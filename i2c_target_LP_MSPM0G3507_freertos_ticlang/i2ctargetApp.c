/*
 * Copyright (c) 2023-2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== i2cTarget.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/* For sleep() */
#include <unistd.h>
/* For memcpy() */
#include <string.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

#ifndef portYIELD_FROM_ISR
#define portYIELD_FROM_ISR(x) do { (void)(x); } while (0)
#endif

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2CTarget.h>
#include <ti/segger/SEGGER_RTT.h>
#include "cmdinterface.h"
#include "hdd_i2c_config.h"
#include "i2ccontroller.h"

#include "pt100.h"
#include "nas2300.h"

/* Driver configuration */
#include "ti_drivers_config.h"
#include "hdd_i2c_config.h"


/* With 0x81 reporting length as a uint8_t, max representable length is 255.
 * Use an 8-byte multiple to keep room for future expansion.
 */
#define READY_PAYLOAD_MAX_LEN_BYTES 248

static uint8_t g_part_id = HDD_I2C_PART_ID;
static uint8_t g_revision_id = HDD_I2C_REVISION_ID;
static uint8_t g_device_count = 0;
static uint8_t g_device_data = 0;
static volatile uint8_t g_regMode = 0x00;   /* reg 0x80: Mode */
static volatile uint8_t g_regReady = 0x00;  /* reg 0x81: Ready */
static bool g_i2cIsReadPhase = false;       /* set by callback before calling cmdHandler() */

static volatile uint32_t g_pt100RequestSeq = 0;
static volatile uint32_t g_nsa2300RequestSeq = 0;

static volatile uint8_t g_readyPayload[READY_PAYLOAD_MAX_LEN_BYTES];
static volatile uint8_t g_readyPayloadLen = 0;
static volatile uint32_t g_readyPayloadTempSeq = 0;
static volatile uint32_t g_readyPayloadForceSeq = 0;

static volatile uint8_t g_readyPayloadReadIdx = 0;
static volatile bool g_data82ReadInProgress = false;

extern TaskHandle_t NSA2300_getTaskHandle(void);

#define TASKSTACKSIZE 640

typedef enum {
    STATE_WAIT_FOR_CMD = 1, /*!< I2C Target is waiting for command byte */
    STATE_PROCESS_ARGS = 2, /*!< I2C Target is processing command arguments */
    STATE_PROCESS_PAYLOAD = 3, /*!< I2C Target is processing command payload */
    STATE_CMD_DONE = 4 /*!< I2C Target has completed processing command */
} ProtocolState;

#define BUFFER_SIZE 32
typedef struct {
    uint8_t id;        /*!< Command ID */
    uint8_t argCount;  /*!< Number of arguments in the command */
    uint8_t dataCount; /*!< Number of data bytes */
    uint8_t args[4];   /*!< Command arguments (4 bytes supported) */
    uint8_t argIdx;    /*!< Internal. Current argument index */
    uint8_t *dataPtr;  /*!< Pointer to data */
    uint16_t dataIdx;  /*!< Internal. Current data index. */
} Command;

/* Constants */
static const char dataString[] = "Hello, I am the target!";

/* Local variables */
uint8_t rxBuffer[BUFFER_SIZE];
uint8_t txBuffer[BUFFER_SIZE];
ProtocolState protocolState;
Command cmd;

/* Function prototypes */
static int_fast16_t i2cTargetCallback(
    I2CTarget_Handle handle, I2CTarget_Event event, uint8_t *val);
static int cmdHandler(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlGetPartId(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlGetDevCount(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlGetDevData(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlSetStatus(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlGetStatus(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlWriteBlock(I2CTarget_Handle handle, uint8_t *data);
static int cmdCtlReadBlock(I2CTarget_Handle handle, uint8_t *data);
/************************************************/
static uint8_t targetAddress;

/* Add prototype */
static int regMode80Handler(I2CTarget_Handle handle, uint8_t *data);
static int regReady81Handler(I2CTarget_Handle handle, uint8_t *data);
static int regData82Handler(I2CTarget_Handle handle, uint8_t *data);


static void i2cErrorHandler(I2C_Transaction *transaction);

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void *i2cDetectThread(void *arg0) {
    uint8_t devCount;
    uint16_t pt100_temp = 0;
    uint32_t nsa2300_pressure = 0;
    uint8_t local[8] = {0};
    static uint8_t downstreamPayload[READY_PAYLOAD_MAX_LEN_BYTES - 8];
    (void)arg0;
    while(true) {
        usleep(HDD_I2C_DEV_DETECT_PERIOD);
        if (detectI2CDevices(HDD_I2C_TARGET_ADDRESS, &devCount)) {
            g_device_count = devCount;
        } else {
            g_device_count = 0;
        }
        g_device_count += 1; /* Always count self */
        SEGGER_RTT_printf(0, "dev count: %d\n", g_device_count);
        if (getPt100Temp_x10(&pt100_temp)) {
            SEGGER_RTT_printf(0, "pt100 temp:%d\n", pt100_temp);
        }
        if (readNsa2300Raw24(&nsa2300_pressure)) {
            SEGGER_RTT_printf(0, "nsa2300 pressure:%d\n", nsa2300_pressure);
        }
        local[0] = (uint8_t)((uint32_t)nsa2300_pressure & 0xFFu);
        local[1] = (uint8_t)(((uint32_t)nsa2300_pressure >> 8) & 0xFFu);
        local[2] = (uint8_t)(((uint32_t)nsa2300_pressure >> 16) & 0xFFu);
        local[3] = (uint8_t)(((uint32_t)nsa2300_pressure >> 24) & 0xFFu);
        /* 2 bytes temp_x10 (little-endian) */
        local[4] = (uint8_t)((uint16_t)pt100_temp & 0xFFu);
        local[5] = (uint8_t)(((uint16_t)pt100_temp >> 8) & 0xFFu);
        /* CRC16 over first 6 bytes, little-endian */
        uint16_t crc = crc16_modbus(local, 6);
        local[6] = (uint8_t)(crc & 0xFFu);
        local[7] = (uint8_t)((crc >> 8) & 0xFFu);
        memcpy((void *)g_readyPayload, local, sizeof(local));

        uint8_t payloadLen = (uint8_t)sizeof(local);

        if (g_device_count > 1) {
            size_t want = 8u * (size_t)(g_device_count - 1);
            size_t maxDown = sizeof(downstreamPayload);

            if (want > maxDown) {
                SEGGER_RTT_printf(0, "Downstream payload too large: want=%u max=%u\n",
                                  (unsigned)want, (unsigned)maxDown);
            } else {
                if (readDeviceData(downstreamPayload, want)) {
                    memcpy((void *)&g_readyPayload[8], downstreamPayload, want);
                    payloadLen = (uint8_t)(payloadLen + (uint8_t)want);
                } else {
                    /* Don't advertise bytes we failed to fetch. */
                    memset((void *)&g_readyPayload[8], 0, want);
                }
            }
        }

        g_regReady = payloadLen;
        g_readyPayloadLen = payloadLen;

    }
}

/*****************************************/
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    I2CTarget_Handle i2cHandle;
    I2CTarget_Params i2cParams;

    /* Configure the LED0 and LED1*/
    GPIO_setConfig(CONFIG_GPIO_LED_0,
        GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | CONFIG_GPIO_LED_0_IOMUX);

    SEGGER_RTT_printf(0, "Starting the I2CTarget example\n");

    /* Open I2CTarget driver */
    I2CTarget_Params_init(&i2cParams);
    i2cParams.eventCallbackFxn = i2cTargetCallback;
    i2cParams.targetAddress    = HDD_I2C_TARGET_ADDRESS;
    i2cHandle = I2CTarget_open(CONFIG_I2C_TARGET_0, &i2cParams);
    if (i2cHandle == NULL) {
        SEGGER_RTT_printf(0, "Error Initializing I2CTarget\n");
        while (1) {
            ;
        }
    } else {
        SEGGER_RTT_printf(0, "I2C Target Initialized!\n");
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_ON);

    /* Initialize state machine */
    protocolState = STATE_WAIT_FOR_CMD;

    /* Start driver, it will now act on I2C traffic */
    I2CTarget_start(i2cHandle);

    while (1) {
        usleep(1000);
    }
}

/*
 *  ======== i2cTargetCallback ========
 */
static int_fast16_t i2cTargetCallback(
    I2CTarget_Handle handle, I2CTarget_Event event, uint8_t *val)
{
    int retCode = I2CTarget_STATUS_SUCCESS;
    if (event == I2CTarget_Event_WRITE_REQUESTED) {
        /* Do nothing. */
        return retCode;
    }
    if (event == I2CTarget_Event_WRITE_RECEIVED) {
        g_i2cIsReadPhase = false;
        /* Controller has sent a byte to us, send to command handler for processing */
        retCode = cmdHandler(handle, val);
    }

    if ((event == I2CTarget_Event_READ_REQUESTED) ||
        (event == I2CTarget_Event_READ_PROCESSED)) {

        g_i2cIsReadPhase = true;

        if (protocolState == STATE_WAIT_FOR_CMD) {
            /* Protocol expects 1 write from controller (CMD), but we got read.
             * Send dummy data.
             */
            *val = 0xff;
            /* We should always return success. */
            retCode = I2CTarget_STATUS_SUCCESS;
        } else {
            retCode = cmdHandler(handle, val);
        }
    }

    if (protocolState == STATE_CMD_DONE) {
        /* Command complete, reset state machine. */
        protocolState = STATE_WAIT_FOR_CMD;
    }

    if (event == I2CTarget_Event_STOP) {
        /* If the controller just finished a *valid* read of Data(0x82), clear
         * Mode/Ready so the next controller cycle must re-trigger.
         *
         * Note: we intentionally do NOT clear state for spurious/early reads of
         * 0x82 when no payload is ready yet.
         */
        if (g_data82ReadInProgress) {
            g_data82ReadInProgress = false;
            g_regReady = 0;
            g_regMode = 0;
            g_readyPayloadReadIdx = 0;
            g_readyPayloadLen = 0;
        }
        /* Stop condition, reset state machine. */
        protocolState = STATE_WAIT_FOR_CMD;
    }

    return retCode;
}

/*
 *  ======== cmdHandler ========
 */
static int cmdHandler(I2CTarget_Handle handle, uint8_t *data)
{
    int retCode = 0; /* Assume success */
    if (protocolState == STATE_WAIT_FOR_CMD) {
        /* The data argument contains command byte */
        cmd.id = *data;

        /* Reset variables */
        cmd.argCount  = 0;
        cmd.argIdx    = 0;
        cmd.dataCount = 0;
        cmd.dataIdx   = 0;
    }

    if (cmd.id == REG_PART_ID_0x00) {
        return cmdCtlGetPartId(handle, data);
    }

    if (cmd.id == REG_DEV_COUNT_0x02) {
        return cmdCtlGetDevCount(handle, data);
    }

    if (cmd.id == REG_DATA_0x03) {
        return cmdCtlGetDevData(handle, data);
    }

    /* --- Add: reg 0x80 handling (memory-mapped style) --- */
    if (cmd.id == REG_MODE_0x80) {
        return regMode80Handler(handle, data);
    }

    /* --- Add: reg 0x81 handling (memory-mapped style) --- */
    if (cmd.id == REG_READY_0x81) {
        return regReady81Handler(handle, data);
    }

    /* --- Add: reg 0x82 payload stream (read N bytes starting at 0x82) --- */
    if (cmd.id == REG_DATA_0x82) {
        return regData82Handler(handle, data);
    }

    if (cmd.id == CMD_CTL_GET_STATUS) {
        /* Controller wants to read a byte from us */
        retCode = cmdCtlGetStatus(handle, data);
    } else if (cmd.id == CMD_CTL_READ_BLOCK) {
        /* Controller wants to read a block of data from us */
        retCode = cmdCtlReadBlock(handle, data);
    } else if (cmd.id == CMD_CTL_SET_STATUS) {
        /* Controller wants to write a byte to us */
        retCode = cmdCtlSetStatus(handle, data);
    } else if (cmd.id == CMD_CTL_WRITE_BLOCK) {
        /* Controller wants to write a block of data to us */
        retCode = cmdCtlWriteBlock(handle, data);
    } else {
        /* Unsupported command */
        return -1;
    }

    return retCode;
}

/*
 *  ======== cmdCtlSetStatus ========
 */
static int cmdCtlSetStatus(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Update state. We expect to receive data the next time around. */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;
        case STATE_PROCESS_ARGS:
            /* No argument processing */
            break;
        case STATE_PROCESS_PAYLOAD:
            /* Data contains the byte to receive */
            rxBuffer[0]   = *data;
            protocolState = STATE_CMD_DONE;
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }
    return 0;
}

/*
 *  ======== cmdCtlGetDevCount ========
 */
static int cmdCtlGetDevCount(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Update state. We'll send data the next time around. */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;
        case STATE_PROCESS_ARGS:
            /* No command processing */
            break;
        case STATE_PROCESS_PAYLOAD:
            /* Give controller the byte value it wants to read. */
            *data         = g_device_count; /* Return the device count */
            protocolState = STATE_CMD_DONE;
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }

    return 0;
}

/*
 *  ======== cmdCtlGetPartId ========
 */
static int cmdCtlGetPartId(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Update state. We'll send data the next time around. */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;
        case STATE_PROCESS_ARGS:
            /* No command processing */
            break;
        case STATE_PROCESS_PAYLOAD:
            /* Give controller the byte value it wants to read. */
            *data         = g_part_id; /* Return the part ID */
            protocolState = STATE_CMD_DONE;
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }

    return 0;
}


/*
 *  ======== cmdCtlGetStatus ========
 */
static int cmdCtlGetStatus(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Update state. We'll send data the next time around. */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;
        case STATE_PROCESS_ARGS:
            /* No command processing */
            break;
        case STATE_PROCESS_PAYLOAD:
            /* Give controller the byte value it wants to read. */
            *data         = rxBuffer[0]; /* Return the last received data */
            protocolState = STATE_CMD_DONE;
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }

    return 0;
}

/*
 *  ======== cmdCtlWriteBlock ========
 */
static int cmdCtlWriteBlock(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Command byte received. Command has 3 additional argument bytes */
            cmd.argCount  = 3;
            protocolState = STATE_PROCESS_ARGS;
            break;
        case STATE_PROCESS_ARGS:
            /* Reading the command arguments */
            if (cmd.argIdx < cmd.argCount) {
                cmd.args[cmd.argIdx++] = *data;

                if (cmd.argIdx == cmd.argCount) {
                    /* All arguments read, prepare for sending response */
                    uint8_t startOffset = cmd.args[0];
                    uint16_t count =
                        ((uint16_t) cmd.args[1] << 8) | (cmd.args[2]);

                    cmd.dataPtr   = &rxBuffer[startOffset];
                    cmd.dataCount = count;
                    protocolState = STATE_PROCESS_PAYLOAD;
                }
            }
            break;
        case STATE_PROCESS_PAYLOAD:
            if (cmd.dataIdx < cmd.dataCount) {
                cmd.dataPtr[cmd.dataIdx++] = *data;
                if (cmd.dataIdx == cmd.dataCount) {
                    protocolState = STATE_CMD_DONE;
                }
            } else {
                protocolState = STATE_CMD_DONE;
            }
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }

    return 0;
}

/*
 *  ======== cmdCtlReadBlock ========
 */
static int cmdCtlReadBlock(I2CTarget_Handle handle, uint8_t *data)
{
    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* Prepare for upcoming command arguments */
            cmd.argCount  = 3;
            protocolState = STATE_PROCESS_ARGS;
            break;
        case STATE_PROCESS_ARGS:
            /* Read command arguments */
            if (cmd.argIdx < cmd.argCount) {
                /* Store argument */
                cmd.args[cmd.argIdx++] = *data;

                if (cmd.argIdx == cmd.argCount) {
                    /* All arguments read, prepare for upcoming command data.
                     * args[0] = startOffset ("address")
                     * args[1] = byteCount (MSB)
                     * args[2] = byteCount (LSB)
                     */
                    uint8_t startOffset = cmd.args[0];
                    uint16_t count =
                        ((uint16_t)(cmd.args[1]) << 8) | cmd.args[2];

                    /* Copy response to TX buffer */
                    memcpy(txBuffer, dataString, count);

                    cmd.dataPtr   = &txBuffer[startOffset];
                    cmd.dataCount = count;
                    protocolState = STATE_PROCESS_PAYLOAD;
                }
            }
            break;
        case STATE_PROCESS_PAYLOAD:
            /* Feed data to controller */
            if (cmd.dataIdx < cmd.dataCount) {
                *data = cmd.dataPtr[cmd.dataIdx++];
                if (cmd.dataIdx == cmd.dataCount) {
                    protocolState = STATE_CMD_DONE;
                }
            } else {
                protocolState = STATE_CMD_DONE;
            }
            break;
        case STATE_CMD_DONE:
            /* We do not expect to get here */
            break;
        default:
            break;
    }

    return 0;
}

/*
 *  ======== i2cErrorHandler ========
 */
static void i2cErrorHandler(I2C_Transaction *transaction)
{
    switch (transaction->status) {
        case I2C_STATUS_TIMEOUT:
            SEGGER_RTT_printf(0, "I2C transaction timed out!");
            break;
        case I2C_STATUS_CLOCK_TIMEOUT:
            SEGGER_RTT_printf(0, "I2C serial clock line timed out!");
            break;
        case I2C_STATUS_ADDR_NACK:
            SEGGER_RTT_printf(0,
                "I2C target address 0x%x not"
                " acknowledged!",
                transaction->targetAddress);
            break;
        case I2C_STATUS_DATA_NACK:
            SEGGER_RTT_printf(0, "I2C data byte not acknowledged!");
            break;
        case I2C_STATUS_ARB_LOST:
            SEGGER_RTT_printf(0, "I2C arbitration to another controller!");
            break;
        case I2C_STATUS_INCOMPLETE:
            SEGGER_RTT_printf(0, "I2C transaction returned before completion!");
            break;
        case I2C_STATUS_BUS_BUSY:
            SEGGER_RTT_printf(0, "I2C bus is already in use!");
            break;
        case I2C_STATUS_CANCEL:
            SEGGER_RTT_printf(0, "I2C transaction cancelled!");
            break;
        case I2C_STATUS_INVALID_TRANS:
            SEGGER_RTT_printf(0, "I2C transaction invalid!");
            break;
        case I2C_STATUS_ERROR:
            SEGGER_RTT_printf(0, "I2C generic error!");
            break;
        default:
            SEGGER_RTT_printf(0, "I2C undefined error case!");
            break;
    }
}

/* reg 0x80 (Mode) R/W:
 * - WRITE: controller sends 0x80 then 1 byte value -> update g_regMode
 * - READ : controller sends 0x80 then repeated-start READ 1 byte -> return g_regMode
 */
static int regMode80Handler(I2CTarget_Handle handle, uint8_t *data)
{
    (void)handle;

    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* After receiving 0x80, next phase can be either:
             * - a WRITE of the value
             * - or a READ request (repeated-start)
             */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        case STATE_PROCESS_PAYLOAD:
            if (g_i2cIsReadPhase) {
                /* Return Mode */
                *data = g_regMode;
            } else {
                /* Update Mode with written value */
                g_regMode = *data;

                /* When Mode switches to 0xD1, notify PT100 thread to sample */
                if (g_regMode == 0xD1) {
                    g_regReady = 0;
                    g_readyPayloadLen = 0;
                    g_readyPayloadReadIdx = 0;
                    g_data82ReadInProgress = false;
                    g_pt100RequestSeq = g_latestTempSeq;
                    g_nsa2300RequestSeq = g_latestForceSeq;

                    /* Invalidate previously built payload */
                    g_readyPayloadTempSeq = g_pt100RequestSeq;
                    g_readyPayloadForceSeq = g_nsa2300RequestSeq;

                    TaskHandle_t pt100Task = PT100_getTaskHandle();
                    if (pt100Task != NULL) {
                        BaseType_t hpw = pdFALSE;
                        vTaskNotifyGiveFromISR(pt100Task, &hpw);
                        portYIELD_FROM_ISR(hpw);
                    }

                    TaskHandle_t nsaTask = NSA2300_getTaskHandle();
                    if (nsaTask != NULL) {
                        BaseType_t hpw2 = pdFALSE;
                        vTaskNotifyGiveFromISR(nsaTask, &hpw2);
                        portYIELD_FROM_ISR(hpw2);
                    }
                } else {
                    /* Optional: clear ready when leaving D1 */
                    g_regReady = 0;
                    g_readyPayloadLen = 0;
                }
            }
            protocolState = STATE_CMD_DONE;
            break;

        default:
            protocolState = STATE_CMD_DONE;
            break;
    }
    return 0;
}

/* reg 0x81 (Ready) R/W:
 * - WRITE: controller sends 0x81 then 1 byte value -> update g_regReady
 * - READ : controller sends 0x81 then repeated-start READ 1 byte -> return g_regReady
 */
static int regReady81Handler(I2CTarget_Handle handle, uint8_t *data)
{
    (void)handle;

    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            /* After receiving 0x81, next phase can be either:
             * - a WRITE of the value
             * - or a READ request (repeated-start)
             */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        case STATE_PROCESS_PAYLOAD:
            if (g_i2cIsReadPhase) {
                /* Return Ready */
                *data = g_regReady;
            } else {
                /* Update Ready with written value */
                g_regReady = *data;
            }
            protocolState = STATE_CMD_DONE;
            break;

        default:
            protocolState = STATE_CMD_DONE;
            break;
    }
    return 0;
}

/* reg 0x03 (Payload stream) R/O:
 * - READ : controller writes 0x03, then repeated-start READ N bytes.
 *          Each subsequent byte clocks out the next payload byte (little-endian stream).
 */
static int cmdCtlGetDevData(I2CTarget_Handle handle, uint8_t *data)
{
    (void)handle;

    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            g_readyPayloadReadIdx = 0;
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        case STATE_PROCESS_PAYLOAD:
            if (g_i2cIsReadPhase) {
                /* Only treat this as a real payload read when we're in the
                 * expected mode and a non-zero payload length has been latched.
                 * This prevents a controller probing 0x82 too early from
                 * accidentally triggering the STOP-time clear.
                 */
                if (g_readyPayloadLen > 0) {
                    if (g_readyPayloadReadIdx < g_readyPayloadLen) {
                        *data = g_readyPayload[g_readyPayloadReadIdx++];
                    } else {
                        *data = 0x00;
                    }
                } else {
                    *data = 0x00;
                }
            } else {
                /* Ignore writes */
            }
            /* Keep streaming until STOP resets the state machine */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        default:
            protocolState = STATE_CMD_DONE;
            break;
    }
    return 0;
}

/* reg 0x82 (Payload stream) R/O:
 * - READ : controller writes 0x82, then repeated-start READ N bytes.
 *          Each subsequent byte clocks out the next payload byte (little-endian stream).
 */
static int regData82Handler(I2CTarget_Handle handle, uint8_t *data)
{
    (void)handle;

    switch (protocolState) {
        case STATE_WAIT_FOR_CMD:
            g_readyPayloadReadIdx = 0;
            g_data82ReadInProgress = false;
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        case STATE_PROCESS_PAYLOAD:
            if (g_i2cIsReadPhase) {
                /* Only treat this as a real payload read when we're in the
                 * expected mode and a non-zero payload length has been latched.
                 * This prevents a controller probing 0x82 too early from
                 * accidentally triggering the STOP-time clear.
                 */
                if ((g_regMode == 0xD1) && (g_readyPayloadLen > 0)) {
                    g_data82ReadInProgress = true;
                    if (g_readyPayloadReadIdx < g_readyPayloadLen) {
                        *data = g_readyPayload[g_readyPayloadReadIdx++];
                    } else {
                        *data = 0x00;
                    }
                } else {
                    *data = 0x00;
                }
            } else {
                /* Ignore writes */
            }
            /* Keep streaming until STOP resets the state machine */
            protocolState = STATE_PROCESS_PAYLOAD;
            break;

        default:
            protocolState = STATE_CMD_DONE;
            break;
    }
    return 0;
}