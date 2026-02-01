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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* For sleep() */
#include <unistd.h>
/* For memcpy() */
#include <string.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2CTarget.h>
#include <ti/segger/SEGGER_RTT.h>

#include "hdd_i2c_config.h"
#include "nas2300.h"
#include "pt100.h"

/* Driver configuration */
#include "ti_drivers_config.h"

/* With 0x81 reporting length as a uint8_t, max representable length is 255.
 * Use an 8-byte multiple to keep room for future expansion.
 */
#define READY_PAYLOAD_MAX_LEN_BYTES 248

static volatile uint8_t g_regMode = 0x00;  /* reg 0x80: Mode */
static volatile uint8_t g_regReady = 0x00; /* reg 0x81: Ready */
static volatile uint8_t
    g_readyPayload[READY_PAYLOAD_MAX_LEN_BYTES]; /* reg 0x82: Data */
static volatile uint8_t g_errorCode = 0x00;      /* reg 0x83: Error Code */

typedef enum {
  STATE_WAIT_FOR_CMD = 1,    /*!< I2C Target is waiting for command byte */
  STATE_PROCESS_ARGS = 2,    /*!< I2C Target is processing command arguments */
  STATE_PROCESS_PAYLOAD = 3, /*!< I2C Target is processing command payload */
  STATE_CMD_DONE = 4         /*!< I2C Target has completed processing command */
} ProtocolState;

typedef struct {
  uint8_t id;        /*!< Command ID */
  uint8_t argCount;  /*!< Number of arguments in the command */
  uint8_t dataCount; /*!< Number of data bytes */
  uint8_t args[4];   /*!< Command arguments (4 bytes supported) */
  uint8_t argIdx;    /*!< Internal. Current argument index */
  uint8_t* dataPtr;  /*!< Pointer to data */
  uint16_t dataIdx;  /*!< Internal. Current data index. */
} Command;

/* Local variables */
uint8_t rxBuffer[BUFFER_SIZE];
uint8_t txBuffer[BUFFER_SIZE];
ProtocolState protocolState;
Command cmd;

/* Function prototypes */
static int_fast16_t i2cTargetCallback(I2CTarget_Handle handle,
                                      I2CTarget_Event event, uint8_t* val);
static int cmdHandler(I2CTarget_Handle handle, uint8_t* data,
                      HDD_I2C_DIRECTION direction);
/************************************************/
static uint8_t targetAddress;

/* Add prototype */
static int regMode80Handler(I2CTarget_Handle handle, uint8_t* data,
                            HDD_I2C_DIRECTION direction);
static int regReady81Handler(I2CTarget_Handle handle, uint8_t* data,
                             HDD_I2C_DIRECTION direction);
static int regData82Handler(I2CTarget_Handle handle, uint8_t* data,
                            HDD_I2C_DIRECTION direction);

static void i2cErrorHandler(I2C_Transaction* transaction);

/*****************************************/
/*
 *  ======== mainThread ========
 */
void* mainThread(void* arg0) {
  I2CTarget_Handle i2cHandle;
  I2CTarget_Params i2cParams;

  /* Configure the LED0 and LED1*/
  GPIO_setConfig(CONFIG_GPIO_LED_0,
                 GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | CONFIG_GPIO_LED_0_IOMUX);

  SEGGER_RTT_printf(0, "Starting the I2CTarget\n");

  /* Open I2CTarget driver */
  I2CTarget_Params_init(&i2cParams);
  i2cParams.eventCallbackFxn = i2cTargetCallback;
  i2cParams.targetAddress = HDD_I2C_TARGET_ADDRESS;
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
static int_fast16_t i2cTargetCallback(I2CTarget_Handle handle,
                                      I2CTarget_Event event, uint8_t* val) {
  int retCode = I2CTarget_STATUS_SUCCESS;
  if (event == I2CTarget_Event_WRITE_REQUESTED) {
    /* Do nothing. */
    return retCode;
  }
  if (event == I2CTarget_Event_WRITE_RECEIVED) {
    /* Controller has sent a byte to us, send to command handler for processing
     */
    retCode = cmdHandler(handle, val, HDD_I2C_DIRECTION_WRITE);
  }

  if ((event == I2CTarget_Event_READ_REQUESTED) ||
      (event == I2CTarget_Event_READ_PROCESSED)) {
    if (protocolState == STATE_WAIT_FOR_CMD) {
      /* Protocol expects 1 write from controller (CMD), but we got read.
       * Send dummy data.
       */
      *val = 0xff;
      /* We should always return success. */
      retCode = I2CTarget_STATUS_SUCCESS;
    } else {
      retCode = cmdHandler(handle, val, HDD_I2C_DIRECTION_READ);
    }
  }

  if (protocolState == STATE_CMD_DONE) {
    /* Command complete, reset state machine. */
    protocolState = STATE_WAIT_FOR_CMD;
  }

  if (event == I2CTarget_Event_STOP) {
    /* Stop condition, reset state machine. */
    protocolState = STATE_WAIT_FOR_CMD;
  }

  return retCode;
}

/*
 *  ======== cmdHandler ========
 */
static int cmdHandler(I2CTarget_Handle handle, uint8_t* data,
                      HDD_I2C_DIRECTION direction) {
  int retCode = 0; /* Assume success */
  if (protocolState == STATE_WAIT_FOR_CMD) {
    /* The data argument contains command byte */
    cmd.id = *data;

    /* Reset variables */
    cmd.argCount = 0;
    cmd.argIdx = 0;
    cmd.dataCount = 0;
    cmd.dataIdx = 0;
  }

  if (cmd.id == REG_MODE_0x80) {
    return regMode80Handler(handle, data, direction);
  } else if (cmd.id == REG_READY_0x81) {
    return regReady81Handler(handle, data, direction);
  } else if (cmd.id == REG_DATA_0x82) {
    return regData82Handler(handle, data, direction);
  } else {
    /* Unsupported command */
    return -1;
  }

  return retCode;
}

/*
 *  ======== i2cErrorHandler ========
 */
static void i2cErrorHandler(I2C_Transaction* transaction) {
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
 * - READ : controller sends 0x80 then repeated-start READ 1 byte -> return
 * g_regMode
 */
static int regMode80Handler(I2CTarget_Handle handle, uint8_t* data,
                            HDD_I2C_DIRECTION direction) {
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
      if (direction == HDD_I2C_DIRECTION_READ) {
        /* Return Mode */
        *data = g_regMode;
      } else {
        SEGGER_RTT_printf(0,
                          "regMode80Handler: processing payload, data=0x%02x\n",
                          (unsigned)(*data));
        g_regMode = *data;
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
 * - READ : controller sends 0x81 then repeated-start READ 1 byte -> return
 * g_regReady
 */
static int regReady81Handler(I2CTarget_Handle handle, uint8_t* data,
                             HDD_I2C_DIRECTION direction) {
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
      // if (g_i2cIsReadPhase) {
      //     if ((g_regMode == 0xD1) &&
      //         (g_latestTempSeq != g_pt100RequestSeq) &&
      //         (g_latestForceSeq != g_nsa2300RequestSeq)) {
      //         /* Build payload once per new pair of samples */
      //         if ((g_readyPayloadTempSeq != g_latestTempSeq) ||
      //             (g_readyPayloadForceSeq != g_latestForceSeq)) {
      //             /* Use raw NSA2300 24-bit pressure value (p24) directly
      //              * in the payload. g_latestForce_p24 holds the low 24 bits
      //              * of the sensor reading (MSB..LSB), so pack them
      //              * little-endian into the first 3 bytes and zero-extend
      //              * the MSB for simplicity.
      //              */
      //             const uint32_t pressure_p24 = (uint32_t)g_latestForce_p24;
      //             const int16_t temp_x10 = g_latestTemp_x10;

      //             /* Current payload format is 8 bytes:
      //              * pressure(4) + temp_x10(2) + crc16_modbus(2)
      //              */
      //             const uint8_t payloadLen = 8;
      //             uint8_t local[8] = {0};
      //             /* 4 bytes pressure (little-endian) - low 24 bits valid */
      //             local[0] = (uint8_t)((pressure_p24) & 0xFFu);
      //             local[1] = (uint8_t)(((pressure_p24) >> 8) & 0xFFu);
      //             local[2] = (uint8_t)(((pressure_p24) >> 16) & 0xFFu);
      //             local[3] = (uint8_t)(((pressure_p24) >> 24) & 0xFFu);
      //             /* 2 bytes temp_x10 (little-endian) */
      //             local[4] = (uint8_t)((uint16_t)temp_x10 & 0xFFu);
      //             local[5] = (uint8_t)(((uint16_t)temp_x10 >> 8) & 0xFFu);

      //             /* CRC16 over first 6 bytes, little-endian */
      //             uint16_t crc = crc16_modbus(local, 6);
      //             local[6] = (uint8_t)(crc & 0xFFu);
      //             local[7] = (uint8_t)((crc >> 8) & 0xFFu);

      //             for (size_t i = 0; i < payloadLen; i++) {
      //                 g_readyPayload[i] = local[i];
      //             }
      //             g_readyPayloadLen = payloadLen;
      //             g_readyPayloadTempSeq = g_latestTempSeq;
      //             g_readyPayloadForceSeq = g_latestForceSeq;
      //         }

      //         g_regReady = g_readyPayloadLen;
      //         *data = g_readyPayloadLen;
      //     } else {
      //         *data = g_regReady;
      //     }
      // } else {
      //     g_regReady = *data;
      // }
      protocolState = STATE_CMD_DONE;
      break;

    default:
      protocolState = STATE_CMD_DONE;
      break;
  }
  return 0;
}

/* reg 0x82 (Payload stream) R/O:
 * - READ : controller writes 0x82, then repeated-start READ N bytes.
 *          Each subsequent byte clocks out the next payload byte (little-endian
 * stream).
 */
static int regData82Handler(I2CTarget_Handle handle, uint8_t* data,
                            HDD_I2C_DIRECTION direction) {
  (void)handle;

  switch (protocolState) {
    case STATE_WAIT_FOR_CMD:
      protocolState = STATE_PROCESS_PAYLOAD;
      break;

    case STATE_PROCESS_PAYLOAD:
      // if (g_i2cIsReadPhase) {
      //     g_data82ReadInProgress = true;
      //     if (g_readyPayloadReadIdx < g_readyPayloadLen) {
      //         *data = g_readyPayload[g_readyPayloadReadIdx++];
      //     } else {
      //         *data = 0x00;
      //     }
      // } else {
      //     /* Ignore writes */
      // }
      /* Keep streaming until STOP resets the state machine */
      protocolState = STATE_PROCESS_PAYLOAD;
      break;

    default:
      protocolState = STATE_CMD_DONE;
      break;
  }
  return 0;
}