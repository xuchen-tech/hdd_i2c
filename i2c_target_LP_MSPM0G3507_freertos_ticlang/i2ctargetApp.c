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
#include "hdd_i2c_payload_manager.h"
#include "i2c_controller.h"
#include "pt100.h"

/* Driver configuration */
#include "ti_drivers_config.h"

/* With 0x81 reporting length as a uint8_t, max representable length is 255.
 * Use an 8-byte multiple to keep room for future expansion.
 */
#define READY_PAYLOAD_MAX_LEN_BYTES 248

/* NOTE: I2CTarget callbacks run in interrupt context.
 * Avoid potentially blocking/slow logging (e.g. SEGGER RTT) inside the I2C ISR
 * path; it can stall the bus and make the controller hang in I2C_transfer().
 */
#define HDD_I2C_TARGET_ISR_LOG 0

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
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];
static ProtocolState protocolState;
static Command cmd;

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

void updateReady(uint8_t ready) { g_regReady = ready; }
void updatePayloadData(uint8_t* data, uint8_t len) {
  if (len > READY_PAYLOAD_MAX_LEN_BYTES) {
    len = READY_PAYLOAD_MAX_LEN_BYTES;
  }
  memcpy((void*)g_readyPayload, (const void*)data, len);
}

/* Maximum size of TX packet */
#define I2C_TX_MAX_PACKET_SIZE (16)

/* Maximum size of RX packet */
#define I2C_RX_MAX_PACKET_SIZE (16)

/* Ring buffer for ISR -> task logging (power-of-two size) */
#define I2C_LOG_RING_SIZE 64
static volatile uint8_t i2cLogRing[I2C_LOG_RING_SIZE];
static volatile uint32_t i2cLogHead = 0; /* written by ISR */
static volatile uint32_t i2cLogTail = 0; /* read by task */
static TaskHandle_t i2cLogTaskHandle = NULL;

/* IRQ diagnostics */
static volatile uint32_t gI2cIrqCount = 0;
static volatile uint32_t gI2cStartCount = 0;
static volatile uint32_t gI2cRxTrigCount = 0;
static volatile uint32_t gI2cStopCount = 0;
static volatile uint32_t gI2cRxDoneCount = 0;
static volatile uint32_t gI2cTxTrigCount = 0;
static volatile uint32_t gI2cTxDoneCount = 0;
static volatile uint32_t gI2cRxOverflowCount = 0;
static volatile uint32_t gI2cDefaultCount = 0;
static volatile uint32_t gI2cLastIidx = 0;
static volatile uint8_t gLastRegAddr = 0x00;
static volatile uint8_t gTxResponseByte = 0xA5;

/* Log task prototype */
static void i2cLogTask(void *pvParameters);

/* Data sent to Controller in response to Read transfer */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {0x00};

/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Controller during a Write transfer */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
uint32_t gRxLen, gRxCount;
/*
 *  ======== mainThread ========
 */
void* mainThread(void* arg0) {
  GPIO_setConfig(CONFIG_GPIO_LED_0,
                 GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | CONFIG_GPIO_LED_0_IOMUX);
  /*
  I2CTarget_Handle i2cHandle;
  I2CTarget_Params i2cParams;

  GPIO_setConfig(CONFIG_GPIO_LED_0,
                 GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | CONFIG_GPIO_LED_0_IOMUX);

  SEGGER_RTT_printf(0, "Starting the I2CTarget\n");

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
  */
  GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
  gTxCount = 0;
  gTxLen   = I2C_TX_MAX_PACKET_SIZE;
  /*
  protocolState = STATE_WAIT_FOR_CMD;

  I2CTarget_start(i2cHandle);
  */
  gRxCount = 0;
  gRxLen   = I2C_RX_MAX_PACKET_SIZE;
  /* Enable RX FIFO trigger interrupt as well as TX FIFO trigger */
  DL_I2C_enableInterrupt(I2C0_INST,
               DL_I2C_INTERRUPT_TARGET_START |
                         DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
       DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                 DL_I2C_INTERRUPT_TARGET_TX_DONE |
                            DL_I2C_INTERRUPT_TARGET_STOP);

  /* Create a low-priority task to drain the ring buffer and print logs
   * (uses blocking / potentially slow calls like SEGGER_RTT_printf)
   * Create the task before enabling the NVIC so the ISR can safely notify
   * the task without racing on the handle being NULL. */
  xTaskCreate(i2cLogTask, "I2CLog", 256, NULL, 1, &i2cLogTaskHandle);

  /* Now enable the I2C IRQ */
  NVIC_EnableIRQ(I2C0_INT_IRQn);
  while (1) {
    sleep(1);
    SEGGER_RTT_printf(
        0,
        "IRQ=%lu START=%lu RXTRIG=%lu TXTRIG=%lu TXDONE=%lu RXDONE=%lu STOP=%lu OF=%lu DEF=%lu IIDX=%lu gRxCount=%lu P0=0x%02x P1=0x%02x\n",
        (unsigned long)gI2cIrqCount, (unsigned long)gI2cStartCount,
        (unsigned long)gI2cRxTrigCount, (unsigned long)gI2cTxTrigCount,
        (unsigned long)gI2cTxDoneCount, (unsigned long)gI2cRxDoneCount,
        (unsigned long)gI2cStopCount, (unsigned long)gI2cRxOverflowCount,
        (unsigned long)gI2cDefaultCount, (unsigned long)gI2cLastIidx,
        (unsigned long)gRxCount, gRxPacket[0], gRxPacket[1]);
  }
}

void I2C0_IRQHandler(void)
{
  static bool dataRx = false;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t iidx = DL_I2C_getPendingInterrupt(I2C0_INST);
  gI2cIrqCount++;
  gI2cLastIidx = iidx;

  switch (iidx) {
    case DL_I2C_IIDX_TARGET_START:
      gI2cStartCount++;
      /* Repeated-start happens between write(reg) and read(data).
       * Do not clear RX context or flush TX FIFO here, otherwise prepared
       * response byte is lost and controller read can block. */
      if (gRxCount == 1) {
        /* Only treat as repeated-start-to-read when exactly one byte
         * (register address) has been received. */
        uint8_t txByte = 0xA5;
        if (gLastRegAddr == REG_MODE_0x80) {
          txByte = g_regMode;
        } else if (gLastRegAddr == REG_READY_0x81) {
          txByte = g_regReady;
        }

        DL_I2C_transmitTargetData(I2C0_INST, txByte);
        if (gTxCount < gTxLen) {
          gTxPacket[gTxCount++] = txByte;
        }
      } else {
        gTxCount = 0;
        DL_I2C_flushTargetTXFIFO(I2C0_INST);
      }
      /* no printing from ISR */
      break;

    case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
      gI2cRxTrigCount++;
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_ON);
      dataRx = true;
      while (DL_I2C_isTargetRXFIFOEmpty(I2C0_INST) != true) {
        uint8_t data = DL_I2C_receiveTargetData(I2C0_INST);

        /* First received byte is register address for subsequent read */
        if (gRxCount == 0) {
          gLastRegAddr = data;
          if (gLastRegAddr == REG_MODE_0x80) {
            gTxResponseByte = g_regMode;
          } else if (gLastRegAddr == REG_READY_0x81) {
            gTxResponseByte = g_regReady;
          } else {
            gTxResponseByte = 0xA5;
          }
        }

        /* byte snapshot for main debug */
        if (gRxCount < gRxLen) {
          gRxPacket[gRxCount++] = data;

          /* Handle register write: [regAddr, value] */
          if ((gRxCount == 2) && (gLastRegAddr == REG_MODE_0x80)) {
            g_regMode = data;
            gTxResponseByte = g_regMode;
          } else if ((gRxCount == 2) && (gLastRegAddr == REG_READY_0x81)) {
            g_regReady = data;
            gTxResponseByte = g_regReady;
          }
        } else {
          gI2cRxOverflowCount++;
        }

        /* ring buffer for task-context logging */
        {
          uint32_t next = (i2cLogHead + 1) & (I2C_LOG_RING_SIZE - 1);
          if (next != i2cLogTail) {
            i2cLogRing[i2cLogHead] = data;
            i2cLogHead = next;
          }
        }
      }

      if (i2cLogTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(i2cLogTaskHandle, &xHigherPriorityTaskWoken);
      }
      break;

    case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
      /* Provide one response byte so controller READ does not stall */
      {
        uint8_t txByte = 0xA5;
        gI2cTxTrigCount++;

        if (gLastRegAddr == REG_MODE_0x80) {
          txByte = g_regMode;
        } else if (gLastRegAddr == REG_READY_0x81) {
          txByte = g_regReady;
        } else if (gLastRegAddr == REG_DATA_0x82) {
          txByte = 0xA5;
        }

        DL_I2C_transmitTargetData(I2C0_INST, txByte);
        if (gTxCount < gTxLen) {
          gTxPacket[gTxCount++] = txByte;
        }
      }
      break;

    case DL_I2C_IIDX_TARGET_TX_DONE:
      gI2cTxDoneCount++;
      break;

    case DL_I2C_IIDX_TARGET_STOP:
      /* no printing from ISR */
      gI2cStopCount++;
      dataRx = false;
      gRxCount = 0;
      gLastRegAddr = 0x00;
      gTxResponseByte = 0xA5;
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
      break;

    case DL_I2C_IIDX_TARGET_RX_DONE:
      gI2cRxDoneCount++;
      gRxCount = 0;
      gLastRegAddr = 0x00;
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
    break;
    case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
    case DL_I2C_IIDX_TARGET_GENERAL_CALL:
    case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
    case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
    default:
      gI2cDefaultCount++;
      break;
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
      *val = 0x00;
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
      // SEGGER_RTT_printf(0, "I2C transaction timed out!");
      break;
    case I2C_STATUS_CLOCK_TIMEOUT:
      // SEGGER_RTT_printf(0, "I2C serial clock line timed out!");
      break;
    case I2C_STATUS_ADDR_NACK:
      // SEGGER_RTT_printf(0,
      //                   "I2C target address 0x%x not"
      //                   " acknowledged!",
      //                   transaction->targetAddress);
      break;
    case I2C_STATUS_DATA_NACK:
      // SEGGER_RTT_printf(0, "I2C data byte not acknowledged!");
      break;
    case I2C_STATUS_ARB_LOST:
      // SEGGER_RTT_printf(0, "I2C arbitration to another controller!");
      break;
    case I2C_STATUS_INCOMPLETE:
      // SEGGER_RTT_printf(0, "I2C transaction returned before completion!");
      break;
    case I2C_STATUS_BUS_BUSY:
      // SEGGER_RTT_printf(0, "I2C bus is already in use!");
      break;
    case I2C_STATUS_CANCEL:
      // SEGGER_RTT_printf(0, "I2C transaction cancelled!");
      break;
    case I2C_STATUS_INVALID_TRANS:
      // SEGGER_RTT_printf(0, "I2C transaction invalid!");
      break;
    case I2C_STATUS_ERROR:
      // SEGGER_RTT_printf(0, "I2C generic error!");
      break;
    default:
      // SEGGER_RTT_printf(0, "I2C undefined error case!");
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
        *data = 0xD1;
      } else {
#if HDD_I2C_TARGET_ISR_LOG
        SEGGER_RTT_printf(0,
                          "regMode80Handler: processing payload, data=0x%02x\n",
                          (unsigned)(*data));
#endif
        const uint8_t prevMode = g_regMode;
        g_regMode = *data;

        if ((prevMode != g_regMode) &&
            (g_regMode == (uint8_t)HDD_I2C_MODE_D1)) {
          // PayloadManager_requestSampleFromISR();
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

/* Background task: drain ring buffer and print logs (runs in task context) */
static void i2cLogTask(void *pvParameters)
{
  (void)pvParameters;

  for (;;) {
    /* Block until ISR notifies us there is data */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Drain buffer */
    while (i2cLogTail != i2cLogHead) {
      uint8_t data = i2cLogRing[i2cLogTail];
      i2cLogTail = (i2cLogTail + 1) & (I2C_LOG_RING_SIZE - 1);
      SEGGER_RTT_printf(0, "I2C0_data: %02x\n", data);
    }
  }
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
      if (direction == HDD_I2C_DIRECTION_READ) {
        /* Return Ready */
        *data = 0x08;
      } else {
#if HDD_I2C_TARGET_ISR_LOG
        SEGGER_RTT_printf(
            0, "regReady81Handler: processing payload, data=0x%02x\n",
            (unsigned)(*data));
#endif
        g_regReady = *data;
        if (g_regReady == 0) {
          g_regMode = HDD_I2C_MODE_IDLE;
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
      cmd.dataCount = 8;
      cmd.dataIdx = 0;
      break;

    case STATE_PROCESS_PAYLOAD:
      if (direction == HDD_I2C_DIRECTION_READ) {
        /* Return next payload byte */
        if (cmd.dataIdx < cmd.dataCount) {
          *data = 0xA5;
          cmd.dataIdx++;
        } else {
          protocolState = STATE_CMD_DONE;
        }
      }
      break;

    default:
      protocolState = STATE_CMD_DONE;
      break;
  }
  return 0;
}