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
#include "hdd_i2c_calibraton.h"
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

static volatile uint8_t g_regMode = 0xD1;  /* reg 0x80: Mode */
static volatile uint8_t g_regReady = 0x8; /* reg 0x81: Ready */
static volatile uint8_t g_readyPayload[READY_PAYLOAD_MAX_LEN_BYTES] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}; /* reg 0x82: Data */
static volatile uint8_t g_errorCode = 0x00;      /* reg 0x83: Error Code */
/* Calibration write: ISR stores 8 received bytes here and sets pending flag; 
 * mainThread will perform the flash write in task context. */
static volatile uint8_t gCalibrationBuf[8];
static volatile bool gCalibrationPending = false;

static volatile uint8_t payloadBuf[2][READY_PAYLOAD_MAX_LEN_BYTES] = {
  {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCA},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}
};

#define INVALID_BUFFER_INDEX 0xFFu
static volatile uint8_t activeIndex = 0;
static volatile uint8_t txReadingBuffer = INVALID_BUFFER_INDEX;
static volatile uint8_t reg82SnapshotIndex;
static volatile uint8_t reg82SnapshotLen;
static volatile uint8_t reg83Snapshot[8];
static volatile uint8_t reg83SnapshotLen;
static volatile uint8_t gReg83TxCount = 0;

volatile size_t wrote = 0;

#define I2C_TARGET_TX_FIFO_CHUNK_SIZE 8u

void updatePayloadData(uint8_t* data, uint8_t len) {
  uint8_t inactive;

    if (len > READY_PAYLOAD_MAX_LEN_BYTES) {
        len = READY_PAYLOAD_MAX_LEN_BYTES;
    }

    taskENTER_CRITICAL();

    inactive = (uint8_t)(activeIndex ^ 1);

    /* 防止覆盖正在发送的 buffer */
    if (inactive == txReadingBuffer) {

        /*
         * 当前 inactive buffer 正被 I2C TX 使用。
         *
         * 双buffer无法继续安全切换。
         *
         * 方案：
         * 1. 丢弃本次更新（推荐）
         * 2. 或等待
         */

        taskEXIT_CRITICAL();

        return;
    }

    taskEXIT_CRITICAL();

    /*
     * 写 inactive buffer
     */

    memcpy((void*)payloadBuf[inactive],
           (const void*)data,
           len);

    /*
     * 原子发布
     */

    taskENTER_CRITICAL();

    g_regReady = (uint8_t)len;
    activeIndex = inactive;

    taskEXIT_CRITICAL();
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
static TaskHandle_t calibrationTaskHandle = NULL;
static volatile uint32_t gCalibrationNotifyCount = 0;

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
static volatile uint8_t gReg82TxCount = 0;
static volatile uint32_t gI2cPauseDepth = 0;

static void i2cTargetLoadReg82Chunk(void) {
  if (gReg82TxCount >= reg82SnapshotLen) {
    return;
  }

  size_t remaining = (size_t)(reg82SnapshotLen - gReg82TxCount);
  size_t chunkLen =
      (remaining > I2C_TARGET_TX_FIFO_CHUNK_SIZE)
          ? I2C_TARGET_TX_FIFO_CHUNK_SIZE
          : remaining;

  wrote = DL_I2C_fillTargetTXFIFO(
      I2C0_INST, (void *)&payloadBuf[reg82SnapshotIndex][gReg82TxCount],
      chunkLen);
  gReg82TxCount = (uint8_t)(gReg82TxCount + (uint8_t)wrote);
}

static void i2cTargetPrepareReg83Snapshot(void) {
  uint32_t *ptr = (uint32_t *)MAIN_BASE_ADDRESS;
  uint32_t low = ptr[0];
  uint32_t high = ptr[1];

  reg83Snapshot[0] = (uint8_t)(low & 0xFFu);
  reg83Snapshot[1] = (uint8_t)((low >> 8) & 0xFFu);
  reg83Snapshot[2] = (uint8_t)((low >> 16) & 0xFFu);
  reg83Snapshot[3] = (uint8_t)((low >> 24) & 0xFFu);
  reg83Snapshot[4] = (uint8_t)(high & 0xFFu);
  reg83Snapshot[5] = (uint8_t)((high >> 8) & 0xFFu);
  reg83Snapshot[6] = (uint8_t)((high >> 16) & 0xFFu);
  reg83Snapshot[7] = (uint8_t)((high >> 24) & 0xFFu);
  reg83SnapshotLen = 8u;
  gReg83TxCount = 0u;
}

static void i2cTargetLoadReg83Chunk(void) {
  if (gReg83TxCount >= reg83SnapshotLen) {
    return;
  }

  size_t remaining = (size_t)(reg83SnapshotLen - gReg83TxCount);
  size_t chunkLen =
      (remaining > I2C_TARGET_TX_FIFO_CHUNK_SIZE)
          ? I2C_TARGET_TX_FIFO_CHUNK_SIZE
          : remaining;

  wrote = DL_I2C_fillTargetTXFIFO(
      I2C0_INST, (void *)&reg83Snapshot[gReg83TxCount], chunkLen);
  gReg83TxCount = (uint8_t)(gReg83TxCount + (uint8_t)wrote);
}
/* Log task prototype */
static void i2cLogTask(void *pvParameters);
static void calibrationTask(void *pvParameters);

/* Expose a small accessor so other tasks can detect heavy I2C activity. */
uint32_t I2C_getIrqCount(void) {
  return gI2cIrqCount;
}

void I2CTarget_pauseService(void) {
  taskENTER_CRITICAL();
  gI2cPauseDepth++;
  if (gI2cPauseDepth == 1u) {
    NVIC_DisableIRQ(I2C0_INT_IRQn);
    DL_I2C_disableInterrupt(I2C0_INST,
                            DL_I2C_INTERRUPT_TARGET_START |
                            DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                            DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                            DL_I2C_INTERRUPT_TARGET_TX_DONE |
                            DL_I2C_INTERRUPT_TARGET_STOP);
  }
  taskEXIT_CRITICAL();
}

void I2CTarget_resumeService(void) {
  taskENTER_CRITICAL();
  if (gI2cPauseDepth != 0u) {
    gI2cPauseDepth--;
    if (gI2cPauseDepth == 0u) {
      DL_I2C_enableInterrupt(I2C0_INST,
                             DL_I2C_INTERRUPT_TARGET_START |
                             DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                             DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                             DL_I2C_INTERRUPT_TARGET_TX_DONE |
                             DL_I2C_INTERRUPT_TARGET_STOP);
      NVIC_EnableIRQ(I2C0_INT_IRQn);
    }
  }
  taskEXIT_CRITICAL();
}

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
  gTxCount = 0;
  gTxLen   = I2C_TX_MAX_PACKET_SIZE;

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
  if (xTaskCreate(i2cLogTask, "I2CLog", 256, NULL, 1, &i2cLogTaskHandle) != pdPASS) {
    SEGGER_RTT_printf(0, "I2CTarget: failed to create i2cLogTask\n");
  }
  /* Create calibration task using static allocation to avoid heap exhaustion */
  static StaticTask_t calibrationTaskTCB;
  static StackType_t calibrationTaskStack[192];
  calibrationTaskHandle = xTaskCreateStatic(
      calibrationTask, "Calib", (uint32_t)(sizeof(calibrationTaskStack) / sizeof(StackType_t)),
      NULL, 1, calibrationTaskStack, &calibrationTaskTCB);
  if (calibrationTaskHandle == NULL) {
    SEGGER_RTT_printf(0, "I2CTarget: failed to create calibrationTask (static)\n");
  }

  /* Now enable the I2C IRQ */
  NVIC_EnableIRQ(I2C0_INT_IRQn);
  while (1) {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
    sleep(1);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_ON);
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
  static bool regPointerLatched = false;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t iidx = DL_I2C_getPendingInterrupt(I2C0_INST);
  gI2cIrqCount++;
  gI2cLastIidx = iidx;

  switch (iidx) {
    case DL_I2C_IIDX_TARGET_START:
      gI2cStartCount++;
      /* Start of a transfer phase.
       * Keep TX FIFO and current register context intact here because a
       * repeated-start READ follows the register-pointer write phase.
       * Clearing `gLastRegAddr` / `gReg82TxCount` here truncates multi-block
       * reads after the first FIFO chunk. Fresh transactions are already
       * reset by STOP/TX_DONE cleanup.
       */
      dataRx = false;
      regPointerLatched = false;
      gRxCount = 0;
      gTxCount = 0;
      /* no printing from ISR */
      break;

    case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
      gI2cRxTrigCount++;
      dataRx = true;
      while (DL_I2C_isTargetRXFIFOEmpty(I2C0_INST) != true) {
        uint8_t data = DL_I2C_receiveTargetData(I2C0_INST);

        /* First received byte after START is register address */
        if (!regPointerLatched) {
          gLastRegAddr = data;
          regPointerLatched = true;
          /* New register pointer received: clear stale TX bytes and preload
           * first response byte to avoid controller blocking at read start. */
          DL_I2C_flushTargetTXFIFO(I2C0_INST);

          if (gLastRegAddr == REG_MODE_0x80) {
            gTxResponseByte = g_regMode;
            DL_I2C_fillTargetTXFIFO(I2C0_INST, (void *)&g_regMode, 1);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_READY_0x81) {
            gTxResponseByte = g_regReady;
            DL_I2C_fillTargetTXFIFO(I2C0_INST, (void *)&g_regReady, 1);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_DATA_0x82) {
            gReg82TxCount = 0;
            gI2cTxDoneCount = 0;
            UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
            reg82SnapshotIndex = activeIndex;
            reg82SnapshotLen = g_regReady;
            txReadingBuffer = reg82SnapshotIndex;
            taskEXIT_CRITICAL_FROM_ISR(saved);
            i2cTargetLoadReg82Chunk();
          } else if (gLastRegAddr == REG_CALIBRATION_0x83) {
            /* Snapshot calibration bytes so a repeated-start READ of 0x83
             * returns the stored calibration payload. */
            i2cTargetPrepareReg83Snapshot();
            i2cTargetLoadReg83Chunk();
          } else {
            gTxResponseByte = 0x00;
            DL_I2C_transmitTargetData(I2C0_INST, gTxResponseByte);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          }
        }

        /* byte snapshot for main debug */
        if (gRxCount < gRxLen) {
          gRxPacket[gRxCount++] = data;

          /* Handle register write: [regAddr, ...data...] */
          if ((gRxCount == 2) && (gLastRegAddr == REG_MODE_0x80)) {
            const uint8_t prevMode = g_regMode;
            g_regMode = data;
            gTxResponseByte = g_regMode;
            if ((prevMode != g_regMode) &&
                (g_regMode == (uint8_t)HDD_I2C_MODE_D1)) {
              PayloadManager_requestSampleFromISR();
            }
          } else if ((gRxCount == 2) && (gLastRegAddr == REG_READY_0x81)) {
            g_regReady = data;
            gTxResponseByte = g_regReady;
          } else if ((gLastRegAddr == REG_CALIBRATION_0x83) && (gRxCount == 9)) {
            /* Received reg + 8 bytes payload. Copy into calibration buffer and
             * notify calibration task. Keep ISR work minimal and ISR-safe. */
            uint8_t i;
            for (i = 0; i < 8; ++i) {
              gCalibrationBuf[i] = gRxPacket[1 + i];
            }
            gCalibrationPending = true;
            if (calibrationTaskHandle != NULL) {
              BaseType_t xHigherPriorityTaskWoken = pdFALSE;
              vTaskNotifyGiveFromISR(calibrationTaskHandle, &xHigherPriorityTaskWoken);
              /* increment a counter for diagnostics (ISR-safe) */
              gCalibrationNotifyCount++;
              portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
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

      break;

    case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
      gI2cTxTrigCount++;
      if (gLastRegAddr == REG_DATA_0x82) {
        i2cTargetLoadReg82Chunk();
      } else if (gLastRegAddr == REG_CALIBRATION_0x83) {
        i2cTargetLoadReg83Chunk();
      }
      break;

    case DL_I2C_IIDX_TARGET_TX_DONE:
      gI2cTxDoneCount++;
      /* Keep transaction state until STOP so multi-chunk REG_DATA_0x82 reads
       * can continue refilling the TX FIFO. Clearing here truncates reads
       * after the first 8-byte chunk.
       */
      break;

    case DL_I2C_IIDX_TARGET_STOP:
      /* no printing from ISR */
      gI2cStopCount++;
      dataRx = false;
      gRxCount = 0;
      gLastRegAddr = 0x00;
      gTxResponseByte = 0x00;
      gReg82TxCount = 0;
      gReg83TxCount = 0;
      reg83SnapshotLen = 0;
      regPointerLatched = false;
      /* 释放发送 buffer */
      txReadingBuffer = INVALID_BUFFER_INDEX;
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
      break;

    case DL_I2C_IIDX_TARGET_RX_DONE:
      gI2cRxDoneCount++;
      /* Keep pointer context for repeated-start READ. Cleared by TX_DONE/STOP. */
      // regPointerLatched = false;
    break;
    case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
    case DL_I2C_IIDX_TARGET_GENERAL_CALL:
    case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
    case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
    default:
      gI2cDefaultCount++;
      break;
  }

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
    }
  }
}

static void calibrationTask(void *pvParameters)
{
  (void)pvParameters;
  for (;;) {
    /* Wait until ISR notifies that an 8-byte calibration payload was received */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint8_t localBuf[8];
    int i;
    /* Copy shared buffer under critical to avoid races with ISR */
    taskENTER_CRITICAL();
    for (i = 0; i < 8; ++i) {
      localBuf[i] = gCalibrationBuf[i];
    }
    gCalibrationPending = false;
    taskEXIT_CRITICAL();

    /* Diagnostic: print notify counter and the captured bytes */
    SEGGER_RTT_printf(0, "CalibrationTask: woke (notify_count=%u)\n", (unsigned)gCalibrationNotifyCount);
    SEGGER_RTT_printf(0, "CalibrationTask: bytes: ");
    for (i = 0; i < 8; ++i) {
      SEGGER_RTT_printf(0, "%02X ", (unsigned)localBuf[i]);
    }
    SEGGER_RTT_printf(0, "\n");

    /* Assemble little-endian 64-bit value from the 8 bytes sent by master */
    uint32_t zero = ((uint32_t)localBuf[0]) |
                ((uint32_t)localBuf[1] << 8) |
                ((uint32_t)localBuf[2] << 16) |
                ((uint32_t)localBuf[3] << 24);

    uint32_t full = ((uint32_t)localBuf[4]) |
                    ((uint32_t)localBuf[5] << 8) |
                    ((uint32_t)localBuf[6] << 16) |
                    ((uint32_t)localBuf[7] << 24);

    if (!hddI2CWriteCalibrationPair(zero, full)) {
      SEGGER_RTT_printf(0, "CalibrationTask: calibration flash write failed\n");
    } else {
      SEGGER_RTT_printf(0,
                        "CalibrationTask: calibration flash write succeeded, zero=0x%08X full=0x%08X\n",
                        zero, full);
    }
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
  }
}