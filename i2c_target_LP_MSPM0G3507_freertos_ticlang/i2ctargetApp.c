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
static volatile uint8_t g_readyPayload[READY_PAYLOAD_MAX_LEN_BYTES] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}; /* reg 0x82: Data */
static volatile uint8_t g_errorCode = 0x00;      /* reg 0x83: Error Code */

static volatile uint8_t payloadBuf[2][READY_PAYLOAD_MAX_LEN_BYTES] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}
};
static volatile uint8_t payloadLen[2] = {8, 8};
static volatile uint8_t activeIndex = 0;
static volatile uint8_t reg82SnapshotIndex;
static volatile uint8_t reg82SnapshotLen;

volatile size_t wrote = 0;

void updateReady(uint8_t ready) { g_regReady = ready; }
void updatePayloadData(uint8_t* data, uint8_t len) {
  if (len > READY_PAYLOAD_MAX_LEN_BYTES) {
    len = READY_PAYLOAD_MAX_LEN_BYTES;
  }
  /* Write into inactive buffer, then publish by atomically swapping active index. */
  uint8_t inactive = (uint8_t)(activeIndex ^ 1);
  memcpy((void*)payloadBuf[inactive], (const void*)data, len);

  /* publish length and swap active index atomically */
  taskENTER_CRITICAL();
  payloadLen[inactive] = (uint8_t)len;
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

/* Log task prototype */
static void i2cLogTask(void *pvParameters);

/* Expose a small accessor so other tasks can detect heavy I2C activity. */
uint32_t I2C_getIrqCount(void) {
  return gI2cIrqCount;
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
  xTaskCreate(i2cLogTask, "I2CLog", 256, NULL, 1, &i2cLogTaskHandle);

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
       * Keep TX FIFO intact here because repeated-start READ depends on the
       * response byte preloaded right after register pointer write. */
      dataRx = false;
      regPointerLatched = false;
      gRxCount = 0;
      gTxCount = 0;
      gReg82TxCount = 0;
      gLastRegAddr = 0x00;
      gTxResponseByte = 0x00;
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
            DL_I2C_transmitTargetData(I2C0_INST, gTxResponseByte);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_READY_0x81) {
            gTxResponseByte = 0x08;
            DL_I2C_transmitTargetData(I2C0_INST, gTxResponseByte);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_DATA_0x82) {
            gReg82TxCount = 0;
            gI2cTxDoneCount = 0;
            reg82SnapshotIndex = activeIndex;
            reg82SnapshotLen = payloadLen[reg82SnapshotIndex];

            size_t remaining = payloadLen[reg82SnapshotIndex];
            wrote = DL_I2C_fillTargetTXFIFO(I2C0_INST,
                            (void*)&payloadBuf[reg82SnapshotIndex][gReg82TxCount], 8);
            gReg82TxCount += (uint8_t)wrote;
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

          /* Handle register write: [regAddr, value] */
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
      break;

    case DL_I2C_IIDX_TARGET_TX_DONE:
      gI2cTxDoneCount++;
      /* Read transaction completed, clear parser context for next frame */
      gRxCount = 0;
      gLastRegAddr = 0x00;
      gReg82TxCount = 0;
      regPointerLatched = false;
      // if (gI2cTxDoneCount == 7) {
      //   DL_I2C_fillTargetTXFIFO(I2C0_INST,
      //                       (void*)&payloadBuf[reg82SnapshotIndex][gReg82TxCount], 8);
      // }
      break;

    case DL_I2C_IIDX_TARGET_STOP:
      /* no printing from ISR */
      gI2cStopCount++;
      dataRx = false;
      gRxCount = 0;
      gLastRegAddr = 0x00;
      gTxResponseByte = 0x00;
      gReg82TxCount = 0;
      regPointerLatched = false;
      GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
      break;

    case DL_I2C_IIDX_TARGET_RX_DONE:
      gI2cRxDoneCount++;
      /* Keep pointer context for repeated-start READ. Cleared by TX_DONE/STOP. */
      regPointerLatched = false;
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