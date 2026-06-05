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
static volatile uint8_t g_regCtrl = REG_CTRL_DEFAULT; /* reg 0x84: Control */
static volatile bool g_nsa2300ReinitRequired = false;  /* set by ISR when input_swap changes */
/* Calibration write: ISR stores 8 received bytes here and sets pending flag; 
 * mainThread will perform the flash write in task context. */
static volatile uint8_t gCalibrationBuf[8];
static volatile bool gCalibrationPending = false;

static volatile uint8_t payloadBuf[2][READY_PAYLOAD_MAX_LEN_BYTES] = {
  {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCA},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}
};

static volatile uint8_t activeIndex = 0;
static volatile uint8_t reg82SnapshotLen;
static volatile uint8_t reg82Snapshot[READY_PAYLOAD_MAX_LEN_BYTES];
static volatile bool    reg82SnapshotValid = false;
static volatile bool    reg82Pending = false;
static volatile uint8_t reg83Snapshot[8];
static volatile uint8_t reg83SnapshotLen;
static volatile uint8_t gReg83TxCount = 0;

void updatePayloadData(uint8_t* data, uint8_t len) {
  uint8_t inactive;

    if (len > READY_PAYLOAD_MAX_LEN_BYTES) {
        len = READY_PAYLOAD_MAX_LEN_BYTES;
    }

    taskENTER_CRITICAL();

    inactive = (uint8_t)(activeIndex ^ 1);

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

uint8_t I2CTarget_getRegCtrl(void) {
  return (uint8_t)g_regCtrl;
}

bool I2CTarget_isNsa2300ReinitRequired(void) {
  return g_nsa2300ReinitRequired;
}

void I2CTarget_clearNsa2300ReinitRequired(void) {
  g_nsa2300ReinitRequired = false;
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
static volatile uint32_t gI2cTxEmptyCount = 0;
static volatile uint32_t gI2cRxOverflowCount = 0;
static volatile uint32_t gI2cTxUnderflowCount = 0;
static volatile uint32_t gI2cDefaultCount = 0;
static volatile uint32_t gI2cLastIidx = 0;
static volatile uint32_t gReg82ShortLoadCount = 0;
static volatile uint32_t gI2cSoftRecoverCount = 0;
static volatile uint32_t gI2cStuckWatchCount = 0;
static volatile uint8_t gLastRegAddr = 0x00;
static volatile uint8_t gTxResponseByte = 0xA5;
static volatile uint8_t gReg82TxCount = 0;
static volatile uint32_t gI2cPauseDepth = 0;

#define I2C_SOFT_RECOVER_CONFIRM_COUNT 3u
#define REG82_PENDING_WATCHDOG_GRACE_SECONDS 2u

static void i2cTargetResetDeferredState(void) {
  gLastRegAddr = 0x00;
  gTxResponseByte = 0x00;
  gReg82TxCount = 0;
  gReg83TxCount = 0;
  reg83SnapshotLen = 0;
}

/* ----- REG_DATA_0x82 TX DMA offload -----------------------------------------
 * A 0x82 data read is normally served by the CPU refilling the 8-byte TX FIFO
 * from reg82Snapshot[] on every TXFIFO_TRIGGER/EMPTY/UNDERFLOW interrupt. For
 * long payloads this is the dominant I2C0 ISR load and can delay the I2C1
 * controller's completion interrupt when cascaded. Routing the I2C0 target TX
 * FIFO-trigger event to the DMA lets hardware stream the snapshot into the
 * FIFO with no per-byte ISR. Channel 0 source = reg82Snapshot (byte,
 * incrementing), destination = I2C0 target TX data register (byte, fixed),
 * trigger = DMA_I2C0_TX_TRIG, single-transfer mode (one byte per trigger). */
#define I2C0_TX_DMA_CHANNEL 0u
static volatile bool gReg82DmaActive = false;
static volatile uint32_t gReg82DmaCount = 0;

static void i2cTargetInitReg82Dma(void) {
  static const DL_DMA_Config cfg = {
      .trigger = DMA_I2C0_TX_TRIG,
      .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
      .transferMode = DL_DMA_SINGLE_TRANSFER_MODE,
      .extendedMode = DL_DMA_NORMAL_MODE,
      .srcWidth = DL_DMA_WIDTH_BYTE,
      .destWidth = DL_DMA_WIDTH_BYTE,
      .srcIncrement = DL_DMA_ADDR_INCREMENT,
      .destIncrement = DL_DMA_ADDR_UNCHANGED,
  };
  DL_DMA_initChannel(DMA, I2C0_TX_DMA_CHANNEL, &cfg);
}

/* Hand the current 0x82 snapshot to the DMA and route the TX FIFO trigger to
 * it. Must run with the TX FIFO already flushed (done at register-pointer
 * latch). Clock stretching covers the first-byte DMA latency. */
static void i2cTargetStartReg82Dma(void) {
  uint8_t len = reg82SnapshotLen;
  if (len == 0u) {
    return;
  }
  /* Silence the CPU TX FIFO interrupts; the DMA now owns the FIFO. The
   * gReg82DmaActive guard in the handlers is the race-proof safety net. */
  DL_I2C_disableInterrupt(I2C0_INST,
                          DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                          DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                          DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW);
  DL_DMA_setSrcAddr(DMA, I2C0_TX_DMA_CHANNEL, (uint32_t)&reg82Snapshot[0]);
  DL_DMA_setDestAddr(DMA, I2C0_TX_DMA_CHANNEL,
                     (uint32_t)&I2C0_INST->SLAVE.STXDATA);
  DL_DMA_setTransferSize(DMA, I2C0_TX_DMA_CHANNEL, len);
  DL_DMA_enableChannel(DMA, I2C0_TX_DMA_CHANNEL);
  DL_I2C_enableDMAEvent(I2C0_INST, DL_I2C_EVENT_ROUTE_1,
                        DL_I2C_DMA_INTERRUPT_TARGET_TXFIFO_TRIGGER);
  gReg82DmaActive = true;
  gReg82DmaCount++;
  /* Diagnostics: snapshot has been fully handed to the DMA engine. */
  gReg82TxCount = len;
}

/* Tear down the 0x82 DMA path and restore CPU-driven FIFO servicing. Safe to
 * call unconditionally; no-op when DMA was not active. */
static void i2cTargetStopReg82Dma(void) {
  if (!gReg82DmaActive) {
    return;
  }
  DL_I2C_disableDMAEvent(I2C0_INST, DL_I2C_EVENT_ROUTE_1,
                         DL_I2C_DMA_INTERRUPT_TARGET_TXFIFO_TRIGGER);
  DL_DMA_disableChannel(DMA, I2C0_TX_DMA_CHANNEL);
  /* Restore exactly the CPU TX FIFO interrupts disabled in start so other
   * registers (e.g. 0x83) keep their pre-DMA servicing baseline. */
  DL_I2C_enableInterrupt(I2C0_INST,
                         DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                         DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                         DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW);
  gReg82DmaActive = false;
}

static void i2cTargetSoftRecover(void) {
  NVIC_DisableIRQ(I2C0_INT_IRQn);
  i2cTargetStopReg82Dma();
  DL_I2C_disableTarget(I2C0_INST);
  DL_I2C_flushTargetTXFIFO(I2C0_INST);
  DL_I2C_flushTargetRXFIFO(I2C0_INST);
  DL_I2C_clearInterruptStatus(I2C0_INST,
                              DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                              DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                              DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                              DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
                              DL_I2C_INTERRUPT_TARGET_STOP |
                              DL_I2C_INTERRUPT_TARGET_START);
  i2cTargetResetDeferredState();
  reg82SnapshotValid = false;
  reg82Pending = false;
  gI2cSoftRecoverCount++;
  DL_I2C_enableTarget(I2C0_INST);
  NVIC_ClearPendingIRQ(I2C0_INT_IRQn);
  NVIC_EnableIRQ(I2C0_INT_IRQn);
}

static void i2cTargetLoadReg82Chunk(void) {
  if (gReg82DmaActive) {
    return; /* DMA owns the TX FIFO for this 0x82 read */
  }
  while (gReg82TxCount < reg82SnapshotLen) {
    if (!DL_I2C_transmitTargetDataCheck(
            I2C0_INST, reg82Snapshot[gReg82TxCount])) {
      break;
    }
    gReg82TxCount++;
  }
}

static void i2cTargetLoadReg82OnRequest(void) {
  if (gReg82DmaActive) {
    return; /* DMA owns the TX FIFO for this 0x82 read */
  }
  if ((gReg82TxCount < reg82SnapshotLen) &&
      ((DL_I2C_getTargetStatus(I2C0_INST) &
        DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) != 0u)) {
    DL_I2C_transmitTargetData(I2C0_INST, reg82Snapshot[gReg82TxCount]);
    gReg82TxCount++;
  }
  i2cTargetLoadReg82Chunk();
}

static void i2cTargetPrepareReg82Snapshot(void) {
  uint8_t len = g_regReady;
  uint8_t index = activeIndex;

  if (len > READY_PAYLOAD_MAX_LEN_BYTES) {
    len = READY_PAYLOAD_MAX_LEN_BYTES;
  }

  memcpy((void *)reg82Snapshot, (const void *)payloadBuf[index], len);
  reg82SnapshotLen = len;
  reg82SnapshotValid = true;
  reg82Pending = true;
  gReg82TxCount = 0u;
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
  while (gReg83TxCount < reg83SnapshotLen) {
    if (!DL_I2C_transmitTargetDataCheck(
            I2C0_INST, reg83Snapshot[gReg83TxCount])) {
      break;
    }
    gReg83TxCount++;
  }
}

static void i2cTargetLoadReg83OnRequest(void) {
  if ((gReg83TxCount < reg83SnapshotLen) &&
      ((DL_I2C_getTargetStatus(I2C0_INST) &
        DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) != 0u)) {
    DL_I2C_transmitTargetData(I2C0_INST, reg83Snapshot[gReg83TxCount]);
    gReg83TxCount++;
  }
  i2cTargetLoadReg83Chunk();
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
                            DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                            DL_I2C_INTERRUPT_TARGET_TX_DONE |
                            DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
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
                             DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                             DL_I2C_INTERRUPT_TARGET_TX_DONE |
                             DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
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
                         DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY |
                         DL_I2C_INTERRUPT_TARGET_TX_DONE |
                         DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
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

  /* One-time DMA channel setup for streaming REG_DATA_0x82 payloads into the
   * I2C0 target TX FIFO. The per-transaction src/dest/size are programmed in
   * i2cTargetStartReg82Dma(); this only configures the static channel mode. */
  i2cTargetInitReg82Dma();

  /* I2C0 target: priority 2 (below I2C1 controller at level 1) so the
   * controller's transfer-complete interrupt can preempt a long I2C0 TX-FIFO
   * refill and its callback fires before the 200ms wait expires. I2C0 clock
   * stretching protects the upstream frame during that preemption. Level 2 is
   * still >= configMAX_SYSCALL boundary, so the calibration
   * vTaskNotifyGiveFromISR remains valid. */
  NVIC_SetPriority(I2C0_INT_IRQn, (2u << 6));
  NVIC_EnableIRQ(I2C0_INT_IRQn);
  while (1) {
    static uint32_t lastIrqCount = 0;
    static uint32_t lastStopCount = 0;
    static uint32_t lastTargetStatus = 0;
    static uint32_t reg82PendingAgeSeconds = 0;
    uint32_t targetStatus;
    bool stuckCandidate;
    bool pendingWindow;

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_OFF);
    sleep(1);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_0_ON);
    targetStatus = DL_I2C_getTargetStatus(I2C0_INST);
    pendingWindow = reg82Pending;
    if (reg82Pending) {
      if (reg82PendingAgeSeconds < REG82_PENDING_WATCHDOG_GRACE_SECONDS) {
        reg82PendingAgeSeconds++;
      } else {
        reg82Pending = false;
        i2cTargetResetDeferredState();
        reg82PendingAgeSeconds = 0u;
        pendingWindow = false;
      }
    } else {
      reg82PendingAgeSeconds = 0u;
    }
    stuckCandidate = (((targetStatus & DL_I2C_TARGET_STATUS_BUS_BUSY) != 0u) &&
                      (gI2cIrqCount == lastIrqCount) &&
                      (gI2cStopCount == lastStopCount) &&
                      (targetStatus == lastTargetStatus));
    if (stuckCandidate) {
      if (gI2cStuckWatchCount < I2C_SOFT_RECOVER_CONFIRM_COUNT) {
        gI2cStuckWatchCount++;
      }
    } else {
      gI2cStuckWatchCount = 0u;
    }
    if (pendingWindow) {
      gI2cStuckWatchCount = 0u;
    } else if (gI2cStuckWatchCount >= I2C_SOFT_RECOVER_CONFIRM_COUNT) {
      i2cTargetSoftRecover();
      gI2cStuckWatchCount = 0u;
    }
    if (!pendingWindow) {
      lastIrqCount = gI2cIrqCount;
      lastStopCount = gI2cStopCount;
      lastTargetStatus = targetStatus;
    }
    SEGGER_RTT_printf(
        0,
        "IRQ=%lu START=%lu RXTRIG=%lu TXTRIG=%lu TXEMPTY=%lu TXDONE=%lu RXDONE=%lu STOP=%lu OF=%lu UF=%lu DEF=%lu IIDX=%lu R82TX=%u R82LEN=%u R82PEND=%u R82SHORT=%lu SOFTREC=%lu STUCK=%lu TSTAT=0x%08lx gRxCount=%lu P0=0x%02x P1=0x%02x\n",
        (unsigned long)gI2cIrqCount, (unsigned long)gI2cStartCount,
        (unsigned long)gI2cRxTrigCount, (unsigned long)gI2cTxTrigCount,
        (unsigned long)gI2cTxEmptyCount,
        (unsigned long)gI2cTxDoneCount, (unsigned long)gI2cRxDoneCount,
        (unsigned long)gI2cStopCount, (unsigned long)gI2cRxOverflowCount,
        (unsigned long)gI2cTxUnderflowCount,
        (unsigned long)gI2cDefaultCount, (unsigned long)gI2cLastIidx,
        (unsigned)gReg82TxCount, (unsigned)reg82SnapshotLen,
        (unsigned)reg82Pending,
        (unsigned long)gReg82ShortLoadCount,
        (unsigned long)gI2cSoftRecoverCount,
        (unsigned long)gI2cStuckWatchCount,
        (unsigned long)targetStatus,
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
       *
       * Do not invalidate the 0x82 data snapshot here. A normal transfer reads
       * 0x81 first, issues STOP, then starts a new transfer for 0x82. The
       * snapshot must survive that START and is released after the 0x82 read
       * completes. */
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
          /* Tear down any 0x82 DMA left over from a previous read before
           * servicing the new register (a fresh 0x82 read restarts it). */
          i2cTargetStopReg82Dma();
          if ((gLastRegAddr != REG_READY_0x81) &&
              (gLastRegAddr != REG_DATA_0x82)) {
            reg82SnapshotValid = false;
            reg82Pending = false;
          }

          if (gLastRegAddr == REG_MODE_0x80) {
            gTxResponseByte = g_regMode;
            DL_I2C_fillTargetTXFIFO(I2C0_INST, (void *)&g_regMode, 1);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_READY_0x81) {
            /* Copy the complete payload when the master reads 0x81. The
             * subsequent 0x82 read is served from this immutable snapshot, so
             * payload manager updates between the two I2C transactions cannot
             * corrupt the frame and trip the master's CRC check. */
            i2cTargetPrepareReg82Snapshot();
            gTxResponseByte = reg82SnapshotLen;
            DL_I2C_fillTargetTXFIFO(I2C0_INST, (void *)&reg82SnapshotLen, 1);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = gTxResponseByte;
            }
          } else if (gLastRegAddr == REG_DATA_0x82) {
            reg82Pending = false;
            gReg82TxCount = 0;
            gI2cTxDoneCount = 0;
            /* If the master reads 0x82 without a prior 0x81 length read, build
             * a best-effort snapshot from the current active payload. */
            if (!reg82SnapshotValid) {
              i2cTargetPrepareReg82Snapshot();
            }
            /* Stream the snapshot to the TX FIFO via DMA (no per-byte ISR).
             * Clock stretching covers the first-byte fill latency. */
            i2cTargetStartReg82Dma();
            if (!gReg82DmaActive && (gReg82TxCount < reg82SnapshotLen)) {
              gReg82ShortLoadCount++;
            }
          } else if (gLastRegAddr == REG_CALIBRATION_0x83) {
            /* Snapshot calibration bytes so a repeated-start READ of 0x83
             * returns the stored calibration payload. */
            i2cTargetPrepareReg83Snapshot();
            i2cTargetLoadReg83Chunk();
          } else if (gLastRegAddr == REG_CTRL_0x84) {
            DL_I2C_fillTargetTXFIFO(I2C0_INST, (void *)&g_regCtrl, 1);
            if (gTxCount < gTxLen) {
              gTxPacket[gTxCount++] = g_regCtrl;
            }
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
            // g_regReady = data;
            gTxResponseByte = g_regReady;
          } else if ((gRxCount == 2) && (gLastRegAddr == REG_CTRL_0x84)) {
            const uint8_t prevCtrl = g_regCtrl;
            g_regCtrl = data;
            if ((prevCtrl ^ data) & REG_CTRL_BIT1_INPUT_SWAP) {
              g_nsa2300ReinitRequired = true;
            }
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

    case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
      gI2cTxEmptyCount++;
      if (gLastRegAddr == REG_DATA_0x82) {
        i2cTargetLoadReg82OnRequest();
      } else if (gLastRegAddr == REG_CALIBRATION_0x83) {
        i2cTargetLoadReg83OnRequest();
      }
      break;

    case DL_I2C_IIDX_TARGET_TX_DONE:
      gI2cTxDoneCount++;
      /* Keep transaction state until STOP so multi-chunk REG_DATA_0x82 reads
       * can continue refilling the TX FIFO. Clearing here truncates reads
       * after the first FIFO chunk. */
      break;

    case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
      gI2cTxUnderflowCount++;
      if (gLastRegAddr == REG_DATA_0x82) {
        i2cTargetLoadReg82OnRequest();
      } else if (gLastRegAddr == REG_CALIBRATION_0x83) {
        i2cTargetLoadReg83OnRequest();
      }
      break;

    case DL_I2C_IIDX_TARGET_STOP:
      /* no printing from ISR */
      gI2cStopCount++;
      /* Always tear down the 0x82 DMA at STOP so the next transaction starts
       * from a clean CPU-served FIFO state. No-op when DMA was not active. */
      i2cTargetStopReg82Dma();
      if (gLastRegAddr == REG_DATA_0x82) {
        reg82SnapshotValid = false;
        reg82Pending = false;
      }
      dataRx = false;
      gRxCount = 0;
      if (!reg82Pending) {
        i2cTargetResetDeferredState();
      }
      regPointerLatched = false;
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
