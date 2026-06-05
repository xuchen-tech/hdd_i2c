#include "i2c_controller.h"

#include <string.h>
#include <math.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* For usleep() */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/segger/SEGGER_RTT.h>
#include <unistd.h>

#include "hdd_i2c_config.h"
#include "i2ctargetApp.h"

/* Driver configuration */
#include "ti_drivers_config.h"

uint32_t calibration_low = 0;
uint32_t calibration_high = 0;
/* calibration_valid indicates whether calibration_low/high contain a valid pair */
static volatile bool calibration_valid = false;

static int32_t nsa2300SignExtend24(uint32_t value) {
  value &= 0x00FFFFFFu;
  if ((value & 0x00800000u) != 0u) {
    value |= 0xFF000000u;
  }
  return (int32_t)value;
}

static uint8_t txBuffer[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];

static I2C_Handle g_i2cHandle;
static I2C_Params g_i2cParams;
static volatile bool g_i2cTransferDone = false;
static volatile bool g_i2cTransferStatus = false;

#define I2C_TRANSFER_WAIT_MS 200
#define I2C_TRANSFER_MAX_ATTEMPTS 4
#define I2C_CLOCK_TIMEOUT_CYCLES (I2C_CLOCK_MHZ * 50000u)
#define I2C1_SDA_MASK (1u << GPIO_I2C1_SDA_PIN)
#define I2C1_SCL_MASK (1u << GPIO_I2C1_SCL_PIN)
#define I2C_BUS_CLEAR_PULSES 9u
#define I2C_BUS_CLEAR_DELAY_CYCLES (I2C_CLOCK_MHZ * 50u)
/* Number of bus-clear cycles attempted inside a single recovery before the
 * downstream slave is declared genuinely stuck.  A slave that is mid-clock-
 * stretch may need more than one set of clock pulses to release SDA. */
#define I2C_BUS_CLEAR_MAX_CYCLES 3u
/* Settle time after reopening the peripheral so a slave that was clock
 * stretching can finish its internal operation before the next transfer. */
#define I2C_RECOVER_SETTLE_US 2000u

static void i2cErrorHandler(I2C_Transaction* transaction);
static void i2cTransferCallback(I2C_Handle handle,
                                I2C_Transaction* transaction,
                                bool transferStatus);
static bool i2cTransferAndWait(I2C_Transaction* transaction);
static void i2cClearBus(void);
static bool i2cRecoverController(void);
static bool i2cShouldRecoverTransaction(const I2C_Transaction* transaction);
static bool i2cBusLinesIdle(void);

static void i2cTransferCallback(I2C_Handle handle,
                                I2C_Transaction* transaction,
                                bool transferStatus) {
  (void)handle;

  g_i2cTransferStatus = transferStatus;
  g_i2cTransferDone = true;

  if (!transferStatus && transaction != NULL) {
    SEGGER_RTT_printf(0, "I2C callback: transfer failed, status=%d\n",
                      (int)transaction->status);
  }
}

static bool i2cRecoverController(void) {
  uint32_t clearCycle;
  bool busIdle = false;

  if (g_i2cHandle != NULL) {
    I2C_close(g_i2cHandle);
    g_i2cHandle = NULL;
  }

  /* Bit-bang clock pulses until the slave releases SDA/SCL.  A single
   * 9-pulse burst is not always enough for a slave that is mid-clock-stretch,
   * so repeat the bus clear and verify the lines actually returned high
   * before declaring the bus usable again. */
  for (clearCycle = 0u; clearCycle < I2C_BUS_CLEAR_MAX_CYCLES; ++clearCycle) {
    i2cClearBus();
    if (i2cBusLinesIdle()) {
      busIdle = true;
      break;
    }
  }

  if (!busIdle) {
    SEGGER_RTT_printf(0,
                      "I2C recovery: bus still held after %u clear cycles\n",
                      (unsigned)I2C_BUS_CLEAR_MAX_CYCLES);
  }

  /* Hard-reset the I2C1 peripheral.  i2cClearBus() only bit-bangs the GPIO
   * pins to release a stuck slave; it does not reset the I2C state machine.
   * Without this reset the peripheral may be stuck mid-transfer and never
   * generate interrupts, so the callback never fires. */
  DL_I2C_reset(I2C1_INST);
  DL_I2C_enablePower(I2C1_INST);
  delay_cycles(POWER_STARTUP_DELAY);

  g_i2cTransferDone = false;
  g_i2cTransferStatus = false;

  g_i2cHandle = I2C_open(CONFIG_I2C_0, &g_i2cParams);
  if (g_i2cHandle == NULL) {
    SEGGER_RTT_printf(0, "I2C recovery failed: reopen controller failed\n");
    return false;
  }

  I2C_setClockTimeout(g_i2cHandle, I2C_CLOCK_TIMEOUT_CYCLES);

  /* Give a clock-stretching slave time to finish its internal operation
   * before the next transfer so the immediately-following retry does not
   * time out again. */
  usleep(I2C_RECOVER_SETTLE_US);

  if (!busIdle) {
    /* Do not claim success while the bus is still physically held low; let
     * the caller retry, which will pulse the clock again. */
    SEGGER_RTT_printf(0, "I2C controller reopened but bus not idle\n");
    return false;
  }

  SEGGER_RTT_printf(0, "I2C controller recovered\n");
  return true;
}

static void i2cClearBus(void) {
  uint32_t pulse;

  DL_GPIO_initDigitalOutput(GPIO_I2C1_IOMUX_SCL);
  DL_GPIO_initDigitalOutput(GPIO_I2C1_IOMUX_SDA);
  DL_GPIO_setPins(GPIOA, I2C1_SCL_MASK | I2C1_SDA_MASK);
  DL_GPIO_disableOutput(GPIOA, I2C1_SCL_MASK | I2C1_SDA_MASK);
  delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);

  for (pulse = 0; pulse < I2C_BUS_CLEAR_PULSES; ++pulse) {
    DL_GPIO_clearPins(GPIOA, I2C1_SCL_MASK);
    DL_GPIO_enableOutput(GPIOA, I2C1_SCL_MASK);
    delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);
    DL_GPIO_setPins(GPIOA, I2C1_SCL_MASK);
    DL_GPIO_disableOutput(GPIOA, I2C1_SCL_MASK);
    delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);

    if ((DL_GPIO_readPins(GPIOA, I2C1_SDA_MASK) & I2C1_SDA_MASK) != 0u) {
      break;
    }
  }

  DL_GPIO_clearPins(GPIOA, I2C1_SDA_MASK);
  DL_GPIO_enableOutput(GPIOA, I2C1_SDA_MASK);
  delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);
  DL_GPIO_setPins(GPIOA, I2C1_SCL_MASK);
  DL_GPIO_disableOutput(GPIOA, I2C1_SCL_MASK);
  delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);
  DL_GPIO_setPins(GPIOA, I2C1_SDA_MASK);
  DL_GPIO_disableOutput(GPIOA, I2C1_SDA_MASK);
  delay_cycles(I2C_BUS_CLEAR_DELAY_CYCLES);

  DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C1_IOMUX_SDA,
      GPIO_I2C1_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
      DL_GPIO_WAKEUP_DISABLE);
  DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C1_IOMUX_SCL,
      GPIO_I2C1_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
      DL_GPIO_WAKEUP_DISABLE);
  DL_GPIO_enableHiZ(GPIO_I2C1_IOMUX_SDA);
  DL_GPIO_enableHiZ(GPIO_I2C1_IOMUX_SCL);
}

static bool i2cBusLinesIdle(void) {
  uint32_t pins = DL_GPIO_readPins(GPIOA, I2C1_SCL_MASK | I2C1_SDA_MASK);
  return (pins & (I2C1_SCL_MASK | I2C1_SDA_MASK)) ==
         (I2C1_SCL_MASK | I2C1_SDA_MASK);
}

static bool i2cTransferAndWait(I2C_Transaction* transaction) {
  const TickType_t timeoutTicks = pdMS_TO_TICKS(I2C_TRANSFER_WAIT_MS);
  const void* const originalWriteBuf = transaction != NULL ? transaction->writeBuf : NULL;
  const size_t originalWriteCount = transaction != NULL ? transaction->writeCount : 0u;
  void* const originalReadBuf = transaction != NULL ? transaction->readBuf : NULL;
  const size_t originalReadCount = transaction != NULL ? transaction->readCount : 0u;
  const uint_least8_t originalTargetAddress =
      transaction != NULL ? transaction->targetAddress : 0u;

  if (transaction == NULL || g_i2cHandle == NULL) {
    return false;
  }

  for (uint32_t attempt = 0; attempt < I2C_TRANSFER_MAX_ATTEMPTS; ++attempt) {
    TickType_t startTick;

    memset(transaction, 0, sizeof(*transaction));
    transaction->writeBuf = (void*)originalWriteBuf;
    transaction->writeCount = originalWriteCount;
    transaction->readBuf = originalReadBuf;
    transaction->readCount = originalReadCount;
    transaction->targetAddress = originalTargetAddress;

    g_i2cTransferDone = false;
    g_i2cTransferStatus = false;

    if (!I2C_transfer(g_i2cHandle, transaction)) {
      i2cErrorHandler(transaction);
      if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS &&
          i2cShouldRecoverTransaction(transaction)) {
        /* Run recovery and retry even if the bus is still held; the next
         * attempt will pulse the clock again.  Aborting here on a
         * still-stuck bus is what caused the cascade to stop updating. */
        (void)i2cRecoverController();
        continue;
      }
      return false;
    }

    startTick = xTaskGetTickCount();
    while (!g_i2cTransferDone) {
      if ((xTaskGetTickCount() - startTick) >= timeoutTicks) {
        SEGGER_RTT_printf(0,
                          "I2C callback wait timed out after %u ms (attempt %lu)\n",
                          (unsigned)I2C_TRANSFER_WAIT_MS,
                          (unsigned long)(attempt + 1u));
        if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS) {
          /* Recover and retry regardless of whether the bus came back idle
           * this cycle; subsequent attempts keep pulsing the clock. */
          (void)i2cRecoverController();
          break; /* exit the while loop, goto next attempt */
        }
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!g_i2cTransferDone) {
      /* Timed out, next attempt */
      continue;
    }

    if (!g_i2cTransferStatus) {
      if (transaction != NULL) {
        i2cErrorHandler(transaction);
      }
      if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS &&
          i2cShouldRecoverTransaction(transaction)) {
        (void)i2cRecoverController();
        continue;
      }
      return false;
    }

    return true;
  }

  return false;
}

static bool i2cShouldRecoverTransaction(const I2C_Transaction* transaction) {
  if (transaction == NULL) {
    return false;
  }

  if ((transaction->status == I2C_STATUS_ADDR_NACK) &&
      (transaction->targetAddress == HDD_I2C_TARGET_ADDRESS)) {
    return false;
  }

  return true;
}

static bool i2cWriteReg8(uint8_t targetAddress, uint8_t* txBuf,
                         size_t txBufSize, uint8_t reg, uint8_t value) {
  I2C_Transaction i2cTransaction = {0};
  if (g_i2cHandle == NULL || txBuf == NULL || txBufSize < 2) {
    return false;
  }

  txBuf[0] = reg;
  txBuf[1] = value;

  i2cTransaction.writeBuf = txBuf;
  i2cTransaction.writeCount = 2;
  i2cTransaction.readBuf = rxBuffer;
  i2cTransaction.readCount = 0;
  i2cTransaction.targetAddress = targetAddress;

  return i2cTransferAndWait(&i2cTransaction);
}

static bool i2cReadReg8(uint8_t targetAddress, uint8_t* txBuf, size_t txBufSize,
                        uint8_t* rxBuf, size_t rxBufSize, uint8_t reg,
                        uint8_t* value) {
  I2C_Transaction i2cTransaction = {0};
  if (g_i2cHandle == NULL || txBuf == NULL || txBufSize < 1 || rxBuf == NULL ||
      rxBufSize < 1 || value == NULL) {
    return false;
  }

  txBuf[0] = reg;
  i2cTransaction.writeBuf = txBuf;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = rxBuf;
  i2cTransaction.readCount = 1;
  i2cTransaction.targetAddress = targetAddress;

  if (!i2cTransferAndWait(&i2cTransaction)) {
    return false;
  }
  *value = rxBuf[0];
  return true;
}

static bool i2cReadRegN(uint8_t targetAddress, uint8_t* txBuf, size_t txBufSize,
                        uint8_t* rxBuf, size_t rxBufSize, uint8_t startReg,
                        uint8_t* out, size_t outLen) {
  I2C_Transaction i2cTransaction = {0};
  if (g_i2cHandle == NULL || txBuf == NULL || txBufSize < 1 || rxBuf == NULL ||
      rxBufSize < outLen || out == NULL) {
    return false;
  }

  txBuf[0] = startReg;
  i2cTransaction.writeBuf = txBuf;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = rxBuf;
  i2cTransaction.readCount = (uint16_t)outLen;
  i2cTransaction.targetAddress = targetAddress;

  if (!i2cTransferAndWait(&i2cTransaction)) {
    return false;
  }

  memcpy(out, rxBuf, outLen);
  return true;
}

static void i2cErrorHandler(I2C_Transaction* transaction) {
  switch (transaction->status) {
    case I2C_STATUS_TIMEOUT:
      SEGGER_RTT_printf(0, "I2C transaction timed out!\n");
      break;
    case I2C_STATUS_CLOCK_TIMEOUT:
      SEGGER_RTT_printf(0, "I2C serial clock line timed out!\n");
      break;
    case I2C_STATUS_ADDR_NACK:
      if (transaction->targetAddress == HDD_I2C_TARGET_ADDRESS) {
        SEGGER_RTT_printf(0,
                          "Optional I2C target address 0x%x not acknowledged\n",
                          transaction->targetAddress);
      } else {
        SEGGER_RTT_printf(0, "I2C target address 0x%x not acknowledged!\n",
                          transaction->targetAddress);
      }
      break;
    case I2C_STATUS_DATA_NACK:
      SEGGER_RTT_printf(0, "I2C data byte not acknowledged!\n");
      break;
    case I2C_STATUS_ARB_LOST:
      SEGGER_RTT_printf(0, "I2C arbitration to another controller!\n");
      break;
    case I2C_STATUS_INCOMPLETE:
      SEGGER_RTT_printf(0, "I2C transaction returned before completion!\n");
      break;
    case I2C_STATUS_BUS_BUSY:
      SEGGER_RTT_printf(0, "I2C bus is already in use!\n");
      break;
    case I2C_STATUS_CANCEL:
      SEGGER_RTT_printf(0, "I2C transaction cancelled!\n");
      break;
    case I2C_STATUS_INVALID_TRANS:
      SEGGER_RTT_printf(0, "I2C transaction invalid!\n");
      break;
    case I2C_STATUS_ERROR:
      SEGGER_RTT_printf(0, "I2C generic error!\n");
      break;
    default:
      SEGGER_RTT_printf(0, "I2C undefined error case! raw status=%d\n",
                        (int)transaction->status);
      break;
  }
}

bool nsa2300Init() {
  I2C_Params_init(&g_i2cParams);
  g_i2cParams.bitRate = I2C_100kHz;
  g_i2cParams.transferMode = I2C_MODE_CALLBACK;
  g_i2cParams.transferCallbackFxn = i2cTransferCallback;
  g_i2cHandle = I2C_open(CONFIG_I2C_0, &g_i2cParams);
  if (g_i2cHandle == NULL) {
    SEGGER_RTT_printf(0, "NSA2300: Error initializing I2C\n");
    return false;
  }
  I2C_setClockTimeout(g_i2cHandle, I2C_CLOCK_TIMEOUT_CYCLES);
  usleep(100000); /* 100ms power-up delay */
  /* Compute P_CONFIG: base value, then add input_swap bit if REG_CTRL bit 1 is set */
  uint8_t pConfig = NSA2300_REG_P_CONFIG_BASE;
  if (I2CTarget_getRegCtrl() & REG_CTRL_BIT1_INPUT_SWAP) {
    pConfig |= NSA2300_REG_P_CONFIG_INPUT_SWAP_BIT;
  }
  SEGGER_RTT_printf(0, "NSA2300: P_CONFIG=0x%02x (input_swap=%u)\n",
                    (unsigned)pConfig,
                    (unsigned)((pConfig >> 6) & 1u));
  if (nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_SYS_CONFIG,
                       NSA2300_REG_SYS_CONFIG_DEFAULT) == false ||
      nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_P_CONFIG,
                       pConfig) == false) {
    SEGGER_RTT_printf(0, "NSA2300: Error writing config registers\n");
    I2C_close(g_i2cHandle);
    g_i2cHandle = NULL;
    return false;
  }

  SEGGER_RTT_printf(0, "NSA2300: I2C initialized successfully\n");
  return true;
}

bool nas2300Deinit() {
  if (g_i2cHandle != NULL) {
    I2C_close(g_i2cHandle);
    g_i2cHandle = NULL;
  }
  return true;
}

bool nsa2300WriteReg8(uint8_t* txBuf, size_t txBufSize, uint8_t reg,
                      uint8_t value) {
  if (!i2cWriteReg8(NAS2300_I2C_ADDRESS, txBuf, txBufSize, reg, value)) {
    SEGGER_RTT_printf(0, "NSA2300: write reg 0x%02x failed\n", (unsigned)reg);
    return false;
  }
  return true;
}

bool nsa2300ReadReg8(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                     size_t rxBufSize, uint8_t reg, uint8_t* value) {
  return i2cReadReg8(NAS2300_I2C_ADDRESS, txBuf, txBufSize, rxBuf, rxBufSize,
                     reg, value);
}

bool nsa2300ReadRegN(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                     size_t rxBufSize, uint8_t startReg, uint8_t* out,
                     size_t outLen) {
  return i2cReadRegN(NAS2300_I2C_ADDRESS, txBuf, txBufSize, rxBuf, rxBufSize,
                     startReg, out, outLen);
}

bool nsa2300StartMeasurement() {
  uint8_t txBuffer[2];
  bool ok;

  if (g_i2cHandle == NULL) {
    SEGGER_RTT_printf(0, "NSA2300: I2C not initialized\n");
    return false;
  }

  SEGGER_RTT_printf(0, "NSA2300: start measurement write begin\n");
  ok = nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_CMD,
                        NSA2300_CMD_SINGLE_PRESSURE_CONVERSION);
  SEGGER_RTT_printf(0, "NSA2300: start measurement write %s\n",
                    ok ? "done" : "failed");
  return ok;
}

bool nsa2300WaitForDataReady() {
  uint8_t status = 0;
  const uint32_t maxPolls = 1000u; /* ~1s at 3ms per poll */
  const useconds_t pollDelayUs = 3000u;
  uint32_t consecutiveFail = 0;

  for (uint32_t poll = 0; poll < maxPolls; poll++) {
    if (nsa2300ReadReg8(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                        NSA2300_REG_STATUS, &status) == false) {
      /* Read failed; count and log if persistent. */
      consecutiveFail++;
      if (consecutiveFail == 1) {
        SEGGER_RTT_printf(0, "NSA2300: STATUS read failed (transient start)\n");
      }
      if (consecutiveFail > 50 && (consecutiveFail % 50) == 0) {
        SEGGER_RTT_printf(0, "NSA2300: STATUS read failing repeatedly (%lu times). I2C handle null? %s\n",
                          (unsigned long)consecutiveFail,
                          (g_i2cHandle == NULL) ? "YES" : "NO");
      }
      if (consecutiveFail >= 3u) {
        /* Bus is persistently stuck (hot-plug glitch held SDA/SCL low).
         * Break now so the caller can reinitialise the I2C controller and
         * the NSA2300 config registers before retrying. */
        SEGGER_RTT_printf(0, "NSA2300: STATUS read failed %lu times; aborting poll\n",
                          (unsigned long)consecutiveFail);
        break;
      }
      usleep(pollDelayUs);
      continue;
    }
    /* successful read, reset fail counter */
    consecutiveFail = 0;
    SEGGER_RTT_printf(0, "NSA2300: STATUS=0x%02x\n", (unsigned)status);
    if ((status & NSA2300_STATUS_DRDY_MASK) != 0) {
      return true;
    }
    usleep(pollDelayUs);
  }

  SEGGER_RTT_printf(0, "NSA2300: DRDY timeout, last STATUS=0x%02x\n",
                    (unsigned)status);
  return false;
}

bool nsa2300ReadPressureRaw24Single(uint32_t* p24) {
  uint8_t raw[3] = {0};

  if (nsa2300ReadRegN(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                      NSA2300_REG_DATA, raw, 3) == false) {
    return false;
  }

  *p24 = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2];

  return true;
}

bool nsa2300ReadPressureOutputSingle(uint32_t* value) {
  uint32_t raw;
  int32_t calibrated;

  if (value == NULL) {
    return false;
  }

  if (!nsa2300ReadPressureRaw24Single(&raw)) {
    return false;
  }

  /* REG_CTRL bit 0: 0=output raw, 1=apply calibration */
  if (!(I2CTarget_getRegCtrl() & REG_CTRL_BIT0_CALIBRATION) || !nsa2300CalibrationEnabled()) {
    *value = raw;
    return true;
  }

  if (!nsa2300RawToPercentX100(raw, &calibrated)) {
    return false;
  }

  *value = (uint32_t)calibrated;
  return true;
}

bool hddI2CWriteReg8(uint8_t* txBuf, size_t txBufSize, uint8_t reg,
                     uint8_t value) {
  if (!i2cWriteReg8(HDD_I2C_TARGET_ADDRESS, txBuf, txBufSize, reg, value)) {
    SEGGER_RTT_printf(0, "HDD: write reg 0x%02x failed\n", (unsigned)reg);
    return false;
  }
  return true;
}

bool hddI2CReadReg8(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                    size_t rxBufSize, uint8_t reg, uint8_t* value) {
  return i2cReadReg8(HDD_I2C_TARGET_ADDRESS, txBuf, txBufSize, rxBuf, rxBufSize,
                     reg, value);
}

bool hddI2CReadRegN(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                    size_t rxBufSize, uint8_t startReg, uint8_t* out,
                    size_t outLen) {
  return i2cReadRegN(HDD_I2C_TARGET_ADDRESS, txBuf, txBufSize, rxBuf, rxBufSize,
                     startReg, out, outLen);
}

bool hddI2CReadMode(uint8_t* mode) {
  if (hddI2CReadReg8(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                     REG_MODE_0x80, mode) == false) {
    SEGGER_RTT_printf(0, "HDD: Failed to read mode register\n");
    return false;
  }
  return true;
}

bool hddI2CWriteMode(HDD_I2C_Mode mode) {
  if (hddI2CWriteReg8(txBuffer, sizeof(txBuffer), REG_MODE_0x80,
                       (uint8_t)mode) == false) {
    SEGGER_RTT_printf(0, "HDD: Failed to write mode register\n");
    return false;
  }
  return true;
}

bool hddI2CReadReady(uint8_t* ready) {
  if (hddI2CReadReg8(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                      REG_READY_0x81, ready) == false) {
    SEGGER_RTT_printf(0, "HDD: Failed to read ready register\n");
    return false;
  }
  return true;
}

bool hddI2CReadData(uint8_t* data, size_t len) {
  if (hddI2CReadRegN(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                     REG_DATA_0x82, data, len) == false) {
    SEGGER_RTT_printf(0, "HDD: Failed to read data register\n");
    return false;
  }
  return true;
}

bool hddI2CWriteReady(uint8_t ready) {
  if (hddI2CWriteReg8(txBuffer, sizeof(txBuffer), REG_READY_0x81,
                      ready) == false) {
    SEGGER_RTT_printf(0, "HDD: Failed to write ready register\n");
    return false;
  }
  return true;
}

bool nsa2300SetCalibration(uint32_t raw_zero_kg, uint32_t raw_full_3000kg) {
  raw_zero_kg &= 0x00FFFFFFu;
  raw_full_3000kg &= 0x00FFFFFFu;

  if (raw_zero_kg == 0u && raw_full_3000kg == 0u) {
    calibration_low = 0u;
    calibration_high = 0u;
    calibration_valid = false;
    SEGGER_RTT_printf(0, "NSA2300: calibration disabled\n");
    return true;
  }

  if (raw_full_3000kg == raw_zero_kg) {
    SEGGER_RTT_printf(0, "NSA2300: invalid calibration (identical points)\n");
    return false;
  }

  calibration_low = raw_zero_kg;
  calibration_high = raw_full_3000kg;
  calibration_valid = true;
  SEGGER_RTT_printf(0,
                    "NSA2300: calibration set: zero=0x%06X full=0x%06X (signed %ld, %ld)\n",
                    (unsigned)calibration_low, (unsigned)calibration_high,
                    (long)nsa2300SignExtend24(calibration_low),
                    (long)nsa2300SignExtend24(calibration_high));
  return true;
}

bool nsa2300RawToPercentX100(uint32_t raw, int32_t *percent_x100) {
  int32_t low;
  int32_t high;
  int32_t signed_raw;
  double frac;

  if (!calibration_valid || percent_x100 == NULL) {
    return false;
  }

  low = nsa2300SignExtend24(calibration_low);
  high = nsa2300SignExtend24(calibration_high);
  signed_raw = nsa2300SignExtend24(raw);
  frac = ((double)signed_raw - (double)low) / ((double)high - (double)low);

  /* Clamp 0..1 */
  if (frac < 0.0)
    frac = 0.0;
  if (frac > 1.0)
    frac = 1.0;

  *percent_x100 = (uint32_t)lround(frac * 10000.0);

  SEGGER_RTT_printf(0,
                    "NSA2300: raw=0x%06X signed=%ld frac=%.4f percent=%lu.%02lu%%\n",
                    (unsigned)(raw & 0x00FFFFFFu),
                    (long)signed_raw,
                    frac,
                    (unsigned long)(*percent_x100 / 100u),
                    (unsigned long)(*percent_x100 % 100u));
  return true;
}

bool nsa2300CalibrationEnabled(void) {
  return calibration_valid &&
         !(calibration_low == 0u && calibration_high == 0u) &&
         (calibration_low != calibration_high);
}