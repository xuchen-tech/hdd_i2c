#include "i2c_controller.h"

#include <string.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* For usleep() */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/segger/SEGGER_RTT.h>
#include <unistd.h>

#include "hdd_i2c_config.h"

/* Driver configuration */
#include "ti_drivers_config.h"

uint32_t calibration_low = 0;
uint32_t calibration_high = 0;
/* calibration_valid indicates whether calibration_low/high contain a valid pair */
static volatile bool calibration_valid = false;

static uint8_t txBuffer[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];

static I2C_Handle g_i2cHandle;
static I2C_Params g_i2cParams;
static volatile bool g_i2cTransferDone = false;
static volatile bool g_i2cTransferStatus = false;

#define I2C_TRANSFER_WAIT_MS 200
#define I2C_TRANSFER_MAX_ATTEMPTS 2

static void i2cErrorHandler(I2C_Transaction* transaction);
static void i2cTransferCallback(I2C_Handle handle,
                                I2C_Transaction* transaction,
                                bool transferStatus);
static bool i2cTransferAndWait(I2C_Transaction* transaction);
static bool i2cRecoverController(void);

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
  if (g_i2cHandle != NULL) {
    I2C_close(g_i2cHandle);
    g_i2cHandle = NULL;
  }

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

  SEGGER_RTT_printf(0, "I2C controller recovered\n");
  return true;
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
      if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS && i2cRecoverController()) {
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
        if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS && i2cRecoverController()) {
          goto retry_transfer;
        }
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!g_i2cTransferStatus) {
      if (transaction != NULL) {
        i2cErrorHandler(transaction);
      }
      if ((attempt + 1u) < I2C_TRANSFER_MAX_ATTEMPTS && i2cRecoverController()) {
        continue;
      }
      return false;
    }

    return true;

retry_transfer:
    continue;
  }

  return false;
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

  if (!i2cTransferAndWait(&i2cTransaction)) {
    return false;
  }
  return true;
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
      SEGGER_RTT_printf(0, "I2C target address 0x%x not acknowledged!\n",
                        transaction->targetAddress);
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
  g_i2cParams.bitRate = I2C_400kHz;
  g_i2cParams.transferMode = I2C_MODE_CALLBACK;
  g_i2cParams.transferCallbackFxn = i2cTransferCallback;
  g_i2cHandle = I2C_open(CONFIG_I2C_0, &g_i2cParams);
  if (g_i2cHandle == NULL) {
    SEGGER_RTT_printf(0, "NSA2300: Error initializing I2C\n");
    return false;
  }
  usleep(100000); /* 100ms power-up delay */
  if (nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_SYS_CONFIG,
                       NSA2300_REG_SYS_CONFIG_DEFAULT) == false ||
      nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_P_CONFIG,
                       NSA2300_REG_P_CONFIG_DEFAULT) == false) {
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

  if (g_i2cHandle == NULL) {
    SEGGER_RTT_printf(0, "NSA2300: I2C not initialized\n");
    return false;
  }

  return nsa2300WriteReg8(txBuffer, sizeof(txBuffer), NSA2300_REG_CMD,
                          NSA2300_CMD_SINGLE_PRESSURE_CONVERSION);
}

bool nsa2300WaitForDataReady() {
  uint8_t status = 0;
  const uint32_t maxPolls = 1000u; /* ~1s at 1ms per poll */
  const useconds_t pollDelayUs = 1000u;
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

  if (!nsa2300CalibrationEnabled()) {
    *value = raw;
    return true;
  }

  if (!nsa2300RawToKg(raw, &calibrated)) {
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
  SEGGER_RTT_printf(0, "NSA2300: calibration set: zero=0x%06X full=0x%06X\n",
                    (unsigned)calibration_low, (unsigned)calibration_high);
  return true;
}

bool nsa2300RawToKg(uint32_t raw, int32_t *kg) {
  if (!calibration_valid || kg == NULL) {
    return false;
  }

  /* Compute fraction = (raw - low) / (high - low), as double for precision. */
  double low = (double)calibration_low;
  double high = (double)calibration_high;
  double r = (double)raw;
  double frac = (r - low) / (high - low);

  /* Clamp 0..1 */
  if (frac < 0.0)
    frac = 0.0;
  if (frac > 1.0)
    frac = 1.0;

  /* Map to 0..3000 kg, return integer kg (rounded) */
  double kgd = frac * 3000.0;
  int32_t out = (int32_t)lround(kgd);
  *kg = out;

  SEGGER_RTT_printf(0, "NSA2300: raw=0x%06X frac=%.4f kg=%ld\n",
                    (unsigned)raw, frac, (long)out);
  return true;
}

bool nsa2300CalibrationEnabled(void) {
  return calibration_valid &&
         !(calibration_low == 0u && calibration_high == 0u);
}