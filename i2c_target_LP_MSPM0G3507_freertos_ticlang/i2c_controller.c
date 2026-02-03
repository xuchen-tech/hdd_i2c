#include "i2c_controller.h"

#include <string.h>
/* For usleep() */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/segger/SEGGER_RTT.h>
#include <unistd.h>

#include "hdd_i2c_config.h"

/* Driver configuration */
#include "ti_drivers_config.h"

static uint8_t txBuffer[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];

static I2C_Handle g_i2cHandle;
static I2C_Params g_i2cParams;

static void i2cErrorHandler(I2C_Transaction* transaction);

static bool i2cWriteReg8(uint8_t targetAddress, uint8_t* txBuf,
                         size_t txBufSize, uint8_t reg, uint8_t value) {
  I2C_Transaction i2cTransaction;
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

  if (!I2C_transfer(g_i2cHandle, &i2cTransaction)) {
    i2cErrorHandler(&i2cTransaction);
    return false;
  }
  return true;
}

static bool i2cReadReg8(uint8_t targetAddress, uint8_t* txBuf, size_t txBufSize,
                        uint8_t* rxBuf, size_t rxBufSize, uint8_t reg,
                        uint8_t* value) {
  I2C_Transaction i2cTransaction;
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

  if (!I2C_transfer(g_i2cHandle, &i2cTransaction)) {
    i2cErrorHandler(&i2cTransaction);
    return false;
  }
  *value = rxBuf[0];
  return true;
}

static bool i2cReadRegN(uint8_t targetAddress, uint8_t* txBuf, size_t txBufSize,
                        uint8_t* rxBuf, size_t rxBufSize, uint8_t startReg,
                        uint8_t* out, size_t outLen) {
  I2C_Transaction i2cTransaction;
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

  if (!I2C_transfer(g_i2cHandle, &i2cTransaction)) {
    i2cErrorHandler(&i2cTransaction);
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
      SEGGER_RTT_printf(0, "I2C undefined error case!\n");
      break;
  }
}

bool nsa2300Init() {
  I2C_Params_init(&g_i2cParams);
  g_i2cParams.bitRate = I2C_400kHz;
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

  for (uint32_t poll = 0; poll < maxPolls; poll++) {
    if (nsa2300ReadReg8(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer),
                        NSA2300_REG_STATUS, &status) == false) {
      /* Read failed; treat as transient and retry. */
      usleep(pollDelayUs);
      continue;
    }
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