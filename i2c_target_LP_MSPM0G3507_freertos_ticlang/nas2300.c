#include "nas2300.h"

#include <string.h>
/* For usleep() */
#include <unistd.h>

/* NSA2300 (user model) register map per user notes */
#define NAS2300_REG_STATUS   0x02
#define NAS2300_REG_P_MSB    0x06
#define NAS2300_REG_CMD      0x30

#define NAS2300_REG_P_CFG0   0xA5
#define NAS2300_REG_P_CFG1   0xA6
#define NAS2300_REG_P_CFG2   0xA7

/* CMD: pressure channel, single conversion */
#define NAS2300_CMD_SINGLE_PRESSURE_CONVERSION 0x09

/* DRDY mask in STATUS (user note: bit0) */
#define NAS2300_STATUS_DRDY_MASK 0x01

volatile int32_t g_latestForce_uV = 0;
volatile uint32_t g_latestForceSeq = 0;

void NAS2300_Config_init(NAS2300_Config *cfg)
{
    if (cfg == NULL) {
        return;
    }

    cfg->i2cAddr = 0x6D;

    /* User-provided init values */
    cfg->p_cfg0_value = 0x12;
    cfg->p_cfg1_value = 0x31;
    cfg->p_cfg2_value = 0x81;

    cfg->fs_uV = 0;
    cfg->dataFormat = NAS2300_DataFormat_OffsetBinary;
}

bool NAS2300_writeReg8(I2C_Handle i2cHandle,
                       I2C_Transaction *transaction,
                       uint8_t *txBuf,
                       size_t txBufSize,
                       uint8_t reg,
                       uint8_t value)
{
    if (i2cHandle == NULL || transaction == NULL || txBuf == NULL) {
        return false;
    }
    if (txBufSize < 2) {
        return false;
    }

    txBuf[0] = reg;
    txBuf[1] = value;
    transaction->writeBuf = txBuf;
    transaction->writeCount = 2;
    transaction->readCount = 0;

    return I2C_transfer(i2cHandle, transaction);
}

bool NAS2300_readReg8(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      uint8_t reg,
                      uint8_t *value)
{
    if (i2cHandle == NULL || transaction == NULL || txBuf == NULL || rxBuf == NULL || value == NULL) {
        return false;
    }
    if (txBufSize < 1 || rxBufSize < 1) {
        return false;
    }

    txBuf[0] = reg;
    transaction->writeBuf = txBuf;
    transaction->writeCount = 1;
    transaction->readBuf = rxBuf;
    transaction->readCount = 1;

    if (!I2C_transfer(i2cHandle, transaction)) {
        return false;
    }

    *value = rxBuf[0];
    return true;
}

bool NAS2300_readRegN(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      uint8_t startReg,
                      uint8_t *out,
                      size_t outLen)
{
    if (i2cHandle == NULL || transaction == NULL || txBuf == NULL || rxBuf == NULL || out == NULL) {
        return false;
    }
    if (txBufSize < 1 || outLen == 0 || outLen > rxBufSize) {
        return false;
    }

    txBuf[0] = startReg;
    transaction->writeBuf = txBuf;
    transaction->writeCount = 1;
    transaction->readBuf = rxBuf;
    transaction->readCount = (uint16_t)outLen;

    if (!I2C_transfer(i2cHandle, transaction)) {
        return false;
    }

    memcpy(out, rxBuf, outLen);
    return true;
}

bool NAS2300_init(I2C_Handle i2cHandle,
                  I2C_Transaction *transaction,
                  uint8_t *txBuf,
                  size_t txBufSize,
                  uint8_t *rxBuf,
                  size_t rxBufSize,
                  const NAS2300_Config *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    transaction->targetAddress = cfg->i2cAddr;

    uint8_t rb0 = 0, rb1 = 0, rb2 = 0;

    if (!NAS2300_writeReg8(i2cHandle, transaction, txBuf, txBufSize, NAS2300_REG_P_CFG0, cfg->p_cfg0_value) ||
        !NAS2300_writeReg8(i2cHandle, transaction, txBuf, txBufSize, NAS2300_REG_P_CFG1, cfg->p_cfg1_value) ||
        !NAS2300_writeReg8(i2cHandle, transaction, txBuf, txBufSize, NAS2300_REG_P_CFG2, cfg->p_cfg2_value)) {
        return false;
    }

    if (!NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_P_CFG0, &rb0) ||
        !NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_P_CFG1, &rb1) ||
        !NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_P_CFG2, &rb2)) {
        return false;
    }

    return (rb0 == cfg->p_cfg0_value) && (rb1 == cfg->p_cfg1_value) && (rb2 == cfg->p_cfg2_value);
}

bool NAS2300_startSinglePressureConversion(I2C_Handle i2cHandle,
                                          I2C_Transaction *transaction,
                                          uint8_t *txBuf,
                                          size_t txBufSize,
                                          uint8_t *rxBuf,
                                          size_t rxBufSize,
                                          const NAS2300_Config *cfg,
                                          uint8_t *cmdReadback)
{
    if (cfg == NULL) {
        return false;
    }

    transaction->targetAddress = cfg->i2cAddr;

    if (!NAS2300_writeReg8(i2cHandle, transaction, txBuf, txBufSize, NAS2300_REG_CMD, NAS2300_CMD_SINGLE_PRESSURE_CONVERSION)) {
        return false;
    }

    if (cmdReadback) {
        uint8_t rb = 0;
        if (!NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_CMD, &rb)) {
            return false;
        }
        *cmdReadback = rb;
    }

    return true;
}

bool NAS2300_waitDrdy(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      const NAS2300_Config *cfg,
                      uint32_t maxPolls,
                      uint32_t pollDelayUs,
                      uint8_t *finalStatus)
{
    if (cfg == NULL) {
        return false;
    }

    transaction->targetAddress = cfg->i2cAddr;

    uint8_t status = 0;
    for (uint32_t polls = 0; polls < maxPolls; polls++) {
        if (!NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_STATUS, &status)) {
            return false;
        }

        if ((status & NAS2300_STATUS_DRDY_MASK) != 0) {
            if (finalStatus) {
                *finalStatus = status;
            }
            return true;
        }

        if (pollDelayUs != 0) {
            usleep(pollDelayUs);
        }
    }

    if (finalStatus) {
        *finalStatus = status;
    }
    return false;
}

bool NAS2300_readPressureRaw24_burst(I2C_Handle i2cHandle,
                                    I2C_Transaction *transaction,
                                    uint8_t *txBuf,
                                    size_t txBufSize,
                                    uint8_t *rxBuf,
                                    size_t rxBufSize,
                                    const NAS2300_Config *cfg,
                                    uint32_t *p24)
{
    if (cfg == NULL || p24 == NULL) {
        return false;
    }

    transaction->targetAddress = cfg->i2cAddr;

    uint8_t bytes[3] = {0};
    if (!NAS2300_readRegN(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_P_MSB, bytes, sizeof(bytes))) {
        return false;
    }

    *p24 = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | (uint32_t)bytes[2];
    return true;
}

bool NAS2300_readPressureRaw24_single(I2C_Handle i2cHandle,
                                     I2C_Transaction *transaction,
                                     uint8_t *txBuf,
                                     size_t txBufSize,
                                     uint8_t *rxBuf,
                                     size_t rxBufSize,
                                     const NAS2300_Config *cfg,
                                     uint32_t *p24)
{
    if (cfg == NULL || p24 == NULL) {
        return false;
    }

    transaction->targetAddress = cfg->i2cAddr;

    uint8_t b0 = 0, b1 = 0, b2 = 0;
    if (!NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, NAS2300_REG_P_MSB, &b0) ||
        !NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, (uint8_t)(NAS2300_REG_P_MSB + 1), &b1) ||
        !NAS2300_readReg8(i2cHandle, transaction, txBuf, txBufSize, rxBuf, rxBufSize, (uint8_t)(NAS2300_REG_P_MSB + 2), &b2)) {
        return false;
    }

    *p24 = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
    return true;
}

int32_t NAS2300_decode_code24(uint32_t u24, NAS2300_DataFormat fmt)
{
    if (fmt == NAS2300_DataFormat_TwosComplement) {
        return (u24 & 0x800000u) ? (int32_t)(u24 | 0xFF000000u) : (int32_t)u24;
    }

    return (int32_t)u24 - 0x800000;
}

int32_t NAS2300_code24_to_uV(int32_t code24, int32_t fs_uV)
{
    if (fs_uV <= 0) {
        return 0;
    }

    return (int32_t)((int64_t)code24 * (int64_t)fs_uV / 8388607LL);
}

void NAS2300_publish_latest_force_uV(int32_t force_uV)
{
    g_latestForce_uV = force_uV;
    g_latestForceSeq++;
}
