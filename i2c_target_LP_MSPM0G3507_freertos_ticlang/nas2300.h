#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <ti/drivers/I2C.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NOTE:
 * This driver is for the NSA2300 sensor (user model name). File name keeps the
 * user-requested spelling: nas2300.h/.c.
 */

typedef enum {
    NAS2300_DataFormat_OffsetBinary = 0,
    NAS2300_DataFormat_TwosComplement = 1,
} NAS2300_DataFormat;

typedef struct {
    uint8_t i2cAddr;           /* 7-bit address (e.g. 0x6D) */

    /* Pressure channel config registers (user-provided init sequence) */
    uint8_t p_cfg0_value;      /* write to 0xA5 */
    uint8_t p_cfg1_value;      /* write to 0xA6 */
    uint8_t p_cfg2_value;      /* write to 0xA7 */

    /* Optional conversion (set fs_uV=0 to disable) */
    int32_t fs_uV;
    NAS2300_DataFormat dataFormat;
} NAS2300_Config;

/* Latest published force (uV) for sharing with other modules */
extern volatile int32_t g_latestForce_uV;
extern volatile uint32_t g_latestForceSeq;

void NAS2300_Config_init(NAS2300_Config *cfg);

/* Low-level reg access helpers (operate on caller-provided tx/rx buffers) */
bool NAS2300_writeReg8(I2C_Handle i2cHandle,
                       I2C_Transaction *transaction,
                       uint8_t *txBuf,
                       size_t txBufSize,
                       uint8_t reg,
                       uint8_t value);

bool NAS2300_readReg8(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      uint8_t reg,
                      uint8_t *value);

bool NAS2300_readRegN(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      uint8_t startReg,
                      uint8_t *out,
                      size_t outLen);

/* Sensor init: write 0xA5/0xA6/0xA7 and verify readback */
bool NAS2300_init(I2C_Handle i2cHandle,
                  I2C_Transaction *transaction,
                  uint8_t *txBuf,
                  size_t txBufSize,
                  uint8_t *rxBuf,
                  size_t rxBufSize,
                  const NAS2300_Config *cfg);

/* Start single pressure conversion (write CMD=0x08 to reg 0x30) */
bool NAS2300_startSinglePressureConversion(I2C_Handle i2cHandle,
                                          I2C_Transaction *transaction,
                                          uint8_t *txBuf,
                                          size_t txBufSize,
                                          uint8_t *rxBuf,
                                          size_t rxBufSize,
                                          const NAS2300_Config *cfg,
                                          uint8_t *cmdReadback);

/* Poll STATUS bit0 (DRDY) until ready or timeout */
bool NAS2300_waitDrdy(I2C_Handle i2cHandle,
                      I2C_Transaction *transaction,
                      uint8_t *txBuf,
                      size_t txBufSize,
                      uint8_t *rxBuf,
                      size_t rxBufSize,
                      const NAS2300_Config *cfg,
                      uint32_t maxPolls,
                      uint32_t pollDelayUs,
                      uint8_t *finalStatus);

/* Read pressure result bytes from 0x06..0x08 */
bool NAS2300_readPressureRaw24_burst(I2C_Handle i2cHandle,
                                    I2C_Transaction *transaction,
                                    uint8_t *txBuf,
                                    size_t txBufSize,
                                    uint8_t *rxBuf,
                                    size_t rxBufSize,
                                    const NAS2300_Config *cfg,
                                    uint32_t *p24);

bool NAS2300_readPressureRaw24_single(I2C_Handle i2cHandle,
                                     I2C_Transaction *transaction,
                                     uint8_t *txBuf,
                                     size_t txBufSize,
                                     uint8_t *rxBuf,
                                     size_t rxBufSize,
                                     const NAS2300_Config *cfg,
                                     uint32_t *p24);

/* Optional helpers for converting 24-bit code to signed and then uV */
int32_t NAS2300_decode_code24(uint32_t u24, NAS2300_DataFormat fmt);
int32_t NAS2300_code24_to_uV(int32_t code24, int32_t fs_uV);

void NAS2300_publish_latest_force_uV(int32_t force_uV);

#ifdef __cplusplus
}
#endif
