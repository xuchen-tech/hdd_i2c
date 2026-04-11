#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <ti/drivers/I2C.h>
#include "hdd_i2c_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize I2C for NSA2300 use */
bool nsa2300Init();
/* Deinitialize I2C for NSA2300 use */
bool nas2300Deinit();
/* Write Data to NSA2300 */
bool nsa2300WriteReg8(uint8_t* txBuf, size_t txBufSize, uint8_t reg,
                      uint8_t value);
/* Read Data from NSA2300 */
bool nsa2300ReadReg8(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                     size_t rxBufSize, uint8_t reg, uint8_t* value);
/*  Read N bytes */
bool nsa2300ReadRegN(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                     size_t rxBufSize, uint8_t startReg, uint8_t* out,
                     size_t outLen);
/* Start a single measurement */
bool nsa2300StartMeasurement();
/* Wait for data ready */
bool nsa2300WaitForDataReady();
/* Read single 24-bit pressure value */
bool nsa2300ReadPressureRaw24Single(uint32_t* p24);

/* Two-point calibration: provide raw reading for 0 kg and for 3000 kg */
bool nsa2300SetCalibration(uint32_t raw_zero_kg, uint32_t raw_full_3000kg);
/* Convert raw reading to kilograms * 10 (kg_x10). Returns false if not calibrated. */
/* Convert raw reading to kilograms as integer (kg). Returns false if not calibrated. */
bool nsa2300RawToKg(uint32_t raw, int32_t *kg);

bool hddI2CWriteReg8(uint8_t* txBuf, size_t txBufSize, uint8_t reg,
                     uint8_t value);
bool hddI2CReadReg8(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                    size_t rxBufSize, uint8_t reg, uint8_t* value);
bool hddI2CReadRegN(uint8_t* txBuf, size_t txBufSize, uint8_t* rxBuf,
                    size_t rxBufSize, uint8_t startReg, uint8_t* out,
                    size_t outLen);

bool hddI2CReadMode(uint8_t* mode);
bool hddI2CWriteMode(HDD_I2C_Mode mode);
bool hddI2CReadReady(uint8_t* ready);
bool hddI2CWriteReady(uint8_t ready);
bool hddI2CReadData(uint8_t* data, size_t len);

bool nsa2300ReadPressureOutputSingle(uint32_t* value);
bool nsa2300CalibrationEnabled(void);
#ifdef __cplusplus
}
#endif
