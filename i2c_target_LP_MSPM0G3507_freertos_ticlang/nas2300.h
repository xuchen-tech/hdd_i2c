#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <ti/drivers/I2C.h>

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
#ifdef __cplusplus
}
#endif
