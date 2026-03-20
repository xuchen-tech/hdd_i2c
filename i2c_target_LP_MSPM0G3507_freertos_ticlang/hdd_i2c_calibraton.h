#ifndef __HDD_I2C_CALIBRATON_H__
#define __HDD_I2C_CALIBRATON_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAIN_BASE_ADDRESS (0x0001F000)

bool hddI2CCalibrationInit(void);
bool hddI2CWriteCalibrationData(uint64_t calibrationData);
bool hddI2CReadCalibrationData(uint32_t* low, uint32_t* high);

#endif /* __HDD_I2C_CALIBRATON_H__ */