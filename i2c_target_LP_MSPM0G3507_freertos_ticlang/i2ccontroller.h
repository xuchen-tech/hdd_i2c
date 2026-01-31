#ifndef __I2CCONTROLLER_H__
#define __I2CCONTROLLER_H__

#include <ti/drivers/I2C.h>
#include <stddef.h>

#include "hdd_i2c_config.h"

bool detectI2CDevices(uint_least8_t targetAddress, uint8_t *devCount);
bool readDeviceData(uint8_t *data, size_t len);
bool readNsa2300Raw24(uint32_t *pressure);
bool readNsa2300Raw24(uint32_t *pressure);
bool readDeviceData(uint8_t *data, size_t len);

#endif /* __I2CCONTROLLER_H__ */