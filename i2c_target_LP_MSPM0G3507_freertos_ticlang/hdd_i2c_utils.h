#ifndef __HDD_I2C_UTILS_H__
#define __HDD_I2C_UTILS_H__

#include <stddef.h>
#include <stdint.h>

uint16_t crc16_modbus(const uint8_t* data, size_t len);

#endif /* __HDD_I2C_UTILS_H__ */