#ifndef __I2CCONTROLLER_H__
#define __I2CCONTROLLER_H__

#include <stdint.h>
#include <stddef.h>

uint16_t crc16_modbus(const uint8_t *data, size_t len);

#endif /* __I2CCONTROLLER_H__ */