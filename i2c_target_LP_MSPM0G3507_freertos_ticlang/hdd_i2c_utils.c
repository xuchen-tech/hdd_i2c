#include "hdd_i2c_utils.h"

/*  ======== crc16_modbus ========
 *  Calculate CRC16-MODBUS over data buffer.
 *
 *  Polynomial: 0xA001
 *  Initial value: 0xFFFF
 *  No reflection, no final XOR
 */
uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}