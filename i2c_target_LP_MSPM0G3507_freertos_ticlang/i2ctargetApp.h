#ifndef __I2CTARGETAPP_H__
#define __I2CTARGETAPP_H__
#include <stdint.h>

void updateReady(uint8_t ready);
void updatePayloadData(uint8_t* data, uint8_t len);

#endif /* __I2CTARGETAPP_H__ */