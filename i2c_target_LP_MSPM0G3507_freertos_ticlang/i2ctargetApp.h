#ifndef __I2CTARGETAPP_H__
#define __I2CTARGETAPP_H__
#include <stdint.h>

void updatePayloadData(uint8_t* data, uint8_t len);
void I2CTarget_pauseService(void);
void I2CTarget_resumeService(void);

#endif /* __I2CTARGETAPP_H__ */