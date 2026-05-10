#ifndef __I2CTARGETAPP_H__
#define __I2CTARGETAPP_H__
#include <stdbool.h>
#include <stdint.h>

void    updatePayloadData(uint8_t* data, uint8_t len);
void    I2CTarget_pauseService(void);
void    I2CTarget_resumeService(void);
uint8_t I2CTarget_getRegCtrl(void);
bool    I2CTarget_isNsa2300ReinitRequired(void);
void    I2CTarget_clearNsa2300ReinitRequired(void);

#endif /* __I2CTARGETAPP_H__ */