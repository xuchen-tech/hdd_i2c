#ifndef __HDD_I2C_PAYLOAD_MANAGER_H__
#define __HDD_I2C_PAYLOAD_MANAGER_H__

#include "pt100.h"
#include "i2c_controller.h"
#include "i2ctargetApp.h"

/* Trigger the payload manager task to run one cycle.
 * Safe to call from ISR context.
 */
void PayloadManager_requestSampleFromISR(void);

#endif /* __HDD_I2C_PAYLOAD_MANAGER_H__ */