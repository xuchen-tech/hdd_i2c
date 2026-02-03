#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/ADC.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

#include "hdd_i2c_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PT100 frontend assumptions (defaults; override at compile time if needed):
 * - Rpullup from VCC to divider node
 * - PT100 from divider node to GND
 * - Optional non-inverting amplifier on the divider node
 */
#ifndef PT100_VCC_UV
#define PT100_VCC_UV (3300000.0)
#endif

#ifndef PT100_PULLUP_OHMS
#define PT100_PULLUP_OHMS (3300.0)
#endif

#ifndef PT100_AMP_GAIN
#define PT100_AMP_GAIN (11.0)
#endif

/*  pt100 initialize */
bool pt100Init(void);
/*  pt100 de-initialize */
bool pt100Deinit(void);
/*  pt100 16bit raw data read */
bool pt100ReadRaw(uint16_t* rawData);
/*  pt100 32bit MicroVolts  */
bool pt100ReadMicroVolts(uint32_t* microVolts);
/*  pt100 temperature in 0.1C units */
bool pt100ReadTemperature_x10(int16_t* temp_x10);
#ifdef __cplusplus
}
#endif
