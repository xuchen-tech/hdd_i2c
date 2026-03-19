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
 * PT1000 frontend assumptions (defaults; override at compile time if needed):
 * - Rpullup from VCC to divider node
 * - PT1000 from divider node to GND
 * - Optional non-inverting amplifier on the divider node
 */
#ifndef PT100_VCC_UV
#define PT100_VCC_UV (3300000.0)
#endif

#ifndef PT100_PULLUP_OHMS
#define PT100_PULLUP_OHMS (33000.0)
#endif

#ifndef PT100_AMP_GAIN
#define PT100_AMP_GAIN (11.0)
#endif

#ifndef PT100_R0_OHMS
#define PT100_R0_OHMS (1000.0)
#endif

#ifndef PT100_TEMP_MIN_C
#define PT100_TEMP_MIN_C (-70.0)
#endif

#ifndef PT100_TEMP_MAX_C
#define PT100_TEMP_MAX_C (200.0)
#endif

/*  PT1000 initialize */
bool pt1000Init(void);
/*  PT1000 de-initialize */
bool pt1000Deinit(void);
/*  PT1000 16-bit raw data read */
bool pt1000ReadRaw(uint16_t* rawData);
/*  PT1000 32-bit microvolt read */
bool pt1000ReadMicroVolts(uint32_t* microVolts);
/*  PT1000 temperature in 0.1C units */
bool pt1000ReadTemperature_x10(int16_t* temp_x10);
#ifdef __cplusplus
}
#endif
