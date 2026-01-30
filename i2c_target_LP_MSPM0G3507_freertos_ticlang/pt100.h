#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADC.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

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
#define PT100_PULLUP_OHMS (49900.0)
#endif

#ifndef PT100_AMP_GAIN
#define PT100_AMP_GAIN (101.0)
#endif

typedef struct {
    double vcc_uV;
    double pullup_ohms;
    double amp_gain;
} PT100_Config;

/* Latest published temperature (0.1C units) for sharing with other modules */
extern volatile int16_t g_latestTemp_x10;
extern volatile uint32_t g_latestTempSeq;

void PT100_Config_init(PT100_Config *cfg);

/*
 * Convert an ADC reading (in microvolts at ADC input) to PT100 temperature.
 * Outputs:
 * - temp_x10: temperature in 0.1C
 * - r_mohm: PT100 resistance in milliohms (for logging/diagnostics)
 * - v_node_uV: divider node voltage (after undoing amplifier gain)
 */
bool PT100_convert_adc_uV_to_temp_x10(const PT100_Config *cfg,
                                     uint32_t adc_uV,
                                     int16_t *temp_x10,
                                     uint32_t *r_mohm,
                                     uint32_t *v_node_uV);

/*
 * One-shot sample helper:
 * - does ADC_convert()
 * - converts raw to uV
 * - converts to temperature
 */
bool PT100_sample(ADC_Handle adc,
                  const PT100_Config *cfg,
                  uint16_t *raw,
                  uint32_t *adc_uV,
                  int16_t *temp_x10,
                  uint32_t *r_mohm,
                  uint32_t *v_node_uV);

/* Publish latest temperature for other threads/modules */
void PT100_publish_latest(int16_t temp_x10);

/*
 * PT100 sampling thread:
 * - waits for task-notify trigger
 * - samples PT100 via ADC
 * - updates g_latestTemp_x10/g_latestTempSeq
 */
void *pt100Thread(void *arg0);

/* Task handle for notifying the PT100 thread */
TaskHandle_t PT100_getTaskHandle(void);

#ifdef __cplusplus
}
#endif
