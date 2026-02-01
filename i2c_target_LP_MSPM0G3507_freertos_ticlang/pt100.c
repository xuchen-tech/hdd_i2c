#include "pt100.h"

#include <math.h>

/* Driver configuration */
#include <ti/segger/SEGGER_RTT.h>

#include "ti_drivers_config.h"

static ADC_Handle g_adc;
static ADC_Params g_adcParams;

volatile int16_t g_latestTemp_x10 = 0;
volatile uint32_t g_latestTempSeq = 0;

bool pt100Init(void) {
  ADC_Params_init(&g_adcParams);
  g_adc = ADC_open(CONFIG_ADC_0, &g_adcParams);
  if (g_adc == NULL) {
    SEGGER_RTT_printf(0, "PT100: Error initializing CONFIG_ADC_0\n");
    return false;
  } else {
    SEGGER_RTT_printf(0, "PT100: CONFIG_ADC_0 initialized successfully\n");
    return true;
  }
}

bool pt100Deinit(void) {
  if (g_adc != NULL) {
    ADC_close(g_adc);
    g_adc = NULL;
    SEGGER_RTT_printf(0, "PT100: CONFIG_ADC_0 deinitialized\n");
  }
  return true;
}

bool pt100ReadRaw(uint16_t* rawData) {
  if (g_adc == NULL) {
    SEGGER_RTT_printf(0, "PT100: ADC not initialized\n");
    return false;
  }

  uint16_t localRaw = 0;
  int_fast16_t res = ADC_convert(g_adc, &localRaw);
  if (res != ADC_STATUS_SUCCESS) {
    SEGGER_RTT_printf(0, "PT100: ADC_convert failed\n");
    return false;
  }

  if (rawData) {
    *rawData = localRaw;
  }
  return true;
}

bool pt100ReadMicroVolts(uint32_t* microVolts) {
  uint16_t localRaw = 0;
  if (!pt100ReadRaw(&localRaw)) {
    return false;
  }

  uint32_t local_uV = ADC_convertRawToMicroVolts(g_adc, localRaw);

  if (microVolts) {
    *microVolts = local_uV;
  }
  return true;
}


static double pt100_tempC_from_ohms(double r_ohm) {
  /* IEC 60751 Callendar–Van Dusen coefficients */
  const double R0 = 100.0;
  const double A = 3.9083e-3;
  const double B = -5.775e-7;
  const double C = -4.183e-12;

  if (r_ohm >= R0) {
    const double z = r_ohm / R0;
    double disc = A * A - 4.0 * B * (1.0 - z);
    if (disc < 0.0) {
      disc = 0.0;
    }
    return (-A + sqrt(disc)) / (2.0 * B);
  }

  /* T < 0C: Newton iteration on full CVD equation */
  double t = -20.0;
  for (int i = 0; i < 12; i++) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double f = R0 * (1.0 + A * t + B * t2 + C * (t - 100.0) * t3) - r_ohm;
    const double df = R0 * (A + 2.0 * B * t + C * (4.0 * t3 - 300.0 * t2));
    if (fabs(df) < 1e-12) {
      break;
    }
    t -= f / df;
  }
  return t;
}


bool PT100_convert_adc_uV_to_temp_x10(const PT100_Config* cfg, uint32_t adc_uV,
                                      int16_t* temp_x10, uint32_t* r_mohm,
                                      uint32_t* v_node_uV) {
  if (cfg == NULL) {
    return false;
  }

  /*
   * If an amplifier is used and the ADC reading is close to the supply rail,
   * the op-amp output is likely saturated and the divider-node voltage can't
   * be reconstructed accurately. Reject instead of producing a misleading
   * temperature.
   */
  const double vcc_uV = cfg->vcc_uV;
  if ((cfg->amp_gain > 1.0) && ((double)adc_uV >= (vcc_uV - 10000.0))) {
    return false;
  }

  /* Undo amplifier gain to get the divider node voltage */
  const double v_div_uV = ((double)adc_uV) / cfg->amp_gain;

  if (!(v_div_uV > 0.0) || !(v_div_uV < (vcc_uV - 1.0))) {
    return false;
  }

  /* Divider: Vnode = Vcc * Rpt / (Rpullup + Rpt) => Rpt = Rpullup * Vnode /
   * (Vcc - Vnode) */
  const double rpt_ohm = cfg->pullup_ohms * v_div_uV / (vcc_uV - v_div_uV);

  const double tempC = pt100_tempC_from_ohms(rpt_ohm);
  long t10 = lround(tempC * 10.0);
  if (t10 > 32767) t10 = 32767;
  if (t10 < -32768) t10 = -32768;

  if (temp_x10) {
    *temp_x10 = (int16_t)t10;
  }
  if (r_mohm) {
    long rm = lround(rpt_ohm * 1000.0);
    if (rm < 0) rm = 0;
    *r_mohm = (uint32_t)rm;
  }
  if (v_node_uV) {
    long vv = lround(v_div_uV);
    if (vv < 0) vv = 0;
    *v_node_uV = (uint32_t)vv;
  }

  return true;
}