#include "pt100.h"

#include <math.h>

/* Driver configuration */
#include <ti/segger/SEGGER_RTT.h>

#include "ti_drivers_config.h"

PT100_Config cfg = {
    .vcc_uV = PT100_VCC_UV,
    .pullup_ohms = PT100_PULLUP_OHMS,
    .amp_gain = PT100_AMP_GAIN,
};

static ADC_Handle g_adc;
static ADC_Params g_adcParams;

/* Center resistance column from doc/pt1000.md, stored in mOhm. */
static const uint32_t g_pt1000_center_r_mohm[] = {
  723350, 727350, 731340, 735340, 739340, 743330, 747320, 751310,
  755300, 759290, 763280, 767260, 771250, 775230, 779210, 783190,
  787170, 791140, 795120, 799090, 803060, 807030, 811000, 814970,
  818940, 822900, 826870, 830830, 834790, 838750, 842710, 846660,
  850620, 854570, 858530, 862480, 866430, 870380, 874320, 878270,
  882220, 886160, 890100, 894040, 897980, 901920, 905860, 909800,
  913730, 917670, 921600, 925530, 929460, 933390, 937320, 941240,
  945170, 949090, 953020, 956940, 960860, 964780, 968700, 972610,
  976530, 980440, 984360, 988270, 992180, 996090, 1000000, 1003910,
  1007810, 1011720, 1015620, 1019530, 1023430, 1027330, 1031230, 1035130,
  1039030, 1042920, 1046820, 1050710, 1054600, 1058490, 1062380, 1066270,
  1070160, 1074050, 1077940, 1081820, 1085700, 1089590, 1093470, 1097350,
  1101230, 1105100, 1108980, 1112860, 1116730, 1120600, 1124470, 1128350,
  1132210, 1136080, 1139950, 1143820, 1147680, 1151550, 1155410, 1159270,
  1163130, 1166990, 1170850, 1174700, 1178560, 1182410, 1186270, 1190120,
  1193970, 1197820, 1201670, 1205520, 1209360, 1213210, 1217050, 1220900,
  1224740, 1228580, 1232420, 1236260, 1240090, 1243930, 1247770, 1251600,
  1255430, 1259260, 1263090, 1266920, 1270750, 1274580, 1278400, 1282230,
  1286050, 1289870, 1293700, 1297520, 1301330, 1305150, 1308970, 1312780,
  1316600, 1320410, 1324220, 1328030, 1331840, 1335650, 1339460, 1343260,
  1347070, 1350870, 1354680, 1358480, 1362280, 1366080, 1369870, 1373670,
  1377470, 1381260, 1385060, 1388850, 1392640, 1396430, 1400220, 1404000,
  1407790, 1411580, 1415360, 1419140, 1422930, 1426710, 1430490, 1434260,
  1438040, 1441820, 1445590, 1449370, 1453140, 1456910, 1460680, 1464450,
  1468220, 1471980, 1475750, 1479510, 1483280, 1487040, 1490800, 1494560,
  1498320, 1502080, 1505830, 1509590, 1513340, 1517100, 1520850, 1524600,
  1528350, 1532100, 1535840, 1539590, 1543330, 1547080, 1550820, 1554560,
  1558300, 1562040, 1565780, 1569520, 1573250, 1576990, 1580720, 1584450,
  1588180, 1591910, 1595640, 1599370, 1603090, 1606820, 1610540, 1614270,
  1617990, 1621710, 1625430, 1629150, 1632860, 1636580, 1640300, 1644010,
  1647720, 1651430, 1655140, 1658850, 1662560, 1666270, 1669970, 1673680,
  1677380, 1681080, 1684780, 1688480, 1692180, 1695880, 1699580, 1703270,
  1706960, 1710660, 1714350, 1718040, 1721730, 1725420, 1729100, 1732790,
  1736480, 1740160, 1743840, 1747520, 1751200, 1754880, 1758560,
};

static bool PT1000_convert_adc_uV_to_temp_x10(const PT100_Config *cfg,
                                             uint32_t adc_uV, int16_t *temp_x10,
                                             uint32_t *r_mohm,
                                             uint32_t *v_node_uV);
static bool pt1000_temp_x10_from_lookup(double r_ohm, int16_t *temp_x10);

bool pt1000Init(void) {
  ADC_Params_init(&g_adcParams);
  g_adc = ADC_open(CONFIG_ADC_0, &g_adcParams);
  if (g_adc == NULL) {
    SEGGER_RTT_printf(0, "PT1000: Error initializing CONFIG_ADC_0\n");
    return false;
  } else {
    SEGGER_RTT_printf(0, "PT1000: CONFIG_ADC_0 initialized successfully\n");
    return true;
  }
}

bool pt1000Deinit(void) {
  if (g_adc != NULL) {
    ADC_close(g_adc);
    g_adc = NULL;
    SEGGER_RTT_printf(0, "PT1000: CONFIG_ADC_0 deinitialized\n");
  }
  return true;
}

bool pt1000ReadRaw(uint16_t *rawData) {
  if (g_adc == NULL) {
    SEGGER_RTT_printf(0, "PT1000: ADC not initialized\n");
    return false;
  }

  uint16_t localRaw = 0;
  int_fast16_t res = ADC_convert(g_adc, &localRaw);
  if (res != ADC_STATUS_SUCCESS) {
    SEGGER_RTT_printf(0, "PT1000: ADC_convert failed\n");
    return false;
  }

  if (rawData) {
    *rawData = localRaw;
  }
  return true;
}

bool pt1000ReadMicroVolts(uint32_t *microVolts) {
  uint16_t localRaw = 0;
  if (!pt1000ReadRaw(&localRaw)) {
    return false;
  }
  uint32_t local_uV = ADC_convertRawToMicroVolts(g_adc, localRaw);

  if (microVolts) {
    *microVolts = local_uV;
  }
  SEGGER_RTT_printf(0, "PT1000: localRaw: %u, ADC microVolts = %u\n", (unsigned)localRaw, (unsigned)local_uV);
  return true;
}

bool pt1000ReadTemperature_x10(int16_t *temp_x10) {
  uint32_t microVolts = 0;
  if (!pt1000ReadMicroVolts(&microVolts)) {
    return false;
  }

  int16_t localTemp_x10 = 0;
  uint32_t local_r_mohm = 0;
  if (!PT1000_convert_adc_uV_to_temp_x10(&cfg, microVolts, &localTemp_x10,
                                         &local_r_mohm, NULL)) {
    return false;
  }

  {
    unsigned int ohm_int = (unsigned int)(((unsigned)local_r_mohm) / 1000U);
    unsigned int ohm_frac = (unsigned int)(((unsigned)local_r_mohm) % 1000U);
    SEGGER_RTT_printf(
        0,
        "PT1000: calculated resistance = %u mOhm (%u.%03u Ohm), temp_x10 = %d\n",
        (unsigned)local_r_mohm, ohm_int, ohm_frac, (int)localTemp_x10);
  }

  if (temp_x10) {
    *temp_x10 = localTemp_x10;
  }

  return true;
}

static bool pt1000_temp_x10_from_lookup(double r_ohm, int16_t *temp_x10) {
  const uint16_t entry_count =
      (uint16_t)(sizeof(g_pt1000_center_r_mohm) / sizeof(g_pt1000_center_r_mohm[0]));
  const int32_t min_temp_x10 = (int32_t)lround(PT100_TEMP_MIN_C * 10.0);
  const uint32_t r_mohm = (uint32_t)lround(r_ohm * 1000.0);

  if ((temp_x10 == NULL) || (entry_count < 2U)) {
    return false;
  }

  if ((r_mohm < g_pt1000_center_r_mohm[0]) ||
      (r_mohm > g_pt1000_center_r_mohm[entry_count - 1U])) {
    return false;
  }

  if (r_mohm == g_pt1000_center_r_mohm[entry_count - 1U]) {
    *temp_x10 = (int16_t)(min_temp_x10 + (int32_t)(entry_count - 1U) * 10);
    return true;
  }

  uint16_t lo = 0U;
  uint16_t hi = (uint16_t)(entry_count - 1U);
  while ((uint16_t)(hi - lo) > 1U) {
    const uint16_t mid = (uint16_t)(lo + (hi - lo) / 2U);
    if (r_mohm < g_pt1000_center_r_mohm[mid]) {
      hi = mid;
    } else {
      lo = mid;
    }
  }

  const uint32_t r_lo = g_pt1000_center_r_mohm[lo];
  const uint32_t r_hi = g_pt1000_center_r_mohm[hi];
  const int32_t base_temp_x10 = min_temp_x10 + (int32_t)lo * 10;

  if (r_hi <= r_lo) {
    return false;
  }

  if (r_mohm == r_lo) {
    *temp_x10 = (int16_t)base_temp_x10;
    return true;
  }

  *temp_x10 =
      (int16_t)(base_temp_x10 +
                (int32_t)lround(((double)(r_mohm - r_lo) * 10.0) /
                                (double)(r_hi - r_lo)));
  return true;
}

static uint16_t pt1000_lookup_entry_count(void) {
  return (uint16_t)(sizeof(g_pt1000_center_r_mohm) /
                    sizeof(g_pt1000_center_r_mohm[0]));
}

static bool PT1000_convert_adc_uV_to_temp_x10(const PT100_Config *cfg,
                                              uint32_t adc_uV, int16_t *temp_x10,
                                              uint32_t *r_mohm,
                                              uint32_t *v_node_uV) {
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

  int16_t local_temp_x10 = 0;
  if (!pt1000_temp_x10_from_lookup(rpt_ohm, &local_temp_x10)) {
    const uint16_t entry_count = pt1000_lookup_entry_count();
    SEGGER_RTT_printf(
        0,
        "PT1000: lookup failed, adc_uV=%u, v_div_uV=%u, r_mohm=%u, range=[%u,%u]\n",
        (unsigned)adc_uV, (unsigned)lround(v_div_uV),
        (unsigned)lround(rpt_ohm * 1000.0),
        (unsigned)g_pt1000_center_r_mohm[0],
        (unsigned)g_pt1000_center_r_mohm[entry_count - 1U]);
    return false;
  }

  if (temp_x10) {
    *temp_x10 = local_temp_x10;
  }
  if (r_mohm) {
    long rm = lround(rpt_ohm * 1000.0);
    if (rm < 0)
      rm = 0;
    *r_mohm = (uint32_t)rm;
  }
  if (v_node_uV) {
    long vv = lround(v_div_uV);
    if (vv < 0)
      vv = 0;
    *v_node_uV = (uint32_t)vv;
  }

  return true;
}