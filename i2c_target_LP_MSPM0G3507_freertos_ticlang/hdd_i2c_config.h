#ifndef __HDD_I2C_CONFIG_H__
#define __HDD_I2C_CONFIG_H__

/*  I2C Target protocol definition  */
#define HDD_I2C_TARGET_ADDRESS 0x50
#define REG_MODE_0x80 0x80
#define REG_READY_0x81 0x81
#define REG_DATA_0x82 0x82

/*  PT100 definition    */
typedef struct {
  double vcc_uV;
  double pullup_ohms;
  double amp_gain;
} PT100_Config;

/*  NAS2300 definition    */
#define NAS2300_I2C_ADDRESS 0x6D

#define NSA2300_REG_PART_ID 0x01
#define NSA2300_REG_STATUS 0x02
// Data_out<23:16> -> 0x06, Data_out<15:8> -> 0x07, Data_out<7:0> -> 0x08
#define NSA2300_REG_DATA 0x06
// Temp_out<15:8> -> 0x09, Temp_out<7:0> -> 0x0A
#define NSA2300_REG_TEMP 0x09
/*
 *  -> 7-4 Sleep_time
 *    Only active in sleep mode conversion
 *    LSB = 62.5ms
 *  -> 3 SCO
 *    start of conversion, automatically return 0 after conversion ends(except
 * sleep mode conversion)
 *  -> 2-0 measurement_ctrl
 *     - 3'b000: single shot temperature signal conversion
 *     - 3'b001: single shot sensor signal conversion
 *     - 3'b010: combined conversion(once temperature conversion immediately
 * followed by once sensor signal conversion)
 *     - 3'b011: sleep mode conversion(periodically perform once combined
 * conversion with an interval time of 'Sleep_time')
 *     - 3'b100: OTP programming mode
 */
#define NSA2300_REG_CMD 0x30
#define NSA2300_REG_SYS_CONFIG 0xA5
#define NSA2300_REG_P_CONFIG 0xA6
#define NSA2300_REG_T_CONFIG 0xA7

#endif /* __HDD_I2C_CONFIG_H__ */