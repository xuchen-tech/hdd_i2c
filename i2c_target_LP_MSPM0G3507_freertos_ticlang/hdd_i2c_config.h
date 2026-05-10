#ifndef __HDD_I2C_CONFIG_H__
#define __HDD_I2C_CONFIG_H__

/*  I2C Target protocol definition  */
#define BUFFER_SIZE 128
#define HDD_I2C_TARGET_ADDRESS 0x50
#define REG_MODE_0x80 0x80
#define REG_READY_0x81 0x81
#define REG_DATA_0x82 0x82
#define REG_CALIBRATION_0x83 0x83
#define REG_CTRL_0x84        0x84

/* REG_CTRL (0x84) bit definitions */
#define REG_CTRL_BIT0_CALIBRATION     (1u << 0)  /* 0=output raw NSA2300 data, 1=apply calibration */
#define REG_CTRL_BIT1_INPUT_SWAP      (1u << 1)  /* 0=NSA2300 input normal, 1=NSA2300 input swap */
#define REG_CTRL_BIT2_AVERAGING       (1u << 2)  /* 0=single reading, 1=enable rolling average */
#define REG_CTRL_AVGCOUNT_MASK        (3u << 3)  /* bits[4:3]: averaging sample count select */
#define REG_CTRL_AVGCOUNT_8           (0u << 3)  /*  8 samples */
#define REG_CTRL_AVGCOUNT_16          (1u << 3)  /* 16 samples (default) */
#define REG_CTRL_AVGCOUNT_32          (2u << 3)  /* 32 samples */
/* Default: input_swap=1 (P_CONFIG=0x71), calibration off, averaging off, count=16 */
#define REG_CTRL_DEFAULT              (REG_CTRL_BIT1_INPUT_SWAP | REG_CTRL_AVGCOUNT_16)

/*  PT1000 definition    */
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

#define NSA2300_REG_SYS_CONFIG_DEFAULT 0x12
#define NSA2300_REG_P_CONFIG_DEFAULT 0x71
#define NSA2300_REG_T_CONFIG_DEFAULT 0x81
/* P_CONFIG base value without input_swap; bit 6 is dynamically set via REG_CTRL bit 1 */
#define NSA2300_REG_P_CONFIG_BASE           0x31u
#define NSA2300_REG_P_CONFIG_INPUT_SWAP_BIT 0x40u

#define NSA2300_CMD_SINGLE_PRESSURE_CONVERSION 0x09
#define NSA2300_STATUS_DRDY_MASK 0x01

/*  thread definition */
#define TASKSTACKSIZE 640

typedef enum {
  HDD_I2C_MODE_IDLE = 0x00,
  HDD_I2C_MODE_D1 = 0xD1,
  HDD_I2C_MODE_D2 = 0xD2,
} HDD_I2C_Mode;

typedef enum {
  HDD_I2C_DIRECTION_WRITE = 0,
  HDD_I2C_DIRECTION_READ = 1,
} HDD_I2C_DIRECTION;

#endif /* __HDD_I2C_CONFIG_H__ */