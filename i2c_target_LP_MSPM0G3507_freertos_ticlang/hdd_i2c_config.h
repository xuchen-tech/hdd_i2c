#ifndef __HDD_I2C_CONFIG_H__
#define __HDD_I2C_CONFIG_H__


#define HDD_I2C_TARGET_ADDRESS 0x50

#define HDD_I2C_PART_ID 0x55

#define REG_PART_ID_0x00 0x00   // Part ID register
#define REG_REV_ID_0x01 0x01    // Rev ID register
#define REG_DEV_COUNT_0x02 0x02 // Device count register
#define REG_DATA_0x03 0x03     // Data register start
#define REG_MODE_0x80 0x80  // mode register
#define REG_READY_0x81 0x81 // ready register
#define REG_DATA_0x82 0x82  // data register

#define HDD_I2C_REVISION_ID 0x01

#define HDD_I2C_CMD_ENUM 0xF1

// 
#define HDD_I2C_DEV_DETECT_PERIOD   500000

// NSA2300
#define HDD_I2C_NSA2300_ADDRESS 0x6D
#define HDD_I2C_NSA2300_MAX_POLLS  3000

#endif /* __HDD_I2C_CONFIG_H__ */