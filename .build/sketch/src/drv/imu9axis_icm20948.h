#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\drv\\imu9axis_icm20948.h"
/**
 * @file imu9axis_icm20948.h
 * @author Masaki Naito
 * @brief Arduino Library for ICM-20948
 * @version 0.1
 * @date 2025-01-16
 * 
 * @copyright UNISEC all rights reserved.
 */


#ifndef IMU9AXIS_ICM20948
#define IMU9AXIS_ICM20948

#include <Arduino.h>

typedef enum {
  ACCEL_FS_2G = 0,
  ACCEL_FS_4G = 1,
  ACCEL_FS_8G = 2,
  ACCEL_FS_16G = 3
} ACCEL_FS_SEL;

typedef enum {
  GYRO_FS_250dps = 0,
  GYRO_FS_500dps = 1,
  GYRO_FS_1000dps = 2,
  GYRO_FS_2000dps = 3
} GYRO_FS_SEL;

typedef enum {
  AK09916_PWR_DOWN           = 0x00,
  AK09916_TRIGGER_MODE       = 0x01,
  AK09916_CONT_MODE_10HZ     = 0x02,
  AK09916_CONT_MODE_20HZ     = 0x04,
  AK09916_CONT_MODE_50HZ     = 0x06,
  AK09916_CONT_MODE_100HZ    = 0x08
} AK09916_OP_MODE;


class Icm20948 {
  public:
    /**
     * @brief Begin transmittion
     */
    void begin(void);

    /**
     * @brief Get acceleration data
     * @note unit is m/s^2, range is +-4G
     */
    void get_accel(float *ax, float *ay, float *az);

    /**
     * @brief Get gyro data
     * @note unit is deg/s, range is +-125deg/s
     */
    void get_gyro(float *gx, float *gy, float *gz);

    /**
     * @brief Get magnetometer data
     * @note unit is uT
     */
    void get_mag(float *mx, float *my, float *mz);

    void print_accel(void);
    void print_gyro(void);
    void print_mag(void);

    void set_accel_scale(ACCEL_FS_SEL scale);
    void set_gyro_scale(GYRO_FS_SEL scale);

    ACCEL_FS_SEL get_accel_scale(void);
    GYRO_FS_SEL get_gyro_scale(void);

  private:
    ACCEL_FS_SEL _accel_scale;
    GYRO_FS_SEL _gyro_scale;
    float _accel_sensitivity;
    float _gyro_sensitivity;

    /* ---------------------
      IMU
    --------------------- */
    typedef enum {
      BANK0,
      BANK1,
      BANK2,
      BANK3
    } ICM20948_BANK;

    const float ACCEL_SCALE_FACTOR[4] = {
      16384.0, 8192.0, 4096.0, 2048.0
    };

    const float GYRO_SCALE_FACTOR[4] = {
      131.0, 65.5, 32.8, 16.4
    };

    // 7bit address
    const uint8_t ICM20948_I2C_ADDR = 0x68;

    // for all banks
    const uint8_t REG_BANK_SEL = 0x7F;

    // for bank0
    const uint8_t ICM20948_WHO_AM_I = 0x00;
    const uint8_t ICM20948_USER_CTRL = 0x03;
    const uint8_t ICM20948_PWR_MGMT_1 = 0x06;
    const uint8_t ICM20948_INT_PIN_CFG = 0x0F;
    const uint8_t ICM20948_ACCEL_XOUT_H = 0x2D;
    const uint8_t ICM20948_GYRO_XOUT_H = 0x33;
    const uint8_t ICM20948_EXT_SLV_SENS_DATA_00 = 0x38;

    // for bank2
    const uint8_t ICM20948_GYRO_CONFIG_1 = 0x01;
    const uint8_t ICM20948_ODR_ALIGN_EN = 0x09;
    const uint8_t ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10;
    const uint8_t ICM20948_ACCEL_CONFIG = 0x14;

    // for bank3
    const uint8_t ICM20948_I2C_MST_ODR_CFG = 0x00;
    const uint8_t ICM20948_I2C_MST_CTRL = 0x01;
    const uint8_t ICM20948_I2C_MST_DELAY_CTRL = 0x02;
    const uint8_t ICM20948_I2C_SLV0_ADDR = 0x03;
    const uint8_t ICM20948_I2C_SLV0_REG = 0x04;
    const uint8_t ICM20948_I2C_SLV0_CTRL = 0x05;
    const uint8_t ICM20948_I2C_SLV0_DO = 0x06;
    const uint8_t ICM20948_I2C_SLV4_ADDR = 0x13;
    const uint8_t ICM20948_I2C_SLV4_REG = 0x14;
    const uint8_t ICM20948_I2C_SLV4_CTRL = 0x15;
    const uint8_t ICM20948_I2C_SLV4_DO = 0x16;
    const uint8_t ICM20948_I2C_SLV4_DI = 0x17;

    const uint8_t ICM20948_BYPASS_EN = 1 << 1;
    const uint8_t ICM20948_I2C_MST_RST = 1 << 1;
    const uint8_t ICM20948_I2C_SLVX_EN = 1 << 7;

    // settings
    const uint8_t ACCEL_DLPF = 6;

    void _reset(void);
    void _wakeup(void);
    void _select_bank(ICM20948_BANK bank);
  
    void _reg_write(uint8_t reg, uint8_t val);
    uint8_t _reg_read(uint8_t reg);
    void _reg_read(uint8_t reg, uint8_t val[], uint8_t len);

    void _ak09916_init(void);
    void _ak09916_reset(void);

    void _ak09916_reg_write(uint8_t reg, uint8_t val);
    uint8_t _ak09916_reg_read(uint8_t reg);
    void _ak09916_reg_read(uint8_t reg, uint8_t val[], uint8_t len);

    /* ---------------------
      MAG(AK09916)
    --------------------- */
    const uint8_t AK09916_I2C_ADDR = 0x0C;

    const uint8_t AK09916_WHO_AM_I = 0x01;
    const uint8_t AK09916_XAXIS_HIGH = 0x11;
    const uint8_t AK09916_CNTL_2 = 0x31;
    const uint8_t AK09916_CNTL_3 = 0x32;

    const uint8_t AK09916_DEVICE_ID = 0x09;
    const uint8_t AK09916_RESET = 1;
    const uint8_t AK09916_READ = 1 << 7;

    const float AK09916_MAG_LSB = 0.15;
};


#endif /* IMU9AXIS_ICM20948 */
