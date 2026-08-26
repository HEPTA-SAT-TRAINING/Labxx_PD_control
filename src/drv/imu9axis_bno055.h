/**
 * @file imu9axis_bno055.h
 * @author Masaki Naito
 * @brief Arduino Library for BNO055
 * @version 0.1
 * @date 2025-06-25
 * 
 * @copyright UNISEC all rights reserved.
 */


#ifndef IMU9AIXS_BNO055_H
#define IMU9AIXS_BNO055_H

#include <Arduino.h>

class Bno055 {
  public:
    /**
     * @brief Begin transmittion with BNO055
     */
    bool begin(void);

    /**
     * @brief Get acceleration data from BNO055
     * @note unit is m/s^2, range is +-4G
     */
    bool sen_acc(float *ax, float *ay, float *az);

    /**
     * @brief Get gyro data from BNO055
     * @note unit is deg/s, range is +-125deg/s
     */
    bool sen_gyro(float *gx, float *gy, float *gz);

    /**
     * @brief Get magnetometer data from BNO055
     * @note unit is uT
     */
    bool sen_mag(float *mx, float *my, float *mz);

    void print_acc(void);
    void print_gyro(void);
    void print_mag(void);

  private:
    bool _write_reg(uint8_t reg, uint8_t value);
    bool _read_reg(uint8_t reg, uint8_t *value);
    bool _read_bytes(uint8_t start_reg, uint8_t *data, size_t length);
    bool _read_vector(uint8_t start_reg, float scale,
                      float *x, float *y, float *z);
    bool _configure_gyro_range_125dps(void);

    const uint8_t I2C_ADDR_BNO055 = 0x28;
    const uint8_t BNO055_CHIP_ID_VALUE = 0xA0;
    const uint8_t BNO055_OPR_MODE_NDOF = 0x0C; // NDOF mode for 9-axis fusion
    const uint8_t BNO055_OPR_MODE_CONFIG = 0x00;
    const uint8_t BNO055_PAGE_0 = 0x00;
    const uint8_t BNO055_PAGE_1 = 0x01;
    const uint8_t BNO055_GYR_RANGE_MASK = 0x07;
    const uint8_t BNO055_GYR_RANGE_125DPS = 0x04;
    bool _initialized = false;

    typedef enum {
      BNO055_CHIP_ID = 0x00,
      BNO055_PAGE_ID = 0x07,
      BNO055_GYR_CONFIG_0 = 0x0A, // Page 1
      BNO055_OPR_MODE = 0x3D,
      BNO055_ACC_DATA_X_LSB = 0x08,
      BNO055_ACC_DATA_X_MSB = 0x09,
      BNO055_ACC_DATA_Y_LSB = 0x0A,
      BNO055_ACC_DATA_Y_MSB = 0x0B,
      BNO055_ACC_DATA_Z_LSB = 0x0C,
      BNO055_ACC_DATA_Z_MSB = 0x0D,
      BNO055_GYR_DATA_X_LSB = 0x14,
      BNO055_GYR_DATA_X_MSB = 0x15,
      BNO055_GYR_DATA_Y_LSB = 0x16,
      BNO055_GYR_DATA_Y_MSB = 0x17,
      BNO055_GYR_DATA_Z_LSB = 0x18,
      BNO055_GYR_DATA_Z_MSB = 0x19,
      BNO055_MAG_DATA_X_LSB = 0x0E,
      BNO055_MAG_DATA_X_MSB = 0x0F,
      BNO055_MAG_DATA_Y_LSB = 0x10,
      BNO055_MAG_DATA_Y_MSB = 0x11,
      BNO055_MAG_DATA_Z_LSB = 0x12,
      BNO055_MAG_DATA_Z_MSB = 0x13,
    } bno055_reg_t;
};


#endif /* IMU9AIXS_BNO055_H */
