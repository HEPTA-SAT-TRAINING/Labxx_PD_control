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
    void begin(void);

    /**
     * @brief Get acceleration data from BNO055
     * @note unit is m/s^2, range is +-4G
     */
    void sen_acc(float *ax, float *ay, float *az);

    /**
     * @brief Get gyro data from BNO055
     * @note unit is deg/s, range is +-125deg/s
     */
    void sen_gyro(float *gx, float *gy, float *gz);

    /**
     * @brief Get magnetometer data from BNO055
     * @note unit is uT
     */
    void sen_mag(float *mx, float *my, float *mz);

    void print_acc(void);
    void print_gyro(void);
    void print_mag(void);

  private:
    void _write_reg(uint8_t reg, uint8_t value);
    uint8_t _read_reg(uint8_t reg);

    const uint8_t I2C_ADDR_BNO055 = 0x28;
    const uint8_t BNO055_OPR_MODE_NDOF = 0x0C; // NDOF mode for 9-axis fusion

    typedef enum {
      BNO055_CHIP_ID = 0x00,
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
