/**
 * @file imu9axis_icm20948.cpp
 * @author Masaki Naito
 * @brief Arduino Library for ICM-20948
 * @version 0.1
 * @date 2025-01-16
 * 
 * @copyright UNISEC all rights reserved.
 */


#include "imu9axis_icm20948.h"

#include <Wire.h>

void Icm20948::begin(void) {
  uint8_t ack;

  Wire.begin();

  _reset();
  delay(10);
  _wakeup();

  ack = _reg_read(ICM20948_WHO_AM_I);
  Serial.print("ack: ");
  Serial.println(ack, HEX);

  _ak09916_init();

  set_accel_scale(ACCEL_FS_2G);
  set_gyro_scale(GYRO_FS_500dps);
}

void Icm20948::get_accel(float *ax, float *ay, float *az) {
  uint8_t buf[6] = {0};

  _select_bank(BANK0);
  _reg_read(ICM20948_ACCEL_XOUT_H, buf, 6);

  *ax = (int16_t)(buf[0] << 8 | buf[1]) / _accel_sensitivity;
  *ay = (int16_t)(buf[2] << 8 | buf[3]) / _accel_sensitivity;
  *az = (int16_t)(buf[4] << 8 | buf[5]) / _accel_sensitivity;
}

void Icm20948::get_gyro(float *gx, float *gy, float *gz) {
  uint8_t buf[6] = {0};

  _select_bank(BANK0);
  _reg_read(ICM20948_GYRO_XOUT_H, buf, 6);

  *gx = (int16_t)(buf[0] << 8 | buf[1]) / _gyro_sensitivity;
  *gy = (int16_t)(buf[2] << 8 | buf[3]) / _gyro_sensitivity;
  *gz = (int16_t)(buf[4] << 8 | buf[5]) / _gyro_sensitivity;
}

void Icm20948::get_mag(float *mx, float *my, float *mz) {
  uint8_t buf[6] = {0};

  _ak09916_reg_write(AK09916_CNTL_2, (uint8_t)AK09916_TRIGGER_MODE);
  _ak09916_reg_read(AK09916_XAXIS_HIGH, buf, 6);

  *mx = (int16_t)(buf[1] << 8 | buf[0]) * AK09916_MAG_LSB;
  *my = (int16_t)(buf[3] << 8 | buf[2]) * AK09916_MAG_LSB;
  *mz = (int16_t)(buf[5] << 8 | buf[4]) * AK09916_MAG_LSB;
}

void Icm20948::print_accel(void) {
  float ax, ay, az;
  get_accel(&ax, &ay, &az);

  Serial.print("ax: ");   Serial.print(ax);
  Serial.print(", ay: "); Serial.print(ay);
  Serial.print(", az: "); Serial.println(az);
}

void Icm20948::print_gyro(void) {
  float gx, gy, gz;
  get_gyro(&gx, &gy, &gz);

  Serial.print("gx: ");   Serial.print(gx);
  Serial.print(", gy: "); Serial.print(gy);
  Serial.print(", gz: "); Serial.println(gz);
}

void Icm20948::print_mag(void) {
  float mx, my, mz;
  get_mag(&mx, &my, &mz);

  Serial.print("mx: ");   Serial.print(mx);
  Serial.print(", my: "); Serial.print(my);
  Serial.print(", mz: "); Serial.println(mz);
}

void Icm20948::set_accel_scale(ACCEL_FS_SEL scale) {
  _accel_scale = scale;
  _accel_sensitivity = ACCEL_SCALE_FACTOR[scale];

  _select_bank(BANK2);
  _reg_write(ICM20948_ACCEL_CONFIG, ACCEL_DLPF << 3 | scale << 1 | 1);
  _reg_write(ICM20948_ACCEL_SMPLRT_DIV_1, 10);
  _reg_write(ICM20948_ODR_ALIGN_EN, 1);
}

void Icm20948::set_gyro_scale(GYRO_FS_SEL scale) {
  _gyro_scale = scale;
  _gyro_sensitivity = GYRO_SCALE_FACTOR[scale];

  _select_bank(BANK2);
  _reg_write(ICM20948_GYRO_CONFIG_1, scale << 1 | 1);
}

/* ----------------------
  private functions
------------------------- */
void Icm20948::_reset(void) {
  _select_bank(BANK0);
  _reg_write(ICM20948_PWR_MGMT_1, 0x81);
  delay(10);
}

void Icm20948::_wakeup(void) {
  // SLEEP disable
  _select_bank(BANK0);
  _reg_write(ICM20948_PWR_MGMT_1, 0x01);
}

void Icm20948::_select_bank(ICM20948_BANK bank) {
  if(bank > 4) {
    return;
  }

  _reg_write(REG_BANK_SEL, (uint8_t)bank << 4);
}

void Icm20948::_reg_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM20948_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t Icm20948::_reg_read(uint8_t reg) {
  uint8_t ret;

  Wire.beginTransmission(ICM20948_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(ICM20948_I2C_ADDR, 1);
  if(Wire.available()) {
    ret = Wire.read();
  }

  return ret;
}

void Icm20948::_reg_read(uint8_t reg, uint8_t val[], uint8_t len) {
  Wire.beginTransmission(ICM20948_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(ICM20948_I2C_ADDR, len);
  if(Wire.available()) {
    for(uint8_t i = 0; i < len; i++) {
      val[i] = Wire.read();
    }
  }
}

void Icm20948::_ak09916_init(void) {
  _select_bank(BANK0);
  _reg_write(ICM20948_INT_PIN_CFG, ICM20948_BYPASS_EN);
  _ak09916_reset();
  _ak09916_reg_write(AK09916_CNTL_2, (uint8_t)AK09916_CONT_MODE_100HZ);

  _select_bank(BANK2);
  _reg_write(ICM20948_ODR_ALIGN_EN, 1);
}

void Icm20948::_ak09916_reset(void) {
  _ak09916_reg_write(AK09916_CNTL_3, 0x01);
}

void Icm20948::_ak09916_reg_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AK09916_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delay(10);
}

uint8_t Icm20948::_ak09916_reg_read(uint8_t reg) {
  uint8_t ret;
  Wire.beginTransmission(AK09916_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(AK09916_I2C_ADDR, 1);
  if(Wire.available()) {
    ret = Wire.read();
  }
  return ret;
}

void Icm20948::_ak09916_reg_read(uint8_t reg, uint8_t val[], uint8_t len) {
  Wire.beginTransmission(AK09916_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(AK09916_I2C_ADDR, len);
  if(Wire.available()) {
    for(uint8_t i = 0; i < len; i++) {
      val[i] = Wire.read();
    }
  }
}
