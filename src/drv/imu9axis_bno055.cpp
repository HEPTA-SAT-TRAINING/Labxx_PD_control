/**
 * @file imu9axis_bno055.cpp
 * @author Masaki Naito
 * @brief Arduino Library for BNO055
 * @version 0.1
 * @date 2025-06-25
 * 
 * @copyright UNISEC all rights reserved.
 */

#include "imu9axis_bno055.h"

#include <Wire.h>

bool Bno055::begin(void) {
  if(millis() < 750) {
    while(millis() < 750) {
      // Wait for 750ms to ensure the sensor is ready
    }
  }

  Wire.begin();
  _initialized = false;

  // Recover from an interrupted previous configuration attempt.
  if (!_write_reg(BNO055_PAGE_ID, BNO055_PAGE_0)) return false;

  uint8_t chip_id = 0;
  if (!_read_reg(BNO055_CHIP_ID, &chip_id) ||
      chip_id != BNO055_CHIP_ID_VALUE) {
    return false;
  }

  if (!_configure_gyro_range_125dps()) {
    return false;
  }

  if (!_write_reg(BNO055_OPR_MODE, BNO055_OPR_MODE_NDOF)) {
    return false;
  }

  delay(10);
  _initialized = true;
  return true;
}

bool Bno055::_configure_gyro_range_125dps(void) {
  // Gyroscope configuration is on register page 1 and must be changed while
  // the BNO055 is in CONFIGMODE. Preserve the bandwidth bits controlled by
  // the fusion mode and replace only the three range bits.
  if (!_write_reg(BNO055_OPR_MODE, BNO055_OPR_MODE_CONFIG)) return false;
  delay(20);
  bool configured = false;
  if (!_write_reg(BNO055_PAGE_ID, BNO055_PAGE_1)) return false;
  uint8_t gyro_config = 0;
  if (_read_reg(BNO055_GYR_CONFIG_0, &gyro_config)) {
    gyro_config = static_cast<uint8_t>(
        (gyro_config & ~BNO055_GYR_RANGE_MASK) | BNO055_GYR_RANGE_125DPS);
    if (_write_reg(BNO055_GYR_CONFIG_0, gyro_config)) {
      uint8_t gyro_config_readback = 0;
      configured =
          _read_reg(BNO055_GYR_CONFIG_0, &gyro_config_readback) &&
          (gyro_config_readback & BNO055_GYR_RANGE_MASK) ==
              BNO055_GYR_RANGE_125DPS;
    }
  }
  const bool page_restored = _write_reg(BNO055_PAGE_ID, BNO055_PAGE_0);
  return configured && page_restored;
}

bool Bno055::sen_acc(float *ax, float *ay, float *az) {
  return _read_vector(BNO055_ACC_DATA_X_LSB, 0.01f, ax, ay, az);
}

bool Bno055::sen_gyro(float *gx, float *gy, float *gz) {
  return _read_vector(BNO055_GYR_DATA_X_LSB, 1.0f / 16.0f,
                      gx, gy, gz);
}

bool Bno055::sen_mag(float *mx,float *my,float *mz) {
  return _read_vector(BNO055_MAG_DATA_X_LSB, 1.0f / 16.0f,
                      mx, my, mz);
}

void Bno055::print_acc(void) {
  float ax, ay, az;

  sen_acc(&ax, &ay, &az);

  Serial.print("ax: ");
  Serial.print(ax);
  Serial.print(" m/s^2, ");
  Serial.print("ay: ");
  Serial.print(ay);
  Serial.print(" m/s^2, ");
  Serial.print("az: ");
  Serial.print(az);
  Serial.println(" m/s^2");
}

void Bno055::print_gyro(void) {
  float gx, gy, gz;

  sen_gyro(&gx, &gy, &gz);

  Serial.print("gx: ");
  Serial.print(gx);
  Serial.print(" deg/s, ");
  Serial.print("gy: ");
  Serial.print(gy);
  Serial.print(" deg/s, ");
  Serial.print("gz: ");
  Serial.print(gz);
  Serial.println(" deg/s");
}

void Bno055::print_mag(void) {
  float mx, my, mz;

  sen_mag(&mx, &my, &mz);

  Serial.print("mx: ");
  Serial.print(mx);
  Serial.print(" uT, ");
  Serial.print("my: ");
  Serial.print(my);
  Serial.print(" uT, ");
  Serial.print("mz: ");
  Serial.print(mz);
  Serial.println(" uT");
}


/* ----------------- 
  Private functions
------------------ */

bool Bno055::_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2C_ADDR_BNO055);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Bno055::_read_reg(uint8_t reg, uint8_t *value) {
  return value != nullptr && _read_bytes(reg, value, 1);
}

bool Bno055::_read_vector(uint8_t start_reg, float scale,
                          float *x, float *y, float *z) {
  if (x == nullptr || y == nullptr || z == nullptr) return false;

  uint8_t data[6] = {0};
  if ((!_initialized && !begin()) ||
      !_read_bytes(start_reg, data, sizeof(data))) {
    _initialized = false;
    if (!begin() || !_read_bytes(start_reg, data, sizeof(data))) {
      *x = 0.0f;
      *y = 0.0f;
      *z = 0.0f;
      return false;
    }
  }

  *x = static_cast<int16_t>(
           (static_cast<uint16_t>(data[1]) << 8) | data[0]) * scale;
  *y = static_cast<int16_t>(
           (static_cast<uint16_t>(data[3]) << 8) | data[2]) * scale;
  *z = static_cast<int16_t>(
           (static_cast<uint16_t>(data[5]) << 8) | data[4]) * scale;
  return true;
}

bool Bno055::_read_bytes(uint8_t start_reg, uint8_t *data,
                         size_t length) {
  if (data == nullptr || length == 0) return false;

  Wire.beginTransmission(I2C_ADDR_BNO055);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) return false;

  const size_t received = Wire.requestFrom(I2C_ADDR_BNO055, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) return false;
    data[i] = Wire.read();
  }
  return true;
}
