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

void Bno055::begin(void) {
  if(millis() < 750) {
    while(millis() < 750) {
      // Wait for 750ms to ensure the sensor is ready
    }
  }

  Wire.begin();
  _write_reg(BNO055_OPR_MODE, BNO055_OPR_MODE_NDOF);

  delay(10);
}

void Bno055::sen_acc(float *ax, float *ay, float *az) {
  uint8_t data[6];

  for(uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(I2C_ADDR_BNO055);
    Wire.write(BNO055_ACC_DATA_X_LSB + i);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR_BNO055, 1);
    if (Wire.available() == 1) {
      data[i] = Wire.read();
    }
  }

  *ax = (int16_t)(data[1] << 8 | data[0]) * 0.01; // m/s^2
  *ay = (int16_t)(data[3] << 8 | data[2]) * 0.01; // m/s^2
  *az = (int16_t)(data[5] << 8 | data[4]) * 0.01; // m/s^2
}

void Bno055::sen_gyro(float *gx, float *gy, float *gz) {
  uint8_t data[6];

  for(uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(I2C_ADDR_BNO055);
    Wire.write(0x14 + i); // Gyro data registers
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR_BNO055, 1);
    if (Wire.available() == 1) {
      data[i] = Wire.read();
    }
  }

  *gx = (int16_t)(data[1] << 8 | data[0]) / 16.0; // deg/s
  *gy = (int16_t)(data[3] << 8 | data[2]) / 16.0; // deg/s
  *gz = (int16_t)(data[5] << 8 | data[4]) / 16.0; // deg/s
}

void Bno055::sen_mag(float *mx,float *my,float *mz) {
  uint8_t data[6];

  for(uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(I2C_ADDR_BNO055);
    Wire.write(0x0E + i); // Magnetometer data registers
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR_BNO055, 1);
    if (Wire.available() == 1) {
      data[i] = Wire.read();
    }
  }

  *mx = (int16_t)(data[1] << 8 | data[0]) / 16.0; // uT
  *my = (int16_t)(data[3] << 8 | data[2]) / 16.0; // uT
  *mz = (int16_t)(data[5] << 8 | data[4]) / 16.0; // uT
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

void Bno055::_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(I2C_ADDR_BNO055);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t Bno055::_read_reg(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR_BNO055);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR_BNO055, 1);
  if (Wire.available() == 1) {
    return Wire.read();
  }

  return 0; // Return 0 if read failed
}
