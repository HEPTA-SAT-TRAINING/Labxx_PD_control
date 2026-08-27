#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\sensor\\hepta_sensor.cpp"
/**
 * @file hepta_sensor.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#include "hepta_sensor.h"

#include <SD.h>
#include <SPI.h>

HeptaSensor::HeptaSensor() {
  adc.begin(_adc_cs_pin);
  analogReadResolution(12);
  pinMode(_temp_pin, INPUT);
}

bool HeptaSensor::save_picture(void) {
  if (cam.begin(C1098_BAUD_RATE_115200, C1098_JPEG_SIZE_VGA)) {
    uint32_t data_len = cam.take_picture();
    if (data_len > 0) {
      File file = SD.open("picture.jpg", FILE_WRITE);
      if (file) {
        uint8_t buf[512];
        uint32_t total_written = 0;
        while (true) {
          int read_size = cam.get_image_data_packet(buf, sizeof(buf));
          if (read_size <= 0) break;
          file.write(buf, read_size);
          total_written += read_size;
        }
        file.close();
        Serial.print("Picture saved successfully. Total bytes: ");
        Serial.println(total_written);
        return true;
      } else {
        Serial.println("Failed to open file for writing.");
      }
    } else {
      Serial.println("No picture data available.");
    }
    return true;
  }
  return false;
}

float HeptaSensor::get_temperature(void) {
  //resistance
  const float R1 = 2500.0;
  const float R2 = 2500.0;
  const float R3 = 110.0;
  const float R4 = 1000.0;
  const float R5 = 68000.0;
  const float Pt = 100.0;
  const float R_1 = 3.0;
  const float R_2 = 2.0;

  //current
  const float I = 0.001;

  //voltage
  const float Vref = 2.5;

  //temperature coefficient
  const float ce = 0.003851;

  float gain = -R5 * I / R4;
  float offset = Vref + I * R3;

  uint16_t adc_volt = analogRead(_temp_pin) * 3.3 / 4096.0;
  float raw_volt = adc_volt * 3.3 * (R_1 + R_2) / R_1;
  float Rth = (raw_volt - offset) / gain + R3;
  float temp = (Rth - Pt) / (ce * Pt);

  return temp;
}
