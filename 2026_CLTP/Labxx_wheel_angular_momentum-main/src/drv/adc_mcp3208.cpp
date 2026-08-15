/**
 * @file adc_mcp3208.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2025-06-25
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#include "adc_mcp3208.h"

#include <SPI.h>

#define START_BIT 0x04
#define MODE_SINGLE 0x02

void AdcMcp3208::begin(uint8_t cs_pin, float ref_vol) {
  _cs_pin = cs_pin;
  _ref_voltage = ref_vol;
  pinMode(_cs_pin, OUTPUT);
}

uint16_t AdcMcp3208::get_raw_data(uint8_t channel) {
  if (channel > 7) {
    Serial.println("Error: Channel must be between 0 and 7.");
    return 0;
  }

  uint8_t command_high = START_BIT | MODE_SINGLE | ((channel & 0x04) >> 2);
  uint8_t command_low = (channel & 0x03) << 6;
  
  uint16_t send_data[3] = {command_high, command_low, 0x00};
  uint16_t ret_data[3] = {0x00};
  spi_block_transaction(send_data, ret_data, 3);

  uint8_t high_byte = ret_data[1] & 0x0F;
  uint8_t low_byte = ret_data[2];
  uint16_t data = (high_byte << 8) | low_byte;

  return data;
}

float AdcMcp3208::get_voltage(uint8_t channel) {
  uint16_t raw_data = get_raw_data(channel);
  return (raw_data * _ref_voltage) / 4096.0; // Convert to voltage
}

void AdcMcp3208::spi_block_transaction(uint16_t* send_data, uint16_t* ret_data, uint8_t Byte_size){
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);

  for(uint8_t i=0; i<Byte_size; i++){
    ret_data[i] = SPI.transfer(send_data[i]);
  }

  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}
