/**
 * @file hepta_eps.cpp
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */


#include "hepta_eps.h"

// HeptaEps::HeptaEps() {
//   adc.begin(_adc_cs_pin); // Initialize ADC with CS pin 17
//   pinMode(_sw_3v3_pin, OUTPUT);
//   digitalWrite(_sw_3v3_pin, LOW); // Ensure 3.3V switch is off initially
//   pinMode(_bat_vol_pin, INPUT);
// }

void HeptaEps::init(void) {
  adc.begin(_adc_cs_pin); // Initialize ADC with CS pin 17
  pinMode(_sw_3v3_pin, OUTPUT);
  digitalWrite(_sw_3v3_pin, LOW); // Ensure 3.3V switch is off initially
  pinMode(_bat_vol_pin, INPUT);
}

void HeptaEps::switch_3V3_on(void) {
  digitalWrite(_sw_3v3_pin, HIGH);
}

void HeptaEps::switch_3V3_off(void) {
  digitalWrite(_sw_3v3_pin, LOW);
}

float HeptaEps::get_battery_voltage(void) {
  float voltage = analogRead(_bat_vol_pin) * (_adc_ref_voltage / _adc_max_value);
  return voltage;
}

float HeptaEps::get_5v_voltage(void) {
  const float resistor_1 = 10000.0;
  const float resistor_2 = 15000.0;

  return adc.get_voltage(ADC_5V_VOLTAGE) * ((resistor_1 + resistor_2) / resistor_2);
}

float HeptaEps::get_3v3_voltage(void) {
  const float resistor_1 = 10000.0;
  const float resistor_2 = 100000.0;

  return adc.get_voltage(ADC_3V3_VOLTAGE) * ((resistor_1 + resistor_2) / resistor_2);
}

float HeptaEps::get_current_discharge(void) {
  return (adc.get_voltage(ADC_CURRENT_DISCHARGE) / galvano_gain) / galvano_resistance;
}

float HeptaEps::get_current_charge(void) {
  return (adc.get_voltage(ADC_CURRENT_CHARGE) / galvano_gain) / galvano_resistance;
}
