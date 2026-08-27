#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\drv\\adc_mcp3208.h"
/**
 * @file adc_mcp3208.h
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2025-06-25
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#ifndef ADC_MCP3208_H
#define ADC_MCP3208_H

#include <Arduino.h>

class AdcMcp3208 {
  public:
    /**
     * @brief Initialize the MCP3208 ADC
     * @param cs_pin Chip Select pin for SPI communication
     * @param ref_vol Reference voltage (default is 3.3V)
     */
    void begin(uint8_t cs_pin, float ref_vol = 3.3);

    /**
     * @brief Get raw data from a specific channel
     * @param channel Channel number (0-7)
     * @return Raw data as a 16-bit unsigned integer
     */
    uint16_t get_raw_data(uint8_t channel);

    /**
     * @brief Get the voltage from a specific channel
     * @param channel Channel number (0-7)
     * @return Voltage as a float
     */
    float get_voltage(uint8_t channel);

  private:
    uint8_t _cs_pin;
    float _ref_voltage;
    void spi_block_transaction(uint16_t* send_data, uint16_t* ret_data, uint8_t Byte_size);
};

#endif /* ADC_MCP_3208_H */
