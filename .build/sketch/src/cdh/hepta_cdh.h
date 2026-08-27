#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\cdh\\hepta_cdh.h"
/**
 * @file hepta_cdh.h
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */

#ifndef HEPTA_CDH_H
#define HEPTA_CDH_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

typedef uint8_t cmd_t;
typedef uint64_t cmd_arg_t; // not yet used, but reserved for future use


class HeptaCdh :public SDClass {
  public:
    void begin(void);
    cmd_t get_command(void);
    bool command_execute(cmd_t cmd, cmd_arg_t arg = 0);

  private:
    const uint8_t _sd_cs_pin = 3;
};


#endif /* HEPTA_CDH_H */
