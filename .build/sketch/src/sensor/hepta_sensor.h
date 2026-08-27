#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\sensor\\hepta_sensor.h"
/**
 * @file hepta_sensor.h
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright UNISEC all rights reserved.
 * 
 */


#ifndef HEPTA_SENSOR_H
#define HEPTA_SENSOR_H

#include <Arduino.h>

#include "../drv/adc_mcp3208.h"
#include "../drv/imu9axis_bno055.h"
#include "../drv/gps_gp1818mk.h"
#include "../drv/camera_c1098.h"


class HeptaSensor :public AdcMcp3208, public Bno055,
                     public Gps1818mk, public CameraC1098 {
  public:
    HeptaSensor();

    bool save_picture(void);

    float get_user_pin_voltage(void);
    float get_temperature(void);

  private:
    AdcMcp3208 adc;
    Bno055 bno055;
    Gps1818mk gps;
    CameraC1098 cam;

    const uint8_t _adc_cs_pin = 17;
    const uint8_t _temp_pin = 27;

    const uint8_t ADC_USER_PIN = 6;
};


#endif /* HEPTA_SENSOR_H */
