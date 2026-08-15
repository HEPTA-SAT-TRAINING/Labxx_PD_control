/**
 * @file gps_gp1818mk.h
 * @author Masaki Naito
 * @brief 
 * @version 0.1
 * @date 2025-02-26
 * 
 * 
 */


#ifndef GPS_1818MK
#define GPS_1818MK

#include <Arduino.h>
#include <SoftwareSerial.h>

class Gps1818mk {
  public:
    void begin(void);
    void read_raw(void);
    bool get_position(float* lat, float* lon, float* alt);
    bool get_velocity(float* velocity, float* heading);
    bool get_all(float* lat, float* lon, float* alt, float* velocity, float* heading);
    bool is_data_available(void);
    void test_gps(void);

  private:
    const pin_size_t _rx_pin = 13; // RX pin for communication
    const pin_size_t _tx_pin = -1; // TX pin for communication (not used)
    SoftwareSerial* _serial = nullptr; // SoftwareSerialインスタンスへのポインタ
    int read_byte(void); // 返り値型をintに修正
    bool wait_serial(void);
    bool get_header(char array[]);
};

#endif /* GPS_1818MK */
