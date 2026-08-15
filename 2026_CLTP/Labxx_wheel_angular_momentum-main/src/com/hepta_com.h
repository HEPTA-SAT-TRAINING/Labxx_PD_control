/**
 * @file hepta_com.h
 * @author Masaki Naito
 * @brief
 * @version 0.1
 * @date 2024-12-08
 * 
 * @copyright UNISEC all rights reserved.
 */

#ifndef HEPTA_COM_H
#define HEPTA_COM_H

#include <Arduino.h>

#include <SoftwareSerial.h>


class HeptaCom {
  public:
    HeptaCom();

    void begin(uint16_t baud_rate);
    char get_char(void);
    void send_char(const char c);

    String get_text(void);
    void send_text(const String text);

  private:
    const uint8_t _rx_pin = 15; // HEPTA-SAT XBee RX
    const uint8_t _tx_pin = 14; // HEPTA-SAT XBee TX

    SoftwareSerial XbeeSerial;
};

#endif /* HEPTA_COM_H */
