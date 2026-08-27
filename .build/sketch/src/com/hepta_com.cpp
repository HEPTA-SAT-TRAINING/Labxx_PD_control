#line 1 "C:\\HEPTA\\2026_CLTP\\Labxx_PD_control-main\\src\\com\\hepta_com.cpp"
#include "hepta_com.h"


HeptaCom::HeptaCom()
    : XbeeSerial(_rx_pin, _tx_pin) {
}

void HeptaCom::begin(uint16_t baud_rate) {
  XbeeSerial.begin(baud_rate);
}

char HeptaCom::get_char(void) {
  return XbeeSerial.read();
}

void HeptaCom::send_char(const char c) {
  XbeeSerial.write(c);
}

void HeptaCom::send_text(const String &text) {
  XbeeSerial.print(text);
}

String HeptaCom::get_text(void) {
  String received_text = "";

  // Check if data is available to read
  while (XbeeSerial.available()) {
    char c = XbeeSerial.read();
    received_text += c; // Append the character to the string
  }

  return received_text; // Return the complete string
}
