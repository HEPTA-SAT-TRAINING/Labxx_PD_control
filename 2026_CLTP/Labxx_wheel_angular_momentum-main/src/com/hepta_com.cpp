#include "hepta_com.h"


HeptaCom::HeptaCom()
    : XbeeSerial(_rx_pin, _tx_pin) { // Initialize SoftwareSerial with RX and TX pins
}

void HeptaCom::begin(uint16_t baud_rate) {
  // Start the SoftwareSerial communication at the specified baud rate
  XbeeSerial.begin(baud_rate);
}

char HeptaCom::get_char(void) {
  return XbeeSerial.read();
}

void HeptaCom::send_char(const char c) {
  XbeeSerial.write(c);
}

void HeptaCom::send_text(String text) {
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
