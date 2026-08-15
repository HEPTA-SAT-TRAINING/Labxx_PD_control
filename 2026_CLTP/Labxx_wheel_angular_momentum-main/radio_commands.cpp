#include <Arduino.h>

#include "radio_commands.h"
#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"

extern HeptaCom com;
extern Bno055 sensor;
extern String radioCommandBuffer;

constexpr size_t COMMAND_BUFFER_SIZE = 64;

// Send a message over the XBee wireless link.
void sendMessage(const String &message) {
  com.send_text(message + "\r\n");
}

// Read the satellite-body angular velocity from the BNO055 gyroscope.
// These values are not calculated from the wheel rpm.
void sendBodyAngularVelocity() {
  float angularVelocityX;
  float angularVelocityY;
  float angularVelocityZ;
  sensor.sen_gyro(&angularVelocityX, &angularVelocityY, &angularVelocityZ);

  String message = "ANGULAR_VELOCITY x=";
  message += String(angularVelocityX, 2) + " deg/s, y=";
  message += String(angularVelocityY, 2) + " deg/s, z=";
  message += String(angularVelocityZ, 2) + " deg/s";
  sendMessage(message);
}

bool isSingleCharacterCommand(char character) {
  character = static_cast<char>(tolower(character));
  return character == 'a' || character == 's' || character == 'h' ||
         character == 'v' || character == 'p' || character == '?';
}

// One-letter radio commands run immediately. An r command is collected until
// Enter is received so that a value such as r2000 can be handled as one command.
void receiveRadioCommands(CommandHandler commandHandler) {
  String receivedText = com.get_text();

  for (unsigned int i = 0; i < receivedText.length(); ++i) {
    char character = receivedText.charAt(i);

    if (isSingleCharacterCommand(character)) {
      commandHandler(String(character));
      radioCommandBuffer = "";
    } else if (character == '\r' || character == '\n') {
      if (radioCommandBuffer.length() > 0) {
        commandHandler(radioCommandBuffer);
        radioCommandBuffer = "";
      }
    } else if (radioCommandBuffer.length() < COMMAND_BUFFER_SIZE - 1) {
      radioCommandBuffer += character;
    } else {
      radioCommandBuffer = "";
      sendMessage("ERR COMMAND_TOO_LONG");
    }
  }
}
