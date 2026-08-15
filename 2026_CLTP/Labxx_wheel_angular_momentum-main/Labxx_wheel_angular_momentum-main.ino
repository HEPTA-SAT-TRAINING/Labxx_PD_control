#include <Arduino.h>
#include <Wire.h>

#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"

/*
 * Reaction wheel control experiment
 *
 * The reaction wheel is controlled from a PC through an XBee wireless link.
 * The BNO055 gyroscope measures the angular velocity of the satellite body.
 *
 * Commands:
 *   r2000  Set the wheel speed to 2000 rpm (press Enter after this command)
 *   a      Start the wheel
 *   s      Stop the wheel
 *   h      Show the wheel status
 *   v      Start continuous angular-velocity output
 *   p      Pause continuous angular-velocity output
 */

// Hardware settings
constexpr uint32_t USB_BAUD_RATE = 9600;
constexpr uint32_t XBEE_BAUD_RATE = 38400;
constexpr uint8_t XBEE_RESET_PIN = 1;
constexpr uint8_t WHEEL_SDA_PIN = 6;
constexpr uint8_t WHEEL_SCL_PIN = 7;
constexpr uint8_t WHEEL_I2C_ADDRESS = 0x64;
constexpr uint32_t WHEEL_I2C_CLOCK_HZ = 100000;

// Experiment settings
constexpr int32_t DEFAULT_WHEEL_SPEED_RPM = 2000;
constexpr int32_t MAX_WHEEL_SPEED_RPM = 3000;
constexpr int32_t WHEEL_SPEED_REGISTER_SCALE = 100;  // 1 rpm = 100 register units
constexpr int32_t WHEEL_MAX_CURRENT = 100000;        // Register unit: 0.01 mA
constexpr float RPM_TO_RAD_PER_SEC = TWO_PI / 60.0f;
constexpr unsigned long ANGULAR_VELOCITY_INTERVAL_MS = 100;
constexpr unsigned long WHEEL_STARTUP_TIMEOUT_MS = 5000;
constexpr unsigned long WHEEL_RETRY_INTERVAL_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 64;

// Devices and program state
HeptaCom radio;
Bno055 imu;
UnitRollerI2C wheel;

String radioCommandBuffer;
String usbCommandBuffer;
int32_t targetWheelSpeedRpm = DEFAULT_WHEEL_SPEED_RPM;
bool wheelIsConnected = false;
bool wheelIsRunning = false;
bool angularVelocityOutputIsEnabled = true;
unsigned long previousAngularVelocityTimeMs = 0;

// Send the same message to both the wireless link and USB.
void sendMessage(const String &message) {
  radio.send_text(message + "\r\n");
  Serial.println(message);
}

// Initialize I2C in the same way as the known-good wheel-check sketch.
void initializeWheelBus() {
  Wire1.setSDA(WHEEL_SDA_PIN);
  Wire1.setSCL(WHEEL_SCL_PIN);
  Wire1.begin();
}

bool initializeWheel() {
  return wheel.begin(&Wire1, WHEEL_I2C_ADDRESS, WHEEL_I2C_CLOCK_HZ);
}

void configureWheel();

// The wheel controller can become ready later than the RP2040. Retry instead
// of permanently marking it disconnected after a single power-on probe.
bool connectWheel(unsigned long timeoutMs) {
  const unsigned long startTimeMs = millis();

  do {
    if (initializeWheel()) {
      wheelIsConnected = true;
      configureWheel();
      return true;
    }
    delay(WHEEL_RETRY_INTERVAL_MS);
  } while (millis() - startTimeMs < timeoutMs);

  wheelIsConnected = false;
  return false;
}

// Put the wheel in a safe state after power-on.
void configureWheel() {
  wheel.setOutput(0);
  wheel.setMode(ROLLER_MODE_SPEED);
  wheel.setSpeed(0);
  wheel.setSpeedMaxCurrent(WHEEL_MAX_CURRENT);
}

void startWheel() {
  if (targetWheelSpeedRpm == 0) {
    sendMessage("ERR SET_SPEED_FIRST");
    return;
  }

  wheel.setMode(ROLLER_MODE_SPEED);
  wheel.setSpeed(targetWheelSpeedRpm * WHEEL_SPEED_REGISTER_SCALE);
  wheel.setOutput(1);
  wheelIsRunning = true;
  sendMessage("OK START " + String(targetWheelSpeedRpm) + " rpm");
}

void stopWheel() {
  wheel.setSpeed(0);
  wheel.setOutput(0);
  wheelIsRunning = false;
  sendMessage("OK STOP");
}

// Set a new target speed. A negative value rotates the wheel in reverse.
void setWheelSpeed(const String &argument) {
  char *endPointer = nullptr;
  long requestedRpm = strtol(argument.c_str(), &endPointer, 10);

  if (argument.length() == 0 || *endPointer != '\0') {
    sendMessage("ERR USAGE R<rpm>");
    return;
  }
  if (requestedRpm < -MAX_WHEEL_SPEED_RPM ||
      requestedRpm > MAX_WHEEL_SPEED_RPM) {
    sendMessage("ERR SPEED_RANGE -3000..3000 rpm");
    return;
  }

  targetWheelSpeedRpm = static_cast<int32_t>(requestedRpm);

  // Apply a new speed immediately when the wheel is already running.
  if (wheelIsRunning) {
    if (targetWheelSpeedRpm == 0) {
      stopWheel();
      return;
    }
    wheel.setSpeed(targetWheelSpeedRpm * WHEEL_SPEED_REGISTER_SCALE);
  }

  sendMessage("OK SPEED " + String(targetWheelSpeedRpm) + " rpm");
}

void sendWheelStatus() {
  String message = "STATUS ";
  message += wheelIsRunning ? "RUNNING" : "STOPPED";
  message += " TARGET=" + String(targetWheelSpeedRpm) + " rpm";
  message += " TARGET_OMEGA=";
  message += String(targetWheelSpeedRpm * RPM_TO_RAD_PER_SEC, 2);
  message += " rad/s";

  if (wheelIsConnected) {
    float actualRpm = wheel.getSpeedReadback() /
                      static_cast<float>(WHEEL_SPEED_REGISTER_SCALE);
    message += " ACTUAL=" + String(actualRpm, 2) + " rpm";
    message += " ACTUAL_OMEGA=";
    message += String(actualRpm * RPM_TO_RAD_PER_SEC, 2);
    message += " rad/s";
  } else {
    message += " WHEEL=DISCONNECTED";
  }
  sendMessage(message);
}

// Read the satellite-body angular velocity from the BNO055 gyroscope.
// These values are not calculated from the wheel rpm.
void sendBodyAngularVelocity() {
  float angularVelocityX;
  float angularVelocityY;
  float angularVelocityZ;
  imu.sen_gyro(&angularVelocityX, &angularVelocityY, &angularVelocityZ);

  String message = "ANGULAR_VELOCITY x=";
  message += String(angularVelocityX, 2) + " deg/s, y=";
  message += String(angularVelocityY, 2) + " deg/s, z=";
  message += String(angularVelocityZ, 2) + " deg/s";
  sendMessage(message);
}

// Interpret one complete command received from USB or XBee.
void executeCommand(String command) {
  command.trim();
  command.toLowerCase();
  if (command.length() == 0) return;

  // Sensor-output commands work even when the wheel is disconnected.
  if (command == "v") {
    angularVelocityOutputIsEnabled = true;
    sendMessage("OK ANGULAR_VELOCITY_OUTPUT ON");
    return;
  }
  if (command == "p") {
    angularVelocityOutputIsEnabled = false;
    sendMessage("OK ANGULAR_VELOCITY_OUTPUT OFF");
    return;
  }

  if (!wheelIsConnected) {
    sendMessage("INFO WHEEL_RECONNECTING");
    if (!connectWheel(WHEEL_STARTUP_TIMEOUT_MS)) {
      sendMessage("ERR WHEEL_DISCONNECTED");
      return;
    }
    sendMessage("I2C WHEEL TRUE");
  }

  if (command == "a") {
    startWheel();
  } else if (command == "s") {
    stopWheel();
  } else if (command == "h" || command == "?") {
    sendWheelStatus();
  } else if (command.startsWith("r")) {
    setWheelSpeed(command.substring(1));
  } else {
    sendMessage("ERR COMMANDS: R<rpm>, A, S, H, V, P");
  }
}

bool isSingleCharacterCommand(char character) {
  character = static_cast<char>(tolower(character));
  return character == 'a' || character == 's' || character == 'h' ||
         character == 'v' || character == 'p' || character == '?';
}

// One-letter radio commands run immediately. An r command is collected until
// Enter is received so that a value such as r2000 can be handled as one command.
void receiveRadioCommands() {
  String receivedText = radio.get_text();

  for (unsigned int i = 0; i < receivedText.length(); ++i) {
    char character = receivedText.charAt(i);

    if (isSingleCharacterCommand(character)) {
      executeCommand(String(character));
      radioCommandBuffer = "";
    } else if (character == '\r' || character == '\n') {
      if (radioCommandBuffer.length() > 0) {
        executeCommand(radioCommandBuffer);
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

// USB commands are collected until Enter is pressed.
void receiveUsbCommands() {
  while (Serial.available() > 0) {
    char character = static_cast<char>(Serial.read());

    if (character == '\r' || character == '\n') {
      if (usbCommandBuffer.length() > 0) {
        executeCommand(usbCommandBuffer);
        usbCommandBuffer = "";
      }
    } else if (usbCommandBuffer.length() < COMMAND_BUFFER_SIZE - 1) {
      usbCommandBuffer += character;
    } else {
      usbCommandBuffer = "";
      sendMessage("ERR COMMAND_TOO_LONG");
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD_RATE);

  // Release XBee reset, then open the wireless serial port.
  pinMode(XBEE_RESET_PIN, OUTPUT);
  digitalWrite(XBEE_RESET_PIN, HIGH);
  radio.begin(XBEE_BAUD_RATE);

  // BNO055 uses Wire; the wheel uses the separate Wire1 bus.
  imu.begin();
  delay(500);

  initializeWheelBus();
  if (connectWheel(WHEEL_STARTUP_TIMEOUT_MS)) {
    sendMessage("I2C WHEEL TRUE");
  } else {
    sendMessage("I2C WHEEL FALSE");
  }

  sendMessage("READY: R<rpm>, A=START, S=STOP, H=STATUS, V=DISPLAY, P=PAUSE");
}

void loop() {
  receiveUsbCommands();
  receiveRadioCommands();

  unsigned long currentTimeMs = millis();
  bool outputIntervalHasElapsed =
      currentTimeMs - previousAngularVelocityTimeMs >=
      ANGULAR_VELOCITY_INTERVAL_MS;

  if (angularVelocityOutputIsEnabled && outputIntervalHasElapsed) {
    sendBodyAngularVelocity();
    previousAngularVelocityTimeMs = currentTimeMs;
  }

  delay(5);
}
