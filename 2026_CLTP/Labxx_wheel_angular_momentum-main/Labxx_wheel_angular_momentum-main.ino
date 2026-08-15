#include <Arduino.h>
#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"
#include "radio_commands.h"

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
constexpr uint32_t XBEE_BAUD_RATE = 38400;
constexpr uint8_t XBEE_RESET_PIN = 1;
constexpr uint8_t WHEEL_SDA_PIN = 6;
constexpr uint8_t WHEEL_SCL_PIN = 7;

// Experiment settings
constexpr int32_t DEFAULT_WHEEL_SPEED_RPM = 2000;
constexpr unsigned long ANGULAR_VELOCITY_INTERVAL_MS = 100;

// Devices and program state
HeptaCom com;
Bno055 sensor;
UnitRollerI2C wheel;

String radioCommandBuffer;
int32_t targetWheelSpeedRpm = DEFAULT_WHEEL_SPEED_RPM;
bool wheelIsConnected = false;
bool wheelIsRunning = false;
bool angularVelocityOutputIsEnabled = false;
unsigned long previousAngularVelocityTimeMs = 0;

// Interpret one complete command received from XBee.
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
    if (!connectWheel(wheel, wheelIsConnected, WHEEL_STARTUP_TIMEOUT_MS,
                      WHEEL_RETRY_INTERVAL_MS)) {
      sendMessage("ERR WHEEL_DISCONNECTED");
      return;
    }
    sendMessage("I2C WHEEL TRUE");
  }

  if (command == "a") {
    sendMessage(startWheel(wheel, targetWheelSpeedRpm, WHEEL_MAX_CURRENT,
                           wheelIsRunning));
  } else if (command == "s") {
    sendMessage(stopWheel(wheel, wheelIsRunning));
  } else if (command == "h" || command == "?") {
    sendMessage(sendWheelStatus(wheel, wheelIsConnected, wheelIsRunning,
                                targetWheelSpeedRpm, RPM_TO_RAD_PER_SEC));
  } else if (command.startsWith("r")) {
    sendMessage(setWheelSpeed(wheel, command.substring(1), MAX_WHEEL_SPEED_RPM,
                              targetWheelSpeedRpm, wheelIsRunning));
  } else {
    sendMessage("ERR COMMANDS: R<rpm>, A, S, H, V, P");
  }
}

void setup() {
  com.begin(XBEE_BAUD_RATE);
  sensor.begin();
  delay(500);

  wheelIsConnected = wheel.begin(&Wire1, WHEEL_SDA_PIN, WHEEL_SCL_PIN,
                                 I2C_ADDR, 100000,
                                 WHEEL_STARTUP_TIMEOUT_MS,
                                 WHEEL_RETRY_INTERVAL_MS);
  if (wheelIsConnected) {
    sendMessage("I2C WHEEL TRUE");
  } else {
    sendMessage("I2C WHEEL FALSE");
  }

  sendMessage("READY: R<rpm>, A=START, S=STOP, H=STATUS, V=DISPLAY, P=PAUSE");
}

void loop() {
  receiveRadioCommands(executeCommand);

  unsigned long currentTimeMs = millis();
  unsigned long timeSinceLastOutputMs =
      currentTimeMs - previousAngularVelocityTimeMs;
  bool isTimeToOutput =
      timeSinceLastOutputMs >= ANGULAR_VELOCITY_INTERVAL_MS;

  if (angularVelocityOutputIsEnabled && isTimeToOutput) {
    sendBodyAngularVelocity();
    previousAngularVelocityTimeMs = currentTimeMs;
  }

  delay(5);
}
