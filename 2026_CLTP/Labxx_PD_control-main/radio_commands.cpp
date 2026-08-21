#include <Arduino.h>
#include "angular_estimation.h"
#include "adcs_conrol.h"
#include "radio_commands.h"
#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"
#include "src/eps/hepta_eps.h"

extern HeptaCom com;
extern HeptaEps eps;
extern Bno055 sensor;
extern UnitRollerI2C wheel;
extern AngularEstimation angularEstimation;
extern AdcsControl adcsControl;
extern String radioCommandBuffer;
extern bool wheelIsConnected, wheelIsRunning, angularVelocityOutputIsEnabled;
extern unsigned long lastControlUpdateMs;
constexpr size_t COMMAND_BUFFER_SIZE = 64;
constexpr unsigned long COMMAND_IDLE_TIMEOUT_MS = 50;
unsigned long lastCommandCharacterTimeMs = 0;

void send_message(const String &message) {
  com.send_text(message);
  com.send_text("\r\n");
  Serial.println(message);
}
void normalize_command(String &command) { command.trim(); command.toLowerCase(); }
void enable_angular_velocity_output() { angularVelocityOutputIsEnabled = true; send_message("OK TELEMETRY ON"); }
void disable_angular_velocity_output() { angularVelocityOutputIsEnabled = false; send_message("OK TELEMETRY OFF"); }

bool ensure_wheel_is_connected() {
  if (wheelIsConnected) return true;
  send_message("INFO WHEEL_RECONNECTING");
  if (!connect_wheel(wheel, wheelIsConnected, WHEEL_STARTUP_TIMEOUT_MS, WHEEL_RETRY_INTERVAL_MS)) {
    send_message("ERR WHEEL_DISCONNECTED"); return false;
  }
  send_message("I2C WHEEL TRUE"); return true;
}

static bool parse_float(const String &text, float &value) {
  if (text.length() == 0) return false;
  char *endPointer = nullptr;
  value = strtof(text.c_str(), &endPointer);
  return endPointer != text.c_str() && *endPointer == '\0' && isfinite(value);
}

void execute_start_command() {
  const unsigned long nowMs = millis();
  const String result =
      adcsControl.start(angularEstimation, wheel, wheelIsRunning, nowMs);
  lastControlUpdateMs = nowMs;
  if (!adcsControl.is_enabled()) {
    send_message(result);
    return;
  }
  send_message(result +
               " VARIABLE_STEP_PD=ON AUTO_UNLOAD=ON TARGET_HOLD=ON YAW_ZEROED");
}

void execute_stop_command() {
  adcsControl.stop(wheel, wheelIsRunning);

  // A stop command must not report success after a lost I2C write. Reconnect
  // forces the Roller's tested configure sequence (output off, speed zero),
  // then send stop once more before acknowledging the command.
  wheelIsConnected = wheel.connect(1000, 100);
  if (!wheelIsConnected) {
    send_message("ERR STOP_UNCONFIRMED WHEEL_DISCONNECTED");
    return;
  }
  wheel.stop();
  wheelIsConnected = wheel.isConnected();
  if (!wheelIsConnected) {
    send_message("ERR STOP_UNCONFIRMED I2C_WRITE_FAILED");
    return;
  }
  send_message("OK STOP CONFIRMED");
}

void execute_set_target_angle_command(const String &text) {
  float value;
  if (!parse_float(text, value)) { send_message("ERR USAGE t<yaw_deg>"); return; }
  adcsControl.set_target_angle_deg(value);
  send_message("OK TARGET_YAW " + String(adcsControl.target_angle_deg(), 2) + " deg");
}

void execute_set_kp_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcsControl.set_proportional_gain(value)) {
    send_message("ERR KP_RANGE: kp0..kp1000 decision/deg"); return;
  }
  send_message("OK Kp " + String(value, 3) + " decision/deg");
}

void execute_set_kd_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcsControl.set_derivative_gain(value)) {
    send_message("ERR KD_RANGE: kd0..kd1000 decision/(deg/s)"); return;
  }
  send_message("OK Kd " + String(value, 3) + " decision/(deg/s)");
}

void execute_status_command() {
  const float estimatedYawDeg = angularEstimation.yaw_deg();
  String message = "STATUS CONTROL=";
  message.reserve(240);
  message += adcsControl.is_enabled() ? "ON" : "OFF";
  message += " MODE=" + String(adcsControl.control_state_name());
  message += " YAW=" + String(estimatedYawDeg, 2) + " deg";
  message += " TARGET_YAW=" + String(adcsControl.target_angle_deg(), 2) + " deg";
  message += " ERROR=" + String(adcsControl.error_deg(estimatedYawDeg), 2) + " deg";
  message += " Kp=" + String(adcsControl.proportional_gain_ma_per_deg(), 3) + " decision/deg";
  message += " Kd=" + String(adcsControl.derivative_gain_ma_per_deg_per_sec(), 3) + " decision/(deg/s)";
  message += " WHEEL_RPM=" + String(adcsControl.wheel_speed_rpm());
  message += " SPEED_CMD=" + String(adcsControl.wheel_speed_command_rpm(), 1) + " rpm";
  message += " SENT=" + String(adcsControl.last_sent_wheel_speed_rpm()) + " rpm";
  send_message(message);
}

void update_attitude_control(unsigned long nowMs) {
  adcsControl.update(angularEstimation, sensor, wheel, nowMs);
  lastControlUpdateMs = nowMs;
}

void send_attitude_telemetry() {
  const float estimatedYawDeg = angularEstimation.yaw_deg();
  // Keep the radio line compact. SoftwareSerial cannot receive while it is
  // transmitting, so shorter lines make commands responsive at 38400 baud.
  String message;
  message.reserve(120);
  message = "ATT ms=" + String(millis());
  message += " yaw=" + String(estimatedYawDeg, 2);
  message += " mode=" + String(adcsControl.control_state_name());
  message += " target=" + String(adcsControl.target_angle_deg(), 2);
  message += " error=" + String(adcsControl.error_deg(estimatedYawDeg), 2);
  message += " rpm=" + String(adcsControl.wheel_speed_rpm());
  message += " cmd=" + String(adcsControl.wheel_speed_command_rpm(), 1);
  message += " sent=" + String(adcsControl.last_sent_wheel_speed_rpm());
  send_message(message);
}

void send_command_error() { send_message("ERR COMMANDS: t<yaw_deg>, kp<decision/deg>, kd<decision/(deg/s)>, a, s, h, v, p"); }
static bool is_single_character_command(char c) {
  c = static_cast<char>(tolower(c));
  return c == 'a' || c == 's' || c == 'h' || c == 'v' || c == 'p' || c == '?';
}
void receive_radio_commands(CommandHandler handler) {
  String received = com.get_text();
  while (Serial.available() > 0) {
    received += static_cast<char>(Serial.read());
  }

  for (unsigned int i = 0; i < received.length(); ++i) {
    const char c = received.charAt(i);

    // The p in "kp" is part of the gain command, not the telemetry-pause
    // command. All other one-letter commands execute immediately.
    const bool isGainPrefix =
        radioCommandBuffer.equalsIgnoreCase("k") &&
        (static_cast<char>(tolower(c)) == 'p' ||
         static_cast<char>(tolower(c)) == 'd');

    if (isGainPrefix) {
      radioCommandBuffer += c;
      lastCommandCharacterTimeMs = millis();
    } else if (is_single_character_command(c)) {
      // Complete a pending value command before a following one-letter
      // command. This supports terminals configured with no line ending.
      if (radioCommandBuffer.length() > 0) {
        handler(radioCommandBuffer);
        radioCommandBuffer = "";
      }
      handler(String(c));
    } else if (c == '\r' || c == '\n') {
      if (radioCommandBuffer.length() > 0) { handler(radioCommandBuffer); radioCommandBuffer = ""; }
    } else if (radioCommandBuffer.length() < COMMAND_BUFFER_SIZE - 1) {
      radioCommandBuffer += c;
      lastCommandCharacterTimeMs = millis();
    }
    else { radioCommandBuffer = ""; send_message("ERR COMMAND_TOO_LONG"); }
  }

  // Execute gain and target commands even when the terminal sends no CR/LF.
  if (radioCommandBuffer.length() > 0 &&
      millis() - lastCommandCharacterTimeMs >= COMMAND_IDLE_TIMEOUT_MS) {
    handler(radioCommandBuffer);
    radioCommandBuffer = "";
  }
}
