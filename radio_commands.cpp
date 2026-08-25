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
extern AngularEstimation angular_estimation;
extern AdcsControl adcs_control;
extern String radioCommandBuffer;
extern bool wheelIsConnected, wheelIsRunning, is_telemetry_enabled;
extern unsigned long last_control_update_ms, last_telemetry_ms;
constexpr size_t COMMAND_BUFFER_SIZE = 64;
constexpr unsigned long COMMAND_END_TIMEOUT_MS = 100;
unsigned long lastcommandcharactertimems = 0;

void send_message(const String &message) {
  com.send_text(message);
  com.send_text("\r\n");
  Serial.println(message);
}
void normalize_command(String &command) { command.trim(); command.toLowerCase(); }
void enable_angular_velocity_output() { is_telemetry_enabled = true; send_message("OK TELEMETRY ON"); }
void disable_angular_velocity_output() { is_telemetry_enabled = false; send_message("OK TELEMETRY OFF"); }

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
      adcs_control.start(angular_estimation, wheel, wheelIsRunning, nowMs);
  last_control_update_ms = nowMs;
  if (!adcs_control.is_enabled()) {
    send_message(result);
    return;
  }
  send_message(result +
               " CONTINUOUS_CURRENT_PD=ON AUTO_UNLOAD=ON TARGET_HOLD=ON YAW_ZEROED");
}

void execute_stop_command() {
  adcs_control.stop(wheel, wheelIsRunning);

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
  adcs_control.set_target_angle_deg(value);
  send_message("OK TARGET_YAW " + String(adcs_control.target_angle_deg(), 2) + " deg");
}

void execute_set_kp_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcs_control.set_proportional_gain(value)) {
    send_message("ERR KP_RANGE: kp0..kp1000 mA/deg"); return;
  }
  send_message("OK Kp " + String(value, 3) + " mA/deg");
}

void execute_set_kd_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcs_control.set_derivative_gain(value)) {
    send_message("ERR KD_RANGE: kd0..kd1000 mA/(deg/s)"); return;
  }
  send_message("OK Kd " + String(value, 3) + " mA/(deg/s)");
}

void execute_status_command() {
  const float estimatedYawDeg = angular_estimation.yaw_deg();
  String message = "STATUS CONTROL=";
  message.reserve(240);
  message += adcs_control.is_enabled() ? "ON" : "OFF";
  message += " MODE=" + String(adcs_control.control_state_name());
  message += " YAW=" + String(estimatedYawDeg, 2) + " deg";
  message += " TARGET_YAW=" + String(adcs_control.target_angle_deg(), 2) + " deg";
  message += " ERROR=" + String(angular_estimation.error_deg(
      adcs_control.target_angle_deg()), 2) + " deg";
  message += " Kp=" + String(adcs_control.proportional_gain_ma_per_deg(), 3) + " mA/deg";
  message += " Kd=" + String(adcs_control.derivative_gain_ma_per_deg_per_sec(), 3) + " mA/(deg/s)";
  message += " WHEEL_RPM=" + String(adcs_control.wheel_speed_rpm());
  message += " CURRENT_CMD=" + String(adcs_control.current_command_ma(), 1) + " mA";
  send_message(message);
}

void send_telemetry() {
  const float estimatedYawDeg = angular_estimation.yaw_deg();
  // Keep the radio line compact. SoftwareSerial cannot receive while it is
  // transmitting, so shorter lines make commands responsive at 38400 baud.
  String message;
  message.reserve(140);
  message = "ATT ms=" + String(millis());
  message += " yaw=" + String(estimatedYawDeg, 2);
  message += " rate=" + String(angular_estimation.yaw_rate_deg_per_sec(), 2);
  message += " mode=" + String(adcs_control.control_state_name());
  message += " target=" + String(adcs_control.target_angle_deg(), 2);
  message += " error=" + String(angular_estimation.error_deg(
      adcs_control.target_angle_deg()), 2);
  message += " rpm=" + String(adcs_control.wheel_speed_rpm());
  message += " cur=" + String(adcs_control.current_command_ma(), 1);
  message += " boost=" + String(adcs_control.friction_compensation_ma(), 1);
  message += " voltage=" + String(eps.get_battery_voltage(), 2);
  send_message(message);
}

static void advance_periodic_timestamp(unsigned long now_ms,
                                       unsigned long interval_ms,
                                       unsigned long &last_update_ms) {
  const unsigned long elapsed_ms = now_ms - last_update_ms;
  // Preserve the normal time grid after a small delay. After missing multiple
  // periods, skip stale updates instead of emitting them in a burst.
  last_update_ms = elapsed_ms < 2 * interval_ms
                       ? last_update_ms + interval_ms
                       : now_ms;
}

void process_telemetry(
    unsigned long now_ms, unsigned long telemetry_interval_ms) {
  if (is_telemetry_enabled && adcs_control.is_enabled() &&
      now_ms - last_telemetry_ms >= telemetry_interval_ms) {
    send_telemetry();
    advance_periodic_timestamp(
        now_ms, telemetry_interval_ms, last_telemetry_ms);
  }
}

void send_command_error() { send_message("ERR COMMANDS: t<yaw_deg>, kp<mA/deg>, kd<mA/(deg/s)>, a, s, h, v, p"); }
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
    const char current_character = received.charAt(i);
    const bool is_gain_command =
        radioCommandBuffer.equalsIgnoreCase("k") &&
        (static_cast<char>(tolower(current_character)) == 'p' ||
         static_cast<char>(tolower(current_character)) == 'd');

    if (is_gain_command) {
      radioCommandBuffer += current_character;
      lastcommandcharactertimems = millis();
    }

    else if (current_character == '\r' || current_character == '\n') {
      if (radioCommandBuffer.length() > 0) {
        handler(radioCommandBuffer);
        radioCommandBuffer = "";
      }
    }

    else if (radioCommandBuffer.length() < COMMAND_BUFFER_SIZE - 1) {
      radioCommandBuffer += current_character;
      lastcommandcharactertimems = millis();
    }

    else {
      radioCommandBuffer = ""; send_message("ERR COMMAND_TOO_LONG");
    }
  }

  if (radioCommandBuffer.length() > 0 &&
      millis() - lastcommandcharactertimems >= COMMAND_END_TIMEOUT_MS) {
    handler(radioCommandBuffer);
    radioCommandBuffer = "";
  }
}
