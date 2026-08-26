#include <Arduino.h>
#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"
#include "src/eps/hepta_eps.h"
#include "angular_estimation.h"
#include "adcs_conrol.h"
#include "radio_commands.h"

/* Z-axis (yaw) reaction-wheel continuous-current PD control.
 * The PD output directly commands motor current with current and slew limits,
 * while wheel speed is monitored for automatic unloading.
 * t90: target yaw 90 deg, kp5: proportional gain in mA/deg,
 * kd1: derivative gain in mA/(deg/s), a: start/reset yaw,
 * S: stop, H: status, V/P: telemetry on/off.
 */
constexpr uint32_t XBEE_BAUD_RATE = 38400;
constexpr uint32_t SERIAL_MONITOR_BAUD_RATE = 38400;
constexpr uint8_t WHEEL_SDA_PIN = 6;
constexpr uint8_t WHEEL_SCL_PIN = 7;
constexpr unsigned long CONTROL_INTERVAL_MS = 10;
// SoftwareSerial at 38400 baud can lose commands and build a transmit backlog
// during long, frequent messages. Keep all monitoring data in one compact line.
constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;

HeptaCom com;
HeptaEps eps;
Bno055 sensor;
UnitRollerI2C wheel;
AngularEstimation angular_estimation;
AdcsControl adcs_control;

String radioCommandBuffer;
bool wheelIsConnected = false;
bool wheelIsRunning = false;
bool is_telemetry_enabled = false;
unsigned long last_control_update_ms = 0;
unsigned long last_telemetry_ms = 0;
// Gyro bias calibration is intentionally RAM-only. Power cycling clears it.
float gyro_bias_z_deg_per_sec = 0.0f;
float measured_gyro_bias_z_deg_per_sec = 0.0f;
bool measured_gyro_bias_is_valid = false;
bool gyro_bias_correction_enabled = false;
bool magnetic_calibration_active = false;
bool magnetic_calibration_enabled = false;
unsigned long magnetic_calibration_started_ms = 0;
unsigned long magnetic_calibration_previous_sample_ms = 0;
unsigned long magnetic_calibration_previous_report_ms = 0;
size_t magnetic_calibration_sample_count = 0;
float magnetic_calibration_min_x_ut = 0.0f;
float magnetic_calibration_max_x_ut = 0.0f;
float magnetic_calibration_min_y_ut = 0.0f;
float magnetic_calibration_max_y_ut = 0.0f;
float magnetic_offset_x_ut = 0.0f;
float magnetic_offset_y_ut = 0.0f;
float magnetic_scale_x = 1.0f;
float magnetic_scale_y = 1.0f;

void execute_command(String command) {
  normalize_command(command);
  if (command.length() == 0) return;
  if (command == "v") enable_angular_velocity_output();
  else if (command == "p") disable_angular_velocity_output();
  else if (command.startsWith("t")) execute_set_target_angle_command(command.substring(1));
  else if (command.startsWith("kp")) execute_set_kp_command(command.substring(2));
  else if (command.startsWith("kd")) execute_set_kd_command(command.substring(2));
  else if (command == "biascal") execute_gyro_bias_calibration_command();
  else if (command == "biassave") execute_gyro_bias_save_command();
  else if (command == "magcal") execute_magnetic_calibration_command();
  else if (command == "magcal?") execute_magnetic_calibration_status_command();
  else if (command == "h" || command == "?") execute_status_command();
  else if (!ensure_wheel_is_connected()) return;
  else if (command == "a") execute_start_command();
  else if (command == "s") execute_stop_command();
  else send_command_error();
}

void update_attitude_control(unsigned long now_ms) {
  angular_estimation.update(sensor, now_ms);

  const float estimated_yaw_deg = angular_estimation.yaw_deg();
  const float yaw_rate_deg_per_sec = angular_estimation.yaw_rate_deg_per_sec();
  const float angle_error_deg =angular_estimation.error_deg(adcs_control.target_angle_deg());

  const float body_pd_command = adcs_control.calculate_body_pd_command(angle_error_deg, yaw_rate_deg_per_sec);

  adcs_control.update_control_state(angle_error_deg, yaw_rate_deg_per_sec, body_pd_command, now_ms);
  adcs_control.command_reaction_wheel(wheel, now_ms);

  last_control_update_ms = now_ms;
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD_RATE);
  com.begin(XBEE_BAUD_RATE);
  sensor.begin();
  eps.init();
  delay(500);
  wheelIsConnected = wheel.begin(WHEEL_SDA_PIN, WHEEL_SCL_PIN);
  send_message(wheelIsConnected ? "I2C WHEEL TRUE" : "I2C WHEEL FALSE");
  send_message("READY: t<yaw_deg>, kp<mA/deg>, kd<mA/(deg/s)>, biascal, biassave, magcal, magcal?, a=START, s=STOP, h=STATUS, v/p=TELEMETRY");
}

void loop() {
  receive_radio_commands(execute_command);
  const unsigned long now_ms = millis();
  process_magnetic_calibration(now_ms);

  // 姿勢制御はテレメトリ表示とは独立した周期で実行する。
  if (adcs_control.is_enabled() && now_ms - last_control_update_ms >= CONTROL_INTERVAL_MS) {
    update_attitude_control(now_ms);
  }

  // 姿勢情報、制御誤差、ホイール情報、電圧を同じ周期で表示する。
  process_telemetry(now_ms, TELEMETRY_INTERVAL_MS);
  delay(1);
}
