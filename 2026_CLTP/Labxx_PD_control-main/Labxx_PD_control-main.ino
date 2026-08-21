#include <Arduino.h>
#include "src/com/hepta_com.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"
#include "src/eps/hepta_eps.h"
#include "angular_estimation.h"
#include "adcs_conrol.h"
#include "radio_commands.h"

/* Z-axis (yaw) reaction-wheel variable-step speed-PD control.
 * The PD decision changes wheel speed by 5, 20, or 60 rpm every 250 ms,
 * with a hard limit of +/-600 rpm and automatic friction-assisted unloading.
 * t90: target yaw 90 deg, kp5: proportional decision gain,
 * kd1: derivative decision gain, a: start/reset yaw,
 * S: stop, H: status, V/P: telemetry on/off.
 */
constexpr uint32_t XBEE_BAUD_RATE = 38400;
constexpr uint32_t SERIAL_MONITOR_BAUD_RATE = 38400;
constexpr uint8_t WHEEL_SDA_PIN = 6;
constexpr uint8_t WHEEL_SCL_PIN = 7;
constexpr unsigned long CONTROL_INTERVAL_MS = 10;
// SoftwareSerial at 38400 baud can lose commands and build a transmit backlog
// during long, frequent messages. One update per second keeps it responsive.
constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;
constexpr unsigned long SERIAL_MONITOR_INTERVAL_MS = 500;

HeptaCom com;
HeptaEps eps;
Bno055 sensor;
UnitRollerI2C wheel;
AngularEstimation angularEstimation;
AdcsControl adcsControl;

String radioCommandBuffer;
bool wheelIsConnected = false;
bool wheelIsRunning = false;
bool angularVelocityOutputIsEnabled = false;
unsigned long lastControlUpdateMs = 0;
unsigned long previousTelemetryTimeMs = 0;
unsigned long previousSerialMonitorTimeMs = 0;

void execute_command(String command) {
  normalize_command(command);
  if (command.length() == 0) return;
  if (command == "v") enable_angular_velocity_output();
  else if (command == "p") disable_angular_velocity_output();
  else if (command.startsWith("t")) execute_set_target_angle_command(command.substring(1));
  else if (command.startsWith("kp")) execute_set_kp_command(command.substring(2));
  else if (command.startsWith("kd")) execute_set_kd_command(command.substring(2));
  else if (command == "h" || command == "?") execute_status_command();
  else if (!ensure_wheel_is_connected()) return;
  else if (command == "a") execute_start_command();
  else if (command == "s") execute_stop_command();
  else send_command_error();
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD_RATE);
  com.begin(XBEE_BAUD_RATE);
  sensor.begin();
  eps.init();
  delay(500);
  wheelIsConnected = wheel.begin(WHEEL_SDA_PIN, WHEEL_SCL_PIN);
  send_message(wheelIsConnected ? "I2C WHEEL TRUE" : "I2C WHEEL FALSE");
  send_message("READY: t<yaw_deg>, kp<decision/deg>, kd<decision/(deg/s)>, a=START, s=STOP, h=STATUS, v/p=TELEMETRY");
}

void loop() {
  receive_radio_commands(execute_command);
  const unsigned long nowMs = millis();

  if (adcsControl.is_enabled() && nowMs - lastControlUpdateMs >= CONTROL_INTERVAL_MS)
    update_attitude_control(nowMs);
  if (angularVelocityOutputIsEnabled && adcsControl.is_enabled() &&
      nowMs - previousTelemetryTimeMs >= TELEMETRY_INTERVAL_MS) {
    send_attitude_telemetry();
    const unsigned long telemetryLagMs = nowMs - previousTelemetryTimeMs;
    // Keep the next display on the original time grid. If a blocking event
    // missed multiple periods, skip old frames instead of sending a burst.
    if (telemetryLagMs < 2 * TELEMETRY_INTERVAL_MS) {
      previousTelemetryTimeMs += TELEMETRY_INTERVAL_MS;
    } else {
      previousTelemetryTimeMs = nowMs;
    }
  }
  if (nowMs - previousSerialMonitorTimeMs >= SERIAL_MONITOR_INTERVAL_MS) {
    const float attitudeErrorDeg =
        adcsControl.error_deg(angularEstimation.yaw_deg());
    const float batteryVoltage = eps.get_battery_voltage();
    Serial.print("attitude_error:");
    Serial.print(attitudeErrorDeg, 2);
    Serial.print(",battery_sat_voltage:");
    Serial.println(batteryVoltage, 2);
    const unsigned long serialMonitorLagMs =
        nowMs - previousSerialMonitorTimeMs;
    if (serialMonitorLagMs < 2 * SERIAL_MONITOR_INTERVAL_MS) {
      previousSerialMonitorTimeMs += SERIAL_MONITOR_INTERVAL_MS;
    } else {
      previousSerialMonitorTimeMs = nowMs;
    }
  }
  delay(1);
}

float AdcsControl::calculate_body_pd_command(
    float estimated_yaw_deg, float yaw_rate_deg_per_sec) const {
  const float theta = estimated_yaw_deg;
  const float thetaref = target_angle_deg_;
  const float angleErrorDeg = normalize_angle_deg(thetaref - theta);
  return kp_ma_per_deg_ * angleErrorDeg -
         kd_ma_per_deg_per_sec_ * yaw_rate_deg_per_sec;
}
