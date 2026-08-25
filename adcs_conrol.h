#ifndef ADCS_CONROL_H
#define ADCS_CONROL_H

#include <Arduino.h>

#include "angular_estimation.h"
#include "src/drv/imu9axis_bno055.h"
#include "src/drv/unit_rolleri2c.hpp"
#include "src/drv/unit_roller_common.hpp"

class AdcsControl {
 public:
  static constexpr int32_t MAX_WHEEL_SPEED_RPM = 600;
  static constexpr float CONTROL_SWITCH_THRESHOLD = 2.0f;
  static constexpr int32_t UNLOAD_START_SPEED_RPM = 590;
  static constexpr int32_t UNLOAD_FINISHED_SPEED_RPM = 10;
  static constexpr float UNLOAD_MIN_ERROR_DEG = 5.0f;
  static constexpr float MOVING_AWAY_YAW_RATE_DEG_PER_SEC = 1.0f;
  static constexpr unsigned long SATURATION_CONFIRM_MS = 1000;
  static constexpr unsigned long SETTLING_CONFIRM_MS = 1000;
  static constexpr float SETTLED_ANGLE_ERROR_DEG = 1.0f;
  static constexpr float SETTLED_YAW_RATE_DEG_PER_SEC = 0.5f;
  static constexpr float TARGET_HOLD_ENTER_ERROR_DEG = 5.0f;
  static constexpr float TARGET_HOLD_ENTER_YAW_RATE_DEG_PER_SEC = 2.0f;
  static constexpr unsigned long TARGET_HOLD_CONFIRM_MS = 200;
  static constexpr float TARGET_HOLD_EXIT_ERROR_DEG = 7.0f;
  static constexpr unsigned long TARGET_HOLD_EXIT_CONFIRM_MS = 500;
  static constexpr unsigned long CURRENT_COMMAND_INTERVAL_MS = 200;
  static constexpr float MAX_COMMAND_CURRENT_MA = 120.0f;
  static constexpr float BREAKAWAY_CURRENT_MA = 80.0f;
  static constexpr float BREAKAWAY_MIN_ERROR_DEG = 5.0f;
  static constexpr float MAX_CURRENT_SLEW_MA_PER_UPDATE = 120.0f;
  static constexpr float UNLOAD_CURRENT_MA = 50.0f;
  static constexpr float CONTROL_DEADBAND_CURRENT_MA = 1.0f;
  static constexpr float STUCK_ERROR_THRESHOLD_DEG = 5.0f;
  static constexpr float STUCK_YAW_RATE_THRESHOLD_DEG_PER_SEC = 1.0f;
  static constexpr unsigned long STUCK_CONFIRM_MS = 1000;
  static constexpr float STUCK_CURRENT_STEP_MA = 2.0f;
  static constexpr float MAX_STUCK_COMPENSATION_MA = 30.0f;
  static constexpr int32_t STUCK_COMP_MAX_WHEEL_SPEED_RPM = 500;
  static constexpr unsigned long WHEEL_READBACK_INTERVAL_MS = 250;
  static constexpr int32_t MAX_VALID_WHEEL_READBACK_RPM = 700;
  static constexpr int32_t MAX_VALID_READBACK_JUMP_RPM = 300;
  static constexpr int32_t MAX_READBACK_COMMAND_ERROR_RPM = 200;
  static constexpr int32_t WHEEL_STOPPED_THRESHOLD_RPM = 10;
  static constexpr unsigned long WHEEL_RECOVERY_DELAY_MS = 2000;
  static constexpr int32_t MAX_CURRENT_REGISTER = 100000;
  static constexpr float MAX_KP_MA_PER_DEG = 1000.0f;
  static constexpr float MAX_KD_MA_PER_DEG_PER_SEC = 1000.0f;
  static constexpr float CURRENT_REGISTER_PER_MA = 100.0f;

  String start(AngularEstimation &estimation, UnitRollerI2C &wheel,
               bool &wheel_is_running, unsigned long now_ms);
  String stop(UnitRollerI2C &wheel, bool &wheel_is_running);
  float calculate_body_pd_command(
      float angle_error_deg, float yaw_rate_deg_per_sec) const;
  void update_control_state(float angle_error_deg,
                            float yaw_rate_deg_per_sec,
                            float body_pd_command,
                            unsigned long now_ms);
  void command_reaction_wheel(UnitRollerI2C &wheel,
                              unsigned long now_ms);

  void set_target_angle_deg(float target_angle_deg);
  bool set_proportional_gain(float kp_ma_per_deg);
  bool set_derivative_gain(float kd_ma_per_deg_per_sec);

  bool is_enabled() const;
  float target_angle_deg() const;
  float proportional_gain_ma_per_deg() const;
  float derivative_gain_ma_per_deg_per_sec() const;
  float current_command_ma() const;
  float friction_compensation_ma() const;
  float current_readback_ma() const;
  int32_t wheel_speed_rpm() const;
  float wheel_speed_command_rpm() const;
  int32_t last_sent_wheel_speed_rpm() const;
  const char *control_state_name() const;

 private:
  enum ControlState {
    CONTROL_STATE_NORMAL,//PD出力を連続的な電流指令として出力
    CONTROL_STATE_SATURATED,//ホイールが回転数上限に達しており，さらに同じ方向へ加速しようとしている状態
    CONTROL_STATE_UNLOADING,//逆向きトルクでホイールを0 rpmへ近づける
    CONTROL_STATE_SETTLING,//アンローディング後にホイール速度が安定するまで待つ
    CONTROL_STATE_TARGET_HOLD//目標付近で連続電流PDにより姿勢を保持する
  };

  static float normalize_angle_deg(float angle_deg);
  bool handle_target_hold(float angle_error_deg,
                          float yaw_rate_deg_per_sec,
                          unsigned long now_ms);
  bool handle_unloading(unsigned long now_ms);
  bool handle_settling(float yaw_rate_deg_per_sec,
                       unsigned long now_ms);
  bool check_wheel_saturation(float angle_error_deg,
                              float yaw_rate_deg_per_sec,
                              float body_pd_command,
                              unsigned long now_ms);
  void update_current_command(float angle_error_deg,
                              float yaw_rate_deg_per_sec,
                              float body_pd_command,
                              unsigned long now_ms);
  void apply_current_command_with_slew(int32_t requested_current);
  bool enabled_ = false;
  float target_angle_deg_ = 0.0f;
  float kp_ma_per_deg_ = 2.0f;
  float kd_ma_per_deg_per_sec_ = 3.0f;
  int32_t current_command_ = 0;
  float friction_compensation_ma_ = 0.0f;
  unsigned long stuck_candidate_started_ms_ = 0;
  int32_t current_readback_ = 0;
  int32_t wheel_speed_rpm_ = 0;
  float wheel_speed_command_rpm_ = 0.0f;
  unsigned long previous_current_command_ms_ = 0;
  unsigned long previous_wheel_command_write_ms_ = 0;
  int32_t last_sent_wheel_speed_rpm_ = 0;
  unsigned long previous_wheel_readback_ms_ = 0;
  unsigned long saturation_started_ms_ = 0;
  unsigned long settling_started_ms_ = 0;
  unsigned long target_hold_candidate_started_ms_ = 0;
  unsigned long target_hold_exit_started_ms_ = 0;
  unsigned long wheel_tracking_error_started_ms_ = 0;
  bool wheel_is_recovering_ = false;
  bool wheel_command_is_ready_ = false;
  ControlState control_state_ = CONTROL_STATE_NORMAL;
};

#endif  // ADCS_CONROL_H
