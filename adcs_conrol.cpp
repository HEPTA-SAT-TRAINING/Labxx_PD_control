#include "adcs_conrol.h"

String AdcsControl::start(AngularEstimation &estimation, UnitRollerI2C &wheel,
                          bool &wheel_is_running, unsigned long now_ms) {
  estimation.reset(now_ms);
  int32_t initial_wheel_speed_rpm = 0;
  wheel_readback_is_valid_ =
      wheel.getSpeedReadbackRpm(&initial_wheel_speed_rpm);
  wheel_speed_rpm_ =
      wheel_readback_is_valid_ ? initial_wheel_speed_rpm : 0;
  consecutive_wheel_read_errors_ = wheel_readback_is_valid_ ? 0 : 1;
  wheel_speed_command_rpm_ = 0.0f;
  previous_current_command_ms_ = now_ms - CURRENT_COMMAND_INTERVAL_MS;
  previous_wheel_command_write_ms_ = now_ms;
  last_sent_wheel_speed_rpm_ = 0;
  previous_wheel_readback_ms_ = now_ms;
  previous_current_readback_ms_ = now_ms;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  wheel_command_is_ready_ = false;
  control_state_ = CONTROL_STATE_NORMAL;
  current_command_ = 0;
  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;
  breakaway_state_started_ms_ = 0;
  breakaway_attempt_count_ = 0;
  current_readback_ = 0;
  last_sent_current_command_ = 0;
  current_readback_is_valid_ = false;
  consecutive_current_tracking_errors_ = 0;
  wheel.resetStalledProtect();
  wheel.setOutput(0);
  wheel.setMode(ROLLER_MODE_CURRENT);
  delay(10);
  wheel.setCurrent(0);
  delay(10);
  wheel.setOutput(1);
  wheel_is_running = true;
  enabled_ = true;
  return "OK START CURRENT_PD=0.0 mA";
}

String AdcsControl::stop(UnitRollerI2C &wheel, bool &wheel_is_running) {
  enabled_ = false;
  current_command_ = 0;
  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;
  current_readback_ = 0;
  last_sent_current_command_ = 0;
  current_readback_is_valid_ = false;
  consecutive_current_tracking_errors_ = 0;
  wheel_speed_command_rpm_ = 0.0f;
  last_sent_wheel_speed_rpm_ = 0;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  wheel_readback_is_valid_ = false;
  consecutive_wheel_read_errors_ = 0;
  breakaway_state_started_ms_ = 0;
  breakaway_attempt_count_ = 0;
  wheel_command_is_ready_ = false;
  control_state_ = CONTROL_STATE_NORMAL;
  wheel.setCurrent(0);
  wheel.setOutput(0);
  wheel_is_running = false;
  return "OK STOP";
}

float AdcsControl::calculate_body_pd_command(float angle_error_deg, float yaw_rate_deg_per_sec) const {
  return kp_ma_per_deg_ * angle_error_deg - kd_ma_per_deg_per_sec_ * yaw_rate_deg_per_sec;
}

void AdcsControl::update_control_state(float angle_error_deg,
                                       float yaw_rate_deg_per_sec,
                                       float body_pd_command,
                                       unsigned long now_ms) {
  if (!enabled_) return;
  wheel_command_is_ready_ = false;

  if (consecutive_wheel_read_errors_ >=
          MAX_CONSECUTIVE_WHEEL_READ_ERRORS ||
      consecutive_current_tracking_errors_ >=
          MAX_CONSECUTIVE_CURRENT_TRACKING_ERRORS) {
    wheel_is_recovering_ = true;
    const bool zero_command_must_be_sent = current_command_ != 0;
    current_command_ = 0;
    friction_compensation_ma_ = 0.0f;
    wheel_command_is_ready_ = zero_command_must_be_sent;
    return;
  }

  // 飽和したホイールへ逆向きトルクを与えて0 rpmへ戻す。
  if (handle_unloading(now_ms)) {
    return;
  }

  // アンロード後、ホイールと衛星本体が静止するまで待つ。
  if (handle_settling(yaw_rate_deg_per_sec, now_ms)) {
    return;
  }

  // 回転数上限へ押し続けている場合はアンローディングへ移行する。
  if (check_wheel_saturation(angle_error_deg, yaw_rate_deg_per_sec, body_pd_command, now_ms)) {
    return;
  }

  if (handle_stiction_recovery(angle_error_deg,
                                yaw_rate_deg_per_sec, now_ms)) {
    return;
  }

  update_current_command(angle_error_deg, yaw_rate_deg_per_sec,
                         body_pd_command, now_ms);
}

bool AdcsControl::handle_unloading(unsigned long now_ms) {
  if (control_state_ != CONTROL_STATE_UNLOADING) return false;

  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;
  breakaway_state_started_ms_ = 0;
  breakaway_attempt_count_ = 0;

  if (now_ms - previous_current_command_ms_ >=
      CURRENT_COMMAND_INTERVAL_MS) {
    previous_current_command_ms_ = now_ms;
    int32_t target_current = 0;
    if (wheel_speed_rpm_ > UNLOAD_FINISHED_SPEED_RPM) {
      target_current = -lroundf(UNLOAD_CURRENT_MA * CURRENT_REGISTER_PER_MA);
    } else if (wheel_speed_rpm_ < -UNLOAD_FINISHED_SPEED_RPM) {
      target_current = lroundf(UNLOAD_CURRENT_MA * CURRENT_REGISTER_PER_MA);
    }
    const int32_t max_delta =
        lroundf(MAX_CURRENT_SLEW_MA_PER_UPDATE * CURRENT_REGISTER_PER_MA);
    current_command_ += constrain(target_current - current_command_,
                                  -max_delta, max_delta);
    wheel_command_is_ready_ = true;

    if (target_current == 0 && current_command_ == 0) {
      control_state_ = CONTROL_STATE_SETTLING;
      settling_started_ms_ = 0;
    }
  }
  return true;
}

bool AdcsControl::handle_settling(float yaw_rate_deg_per_sec,
                                  unsigned long now_ms) {
  if (control_state_ != CONTROL_STATE_SETTLING) return false;

  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;

  // SETTLING中はトルクを0に保つ。
  current_command_ = 0;
  wheel_command_is_ready_ = true;

  // ホイールと衛星本体の停止が連続して確認できたら通常制御へ戻す。
  const bool wheel_is_stopped =
      labs(static_cast<long>(wheel_speed_rpm_)) <=
      UNLOAD_FINISHED_SPEED_RPM;
  const bool body_is_stopped =
      fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC;
  if (wheel_is_stopped && body_is_stopped) {
    if (settling_started_ms_ == 0) settling_started_ms_ = now_ms;
    if (now_ms - settling_started_ms_ >= SETTLING_CONFIRM_MS) {
      control_state_ = CONTROL_STATE_NORMAL;
      saturation_started_ms_ = 0;
      settling_started_ms_ = 0;
      previous_current_command_ms_ = now_ms;
    }
  } else {
    settling_started_ms_ = 0;
  }
  return true;
}

bool AdcsControl::check_wheel_saturation(float angle_error_deg,
                                         float yaw_rate_deg_per_sec,
                                         float body_pd_command,
                                         unsigned long now_ms) {
  const bool pushing_positive_limit =
      wheel_speed_rpm_ >= UNLOAD_START_SPEED_RPM &&
      body_pd_command > CONTROL_SWITCH_THRESHOLD;
  const bool pushing_negative_limit =
      wheel_speed_rpm_ <= -UNLOAD_START_SPEED_RPM &&
      body_pd_command < -CONTROL_SWITCH_THRESHOLD;
  const bool unload_is_needed =
      fabsf(angle_error_deg) > UNLOAD_MIN_ERROR_DEG &&
      (pushing_positive_limit || pushing_negative_limit);

  if (!unload_is_needed) {
    saturation_started_ms_ = 0;
    if (control_state_ == CONTROL_STATE_SATURATED) {
      control_state_ = CONTROL_STATE_NORMAL;
    }
    return false;
  }

  if (saturation_started_ms_ == 0) {
    saturation_started_ms_ = now_ms;
    control_state_ = CONTROL_STATE_SATURATED;
  }
  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;
  // Do not continue accelerating the wheel while saturation is confirmed.
  current_command_ = 0;
  wheel_command_is_ready_ = true;

  // 目標から離れ始めた場合は即時、それ以外は継続確認後にアンロードする。
  const bool moving_away_from_target =
      angle_error_deg * yaw_rate_deg_per_sec < 0.0f &&
      fabsf(yaw_rate_deg_per_sec) >= MOVING_AWAY_YAW_RATE_DEG_PER_SEC;
  if (moving_away_from_target ||
      now_ms - saturation_started_ms_ >= SATURATION_CONFIRM_MS) {
    control_state_ = CONTROL_STATE_UNLOADING;
    previous_current_command_ms_ = now_ms - CURRENT_COMMAND_INTERVAL_MS;
    saturation_started_ms_ = 0;
    return true;
  }
  return true;
}

bool AdcsControl::handle_stiction_recovery(
    float angle_error_deg, float yaw_rate_deg_per_sec,
    unsigned long now_ms) {
  if (control_state_ == CONTROL_STATE_BREAKAWAY_PULSE) {
    if (fabsf(yaw_rate_deg_per_sec) >
        STUCK_YAW_RATE_THRESHOLD_DEG_PER_SEC) {
      control_state_ = CONTROL_STATE_NORMAL;
      breakaway_state_started_ms_ = 0;
      breakaway_attempt_count_ = 0;
      current_command_ = 0;
      friction_compensation_ma_ = 0.0f;
      wheel_command_is_ready_ = true;
      return true;
    }
    if (now_ms - breakaway_state_started_ms_ >= BREAKAWAY_PULSE_MS) {
      current_command_ = 0;
      friction_compensation_ma_ = 0.0f;
      wheel_command_is_ready_ = true;
      control_state_ = CONTROL_STATE_RESPONSE_CHECK;
      breakaway_state_started_ms_ = now_ms;
    }
    return true;
  }

  if (control_state_ == CONTROL_STATE_RESPONSE_CHECK) {
    current_command_ = 0;
    friction_compensation_ma_ = 0.0f;
    if (fabsf(yaw_rate_deg_per_sec) >
        STUCK_YAW_RATE_THRESHOLD_DEG_PER_SEC) {
      control_state_ = CONTROL_STATE_NORMAL;
      breakaway_state_started_ms_ = 0;
      breakaway_attempt_count_ = 0;
      return false;
    }
    if (now_ms - breakaway_state_started_ms_ <
        BREAKAWAY_RESPONSE_CHECK_MS) {
      return true;
    }

    const bool retry_is_safe =
        breakaway_attempt_count_ < MAX_BREAKAWAY_ATTEMPTS &&
        labs(static_cast<long>(wheel_speed_rpm_)) <
            STUCK_COMP_MAX_WHEEL_SPEED_RPM;
    if (!retry_is_safe) {
      control_state_ = CONTROL_STATE_STICTION_STOP;
      wheel_command_is_ready_ = true;
      return true;
    }

    ++breakaway_attempt_count_;
    breakaway_state_started_ms_ = now_ms;
    control_state_ = CONTROL_STATE_BREAKAWAY_PULSE;
    const float pulse_current_ma = angle_error_deg >= 0.0f
                                       ? BREAKAWAY_CURRENT_MA
                                       : -BREAKAWAY_CURRENT_MA;
    current_command_ =
        lroundf(pulse_current_ma * CURRENT_REGISTER_PER_MA);
    friction_compensation_ma_ = BREAKAWAY_CURRENT_MA;
    wheel_command_is_ready_ = true;
    return true;
  }

  if (control_state_ == CONTROL_STATE_STICTION_STOP) {
    current_command_ = 0;
    friction_compensation_ma_ = 0.0f;
    return true;
  }

  const bool body_is_stuck =
      control_state_ == CONTROL_STATE_NORMAL &&
      fabsf(angle_error_deg) >= STUCK_ERROR_THRESHOLD_DEG &&
      fabsf(yaw_rate_deg_per_sec) <=
          STUCK_YAW_RATE_THRESHOLD_DEG_PER_SEC &&
      labs(static_cast<long>(wheel_speed_rpm_)) <
          STUCK_COMP_MAX_WHEEL_SPEED_RPM;
  if (!body_is_stuck) {
    stuck_candidate_started_ms_ = 0;
    breakaway_attempt_count_ = 0;
    return false;
  }

  if (stuck_candidate_started_ms_ == 0) {
    stuck_candidate_started_ms_ = now_ms;
    return false;
  }
  if (now_ms - stuck_candidate_started_ms_ < STUCK_CONFIRM_MS) {
    return false;
  }

  stuck_candidate_started_ms_ = 0;
  breakaway_attempt_count_ = 1;
  breakaway_state_started_ms_ = now_ms;
  control_state_ = CONTROL_STATE_BREAKAWAY_PULSE;
  const float pulse_current_ma = angle_error_deg >= 0.0f
                                     ? BREAKAWAY_CURRENT_MA
                                     : -BREAKAWAY_CURRENT_MA;
  current_command_ = lroundf(pulse_current_ma * CURRENT_REGISTER_PER_MA);
  friction_compensation_ma_ = BREAKAWAY_CURRENT_MA;
  wheel_command_is_ready_ = true;
  return true;
}

void AdcsControl::update_current_command(float angle_error_deg,
                                         float yaw_rate_deg_per_sec,
                                         float body_pd_command,
                                         unsigned long now_ms) {
  if (now_ms - previous_current_command_ms_ < CURRENT_COMMAND_INTERVAL_MS) {
    return;
  }
  previous_current_command_ms_ = now_ms;

  float requested_current_ma = body_pd_command;
  friction_compensation_ma_ = 0.0f;
  requested_current_ma = constrain(
      requested_current_ma, -MAX_COMMAND_CURRENT_MA,
      MAX_COMMAND_CURRENT_MA);
  if (fabsf(angle_error_deg) <= SETTLED_ANGLE_ERROR_DEG && fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC) {
    requested_current_ma = 0.0f;
  } 
  else if (fabsf(requested_current_ma) < CONTROL_DEADBAND_CURRENT_MA) {
    requested_current_ma = 0.0f;
  }

  // Reduce only the current that would accelerate an already fast wheel.
  // Braking current remains available at full strength. This makes reaching
  // the unloading threshold less frequent without delaying a necessary
  // unload once the wheel is actually saturated.
  const bool current_accelerates_wheel =
      wheel_readback_is_valid_ &&
      requested_current_ma * static_cast<float>(wheel_speed_rpm_) > 0.0f;
  if (current_accelerates_wheel) {
    const float taper_width_rpm = static_cast<float>(
        MAX_WHEEL_SPEED_RPM - WHEEL_CURRENT_TAPER_START_RPM);
    const float speed_above_taper_rpm =
        fabsf(static_cast<float>(wheel_speed_rpm_)) -
        static_cast<float>(WHEEL_CURRENT_TAPER_START_RPM);
    const float current_scale =
        1.0f - constrain(speed_above_taper_rpm / taper_width_rpm,
                         0.0f, 1.0f);
    requested_current_ma *= current_scale;
  }

  const int32_t requested_current = lroundf(requested_current_ma * CURRENT_REGISTER_PER_MA);
  const int32_t max_delta = lroundf(MAX_CURRENT_SLEW_MA_PER_UPDATE * CURRENT_REGISTER_PER_MA);
  current_command_ += constrain(requested_current - current_command_, -max_delta, max_delta);
  wheel_command_is_ready_ = true;
}

void AdcsControl::command_reaction_wheel(UnitRollerI2C &wheel,
                                         unsigned long now_ms) {
  if (!enabled_) return;

  if (now_ms - previous_wheel_readback_ms_ >=
      WHEEL_READBACK_INTERVAL_MS) {
    previous_wheel_readback_ms_ = now_ms;
    int32_t measured_speed_rpm = 0;
    const bool read_succeeded =
        wheel.getSpeedReadbackRpm(&measured_speed_rpm);
    if (read_succeeded) {
      const bool recovering_from_read_error =
          consecutive_wheel_read_errors_ > 0;
      wheel_readback_is_valid_ = true;
      consecutive_wheel_read_errors_ = 0;
      wheel_is_recovering_ = false;

      const bool speed_is_in_range =
          labs(static_cast<long>(measured_speed_rpm)) <=
          MAX_VALID_WHEEL_READBACK_RPM;
      const bool speed_jump_is_valid =
          labs(static_cast<long>(measured_speed_rpm) -
               static_cast<long>(wheel_speed_rpm_)) <=
          MAX_VALID_READBACK_JUMP_RPM;
      // After a read failure, accept the first plausible real value even if
      // it moved more than the normal per-sample jump limit. This prevents a
      // false 0 rpm from making the readback permanently stick at zero.
      if (speed_is_in_range &&
          (speed_jump_is_valid || recovering_from_read_error)) {
        wheel_speed_rpm_ = measured_speed_rpm;
      }
    } else {
      wheel_readback_is_valid_ = false;
      if (consecutive_wheel_read_errors_ < UINT8_MAX) {
        ++consecutive_wheel_read_errors_;
      }
      if (consecutive_wheel_read_errors_ >=
          MAX_CONSECUTIVE_WHEEL_READ_ERRORS) {
        wheel_is_recovering_ = true;
        current_command_ = 0;
        friction_compensation_ma_ = 0.0f;
        wheel.setCurrent(0);
        previous_wheel_command_write_ms_ = now_ms;
      }
    }
  }

  // Current readback is diagnostic and does not need to run at the faster
  // speed-feedback rate. Avoid a second I2C transaction after a failed speed
  // read, because a disconnected or busy wheel can otherwise stall telemetry.
  if (wheel_readback_is_valid_ &&
      now_ms - previous_current_readback_ms_ >=
          CURRENT_READBACK_INTERVAL_MS) {
    previous_current_readback_ms_ = now_ms;
    int32_t measured_current = 0;
    if (wheel.getCurrentReadback(&measured_current)) {
      current_readback_ = measured_current;
      current_readback_is_valid_ = true;
      const int32_t max_tracking_error =
          lroundf(MAX_CURRENT_TRACKING_ERROR_MA * CURRENT_REGISTER_PER_MA);
      const bool command_is_material =
          labs(static_cast<long>(last_sent_current_command_)) >=
          lroundf(10.0f * CURRENT_REGISTER_PER_MA);
      const bool current_tracks_command =
          labs(static_cast<long>(current_readback_) -
               static_cast<long>(last_sent_current_command_)) <=
          max_tracking_error;
      if (command_is_material && !current_tracks_command) {
        if (consecutive_current_tracking_errors_ < UINT8_MAX) {
          ++consecutive_current_tracking_errors_;
        }
      } else {
        consecutive_current_tracking_errors_ = 0;
      }
    } else {
      current_readback_is_valid_ = false;
      consecutive_current_tracking_errors_ = 0;
    }

    if (consecutive_current_tracking_errors_ >=
        MAX_CONSECUTIVE_CURRENT_TRACKING_ERRORS) {
      wheel_is_recovering_ = true;
      current_command_ = 0;
      friction_compensation_ma_ = 0.0f;
      wheel_command_is_ready_ = true;
    }
  }

  if (!wheel_command_is_ready_) return;
  wheel_command_is_ready_ = false;
  wheel.setCurrent(current_command_);
  last_sent_current_command_ = current_command_;
  previous_wheel_command_write_ms_ = now_ms;
}

void AdcsControl::set_target_angle_deg(float target_angle_deg) {
  target_angle_deg_ = normalize_angle_deg(target_angle_deg);
  // A new target always starts a new manoeuvre and cancels unloading for the
  // previous target.
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  friction_compensation_ma_ = 0.0f;
  stuck_candidate_started_ms_ = 0;
  breakaway_state_started_ms_ = 0;
  breakaway_attempt_count_ = 0;
  wheel_command_is_ready_ = false;
  control_state_ = CONTROL_STATE_NORMAL;
}

bool AdcsControl::set_proportional_gain(float kp_ma_per_deg) {
  if (!isfinite(kp_ma_per_deg) || kp_ma_per_deg < 0.0f ||
      kp_ma_per_deg > MAX_KP_MA_PER_DEG) {
    return false;
  }
  kp_ma_per_deg_ = kp_ma_per_deg;
  return true;
}

bool AdcsControl::set_derivative_gain(float kd_ma_per_deg_per_sec) {
  if (!isfinite(kd_ma_per_deg_per_sec) || kd_ma_per_deg_per_sec < 0.0f ||
      kd_ma_per_deg_per_sec > MAX_KD_MA_PER_DEG_PER_SEC) {
    return false;
  }
  kd_ma_per_deg_per_sec_ = kd_ma_per_deg_per_sec;
  return true;
}

bool AdcsControl::is_enabled() const { return enabled_; }
float AdcsControl::target_angle_deg() const { return target_angle_deg_; }
float AdcsControl::proportional_gain_ma_per_deg() const {
  return kp_ma_per_deg_;
}
float AdcsControl::derivative_gain_ma_per_deg_per_sec() const {
  return kd_ma_per_deg_per_sec_;
}

float AdcsControl::current_command_ma() const {
  return current_command_ / CURRENT_REGISTER_PER_MA;
}
float AdcsControl::friction_compensation_ma() const {
  return friction_compensation_ma_;
}
float AdcsControl::current_readback_ma() const {
  return current_readback_ / CURRENT_REGISTER_PER_MA;
}
bool AdcsControl::current_readback_is_valid() const {
  return current_readback_is_valid_;
}
int32_t AdcsControl::wheel_speed_rpm() const { return wheel_speed_rpm_; }
bool AdcsControl::wheel_readback_is_valid() const {
  return wheel_readback_is_valid_;
}
float AdcsControl::wheel_speed_command_rpm() const {
  return wheel_speed_command_rpm_;
}
int32_t AdcsControl::last_sent_wheel_speed_rpm() const {
  return last_sent_wheel_speed_rpm_;
}
const char *AdcsControl::control_state_name() const {
  if (!enabled_) return "STOPPED";
  if (wheel_is_recovering_) return "WHEEL_RECOVERY";
  switch (control_state_) {
    case CONTROL_STATE_SATURATED:
      return "SATURATED";
    case CONTROL_STATE_UNLOADING:
      return "UNLOADING";
    case CONTROL_STATE_SETTLING:
      return "SETTLING";
    case CONTROL_STATE_BREAKAWAY_PULSE:
      return "BREAKAWAY";
    case CONTROL_STATE_RESPONSE_CHECK:
      return "RESPONSE_CHECK";
    case CONTROL_STATE_STICTION_STOP:
      return "STICTION_STOP";
    default:
      return "CONTROL";
  }
}

float AdcsControl::normalize_angle_deg(float angle_deg) {
  while (angle_deg >= 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}
