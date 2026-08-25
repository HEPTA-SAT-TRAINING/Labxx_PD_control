#include "angular_estimation.h"

void AngularEstimation::reset(unsigned long now_ms) {
  yaw_deg_ = 0.0f;
  previous_gyro_z_deg_per_sec_ = 0.0f;
  previous_sample_ms_ = now_ms;
  has_previous_sample_ = false;
}

void AngularEstimation::update(Bno055 &sensor, unsigned long now_ms) {
  float gyro_x_deg_per_sec;
  float gyro_y_deg_per_sec;
  float gyro_z_deg_per_sec;
  sensor.sen_gyro(&gyro_x_deg_per_sec, &gyro_y_deg_per_sec, &gyro_z_deg_per_sec);

  if (has_previous_sample_) {
    //一次ローパスフィルタでジャイロのノイズを低減する。フィルタ係数はGYRO_FILTER_ALPHAで調整する。
    gyro_z_deg_per_sec = GYRO_FILTER_ALPHA * gyro_z_deg_per_sec + (1.0f - GYRO_FILTER_ALPHA) * previous_gyro_z_deg_per_sec_;
    const unsigned long sample_interval_ms = now_ms - previous_sample_ms_;
    if (sample_interval_ms <= MAX_GYRO_INTEGRATION_INTERVAL_MS) {
      const float dt_sec = sample_interval_ms / 1000.0f;
      //台形積分
      yaw_deg_ += (previous_gyro_z_deg_per_sec_ + gyro_z_deg_per_sec) * 0.5f * dt_sec;
      yaw_deg_ = normalize_angle_deg(yaw_deg_);
    }
  }

  previous_gyro_z_deg_per_sec_ = gyro_z_deg_per_sec;
  previous_sample_ms_ = now_ms;
  has_previous_sample_ = true;
}

float AngularEstimation::yaw_deg() const { return yaw_deg_; }
float AngularEstimation::yaw_rate_deg_per_sec() const {
  return previous_gyro_z_deg_per_sec_;
}

float AngularEstimation::error_deg(float target_yaw_deg) const {
  return normalize_angle_deg(target_yaw_deg - yaw_deg_);
}

float AngularEstimation::normalize_angle_deg(float angle_deg) {
  while (angle_deg >= 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}
