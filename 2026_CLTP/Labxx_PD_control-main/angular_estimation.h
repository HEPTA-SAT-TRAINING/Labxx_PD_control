#ifndef ANGULAR_ESTIMATION_H
#define ANGULAR_ESTIMATION_H

#include <Arduino.h>

#include "src/drv/imu9axis_bno055.h"

// Estimates the Z-axis attitude by trapezoidal integration of gyro data.
class AngularEstimation {
 public:
  static constexpr float GYRO_FILTER_ALPHA = 0.2f;
  static constexpr unsigned long MAX_GYRO_INTEGRATION_INTERVAL_MS = 100;
  void reset(unsigned long now_ms);
  void update(Bno055 &sensor, unsigned long now_ms);
  float yaw_deg() const;
  float yaw_rate_deg_per_sec() const;

 private:
  static float normalize_angle_deg(float angle_deg);

  float yaw_deg_ = 0.0f;
  float previous_gyro_z_deg_per_sec_ = 0.0f;
  unsigned long previous_sample_ms_ = 0;
  bool has_previous_sample_ = false;
};

#endif  // ANGULAR_ESTIMATION_H
