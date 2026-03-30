#pragma once
#include "freertos/idf_additions.h"
#include <cmath>

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

inline float getYawDifference(const Eigen::Vector3f &v_gps,
                              const Eigen::Vector3f &v_imu) {
  float yaw_gps = std::atan2(v_gps.y(), v_gps.x());
  float yaw_imu = std::atan2(v_imu.y(), v_imu.x());

  float delta_yaw = yaw_gps - yaw_imu;

  return std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
}

struct nav_compl {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  float yaw_offset = 0.0f;

  // Time Constants per axis (X, Y, Z)
  // Lower = faster tracking of GPS; Higher = smoother/more IMU trust
  Eigen::Vector3f tau_gps_pos = {0.5f, 0.5f, 0.5f};
  Eigen::Vector3f tau_gps_vel = {1.0f, 1.0f, INFINITY};

  Eigen::Vector3f tau_baro_pos = {INFINITY, INFINITY, 5.0f};
  Eigen::Vector3f tau_baro_vel = {INFINITY, INFINITY, 10.0f};

  float tau_yaw = 2.0f; // Yaw remains a scalar

  void predict(float dt, Eigen::Vector3f accel) {
    // Rotate body-frame accel to world-frame
    Eigen::Vector3f accel_rotated =
        Eigen::AngleAxisf(this->yaw_offset, Eigen::Vector3f::UnitZ()) * accel;

    Eigen::Vector3f next_velocity = this->velocity + (accel_rotated * dt);

    // Trapezoidal integration for position
    this->position += (this->velocity + next_velocity) * 0.5f * dt;
    this->velocity = next_velocity;
  }

  void measure_gps(float dt, Eigen::Vector3f gps_pos, Eigen::Vector3f gps_vel) {
    // Calculate Alpha vectors using element-wise operations
    // Formula: alpha = dt / (tau + dt)
    Eigen::Vector3f alpha_pos = dt / (tau_gps_pos.array() + dt);
    Eigen::Vector3f alpha_vel = dt / (tau_gps_vel.array() + dt);
    float alpha_yaw = dt / (tau_yaw + dt);

    // 1. Position Update (Element-wise LPF)
    // res = (1 - alpha) * state + alpha * measurement
    this->position =
        (Eigen::Vector3f::Ones() - alpha_pos).array() * this->position.array() +
        alpha_pos.array() * gps_pos.array();

    // 2. Yaw Correction (only if moving > 1.0 m/s)
    if (gps_vel.norm() > 1.0f) {
      float yaw_delta = getYawDifference(gps_vel, this->velocity);
      this->yaw_offset += yaw_delta * alpha_yaw;

      this->yaw_offset =
          std::atan2(std::sin(this->yaw_offset), std::cos(this->yaw_offset));
    }

    // 3. Velocity Update (Element-wise LPF)
    this->velocity =
        (Eigen::Vector3f::Ones() - alpha_vel).array() * this->velocity.array() +
        alpha_vel.array() * gps_vel.array();
  }

  void measure_baro(float dt, Eigen::Vector3f baro_pos,
                    Eigen::Vector3f baro_vel) {
    // Calculate Alpha vectors using element-wise operations
    // Formula: alpha = dt / (tau + dt)
    Eigen::Vector3f alpha_pos = dt / (tau_baro_pos.array() + dt);
    Eigen::Vector3f alpha_vel = dt / (tau_baro_vel.array() + dt);

    this->position =
        (Eigen::Vector3f::Ones() - alpha_pos).array() * this->position.array() +
        alpha_pos.array() * baro_pos.array();

    this->velocity =
        (Eigen::Vector3f::Ones() - alpha_vel).array() * this->velocity.array() +
        alpha_vel.array() * baro_vel.array();
  }
};

inline SemaphoreHandle_t nav_mutex = NULL;
inline nav_compl nav_filter;
