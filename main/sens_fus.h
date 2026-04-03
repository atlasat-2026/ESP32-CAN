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

struct sens_fus_compl {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f last_accel_world = Eigen::Vector3f::Zero();
  float yaw_offset = 0.0f;

  /*
   * Tau is the time that the filter takes to reach 1-e^(-1) of the difference
   * to a constant target
   * so at t=tau, were 63% of the way there
   * at t=3*tau, were 95% of the way there
   */
  Eigen::Vector3f tau_gps_pos = {4.0f, 4.0f, 10.0f};
  Eigen::Vector3f tau_gps_vel = {5.0f, 5.0f, INFINITY};

  Eigen::Vector3f tau_baro_pos = {INFINITY, INFINITY, 5.0f};
  Eigen::Vector3f tau_baro_vel = {INFINITY, INFINITY, 10.0f};

  float tau_yaw = 10.0f; // Yaw remains a scalar

  void predict(float dt, Eigen::Vector3f accel) {
    Eigen::Vector3f accel_rotated =
        Eigen::AngleAxisf(this->yaw_offset, Eigen::Vector3f::UnitZ()) * accel;

    Eigen::Vector3f next_velocity =
        this->velocity + (this->last_accel_world + accel_rotated) * 0.5f * dt;

    this->position += (this->velocity + next_velocity) * 0.5f * dt;

    this->velocity = next_velocity;
    this->last_accel_world = accel_rotated;
  }

  void measure_gps(float dt, Eigen::Vector3f gps_pos, Eigen::Vector3f gps_vel) {
    // alpha = dt / (tau + dt)
    Eigen::Vector3f alpha_pos = dt / (tau_gps_pos.array() + dt);
    Eigen::Vector3f alpha_vel = dt / (tau_gps_vel.array() + dt);
    float alpha_yaw = dt / (tau_yaw + dt);

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

    // res = (1 - alpha) * state + alpha * measurement
    this->velocity =
        (Eigen::Vector3f::Ones() - alpha_vel).array() * this->velocity.array() +
        alpha_vel.array() * gps_vel.array();
  }

  void measure_baro(float dt, Eigen::Vector3f baro_pos,
                    Eigen::Vector3f baro_vel) {
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

inline SemaphoreHandle_t sens_fus_mutex = NULL;
inline sens_fus_compl sens_fus;
