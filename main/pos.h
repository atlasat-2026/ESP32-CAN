
#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

inline float getYawDifference(const Eigen::Vector3f &v_gps,
                              const Eigen::Vector3f &v_imu) {
  // 1. Extract the 2D components (Project to XY plane)
  float yaw_gps = std::atan2(v_gps.y(), v_gps.x());
  float yaw_imu = std::atan2(v_imu.y(), v_imu.x());

  // 2. Calculate the raw difference
  float delta_yaw = yaw_gps - yaw_imu;

  // 3. Normalize the angle to [-PI, PI]
  while (delta_yaw > M_PI)
    delta_yaw -= 2.0 * M_PI;
  while (delta_yaw < -M_PI)
    delta_yaw += 2.0 * M_PI;

  return delta_yaw;
}

struct nav_compl {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  float yaw_offset = 0.0;
  float yaw_delta_weight = 0.04;

  Eigen::Vector3f gps_weight_vel = Eigen::Vector3f::Constant(0.03f);
  Eigen::Vector3f gps_weight_pos = Eigen::Vector3f::Constant(0.03f);

  void predict(float dt, Eigen::Vector3f accel) {
    Eigen::Vector3f accel_rotated =
        Eigen::AngleAxisf(this->yaw_offset, Eigen::Vector3d::UnitZ()) * accel;

    Eigen::Vector3f next_velocity = this->velocity + (accel_rotated * dt);

    this->position += (this->velocity + next_velocity) * 0.5f * dt;

    this->velocity = next_velocity;
  }

  void measure(Eigen::Vector3f gps_pos, Eigen::Vector3f gps_vel) {
    this->position = (Eigen::Vector3f::Ones() - gps_weight_pos)
                         .cwiseProduct(this->position) +
                     gps_weight_pos.cwiseProduct(gps_pos);

    if (gps_vel.norm() > 1) {
      float yaw_delta = getYawDifference(gps_vel, this->velocity);
      yaw_offset =
          yaw_offset * (1 - yaw_delta_weight) + yaw_delta * yaw_delta_weight;
    }

    this->velocity = (Eigen::Vector3f::Ones() - gps_weight_vel)
                         .cwiseProduct(this->velocity) +
                     gps_weight_vel.cwiseProduct(gps_vel);
  }
};
