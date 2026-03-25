#include <Eigen/Dense>

struct nav_compl {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

  Eigen::Vector3f gps_weight_vel = Eigen::Vector3f::Constant(0.01f);
  Eigen::Vector3f gps_weight_pos = Eigen::Vector3f::Constant(0.01f);

  void predict(float dt, Eigen::Vector3f accel) {
    Eigen::Vector3f next_velocity = this->velocity + (accel * dt);

    this->position += (this->velocity + next_velocity) * 0.5f * dt;

    this->velocity = next_velocity;
  }

  void measure(Eigen::Vector3f gps_pos, Eigen::Vector3f gps_vel) {
    this->position = (Eigen::Vector3f::Ones() - gps_weight_pos)
                         .cwiseProduct(this->position) +
                     gps_weight_pos.cwiseProduct(gps_pos);

    this->velocity = (Eigen::Vector3f::Ones() - gps_weight_vel)
                         .cwiseProduct(this->velocity) +
                     gps_weight_vel.cwiseProduct(gps_vel);
  }
};
