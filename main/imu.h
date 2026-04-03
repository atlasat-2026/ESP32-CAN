#pragma once

#include "drone_controller.h"

#include "freertos/idf_additions.h"
#include <cstdint>

#ifdef LOW
#undef LOW
#endif
#ifdef HIGH
#undef HIGH
#endif

#include "BNO08x.hpp"

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

struct imu_state {
  Vec3C accel = {0, 0, 0};
  QuatC rot = {0, 0, 0, 1};
  int64_t last_time = -1;
  Eigen::Vector3f angvel;
  Eigen::Vector3f rot_euler;
};

void setup_imu();

inline SemaphoreHandle_t imu_state_mutex = NULL;
inline imu_state imu_state_var;
