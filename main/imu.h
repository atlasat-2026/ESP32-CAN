#pragma once

#include "drone_controller.h"
#include <cstdint>

#ifdef LOW
#undef LOW
#endif
#ifdef HIGH
#undef HIGH
#endif

#include "BNO08x.hpp"

struct imu_state {
  Vec3C accel = {0, 0, 0};
  QuatC rot = {0, 0, 0, 1};
  int64_t last_time = -1;
};

BNO08x *setup_imu(imu_state *state);
