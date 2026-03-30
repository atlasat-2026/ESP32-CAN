#pragma once

#include <cmath>
#include <cstdint>

#include "BNO08x.hpp"
#include "drone_controller.h"

struct imu_state {
  Vec3C accel = {0, 0, 0};
  QuatC rot = {0, 0, 0, 1};
  int64_t last_time = -1;
};

BNO08x *setup_imu(imu_state *state);
