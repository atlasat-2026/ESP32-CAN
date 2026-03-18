#pragma once

#include "BNO08x.hpp"
#include "BNO08xGlobalTypes.hpp"
#include "drone_controller.h"
#include <cmath>
#include <cstdint>

struct imu_state {
  Vec3C accel = {0, 0, 0};
  Vec3C last_accel = {0, 0, 0};
  QuatC rot = {0, 0, 0, 1};
  Vec3C vel = {0, 0, 0};
  int64_t last_time = -1;
  Vec3C euler_ang = {0, 0, 0};
};

BNO08x *setup_imu(imu_state *state);

extern SemaphoreHandle_t imuStateMutex;
