#pragma once

#include "Eigen/Core"
#include "drone_comms.h"
#include "drone_controller.h"

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

void setup_drone();

inline dcont::Vec3C v3f_to_v3c(Eigen::Vector3f v) { return {v[0], v[1], v[2]}; }

inline dcont::StackedController *drone_controller = nullptr;
inline dcont::ModeInput current_input_mode;

void drone_controller_task(void *params);

void motor_throttles_task(void *params);

inline float motor_throttles[4];
inline bool killswitch_active;
