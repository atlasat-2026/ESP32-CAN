#pragma once

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "drone_comms.h"
#include "drone_controller.h"

void setup_drone();

inline dcont::Vec3C v3f_to_v3c(Eigen::Vector3f v) { return {v[0], v[1], v[2]}; }

inline dcont::StackedController *drone_controller = nullptr;
inline dcont::ModeInput current_input_mode = dcont::ModeInput::Acro;

void drone_controller_task(void *params);

void motor_throttles_task(void *params);

// 3     4     2     1
inline float motor_throttles[4] = {0.01, 0.01, 0.01, 0.01};
inline bool killswitch_active = false;
