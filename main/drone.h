#pragma once

#include "esp_log.h"
#include "nav.h"

#include "esp32-hal.h"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdatomic.h>
#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "drone_comms.h"
#include "drone_controller.h"

#include "packet_handler.h"

#include "imu.h"
#include "packet_handler.h"
#include "sens_fus.h"

#define CONNECTION_LOST_THRESHOLD 200

#define MAX_LANDING_LINVEL 1.0

void setup_drone();

inline dcont::Vec3C v3f_to_v3c(Eigen::Vector3f v) { return {v[0], v[1], v[2]}; }

void drone_controller_task(void *params);

void motor_throttles_task(void *params);

// 3     4     2     1
inline float *motor_throttles = nullptr;
inline std::atomic_bool killswitch_active = false;

dcont::ControllerConfig default_config();

struct drone_cont_state {
  bool angvel_stablilized;
  bool fall_vel_stabilized;
  dcont::Input last_input;

  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
  Eigen::Quaternionf rot;
  Eigen::Vector3f angvel;

  dcont::StackedController *drone_controller;
  atomic_uint_fast8_t current_input_mode = INPUT_TYPE::ACRO;

  void init() { this->drone_controller = dcont::create(default_config()); }

  void drone_cont_stabilize() {
    // 1. ANGLE VELOCITY STABILIZATION
    // Kill the spin first to ensure sensors/control loops can work accurately.
    if (!this->angvel_stablilized) {
      dcont::set_input(drone_controller,
                       dcont::Input{.joystick = {.throttle_input = 0.0,
                                                 .roll_input = 0.0,
                                                 .yaw_input = 0.0,
                                                 .pitch_input = 0.0},
                                    .acceleration = {0.0, 0.0, 0.0},
                                    .rotation = {0.0, 0.0, 0.0},
                                    .velocity = {0.0, 0.0, 0.0},
                                    .position = {0.0, 0.0, 0.0},
                                    .mode = dcont::ModeInput::Acro});
      if (this->angvel.norm() < 1.0) {
        this->angvel_stablilized = true;
      }
    }

    // 2. FALL VELOCITY STABILIZATION (WITH PROP WASH AVOIDANCE)
    // Instead of climbing straight up, we move at an angle.
    if (!this->fall_vel_stabilized) {
      dcont::set_input(drone_controller,
                       dcont::Input{.joystick = {.throttle_input = 1.0,
                                                 .roll_input = 0.0,
                                                 .yaw_input = 0.0,
                                                 .pitch_input = 0.0},
                                    .acceleration = {0.0, 0.0, 0.0},
                                    .rotation = {0.0, 0.05 * M_PI, 0.0},
                                    .velocity = {0.0, 0.0, 0.0},
                                    .position = {0.0, 0.0, 0.0},
                                    .mode = dcont::ModeInput::Rotation});

      if (this->vel.z() - 1.0 >= 0) {
        this->fall_vel_stabilized = true;
      }
    }

    dcont::set_input(drone_controller,
                     dcont::Input{.joystick = {.throttle_input = 0.0,
                                               .roll_input = 0.0,
                                               .yaw_input = 0.0,
                                               .pitch_input = 0.0},
                                  .acceleration = {0.0, 0.0, 0.0},
                                  .rotation = {0.0, 0.0, 0.0},
                                  .velocity = {0.0, 0.0, 0.0},
                                  .position = {0.0, 0.0, 0.0},
                                  .mode = dcont::ModeInput::Velocity});
  }

  inline bool stabilization_done() {
    return this->fall_vel_stabilized && this->angvel_stablilized;
  }

  void fetch_sens() {
    static imu_state imu_state_local;
    if (imu_state_mutex && xSemaphoreTake(imu_state_mutex, 1)) {
      imu_state_local = imu_state_var;
      xSemaphoreGive(imu_state_mutex);
      this->angvel = imu_state_local.angvel;
    }
    if (sens_fus_mutex && xSemaphoreTake(sens_fus_mutex, 1)) {
      this->pos = sens_fus.position;
      this->vel = sens_fus.velocity;
      xSemaphoreGive(sens_fus_mutex);
    }

    this->rot = imu_state_var.rot;
  }

  void update_controller_sens() {

    dcont::set_cur_time(drone_controller, millis() / 1000.0f);
    dcont::set_cur_angvel(drone_controller, v3f_to_v3c(this->angvel));
    dcont::set_cur_linvel(drone_controller, v3f_to_v3c(this->vel));
    dcont::set_cur_pos(drone_controller, v3f_to_v3c(this->pos));

    dcont::set_cur_rot(drone_controller, dcont::QuatC{.i = this->rot.x(),
                                                      .j = this->rot.y(),
                                                      .k = this->rot.z(),
                                                      .w = this->rot.w()});
  }

  void update_throttles() {
    auto thrt_var = dcont::get_throttles(drone_controller);
    auto thrt = thrt_var.values;
    memcpy(motor_throttles, thrt, sizeof(float) * 4);
    // ESP_LOGI("CONT", "thr: %f, %f, %f, %f", thrt[0], thrt[1], thrt[2],
    // thrt[3]); ESP_LOGE("CONT", "UPDATE THROTTLES");
  }

  void update_input() {

    packet_controller_input cont_input;

    switch (atomic_load(&this->current_input_mode)) {
    case INPUT_TYPE::ACRO: {
      if (controller_input_semaphore &&
          xSemaphoreTake(controller_input_semaphore, 10)) {
        // if (millis() - time_last_controller > CONNECTION_LOST_THRESHOLD) {
        //   current_controller_input = {0, 0, 0, 0};
        // }
        cont_input = current_controller_input;

        xSemaphoreGive(controller_input_semaphore);

        auto inp = dcont::Input{.joystick = {.throttle_input = 0.5,
                                             .roll_input = cont_input.rx,
                                             .yaw_input = cont_input.lx,
                                             .pitch_input = cont_input.ry},
                                .acceleration = {0.0, 0.0, 0.0},
                                .rotation = {0.0, 0.0, 0.0},
                                .velocity = {0.0, 0.0, 0.0},
                                .position = {0.0, 0.0, 0.0},
                                .mode = dcont::ModeInput::Rotation};

        dcont::set_input(drone_controller, inp);

        this->last_input = inp;
      } else {
        drone_cont_stabilize();
      }
    } break;

    case INPUT_TYPE::AUTO_NAV: {
      if (!stabilization_done()) {
        drone_cont_stabilize();
      } else if (xSemaphoreTake(nav_mutex, 10)) {
        waypoint wayp = nav_man.get_current_waypoint();
        if (nav_man.current_waypoint == 8) {
          dcont::set_max_linvel(this->drone_controller, MAX_LANDING_LINVEL);
        }

        xSemaphoreGive(nav_mutex);
        if (wayp.coords_in_axis == std::nullopt) {
          drone_cont_stabilize();
        } else {

          auto coords = wayp.coords_in_axis.value_or(Eigen::Vector3f::Zero());

          dcont::set_input(
              drone_controller,
              dcont::Input{.joystick = {.throttle_input = 0.0,
                                        .roll_input = 0.0,
                                        .yaw_input = 0.0,
                                        .pitch_input = 0.0},
                           .acceleration = {0.0, 0.0, 0.0},
                           .rotation = {0.0, 0.0, 0.0},
                           .velocity = {0.0, 0.0, 0.0},
                           .position = {coords.x(), coords.y(), coords.z()},
                           .mode = dcont::ModeInput::Position});
        }
      } else {
        drone_cont_stabilize();
      }
      break;
    }
    default:
    case INPUT_TYPE::STABILIZE_FALL: {
      drone_cont_stabilize();
      break;
    }
    }
  }
  void update() {
    this->fetch_sens();
    this->update_controller_sens();
    this->update_input();
    this->update_throttles();
  }
};

inline drone_cont_state *drone_cont = nullptr;
