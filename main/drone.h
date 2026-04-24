#pragma once

#include "Eigen/Core"
#include "esp_log.h"
#include "gps.h"
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

#define MAX_LANDING_LINVEL 2.0

void setup_drone();

inline dcont::Vec3C v3f_to_v3c(Eigen::Vector3f v) { return {v[0], v[1], v[2]}; }

void drone_controller_task(void *params);

void motor_throttles_task(void *params);

// 3     4     2     1
inline float *motor_throttles = nullptr;
inline std::atomic_bool killswitch_active = true;

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
  atomic_uint_fast8_t current_input_mode = INPUT_TYPE::AUTO_NAV;

  void init() { this->drone_controller = dcont::create(default_config()); }

  void drone_cont_stabilize() {

    // 1. ANGLE VELOCITY STABILIZATION
    // Kill the spin first to ensure sensors/control loops can work accurately.
    if (!this->angvel_stablilized) {

      if (this->angvel.norm() < 1.0) {
        this->angvel_stablilized = true;
      } else {
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
      }
    }

    // 2. FALL VELOCITY STABILIZATION (WITH PROP WASH AVOIDANCE)
    // Instead of climbing straight up, we move at an angle.
    if (!this->fall_vel_stabilized) {
      if (this->vel.z() - 1.0 >= 0) {
        this->fall_vel_stabilized = true;
      } else {
        dcont::set_input(drone_controller,
                         dcont::Input{.joystick = {.throttle_input = 0.6,
                                                   .roll_input = 0.0,
                                                   .yaw_input = 0.0,
                                                   .pitch_input = 0.0},
                                      .acceleration = {0.0, 0.0, 0.0},
                                      .rotation = {0.0, M_PI / 8, 0.0},
                                      .velocity = {0.0, 0.0, 0.0},
                                      .position = {0.0, 0.0, 0.0},
                                      .mode = dcont::ModeInput::Rotation});
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

  Eigen::Vector3f velocity_hist[25];
  int velocity_hist_index = 0;

  void fetch_sens() {
    static imu_state imu_state_local;
    if (imu_state_mutex && xSemaphoreTake(imu_state_mutex, 1)) {
      imu_state_local = imu_state_var;
      xSemaphoreGive(imu_state_mutex);
      this->angvel = imu_state_local.angvel;
      this->rot = imu_state_local.rot;
    }
    if (sens_fus_mutex && xSemaphoreTake(sens_fus_mutex, 1)) {
      this->pos = sens_fus.position;
      velocity_hist[velocity_hist_index] = sens_fus.velocity;
      velocity_hist_index++;
      velocity_hist_index = velocity_hist_index % 25;
      xSemaphoreGive(sens_fus_mutex);
    }
    this->vel = Eigen::Vector3f::Zero();
    for (int i = 0; i < 25; i++) {
      this->vel += velocity_hist[i] / 25.0f;
    }
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
  }

  void update_input() {

    packet_controller_input c;
    bool input_was_set = false;

    switch (atomic_load(&this->current_input_mode)) {

    case INPUT_TYPE::ACRO: {
      if (controller_input_semaphore &&
          xSemaphoreTake(controller_input_semaphore, 10)) {
        c = current_controller_input;

        xSemaphoreGive(controller_input_semaphore);
        float ly = 0.6 * c.ly + 0.15;

        auto inp = dcont::Input{.joystick = {.throttle_input = ly,
                                             .roll_input = c.rx,
                                             .yaw_input = c.lx,
                                             .pitch_input = -c.ry},
                                .acceleration = {0.0, 0.0, 0.0},
                                .rotation = {0.0, 0.0, 0.0},
                                .velocity = {0.0, 0.0, 0.0},
                                .position = {0.0, 0.0, 0.0},
                                .mode = dcont::ModeInput::Acro};

        dcont::set_input(drone_controller, inp);
        this->last_input = inp;
        input_was_set = true;
      }
    } break;

    case INPUT_TYPE::LEVEL: {
      if (controller_input_semaphore &&
          xSemaphoreTake(controller_input_semaphore, 10)) {
        c = current_controller_input;

        xSemaphoreGive(controller_input_semaphore);

        float ly = 0.6 * c.ly + 0.15;

        auto inp =
            dcont::Input{.joystick = {.throttle_input = ly,
                                      .roll_input = c.rx,
                                      .yaw_input = c.lx,
                                      .pitch_input = -c.ry},
                         .acceleration = {0.0, 0.0, 0.0},
                         .rotation = {-c.ry * 7 * 0.087f, c.rx * 7 * 0.087f,
                                      c.lx * (float)M_PI},
                         .velocity = {0.0, 0.0, 0.0},
                         .position = {0.0, 0.0, 0.0},
                         .mode = dcont::ModeInput::Rotation};

        dcont::set_input(drone_controller, inp);
        this->last_input = inp;
        input_was_set = true;
      }
    } break;

    case INPUT_TYPE::VELOCITY: {
      if (controller_input_semaphore &&
          xSemaphoreTake(controller_input_semaphore, 10)) {
        c = current_controller_input;

        xSemaphoreGive(controller_input_semaphore);

        auto inp = dcont::Input{
            .joystick = {.throttle_input = 0.0,
                         .roll_input = 0.0,
                         .yaw_input = 0.0,
                         .pitch_input = 0.0},
            .acceleration = {c.rx * 6.0f, c.ry * 6.0f, c.ly * 6.0f},
            .rotation = {0.0, 0.0, 0.0},
            .velocity = {c.rx * 5.0f, c.ry * 5.0f, c.ly * 5.0f},
            .position = {0.0, 0.0, 0.0},
            .mode = dcont::ModeInput::Acceleration};

        dcont::set_input(drone_controller, inp);
        this->last_input = inp;
        input_was_set = true;
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

          auto coords = wayp.coords_in_axis.value();

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
          input_was_set = true;
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
    if (!input_was_set) {
      if (atomic_load(&this->current_input_mode) != INPUT_TYPE::AUTO_NAV) {
        auto inp = dcont::Input{.joystick = {.throttle_input = 0.0,
                                             .roll_input = 0.0,
                                             .yaw_input = 0.0,
                                             .pitch_input = 0.0},
                                .acceleration = {0.0, 0.0, 0.0},
                                .rotation = {0.0, 0.0, 0.0},
                                .velocity = {0.0, 0.0, 0.0},
                                .position = {0.0, 0.0, 0.0},
                                .mode = dcont::ModeInput::Acro};

        dcont::set_input(drone_controller, inp);
        this->last_input = inp;
      } else {
        this->drone_cont_stabilize();
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
