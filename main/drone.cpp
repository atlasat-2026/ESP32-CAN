#include "drone.h"

#include "DShotRMT.h"
#include "Eigen/Core"
#include "driver/rmt_tx.h"
#include "drone_comms.h"
#include "drone_controller.h"

#include "dshot_definitions.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "imu.h"
#include "nav.h"
#include "packet_handler.h"
#include "sens_fus.h"
#include "soc/gpio_num.h"
#include <cstdint>
#include <cstring>
#include <optional>

#define CONTROLLER_TASK_FREQUENCY 1000.0;

dcont::ControllerConfig default_config() {
  dcont::ControllerConfig config;

  // 1. Configure the PID Stack
  // Note: kp, ki, kd are arrays of 3 [roll, pitch, yaw]

  // Position Loop (Position -> Velocity)
  config.stack.position_pid = {
      {1.0f, 1.0f, 1.0f}, // kp
      {0.0f, 0.0f, 0.0f}, // ki
      {0.0f, 0.0f, 0.0f}, // kd
      5.0f                // frequency (Hz)
  };

  // Velocity Loop (Velocity -> Acceleration/Tilt)
  config.stack.linvel_pid = {
      {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 15.0f};
  // Rotation Loop (Angle -> Angular Rate)
  config.stack.rotation_pid = {
      {4.0f, 4.0f, 4.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 50.0f};

  // Rate Loop (Angular Rate -> Torque) - The "Inner" Loop
  config.stack.rate_pid = {{0.1f, 0.1f, 1.0f},
                           {0.01f, 0.01f, 0.01f},
                           {0.001f, 0.001f, 0.001f},
                           100.0f};
  // 2. Set Constraints
  config.stack.max_rate = 3.14f;   // ~180 degrees/s
  config.stack.max_linvel = 10.0f; // 10 m/s

  // 3. Physical Drone Properties
  config.mass = 0.350f;     // kg
  config.max_thrust = 2.6f; // Newtons
  config.max_torque = 0.5f; // Nm
  /*
    * roll, pitch, yaw
      {1.0f, -1.0f, 1.0f},   // Front Right
      {-1.0f, -1.0f, -1.0f}, // Front Left
      {-1.0f, 1.0f, 1.0f},   // Rear Left
      {1.0f, 1.0f, -1.0f}    // Rear Right
   */

  float mixer[4][3] = {
      {1.0f, -1.0f, 1.0f},   // Front Right
      {-1.0f, -1.0f, -1.0f}, // Front Left
      {-1.0f, 1.0f, 1.0f},   // Rear Left
      {1.0f, 1.0f, -1.0f}    // Rear Right
  };

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      config.motor_map[i][j] = mixer[i][j];
    }
  }

  return config;
}

void setup_drone() { drone_controller = dcont::create(default_config()); }

void drone_cont_stabilize() {
  // TODO: Implement stabilization. if |angvel| > something, we should make it 0
  // first, then if falling fast, pull rotation to the side, then when falling
  // is stabilized, turn slowly up until pointing up. let velocity dissipate
  // afterwards
}

void drone_controller_task(void *params) {
  setup_drone();
  uint8_t wait_ms = 1000.0 / CONTROLLER_TASK_FREQUENCY;

  imu_state imu_state_local;
  Eigen::Vector3f position_local = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_local = Eigen::Vector3f::Zero();

  while (true) {
    if (imu_state_mutex && xSemaphoreTake(imu_state_mutex, 1)) {
      imu_state_local = imu_state_var;
      xSemaphoreGive(imu_state_mutex);
    }
    if (sens_fus_mutex && xSemaphoreTake(sens_fus_mutex, 1)) {
      position_local = sens_fus.position;
      velocity_local = sens_fus.velocity;
      xSemaphoreGive(sens_fus_mutex);
    }

    dcont::set_cur_time(drone_controller, millis() / 1000.0f);
    dcont::set_cur_angvel(drone_controller, v3f_to_v3c(imu_state_local.angvel));
    dcont::set_cur_linvel(drone_controller, v3f_to_v3c(velocity_local));
    dcont::set_cur_pos(drone_controller, v3f_to_v3c(position_local));
    dcont::set_cur_rot(drone_controller, imu_state_local.rot);

    packet_controller_input cont_input;
    if (current_input_mode == dcont::ModeInput::Acro &&
        xSemaphoreTake(controller_input_semaphore, 10)) {
      cont_input = current_controller_input;

      xSemaphoreGive(controller_input_semaphore);
    }

    waypoint wayp;
    if (current_input_mode == dcont::ModeInput::Position &&
        xSemaphoreTake(nav_mutex, 10)) {
      wayp = nav_man.get_current_waypoint();

      xSemaphoreGive(nav_mutex);
    }
    if (current_input_mode == dcont::ModeInput::Position &&
        wayp.coords_in_axis == std::nullopt) {
      drone_cont_stabilize();
    } else {
      auto coords = wayp.coords_in_axis.value_or(Eigen::Vector3f::Zero());

      dcont::set_input(drone_controller,
                       dcont::Input{{cont_input.ly, cont_input.lx,
                                     cont_input.rx, cont_input.ry},
                                    {0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0},
                                    {coords.x(), coords.y(), coords.z()},
                                    current_input_mode});
    }

    memcpy(dcont::get_throttles(drone_controller).values, motor_throttles,
           sizeof(motor_throttles));

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
  }
}

const gpio_num_t motor_pins[4] = {GPIO_NUM_46, GPIO_NUM_16, GPIO_NUM_14,
                                  GPIO_NUM_15};

// const bool reversed[4] = {false, true, false, false};

DShotRMT *motors[4];
void motor_throttles_task(void *params) {

  for (int i = 0; i < 4; i++) {
    motors[i] = new DShotRMT(motor_pins[i], DSHOT300, false);
    motors[i]->begin();
  }

  // ARM
  unsigned long armTime = millis();
  while (millis() - armTime < 5000) {
    for (int i = 0; i < 4; i++) {
      motors[i]->sendThrottlePercent(0);
    }
    vTaskDelay(1);
  }

  while (true) {
    for (int i = 0; i < 4; i++) {
      motors[i]->sendThrottlePercent(motor_throttles[i] * 100.0f);
    }
    vTaskDelay(1);
  }
}
