#include "drone.h"

#include "drone_controller.h"

#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "imu.h"
#include "sens_fus.h"
#include <cstdint>

#define CONTROLLER_TASK_FREQUENCY 400.0;

dcont::ControllerConfig default_config() {
  dcont::ControllerConfig config;

  // 1. Configure the PID Stack
  // Note: kp, ki, kd are arrays of 3 [roll, pitch, yaw]

  // Position Loop (Position -> Velocity)
  config.stack.position_pid = {
      {1.0f, 1.0f, 1.0f}, // kp
      {0.0f, 0.0f, 0.0f}, // ki
      {0.0f, 0.0f, 0.0f}, // kd
      25.0f               // frequency (Hz)
  };

  // Velocity Loop (Velocity -> Acceleration/Tilt)
  config.stack.linvel_pid = {
      {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 50.0f};

  // Rotation Loop (Angle -> Angular Rate)
  config.stack.rotation_pid = {
      {4.0f, 4.0f, 4.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 200.0f};

  // Rate Loop (Angular Rate -> Torque) - The "Inner" Loop
  config.stack.rate_pid = {{0.2f, 0.2f, 2.0f},
                           {0.05f, 0.05f, 0.05f},
                           {0.003f, 0.003f, 0.001f},
                           600.0f};

  // 2. Set Constraints
  config.stack.max_rate = 3.14f;   // ~180 degrees/s
  config.stack.max_linvel = 10.0f; // 10 m/s

  // 3. Physical Drone Properties
  config.mass = 0.350f;     // kg
  config.max_thrust = 2.6f; // Newtons
  config.max_torque = 0.5f; // Nm

  // roll, pitch, yaw for each motor
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

void drone_controller_task(void *params) {
  setup_drone();
  uint8_t wait_ms = 1000.0 / CONTROLLER_TASK_FREQUENCY;

  imu_state imu_state_local;
  Eigen::Vector3f position_local = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_local = Eigen::Vector3f::Zero();

  while (true) {
    if (xSemaphoreTake(imu_state_mutex, 1)) {
      imu_state_local = imu_state_var;
      xSemaphoreGive(imu_state_mutex);
    }
    if (xSemaphoreTake(sens_fus_mutex, 1)) {
      position_local = sens_fus.position;
      velocity_local = sens_fus.velocity;
      xSemaphoreGive(sens_fus_mutex);
    }

    dcont::set_cur_time(drone_controller, millis() / 1000.0f);
    dcont::set_cur_angvel(drone_controller, v3f_to_v3c(imu_state_local.angvel));
    dcont::set_cur_linvel(drone_controller, v3f_to_v3c(velocity_local));
    dcont::set_cur_pos(drone_controller, v3f_to_v3c(position_local));
    dcont::set_cur_rot(drone_controller, imu_state_local.rot);

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
  }
}
