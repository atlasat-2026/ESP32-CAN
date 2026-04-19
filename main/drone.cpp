#include "drone.h"

#include "DShotRMT.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "driver/rmt_tx.h"
#include "drone_comms.h"
#include "drone_controller.h"

#include "dshot_definitions.h"
#include "esp32-hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "imu.h"
#include "nav.h"
#include "packet_handler.h"
#include "sens_fus.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdlib.h>

#define CONTROLLER_TASK_FREQUENCY 1000.0;

dcont::ControllerConfig default_config() {
  dcont::ControllerConfig config;

  // 1. Configure the PID Stack
  // Note: kp, ki, kd are arrays of 3 [roll, pitch, yaw]

  // Position Loop (Position -> Velocity)
  config.stack.position_pid = {
      .kp = {1.0f, 1.0f, 1.0f}, // kp
      .ki = {0.0f, 0.0f, 0.0f}, // ki
      .kd = {0.0f, 0.0f, 0.0f}, // kd
      .frequency = 25.0f        // frequency (Hz)
  };

  // Velocity Loop (Velocity -> Acceleration/Rotation)
  config.stack.linvel_pid = {.kp = {1.0f, 1.0f, 1.0f},
                             .ki = {0.01f, 0.01f, 0.01f},
                             .kd = {0.0f, 0.0f, 0.0f},
                             .frequency = 50.0f};
  // Rotation Loop (Rotation/Accel -> Angular Rate)
  config.stack.rotation_pid = {.kp = {4.0f, 4.0f, 4.0f},
                               .ki = {1.0f, 1.0f, 1.0f},
                               .kd = {0.0f, 0.0f, 0.0f},
                               .frequency = 200.0f};

  // Rate Loop (Angular Rate -> Torque) - The "Inner" Loop
  config.stack.rate_pid = {.kp = {0.1f, 0.1f, 1.0f},
                           .ki = {0.01f, 0.01f, 0.01f},
                           .kd = {0.001f, 0.001f, 0.001f},
                           .frequency = 1000.0f};
  // 2. Set Constraints
  config.stack.max_rate = 3.14f;  // ~180 degrees/s
  config.stack.max_linvel = 3.0f; // 10 m/s

  // 3. Physical Drone Properties
  config.mass = 0.350f;     // kg
  config.max_thrust = 2.6f; // Newtons
  config.max_torque = 0.5f; // Nm

  float mixer[4][3] = {
      // x, y, z

      {-1.0, -1.0, -1.0}, // Rear Right
      {-1.0, 1.0, 1.0},   // Rear Left
      {1.0, -1.0, 1.0},   // Front Right
      {1.0, 1.0, -1.0},   // Front Left
  };

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      config.motor_map[i][j] = mixer[i][j];
    }
  }

  return config;
}

constexpr uint8_t wait_ms = 1000.0 / CONTROLLER_TASK_FREQUENCY;

void drone_controller_task(void *params) {
  drone_cont = new drone_cont_state;
  drone_cont->init();

  while (true) {
    drone_cont->update();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

const gpio_num_t motor_pins[4] = {GPIO_NUM_46, GPIO_NUM_16, GPIO_NUM_14,
                                  GPIO_NUM_15};

// const gpio_num_t motor_pins[4] = {GPIO_NUM_15, GPIO_NUM_14, GPIO_NUM_46,
//                                   GPIO_NUM_16};

DShotRMT *motors[4];
void motor_throttles_task(void *params) {
  motor_throttles = (float *)malloc(sizeof(float) * 4);

  for (int i = 0; i < 4; i++) {
    motor_throttles[i] = 0.02;
    motors[i] = new DShotRMT(motor_pins[i], DSHOT300, false);
    motors[i]->begin();
  }

  // ARM
  unsigned long armTime = millis();
  while (millis() - armTime < 5000) {
    for (int i = 0; i < 4; i++) {
      motors[i]->sendThrottlePercent(0);
    }
    vTaskDelay(2);
  }

  while (true) {
    for (int i = 0; i < 4; i++) {
      float throttle =
          std::clamp(motor_throttles[i], 0.0f, 1.0f) * 100.0f * 0.3f;
      if (atomic_load(&killswitch_active)) {
        throttle = 0.0;
      }
      motors[i]->sendThrottlePercent(throttle);
    }
    vTaskDelay(2);
  }
}
