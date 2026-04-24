#include "drone.h"

#include "DShotRMT.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "driver/rmt_tx.h" #include "drone_comms.h"
#include "drone_controller.h"
#include "dshot_definitions.h"
#include "esp32-hal.h"
#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "esp_timer.h" #include "imu.h" #include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "logger.h"
#include "nav.h"
#include "packet_handler.h"
#include "sens_fus.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdlib.h>

#define CONTROLLER_TASK_FREQUENCY 400.0;

dcont::ControllerConfig default_config() {
  dcont::ControllerConfig config;

  // 1. Configure the PID Stack
  // Note: kp, ki, kd are arrays of 3 [roll, pitch, yaw]

  // Position Loop (Position -> Velocity)
  config.stack.position_pid = {
      .kp = {0.1f, 0.1f, 0.1f}, // kp
      .ki = {0.0f, 0.0f, 0.0f}, // ki
      .kd = {0.0f, 0.0f, 0.0f}, // kd
      .frequency = 5.0f         // frequency (Hz)
  };

  // Velocity Loop (Velocity -> Acceleration/Rotation)
  config.stack.linvel_pid = {.kp = {0.2f, 0.2f, 0.2f},
                             .ki = {0.01f, 0.01f, 0.01f},
                             // .ki = {0.01f, 0.01f, 0.01f},
                             .kd = {0.0f, 0.0f, 0.0f},
                             .integral_cap = {0.1f, 0.1f, 2.0f},
                             .frequency = 10.0f};

  // Rotation Loop (Rotation/Accel -> Angular Rate)
  config.stack.rotation_pid = {
      .kp = {8.0f, 8.0f, 4.0f},
      .ki = {0.2f, 0.2f, 0.2f},
      .kd = {0.0f, 0.0f, 0.0f},
      .integral_cap = {2.0f, 2.0f, 2.0f},
      .frequency = 100.0f,
  };

  // Rate Loop (Angular Rate -> Torque) - The "Inner" Loop
  config.stack.rate_pid = {
      .kp = {0.02f, 0.02f, 0.2},
      // .kp = {0.11f, 0.07f, 0.375},
      // .kp = {0.05f, 0.05f, 2.0f},
      .ki = {0.00f, 0.00f, 0.0f},
      .kd = {0.00f, 0.00f, 0.0f},
      .integral_cap = {1.0f, 1.0f, 1.0f},
      .frequency = 400.0f,
  };

  config.stack.max_rate = 3.14f; // rad/s
  config.stack.max_linvel = 4.0f;
  config.stack.max_accel = 3.0;

  config.mass = 0.325f;     // kg
  config.max_thrust = 1.8f; // Newtons
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

const gpio_num_t motor_pins[4] = {GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_46,
                                  GPIO_NUM_15};
DShotRMT *motors[4];

void drone_controller_task(void *params) {
  drone_cont = new drone_cont_state;
  drone_cont->init();

  motor_throttles = (float *)malloc(sizeof(float) * 4);

  for (int i = 0; i < 4; i++) {
    motor_throttles[i] = 0.0;
    motors[i] = new DShotRMT(motor_pins[i], DSHOT300, false);
    motors[i]->begin();
  }
  unsigned long armTime = millis();
  while (millis() - armTime < 5000) {
    for (int i = 0; i < 4; i++) {
      motors[i]->sendThrottlePercent(0);

      motor_throttles[i] = 0.0;
    }
    vTaskDelay(2);
  }

  while (true) {
    drone_cont->update();
    for (int i = 0; i < 4; i++) {
      float throttle = std::clamp(motor_throttles[i], 0.0f, 0.6f) * 100.0f;
      if (atomic_load(&killswitch_active)) {
        throttle = 0.0;
        dcont::reset_pid_states(drone_cont->drone_controller);
      }
      motors[i]->sendThrottlePercent(std::clamp(throttle, 0.0f, 60.0f));
    }

    vTaskDelay(1);
  }
}
