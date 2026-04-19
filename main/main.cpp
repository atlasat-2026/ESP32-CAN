#include "Eigen/Core"
#include "driver/gpio.h"
#include "drone.h"
#include "drone_comms.h"
#include "drone_controller.h"
#include "esp32-hal.h" #include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>

#include "env_sens.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "packet_handler.h"
#include "radio.h"
#include "sens_fus.h"
#include "servo.h"

static const char *TAG = "MAIN";

#define TIME_RELEASE_QUEUE_TO_ACTIVATION 500

extern "C" void app_main(void) {

  sens_fus_mutex = xSemaphoreCreateMutex();
  configASSERT(sens_fus_mutex);
  nav_mutex = xSemaphoreCreateMutex();

  initArduino();
  gpio_install_isr_service(0);
  Serial.begin(115200);

  xTaskCreatePinnedToCore(radio_task,   // Function name
                          "radio_rxtx", // Name for debugging
                          4096,         // Stack size in bytes
                          NULL,         // Parameters
                          5,            // Priority (higher = more urgent)
                          NULL,         // Task handle
                          0             // Core ID
  );

  xTaskCreatePinnedToCore(env_sens::baro_poll_task, "baro_poll", 8192, NULL, 5,
                          NULL, 0);

  xTaskCreatePinnedToCore(gps_poll_task, "gps_poll", 8192, NULL, 5, NULL, 0);

  // xTaskCreatePinnedToCore(drone_controller_task,   // Function name
  //                         "drone_controller_task", // Name for debugging
  //                         1024 * 32,               // Stack size in bytes
  //                         NULL,                    // Parameters
  //                         20,   // Priority (higher = more urgent)
  //                         NULL, // Task handle
  //                         1     // Core ID
  // );

  xTaskCreatePinnedToCore(motor_throttles_task,   // Function name
                          "motor_throttles_task", // Name for debugging
                          1024 * 4,               // Stack size in bytes
                          NULL,                   // Parameters
                          24,   // Priority (higher = more urgent)
                          NULL, // Task handle
                          1     // Core ID
  );

  setup_imu();
  servo_init();
  ESP_LOGI("MAIN", "All tasks spawned. Main loop free.");

  Eigen::Vector3f local_pos = {0, 0, 0};
  Eigen::Vector3f local_vel = {0, 0, 0};
  bool nav_data_ready = false;

  uint64_t last_print_time = 0;
  uint64_t last_position_broadcast_time = 0;
  uint64_t last_status_broadcast_time = 0;
  uint64_t time_activation_queue = 0;
  bool released = false;

  while (true) {
    while (packet_rx_queue &&
           xQueueReceive(packet_rx_queue, &packet_data[0], 20)) {
      handle_packet(&packet_data[0]);
    }

    if (millis() > last_position_broadcast_time + 200 && packet_tx_queue) {
      send_packet_getter(PACKET_TYPE::INFO_DRONE_POSITION);
      last_position_broadcast_time = millis();
    }
    // auto vel = sens_fus.velocity;
    // ESP_LOGI("GPSvsIMU", "gps_vel: %f, imu_vel: %f",
    //          gps->velocity().value().norm(),
    //          Eigen::Vector3f(vel.x(), vel.y(), 0.0).norm());

    if (millis() > last_status_broadcast_time + 500 && packet_tx_queue) {
      send_packet_getter(PACKET_TYPE::INFO_DRONE_STATUS);
      last_status_broadcast_time = millis();
    }

    if (nav_mutex && xSemaphoreTake(nav_mutex, 10)) {

      if (gps->gps_avaliable()) {
        waypoint current_wayp = nav_man.get_current_waypoint();
        auto pos = sens_fus.position;

        if ((current_wayp.coords_in_axis.value_or(
                 Eigen::Vector3f(INFINITY, INFINITY, INFINITY)) -
             pos)
                .norm() < 1.0) {
          nav_man.waypoint_reached();
        }
      }

      xSemaphoreGive(nav_mutex);
    }

    // Release
    if (imu_state_var.lin_accel_global.z < -8.0 && !released) {
      released = true;
      time_activation_queue = millis();
    }

    if (released && drone_cont &&
        std::atomic_load(&drone_cont->current_input_mode) ==
            INPUT_TYPE::AUTO_NAV &&
        millis() - time_activation_queue > TIME_RELEASE_QUEUE_TO_ACTIVATION) {

      dcont::reset_pid_states(drone_cont->drone_controller);
      servo_set(SERVO_OPTIONS::UP);

      xTaskCreate(
          [](void *pvParameters) {
            vTaskDelay(pdMS_TO_TICKS(700));
            std::atomic_store(&killswitch_active, false);
          },
          "lambda_task", 2048, NULL, 5, NULL);
    }

    // Logging
    if (millis() > last_print_time + 1000) {
      last_print_time = millis();

      std::optional<Eigen::Vector3f> coords;
      float lat, lon, alt;
      bool gps_values = false;
      bool fix = false;
      uint8_t sat_count = 0;
      if (gps_mutex && xSemaphoreTake(gps_mutex, (TickType_t)20) == pdTRUE) {
        if (gps->gps_avaliable()) {
          coords = gps->get_coordinates();
          lat = gps->gps->latitudeDegrees;
          lon = gps->gps->longitudeDegrees;
          alt = gps->gps->altitude;
          gps_values = true;
        }
        sat_count = gps->gps->satellites;
        fix = gps->gps->fix;
        xSemaphoreGive(gps_mutex);

        if (gps_values) {

          ESP_LOGI(TAG,
                   "loc -> lat: %f, long: %f, height: %f, sat_c: %d, fix: %b",
                   lat, lon, alt, sat_count, fix);
        }

        if (coords.has_value()) {
          auto D_pos = coords.value();

          ESP_LOGI(TAG, "    -> D(pos): (%f, %f, %f)", D_pos[0], D_pos[1],
                   D_pos[2]);
        }
      }
      env_sens::dbg_sens();

      if (sens_fus_mutex &&
          xSemaphoreTake(sens_fus_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_pos = sens_fus.position;
        local_vel = sens_fus.velocity;
        nav_data_ready = true;
        xSemaphoreGive(sens_fus_mutex);
      }

      if (nav_data_ready) {
        ESP_LOGI(TAG, "nav(pos): (%f, %f, %f)", local_pos[0], local_pos[1],
                 local_pos[2]);
        ESP_LOGI(TAG, "nav(vel): (%f, %f, %f)", local_vel[0], local_vel[1],
                 local_vel[2]);
      }

      if (motor_throttles != nullptr) {

        ESP_LOGI(TAG, "Throttles: (%f, %f, %f, %f)", motor_throttles[0],
                 motor_throttles[1], motor_throttles[2], motor_throttles[3]);
      }

      if (time_last_controller - millis() < CONNECTION_LOST_THRESHOLD) {

        ESP_LOGI(TAG, "Controller: (%f, %f), (%f, %f)",
                 current_controller_input.lx, current_controller_input.ly,
                 current_controller_input.rx, current_controller_input.ry);
      }
      // ESP_LOGI(TAG, "ROT: (%f, %f, %f)", imu_state_var.rot_euler.x(),
      //          imu_state_var.rot_euler.y(), imu_state_var.rot_euler.z());
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
