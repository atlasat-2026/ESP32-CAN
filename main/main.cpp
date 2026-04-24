#include "Eigen/Core"
#include "driver/gpio.h"
#include "drone.h"
#include "drone_comms.h"
#include "drone_controller.h"
#include "esp32-hal.h"
#include "esp_log.h"
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
#include "logger.h"
#include "nav.h"
#include "packet_handler.h"
#include "portmacro.h"
#include "radio.h"
#include "sens_fus.h"

static const char *TAG = "MAIN";
#define TIME_RELEASE_QUEUE_TO_ACTIVATION 1000

bool first_wayp_was_set = false;

extern "C" void app_main(void) {

  sens_fus_mutex = xSemaphoreCreateMutex();
  nav_mutex = xSemaphoreCreateMutex();

  initArduino();
  gpio_install_isr_service(0);
  Serial.begin(115200);

  xTaskCreatePinnedToCore(logger_task,   // Function name
                          "logger_task", // Name for debugging
                          2048 * 8,      // Stack size in bytes
                          NULL,          // Parameters
                          2,             // Priority (higher = more urgent)
                          NULL,          // Task handle
                          0              // Core ID
  );

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

  xTaskCreate(
      [](void *pvParameters) {
        while (true) {
          while (
              packet_rx_queue &&
              xQueueReceive(packet_rx_queue, &packet_data[0], portMAX_DELAY)) {
            handle_packet(&packet_data[0]);
          }
          vTaskDelay(1);
        }
      },
      "task_recv_task", 8192, NULL, 5, NULL);

  setup_imu();

  xTaskCreatePinnedToCore(drone_controller_task,   // Function name
                          "drone_controller_task", // Name for debugging
                          1024 * 32,               // Stack size in bytes
                          NULL,                    // Parameters
                          20,   // Priority (higher = more urgent)
                          NULL, // Task handle
                          0     // Core ID
  );

  ESP_LOGI("MAIN", "All tasks spawned. Main loop free.");

  Eigen::Vector3f local_pos = {0, 0, 0};
  Eigen::Vector3f local_vel = {0, 0, 0};
  bool sens_fus_data_ready = false;

  uint64_t last_print_time = 0;
  uint64_t last_position_broadcast_time = 0;
  uint64_t last_status_broadcast_time = 0;
  uint64_t time_activation_queue = 0;
  bool released = false;
  bool active = false;

  while (true) {

    if (millis() > last_position_broadcast_time + 100 && packet_tx_queue) {
      send_packet_getter(PACKET_TYPE::INFO_DRONE_POSITION);
      last_position_broadcast_time = millis();
    }

    if (millis() > last_status_broadcast_time + 5000 && packet_tx_queue) {
      send_packet_getter(PACKET_TYPE::INFO_DRONE_STATUS);
      for (uint8_t i = 0; i < 8; i++) {
        send_packet_getter(
            (PACKET_TYPE)(PACKET_TYPE::DRONE_NAV_WAYPOINT_0 + i));
      }
      last_status_broadcast_time = millis();
    }

    if (nav_mutex && xSemaphoreTake(nav_mutex, 10)) {

      if (gps_mutex && xSemaphoreTake(gps_mutex, 10)) {

        if (gps && gps->gps_avaliable()) {
          waypoint current_wayp = nav_man.get_current_waypoint();
          auto pos = sens_fus.position;

          if ((current_wayp.coords_in_axis.value_or(
                   Eigen::Vector3f(INFINITY, INFINITY, INFINITY)) -
               pos)
                  .norm() < 2.0) {
            nav_man.waypoint_reached();
          }
        }
        xSemaphoreGive(gps_mutex);
      }

      xSemaphoreGive(nav_mutex);
    }

    // Release
    if (xSemaphoreTake(imu_state_mutex, 1)) {

      if (imu_state_var.lin_accel_global.z < -7.0 && !released) {
        released = true;
        time_activation_queue = millis();
      }
      xSemaphoreGive(imu_state_mutex);
    }

    if (released && drone_cont &&
        std::atomic_load(&drone_cont->current_input_mode) ==
            INPUT_TYPE::AUTO_NAV &&
        millis() - time_activation_queue > TIME_RELEASE_QUEUE_TO_ACTIVATION &&
        !active) {

      dcont::reset_pid_states(drone_cont->drone_controller);

      if (xSemaphoreTake(imu_state_mutex, 100)) {

        if (imu_state_var.lin_accel_global.z < -7.0) {
          active = true;
          xTaskCreate(
              [](void *pvParameters) {
                vTaskDelay(pdMS_TO_TICKS(100));
                std::atomic_store(&killswitch_active, false);
                vTaskDelete(NULL);
              },
              "lambda_task", 4096, NULL, 5, NULL);
        } else {
          released = false;
        }
        xSemaphoreGive(imu_state_mutex);
      }
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

          if (!first_wayp_was_set) {
            first_wayp_was_set = true;
            xTaskCreate(
                [](void *pvParameters) {
                  Eigen::Vector3f coords = Eigen::Vector3f::Zero();
                  vTaskDelay(10000);
                  int count = 0;
                  for (int i = 0; i < 10; i++) {
                    vTaskDelay(4000);

                    if (gps_mutex &&
                        xSemaphoreTake(gps_mutex, (TickType_t)20) == pdTRUE) {

                      auto lat = gps->gps->latitudeDegrees;
                      auto lon = gps->gps->longitudeDegrees;
                      coords +=
                          Eigen::Vector3f(lat, lon, sens_fus.position.z());
                      count++;

                      xSemaphoreGive(gps_mutex);
                    } else {
                      ESP_LOGE("FIRST_WAYP", "FAILED TO GET MUTEX ON AVG");
                    }
                  }
                  coords = coords / (float)count;

                  if (gps_mutex &&
                      xSemaphoreTake(gps_mutex, (TickType_t)20) == pdTRUE) {

                    gps->origin_lat = coords.x();
                    gps->origin_long = coords.y();
                    xSemaphoreGive(gps_mutex);
                  } else {
                    ESP_LOGE("FIRST_WAYP", "FAILED TO GET MUTEX ON ORIGIN");
                  }

                  if (nav_mutex && xSemaphoreTake(nav_mutex, 1000)) {

                    nav_man.waypoints[0].coords = coords;
                    nav_man.current_waypoint = 0;
                    nav_man.waypoints[0].active = true;
                    nav_man.waypoints[0].landing = true;
                    xSemaphoreGive(nav_mutex);
                  } else {
                    ESP_LOGE("FIRST_WAYP", "FAILED TO GET NAV MUTEX");
                    first_wayp_was_set = false;
                  }

                  vTaskDelete(NULL);
                },
                "lambda_task_gps_init", 8192, NULL, 5, NULL);
          }
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
        sens_fus_data_ready = true;
        xSemaphoreGive(sens_fus_mutex);
      }

      auto wayp = nav_man.get_current_waypoint().coords;
      auto wayp_axis = nav_man.get_current_waypoint().coords_in_axis.value_or(
          Eigen::Vector3f::Zero());
      ESP_LOGI(TAG, "wayp(pos): (%f, %f, %f)", wayp.x(), wayp.y(), wayp.z());
      ESP_LOGI(TAG, "wayp_axis(pos): (%f, %f, %f)", wayp_axis.x(),
               wayp_axis.y(), wayp_axis.z());

      if (sens_fus_data_ready) {
        ESP_LOGI(TAG, "sens_fus(pos): (%f, %f, %f)", local_pos[0], local_pos[1],
                 local_pos[2]);
        ESP_LOGI(TAG, "sens_fus(vel): (%f, %f, %f)", local_vel[0], local_vel[1],
                 local_vel[2]);
      }

      if (motor_throttles != nullptr) {
        ESP_LOGI(TAG, "Throttles: (%f, %f, %f, %f), kill: %d",
                 motor_throttles[0], motor_throttles[1], motor_throttles[2],
                 motor_throttles[3], std::atomic_load(&killswitch_active));
      }

      if (xSemaphoreTake(controller_input_semaphore, 10)) {
        ESP_LOGI(TAG, "Controller: (%f, %f), (%f, %f)",
                 current_controller_input.lx, current_controller_input.ly,
                 current_controller_input.rx, current_controller_input.ry);
        xSemaphoreGive(controller_input_semaphore);
      }
      ESP_LOGI(TAG, "ROT: (%f, %f, %f)", imu_state_var.rot_euler.x(),
               imu_state_var.rot_euler.y(), imu_state_var.rot_euler.z());
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
