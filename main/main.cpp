
#include "driver/gpio.h"
#include "drone_comms.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "imu.h"
#include <Arduino.h>

#include "env_sens.h"
#include "gps.h"
#include "nav.h"
#include "radio.h"

static const char *TAG = "MAIN";

extern "C" void app_main(void) {

  nav_mutex = xSemaphoreCreateMutex();
  initArduino();
  gpio_install_isr_service(0);
  Serial.begin(115200);

  static imu_state imu_state;
  auto _ = setup_imu(&imu_state);

  // xTaskCreatePinnedToCore(radio_rx_task, // Function name
  //                         "radio_rx",    // Name for debugging
  //                         4096,          // Stack size in bytes
  //                         NULL,          // Parameters
  //                         1,             // Priority (higher = more urgent)
  //                         NULL,          // Task handle
  //                         1              // Core ID
  // );
  //
  xTaskCreate(env_sens::baro_poll_task, "baro_poll", 8192, NULL, 1, NULL);

  xTaskCreate(gps_poll_task, "gps_poll", 8192, NULL, 1, NULL);

  ESP_LOGI("MAIN", "All tasks spawned. Main loop free.");

  Eigen::Vector3f local_pos = {0, 0, 0};
  Eigen::Vector3f local_vel = {0, 0, 0};
  bool nav_data_ready = false;
  while (true) {
    // if (controller_input_semaphore &&
    //     xSemaphoreTake(controller_input_semaphore, (TickType_t)30) == pdTRUE)
    //     {
    //   inp = current_controller_input;
    //   xSemaphoreGive(controller_input_semaphore);
    //
    //   ESP_LOGI(TAG, "CONT INPUT: (%f, %f), (%f,%f)", inp.lx, inp.ly, inp.rx,
    //            inp.ry);
    // }

    if (gps_mutex && xSemaphoreTake(gps_mutex, (TickType_t)10) == pdTRUE) {
      if (gps->gps_avaliable()) {
        ESP_LOGI(TAG, "loc -> lat: %f, long: %f, height: %f",
                 gps->gps->latitudeDegrees, gps->gps->longitudeDegrees,
                 gps->gps->altitude);

        auto D_pos_cond = gps->get_coordinates();
        if (D_pos_cond.has_value()) {
          auto D_pos = D_pos_cond.value();

          ESP_LOGI(TAG, "    -> D(pos): (%f, %f, %f)", D_pos[0], D_pos[1],
                   D_pos[2]);
        }
      }
      xSemaphoreGive(gps_mutex);
    }
    env_sens::dbg_sens();

    if (nav_mutex && xSemaphoreTake(nav_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_pos = nav_filter.position;
      local_vel = nav_filter.velocity;
      nav_data_ready = true;
      xSemaphoreGive(nav_mutex); // RELEASE IMMEDIATELY
    }

    if (nav_data_ready) {
      ESP_LOGI(TAG, "nav(pos): (%f, %f, %f)", local_pos[0], local_pos[1],
               local_pos[2]);
      ESP_LOGI(TAG, "nav(vel): (%f, %f, %f)", local_vel[0], local_vel[1],
               local_vel[2]);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
