
#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include "Eigen/Core"

#include "driver/gpio.h"
#include "drone.h"
#include "drone_comms.h"
#include "esp32-hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <cstddef>
#include <cstdint>
#include <optional>

#include "env_sens.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "packet_handler.h"
#include "radio.h"
#include "sens_fus.h"

static const char *TAG = "MAIN";

extern "C" void app_main(void) {

  sens_fus_mutex = xSemaphoreCreateMutex();
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
  //
  xTaskCreate(env_sens::baro_poll_task, "baro_poll", 8192, NULL, 1, NULL);
  //
  xTaskCreate(gps_poll_task, "gps_poll", 8192, NULL, 5, NULL);
  //
  xTaskCreatePinnedToCore(drone_controller_task,   // Function name
                          "drone_controller_task", // Name for debugging
                          1024 * 32,               // Stack size in bytes
                          NULL,                    // Parameters
                          20,   // Priority (higher = more urgent)
                          NULL, // Task handle
                          1     // Core ID
  );

  // xTaskCreatePinnedToCore(motor_throttles_task,   // Function name
  //                         "motor_throttles_task", // Name for debugging
  //                         1024 * 4,               // Stack size in bytes
  //                         NULL,                   // Parameters
  //                         30,   // Priority (higher = more urgent)
  //                         NULL, // Task handle
  //                         1     // Core ID
  // );

  ESP_LOGI("MAIN", "All tasks spawned. Main loop free.");

  setup_imu();

  Eigen::Vector3f local_pos = {0, 0, 0};
  Eigen::Vector3f local_vel = {0, 0, 0};
  bool nav_data_ready = false;

  uint64_t last_print_time = 0;
  while (true) {
    while (packet_tx_queue &&
           xQueueReceive(packet_tx_queue, &packet_data[0], 1)) {
      handle_packet(&packet_data[0]);
    }

    if (millis() > last_print_time + 1000) {
      last_print_time = millis();

      std::optional<Eigen::Vector3f> coords;
      float lat, lon, alt;
      if (gps_mutex && xSemaphoreTake(gps_mutex, (TickType_t)10) == pdTRUE) {
        if (gps->gps_avaliable()) {
          coords = gps->get_coordinates();
          lat = gps->gps->latitudeDegrees;
          lon = gps->gps->longitudeDegrees;
          alt = gps->gps->altitude;
        }
        xSemaphoreGive(gps_mutex);

        if (coords.has_value()) {
          auto D_pos = coords.value();

          ESP_LOGI(TAG, "loc -> lat: %f, long: %f, height: %f", lat, lon, alt);
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
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
