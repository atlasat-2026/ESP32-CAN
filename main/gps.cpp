#include "gps.h"

#include "esp32-hal.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "sens_fus.h"
#include <cstdint>

static const char *TAG = "GPS_TASK";

void gps_poll_task(void *_) {
  gps = new GPS();
  gps->begin();
  gps_mutex = xSemaphoreCreateMutex();

  ESP_LOGI(TAG, "GPS TASK INIT.");

  uint64_t last_time = millis();
  while (true) {
    bool available = false;
    Eigen::Vector2f local_vel = Eigen::Vector2f::Zero();
    std::optional<Eigen::Vector3f> local_coords;

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      gps->poll();

      if (gps->gps_avaliable()) {
        local_vel = gps->velocity().value_or(Eigen::Vector2f::Zero());
        local_coords = gps->get_coordinates();

        available = local_coords.has_value();
      }
      xSemaphoreGive(gps_mutex);

    } else {
      ESP_LOGE(TAG, "FAILED TO GET GPS MUTEX");
    }

    if (sens_fus_mutex) {
      if (available) {

        uint64_t current_time = millis();
        if (xSemaphoreTake(sens_fus_mutex, 50) == pdTRUE) {
          sens_fus.measure_gps(
              (current_time - last_time) / 1000.0f, local_coords.value(),
              Eigen::Vector3f(local_vel.x(), local_vel.y(), 0.0f));

          xSemaphoreGive(sens_fus_mutex);
          last_time = current_time;
        } else {

          ESP_LOGE(TAG, "Sens_fus busy, skipping GPS update.");
        }
      } else {
        if (xSemaphoreTake(sens_fus_mutex, 50) == pdTRUE) {
          sens_fus.gps_lost();
          xSemaphoreGive(sens_fus_mutex);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 10Hz polling
  }
}
