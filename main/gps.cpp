#include "gps.h"
#include "Eigen/Core"
#include "esp_log.h"
#include "sens_fus.h"

static const char *TAG = "GPS_TASK";

void gps_poll_task(void *_) {
  gps = new GPS();
  gps->begin();
  gps_mutex = xSemaphoreCreateMutex();

  ESP_LOGI(TAG, "GPS TASK INIT.");

  while (true) {
    bool has_new_data = false;
    bool available = false;
    Eigen::Vector2f local_vel = Eigen::Vector2f::Zero();
    std::optional<Eigen::Vector3f> local_coords;

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      gps->poll();

      if (gps->gps_avaliable()) {
        available = true;
        local_vel = gps->velocity().value_or(Eigen::Vector2f::Zero());
        local_coords = gps->get_coordinates();
      }
      xSemaphoreGive(gps_mutex);

      has_new_data = local_coords.has_value();
    } else {
      ESP_LOGE(TAG, "FAILED TO GET GPS MUTEX");
    }

    if (has_new_data && available && sens_fus_mutex) {
      if (xSemaphoreTake(sens_fus_mutex, 50) == pdTRUE) {
        sens_fus.measure_gps(
            1.0f, local_coords.value(),
            Eigen::Vector3f(local_vel.x(), local_vel.y(), 0.0f));

        ESP_LOGI(TAG, "applied gps measure to sens_fus");
        xSemaphoreGive(sens_fus_mutex);
      } else {

        ESP_LOGE(TAG, "Sens_fus busy, skipping GPS update.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz polling
  }
}
