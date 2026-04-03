#include "gps.h"
#include "esp_log.h"
#include "sens_fus.h"

static const char *TAG = "GPS_TASK";

void gps_poll_task(void *_) {

  gps = new GPS();
  gps->begin();
  gps_mutex = xSemaphoreCreateMutex();

  ESP_LOGI(TAG, "GPS TASK INIT.");

  while (true) {
    if (xSemaphoreTake(gps_mutex, (TickType_t)1) == pdTRUE) {

      // ESP_LOGI(TAG, "Polling gps.");
      gps->poll();
      if (gps->gps_avaliable() && sens_fus_mutex &&
          xSemaphoreTake(sens_fus_mutex, (TickType_t)2) == pdTRUE) {
        auto vel = gps->velocity().value_or(Eigen::Vector2f::Zero());
        auto coords = gps->get_coordinates();
        if (coords.has_value()) {
          sens_fus.measure_gps(1.0, gps->get_coordinates().value(),
                               Eigen::Vector3f(vel.x(), vel.y(), 0.0));
        }
        xSemaphoreGive(sens_fus_mutex);
      }
      xSemaphoreGive(gps_mutex);
    } else {
      ESP_LOGE(TAG, "FAILED TO GET GPS MUTEX");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
