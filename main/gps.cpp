#include "gps.h"
#include "esp_log.h"
#include "nav.h"

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
      if (gps->gps_avaliable() && nav_mutex &&
          xSemaphoreTake(nav_mutex, (TickType_t)2) == pdTRUE) {
        auto vel = gps->velocity().value_or(Eigen::Vector2f::Zero());
        auto coords = gps->get_coordinates();
        if (coords.has_value()) {
          nav_filter.measure_gps(1.0, gps->get_coordinates().value(),
                                 Eigen::Vector3f(vel.x(), vel.y(), 0.0));
        }
        xSemaphoreGive(nav_mutex);
      }
      xSemaphoreGive(gps_mutex);
    } else {
      ESP_LOGE(TAG, "FAILED TO GET GPS MUTEX");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
