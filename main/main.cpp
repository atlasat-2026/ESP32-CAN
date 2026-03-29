
#include "esp32-hal.h"
#include "esp_log.h"
#include <cmath>
#include <cstdio>

#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "gps.h"

static const constexpr char *TAG = "Main";

SemaphoreHandle_t gps_mutex;
GPS *gps = nullptr;

void gps_poll_task(void *_) {
  while (true) {
    if (xSemaphoreTake(gps_mutex, (TickType_t)10) == pdTRUE) {
      gps->poll();
      xSemaphoreGive(gps_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void gps_poll_init() {
  gps_mutex = xSemaphoreCreateMutex();
  if (gps_mutex != NULL) {
    xTaskCreate(gps_poll_task, "Writer", 2048, NULL, 1, NULL);
  }
}

extern "C" void app_main(void) {
  initArduino();
  gps = new GPS();
  gps_poll_init();

  while (true) {
    if (xSemaphoreTake(gps_mutex, (TickType_t)10) == pdTRUE) {
      ESP_LOGI(TAG, "loc -> lat: %f, long: %f, height: %f",
               gps->gps.latitudeDegrees, gps->gps.longitudeDegrees,
               gps->gps.altitude);
      if (gps->gps_avaliable()) {

        auto D_pos = gps->get_coordinates().value();
        ESP_LOGI(TAG, "    -> D(pos): (%f, %f, %f)", D_pos[0], D_pos[1],
                 D_pos[2]);
      }
      xSemaphoreGive(gps_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
