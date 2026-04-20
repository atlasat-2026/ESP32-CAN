#include "logger.h"

#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

void logger_task(void *pvParameters) {
  logQueue = xQueueCreate(10, sizeof(char *));

  char *string_to_log = nullptr;

  while (true) {
    if (xQueueReceive(logQueue, &string_to_log, portMAX_DELAY) == pdTRUE) {
      if (string_to_log != nullptr) {
        ESP_LOGI("LOGGER", "DCONT_DBG: %s", string_to_log);

        free(string_to_log);
      }
    }
  }
}
