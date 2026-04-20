#include "logger.h"

#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

void init_logging_queue() { logQueue = xQueueCreate(10, sizeof(char *)); }

void logger_task(void *pvParameters) {
  char *string_to_log = nullptr;
  while (true) {
    if (xQueueReceive(logQueue, string_to_log, portMAX_DELAY) == pdTRUE) {

      ESP_LOGI("LOGGER", "DCONT_DBG: %s", string_to_log);
      free(string_to_log);
    }
  }
}
