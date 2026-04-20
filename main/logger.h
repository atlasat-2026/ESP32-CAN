#pragma once

#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

inline QueueHandle_t logQueue = nullptr;

void logger_task(void *pvParameters);
