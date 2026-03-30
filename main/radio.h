#pragma once

#include "drone_comms.h"
#include "freertos/idf_additions.h"

inline QueueHandle_t packet_queue = NULL;
inline SemaphoreHandle_t controller_input_semaphore = NULL;
inline controller_input current_controller_input;

void radio_rx_task(void *pvParameters);
