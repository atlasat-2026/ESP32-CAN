#pragma once

#include "drone_comms.h"
#include "freertos/idf_additions.h"

inline QueueHandle_t packet_rx_queue = NULL;
inline QueueHandle_t packet_tx_queue = NULL;
inline SemaphoreHandle_t controller_input_semaphore = NULL;
inline packet_controller_input current_controller_input;

void radio_task(void *pvParameters);
