#pragma once

#include "drone_comms.h"
#include "freertos/idf_additions.h"

inline QueueHandle_t packet_rx_queue = NULL;
inline QueueHandle_t packet_tx_queue = NULL;

void radio_task(void *pvParameters);
