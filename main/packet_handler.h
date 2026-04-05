#pragma once

#include <cstdint>

#include "drone_comms.h"
#include "freertos/idf_additions.h"

void handle_packet(uint8_t *packet_addr);

inline SemaphoreHandle_t controller_input_semaphore = NULL;
inline packet_controller_input current_controller_input;
