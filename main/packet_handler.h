#pragma once

#include <cstdint>

#include "drone_comms.h"
#include "esp32-hal.h"
#include "freertos/idf_additions.h"

void handle_packet(uint8_t *packet_addr);

void send_packet_getter(PACKET_TYPE requested_type);

inline SemaphoreHandle_t controller_input_semaphore = NULL;
inline packet_controller_input current_controller_input;

inline uint64_t time_last_controller = 0;
