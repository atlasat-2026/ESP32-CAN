#pragma once

#include <cstdint>

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include "drone_comms.h"
#include "freertos/idf_additions.h"
#include <Eigen/Dense>

void handle_packet(uint8_t *packet_addr);

inline SemaphoreHandle_t controller_input_semaphore = NULL;
inline packet_controller_input current_controller_input;
