#pragma once

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "freertos/idf_additions.h"
namespace env_sens {

void setup();

float get_temperature();

float get_pressure();

void dbg_sens();

float get_altitude();
void baro_poll_task(void *_);

} // namespace env_sens

inline SemaphoreHandle_t baro_mutex = NULL;
