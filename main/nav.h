#pragma once

#include "Eigen/Core"
#include "freertos/idf_additions.h"
#include <cstdint>

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#define WAYPOINT_COUNT 8

struct waypoint {
  Eigen::Vector3f coords; // lon, lat, alt
  bool active;            // active or to be skipped
};

struct drone_nav {
  waypoint waypoints[WAYPOINT_COUNT];
  uint8_t current_waypoint;

  void set_active_mask(uint8_t mask) {
    for (int i = 0; i < WAYPOINT_COUNT; i++) {
      this->waypoints[i].active = (mask & (1 << i)) != 0;
    }
  }

  uint8_t get_active_mask() {
    uint8_t mask = 0;
    for (int i = 0; i < WAYPOINT_COUNT; i++) {
      if (waypoints[i].active) {
        mask |= (1 << i);
      }
    }
    return mask;
  }
};

inline SemaphoreHandle_t nav_mutex = NULL;
inline drone_nav nav_man;
