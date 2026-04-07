#pragma once
#include "freertos/idf_additions.h"
#include "gps.h"
#include <cmath>
#include <cstdint>
#include <optional>

#define WAYPOINT_COUNT 8

struct waypoint {
  Eigen::Vector3f coords; // lat, lon, alt
  std::optional<Eigen::Vector3f> coords_in_axis = std::nullopt;
  bool active = false; // active or to be skipped
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

  waypoint get_current_waypoint() {
    waypoint wayp = this->waypoints[this->current_waypoint];
    if (!wayp.coords_in_axis.has_value()) {
      auto axis =
          gps->waypoint_to_xyz(wayp.coords[0], wayp.coords[1], wayp.coords[2]);
      wayp.coords_in_axis = axis;
    }
    return wayp;
  }
};

inline SemaphoreHandle_t nav_mutex = NULL;
inline drone_nav nav_man;
