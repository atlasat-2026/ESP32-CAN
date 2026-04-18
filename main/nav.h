#pragma once

#include "drone_controller.h"

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
  bool landing = false;
};

struct drone_nav {
  waypoint waypoints[WAYPOINT_COUNT + 1];
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

  void waypoint_reached() {
    waypoint wayp = waypoints[this->current_waypoint];
    if (wayp.landing) {
      this->waypoints[8] = waypoint{
          .coords = Eigen::Vector3f(wayp.coords.x(), wayp.coords.y(), 0.0f),
          .coords_in_axis = std::nullopt};
      current_waypoint = 8;

      return;
    }

    int i = this->current_waypoint + 1;
    while (i != this->current_waypoint) {
      bool active = waypoints[i].active;
      if (active) {
        this->current_waypoint = i;
      }
      i++;
      i = i % 8;
    }
  }
};

inline SemaphoreHandle_t nav_mutex = NULL;
inline drone_nav nav_man;
