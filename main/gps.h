#pragma once

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "HardwareSerial.h"

#include "esp_log.h"
#include <Adafruit_GPS.h>
#include <cmath>
#include <optional>

const float TO_RAD = M_PI / 180.0f;
const float KNOTS_TO_M_SEC = 0.5144444f;
const float earth_radius = 6371000.0f;

#define GPS_RX_PIN 18
#define GPS_TX_PIN 17

struct GPS {
  Adafruit_GPS *gps;
  float origin_lat;
  float origin_long;

  GPS() {

    origin_lat = INFINITY;
    origin_long = INFINITY;
  }

  void begin() {
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    this->gps = new Adafruit_GPS(&Serial2);
    // this->gps->begin(9600);
    this->gps->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    this->gps->sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    this->gps->sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  }

  bool gps_avaliable() { return this->gps->fix; }

  std::optional<Eigen::Vector2f> velocity() {
    if (!this->gps->fix) {
      return std::nullopt;
    }
    float speed = this->gps->speed * KNOTS_TO_M_SEC;
    float angle = this->gps->angle * TO_RAD;

    return Eigen::Vector2f(cos(M_PI_2 - angle), sin(M_PI_2 - angle)) * speed;
  }

  std::optional<Eigen::Vector3f>
  waypoint_to_xyz(float latitude, float longitude, float height) {
    if (this->origin_lat == INFINITY || this->origin_long == INFINITY) {
      if (this->gps_avaliable()) {
        this->origin_lat = this->gps->latitudeDegrees;
        this->origin_long = this->gps->longitudeDegrees;
      } else {
        return std::nullopt;
      }
    }

    float dLat = (latitude - this->origin_lat) * TO_RAD;
    float dLon = (longitude - this->origin_long) * TO_RAD;

    float meanLat = (this->origin_lat) * TO_RAD;

    float y = dLat * earth_radius;
    float x = dLon * earth_radius * std::cos(meanLat);
    float z = this->gps->altitude;

    return Eigen::Vector3f(x, y, z);
  }

  std::optional<Eigen::Vector3f> get_coordinates() {
    if (this->gps->fix == false) {
      return std::nullopt;
    }
    float latitude = this->gps->latitudeDegrees;
    float longitude = this->gps->longitudeDegrees;

    return this->waypoint_to_xyz(latitude, longitude, this->gps->altitude);
  }

  void poll() {
    char c = this->gps->read();
    if (this->gps->newNMEAreceived()) {
      char *line = this->gps->lastNMEA();
      ESP_LOGI("GPS", "NMEA LINE: %s", line);
      this->gps->parse(line);
    }
  }
};

inline SemaphoreHandle_t gps_mutex;
inline GPS *gps = nullptr;

void gps_poll_task(void *_);
