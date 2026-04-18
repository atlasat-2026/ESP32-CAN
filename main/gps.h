#pragma once

#include <cstdint>
#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "HardwareSerial.h"

#include "esp_log.h"
#include <Adafruit_GPS.h> #include <cmath>
#include <ctime>
#include <optional>

const float TO_RAD = M_PI / 180.0f;
const float KNOTS_TO_M_SEC = 0.5144444f;
const float earth_radius = 6371000.0f;

#define GPS_TX_PIN 18
#define GPS_RX_PIN 17

struct GPS {
  Adafruit_GPS *gps;
  float origin_lat;
  float origin_long;

  GPS() {

    origin_lat = INFINITY;
    origin_long = INFINITY;
  }

  uint64_t unix_timestamp_millis() {
    struct tm t;

    t.tm_year = (2000 + this->gps->year) - 1900;
    t.tm_mon = this->gps->month - 1;
    t.tm_mday = this->gps->day;
    t.tm_hour = this->gps->hour;
    t.tm_min = this->gps->minute;
    t.tm_sec = this->gps->seconds;
    t.tm_isdst = 0; // GPS is always UTC, so no Daylight Savings

    // mktime converts struct tm to time_t (Unix timestamp)
    time_t unixTime = mktime(&t);

    // Calculate how many milliseconds have passed since the GPS updated its
    // data
    unsigned long msSinceUpdate = millis() - this->gps->lastDate;

    // Return the base timestamp plus the elapsed seconds
    return (uint64_t)unixTime * 1000 + msSinceUpdate + gps->milliseconds;
  }

  uint64_t unix_timestamp_seconds() {
    return this->unix_timestamp_millis() / 1000;
  }

  void begin() {
    Serial2.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
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
    for (int i = 0; i < 50; i++) {
      char _ = this->gps->read();
    }
    if (this->gps->newNMEAreceived()) {
      char *line = this->gps->lastNMEA();
      // ESP_LOGI("GPS", "NMEA LINE: %s", line);
      this->gps->parse(line);
    }
  }
};

inline SemaphoreHandle_t gps_mutex;
inline GPS *gps = nullptr;

void gps_poll_task(void *_);
