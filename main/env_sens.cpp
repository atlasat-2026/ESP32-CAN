#include "esp_log.h"
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

static const constexpr char *TAG = "IMU";

namespace env_sens {

void setup() {

  if (!bme.begin()) {

    ESP_LOGE(TAG, "Couldn't find a valid sensor");
    return;
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
}

float get_temperature() {
  sensors_event_t temp_event;
  bme_temp->getEvent(&temp_event);
  return temp_event.temperature;
}

float get_pressure() {
  sensors_event_t e;
  bme_pressure->getEvent(&e);
  return e.pressure;
}

float calculateAltitude(float pressure, float seaLevelPressure,
                        float tempCelsius) {
  float altitude =
      (((pow((seaLevelPressure / pressure), (1.0 / 5.257))) - 1.0) *
       (tempCelsius + 273.15)) /
      0.0065;
  return altitude;
}

float get_altitude() {
  return calculateAltitude(get_pressure(), SEALEVELPRESSURE_HPA,
                           get_temperature());
}

void dbg_sens() {
  ESP_LOGI(TAG, "T (ºC): %f, P (hPa): %f, Alt (m): %f", get_temperature(),
           get_pressure(), get_altitude());
}

} // namespace env_sens
