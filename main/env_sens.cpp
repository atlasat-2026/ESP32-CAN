#include "env_sens.h"
#include "esp_log.h"
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Wire.h>

#include "freertos/idf_additions.h"
#include "sens_fus.h"

#define SEALEVELPRESSURE_HPA (1030)

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

static const constexpr char *TAG = "BARO";

namespace env_sens {
void setup() {

  baro_mutex = xSemaphoreCreateMutex();

  if (!bme.begin()) {

    ESP_LOGE(TAG, "Couldn't find a valid sensor");
    return;
  }
  ESP_LOGI(TAG, "BARO SETUP COMPLETE.");
  bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X8,
                  Adafruit_BME280::SAMPLING_X8, Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_20);

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
      (((std::pow((seaLevelPressure / pressure), (1.0 / 5.257))) - 1.0) *
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

void baro_poll_task(void *_) {
  env_sens::setup();

  float last_alt = env_sens::get_altitude();
  uint32_t last_time = xTaskGetTickCount();

  float filtered_alt = last_alt;
  const float alt_lpf = 0.1f;

  while (true) {
    uint32_t now = xTaskGetTickCount();
    float dt = (now - last_time) * portTICK_PERIOD_MS / 1000.0f;

    if (dt > 0.001f) { // Prevent division by zero
      float current_alt = env_sens::get_altitude();

      filtered_alt = (alt_lpf * current_alt) + (1.0f - alt_lpf) * filtered_alt;

      float v_z = (filtered_alt - last_alt) / dt;

      Eigen::Vector3f baro_pos = sens_fus.position;
      baro_pos.z() = filtered_alt;

      Eigen::Vector3f baro_vel = sens_fus.velocity;
      baro_vel.z() = v_z;
      if (sens_fus_mutex &&
          xSemaphoreTake(sens_fus_mutex, (TickType_t)20) == pdTRUE) {

        // Update the filter with Baro data
        sens_fus.measure_baro(dt, baro_pos, baro_vel);

        xSemaphoreGive(sens_fus_mutex);
      }

      last_alt = filtered_alt;
      last_time = now;
    }

    // BME280 in your config has a 10ms standby, so 20ms-50ms poll is ideal
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

} // namespace env_sens
