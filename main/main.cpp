#include "HardwareSerial.h"
#include "dshot_utils.h"

#include "esp32-hal.h"
#include "esp_log.h"
#include <cmath>
#include <iostream>

#include "DShotRMT.h"
#include "env_sens.h"

static const constexpr char *TAG = "Main";
const gpio_num_t MOTOR_PIN = GPIO_NUM_22;

DShotRMT motor(MOTOR_PIN, DSHOT300, true);

extern "C" void app_main(void) {
  initArduino();
  printf("beg: %d", motor.begin().result_code);

  ESP_LOGI(TAG, "Arming ESC...");
  // Send 0 throttle for 4 seconds to arm
  unsigned long armTime = millis();
  while (millis() - armTime < 4000) {
    motor.sendThrottlePercent(0);
    // delay(1);
  }
  delay(50);

  ESP_LOGI(TAG, "Motor Armed. Ramping up...");
  // Send 10% throttle for 5 seconds
  unsigned long runTime = millis();
  while (millis() - runTime < 5000) {
    auto res = motor.sendThrottlePercent(10);
    printf("%d, %d\n", res.result_code, res.erpm);
    //
    // ESP_LOGI(TAG, "current: %d",
    // motor.getTelemetry().telemetry_data.current);
    // delay(1);
  }

  ESP_LOGI(TAG, "Stopping motor");
  motor.sendThrottlePercent(0);
}
