#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdint>
#include <stdio.h>

#include "env_sens.h"
#include "freertos/idf_additions.h"
#include "imu.h"

static const constexpr char *TAG = "Main";

extern "C" void app_main(void) {
  env_sens::setup();

  imu_state imu_state;
  BNO08x *imu = setup_imu(&imu_state);
  if (imu == nullptr) {
    ESP_LOGE(TAG, "IMU setup failed.");
    return;
  }

  ESP_LOGE(TAG, "IMU setup sucess.");

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(300));
    if (xSemaphoreTake(imuStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ESP_LOGI(TAG, "accel - %f, %f, %f", imu_state.accel.x, imu_state.accel.y,
               imu_state.accel.z);
      ESP_LOGI(TAG, "; euler - %f, %f, %f\n", imu_state.euler_ang.x,
               imu_state.euler_ang.y, imu_state.euler_ang.z);
      xSemaphoreGive(imuStateMutex);
    }
    env_sens::dbg_sens();
  }
}
