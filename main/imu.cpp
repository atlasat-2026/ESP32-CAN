#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "hal/spi_types.h"
#include "sens_fus.h"

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

static const char *TAG = "IMU";

BNO08x *setup_imu(imu_state *state) {
  BNO08x *imu = new BNO08x(
      bno08x_config_t(SPI2_HOST, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_25,
                      GPIO_NUM_33, GPIO_NUM_36, GPIO_NUM_32, 2000000, false));

  if (!imu->initialize()) {
    ESP_LOGE(TAG, "BNO08x Init failure.");
    return nullptr;
  }

  imu->dynamic_calibration_autosave_enable();
  imu->dynamic_calibration_enable(BNO08xCalSel::all);

  imu->rpt.rv_game.enable(2500UL);
  imu->rpt.linear_accelerometer.enable(2500UL);

  imu->register_cb([imu, state]() {
    if (imu->rpt.rv_game.has_new_data()) {
      auto sens_rot = imu->rpt.rv_game.get_quat();
      state->rot = {sens_rot.i, sens_rot.j, sens_rot.k, sens_rot.real};
    }

    if (imu->rpt.linear_accelerometer.has_new_data()) {

      auto sens_accel = imu->rpt.linear_accelerometer.get();
      state->accel = {sens_accel.x, sens_accel.y, sens_accel.z};

      int64_t current_time = esp_timer_get_time();

      if (state->last_time != -1) {
        float dt = ((float)(current_time - state->last_time)) / 1000000.0f;
        Vec3C accel_global = apply_rot(&state->accel, &state->rot);

        if (xSemaphoreTake(sens_fus_mutex, (TickType_t)2) == pdTRUE) {
          sens_fus.predict(dt, Eigen::Vector3f(accel_global.x, accel_global.y,
                                               accel_global.z));
          xSemaphoreGive(sens_fus_mutex);
        } else {
          ESP_LOGE(TAG, "Failed to get mutex.");
        }
      }
      state->last_time = current_time;
    }
  });

  ESP_LOGI(TAG, "IMU Setup Success.");
  return imu;
}
