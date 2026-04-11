#include "imu.h"
#include "Esp.h"

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

#include "esp_log.h"
#include "esp_timer.h"
#include "hal/spi_types.h"
#include "sens_fus.h"

#include "freertos/idf_additions.h"

static const char *TAG = "IMU";

void setup_imu() {
  imu_state *local_state = new imu_state;
  imu_state_mutex = xSemaphoreCreateMutex();

  BNO08x *imu = new BNO08x(bno08x_config_t(
      SPI2_HOST, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_25, // TODO: FIX
      GPIO_NUM_33, GPIO_NUM_36, GPIO_NUM_32, 2000000, false));

  if (!imu->initialize()) {
    ESP_LOGE(TAG, "BNO08x Init failure.");
    ESP.restart();
  }

  imu->dynamic_calibration_autosave_enable();
  imu->dynamic_calibration_enable(BNO08xCalSel::all);

  imu->rpt.rv_game.enable(2500UL);
  imu->rpt.linear_accelerometer.enable(2500UL);
  imu->rpt.cal_gyro.enable(2500UL);

  imu->register_cb([imu, local_state]() {
    bool needs_updating = false;
    if (sens_fus_mutex == NULL || imu_state_mutex == NULL)
      return;

    if (imu->rpt.rv_game.has_new_data()) {

      needs_updating = true;

      auto sens_rot = imu->rpt.rv_game.get_quat();
      auto sens_euler = imu->rpt.rv_game.get_euler();
      local_state->rot = {sens_rot.i, sens_rot.j, sens_rot.k, sens_rot.real};
      local_state->rot_euler = {sens_euler.x, sens_euler.y, sens_euler.z};
    }

    if (imu->rpt.cal_gyro.has_new_data()) {

      needs_updating = true;

      auto cal_gyro = imu->rpt.cal_gyro.get();
      local_state->angvel = {cal_gyro.x, cal_gyro.y, cal_gyro.z};
    }

    if (imu->rpt.linear_accelerometer.has_new_data()) {

      needs_updating = true;

      auto sens_accel = imu->rpt.linear_accelerometer.get();
      local_state->accel = {sens_accel.x, sens_accel.y, sens_accel.z};
      int64_t current_time = esp_timer_get_time();

      if (local_state->last_time != -1) {

        float dt =
            ((float)(current_time - local_state->last_time)) / 1000000.0f;
        dcont::Vec3C accel_global =
            dcont::apply_rot(&local_state->accel, &local_state->rot);

        if (xSemaphoreTake(sens_fus_mutex, (TickType_t)2) == pdTRUE) {

          // ESP_LOGE(TAG, "accel: (%f, %f, %f), dt: %f", accel_global.x,
          //          accel_global.y, accel_global.z, dt);

          sens_fus.predict(dt, Eigen::Vector3f(accel_global.x, accel_global.y,
                                               accel_global.z));

          // ESP_LOGE(TAG, "vel: (%f, %f, %f)", sens_fus.velocity.x(),
          // sens_fus.velocity.z(), sens_fus.velocity.y());
          xSemaphoreGive(sens_fus_mutex);
        } else {
          ESP_LOGE(TAG, "Failed to get sens_fus mutex.");
        }
      }
      local_state->last_time = current_time;
    }

    if (needs_updating) {
      if (xSemaphoreTake(imu_state_mutex, 0)) {
        imu_state_var = *local_state;
        xSemaphoreGive(imu_state_mutex);
      }
    }
  });

  ESP_LOGI(TAG, "IMU Setup Success.");
}
