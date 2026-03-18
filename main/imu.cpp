#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"

static const char *TAG = "IMU";

SemaphoreHandle_t imuStateMutex = NULL;

BNO08x *setup_imu(imu_state *state) {
  BNO08x *imu = new BNO08x();

  if (!imu->initialize()) {
    ESP_LOGE(TAG, "BNO08x Init failure.");
    return nullptr;
  }

  if (imuStateMutex == NULL) {
    imuStateMutex = xSemaphoreCreateMutex();
  }

  imu->dynamic_calibration_autosave_enable();
  imu->dynamic_calibration_enable(BNO08xCalSel::all);

  imu->rpt.rv_game.enable(2500UL);
  imu->rpt.linear_accelerometer.enable(2500UL);
  // imu->rpt.rv_game.tare();

  imu->register_cb([imu, state]() {
    if (imu->rpt.rv_game.has_new_data()) {
      auto sens_rot = imu->rpt.rv_game.get_quat();
      auto euler = imu->rpt.rv_game.get_euler();
      if (xSemaphoreTake(imuStateMutex, 0) == pdTRUE) {
        state->euler_ang = {euler.x, euler.y, euler.z};
        state->rot = {sens_rot.i, sens_rot.j, sens_rot.k, sens_rot.real};
        xSemaphoreGive(imuStateMutex);
      }
    }

    if (imu->rpt.linear_accelerometer.has_new_data()) {

      if (xSemaphoreTake(imuStateMutex, 0) == pdTRUE) {
        auto sens_accel = imu->rpt.linear_accelerometer.get();
        state->accel = {sens_accel.x, sens_accel.y, sens_accel.z};

        int64_t current_time = esp_timer_get_time();

        if (state->last_time != -1) {
          float dt = (current_time - state->last_time) / 1000000.0f;

          Vec3C delta = apply_rot(&state->accel, &state->rot);

          // Trapezoidal Integration for Velocity: v = v0 + 0.5 * (a + a0) * dt
          delta.x += state->last_accel.x;
          delta.y += state->last_accel.y;
          delta.z += state->last_accel.z;

          multiply_vec_by(&delta, dt * 0.5f);
          add_to_vec(&state->vel, &delta);

          state->last_accel = state->accel;
        }
        state->last_time = current_time;

        xSemaphoreGive(imuStateMutex);
      }
    }
  });

  ESP_LOGI(TAG, "IMU Setup Success.");
  return imu;
}
