
#include "BNO08x.hpp"
#include "BNO08xGlobalTypes.hpp"
#include "drone_controller.h"
#include "esp_timer.h"
#include <cstdint>
#include <stdio.h>

static const constexpr char *TAG = "Main";

static struct Vec3C accel = {0, 0, 0};
static struct QuatC rot = {0, 0, 0, 0};
static struct Vec3C pos = {0, 0, 0};
static struct Vec3C vel = {0, 0, 0};
static int64_t last_time = -1;

extern "C" void app_main(void) {
  BNO08x *imu = new BNO08x();

  if (!imu->initialize()) {
    ESP_LOGE(TAG, "Init failure");
    return;
  }

  // Use the pointer in your lambdas
  imu->rpt.rv_game.enable(2500UL);
  imu->rpt.linear_accelerometer.enable(10000UL);
  imu->dynamic_calibration_autosave_enable();

  imu->register_cb([imu]() {
    if (imu->rpt.linear_accelerometer.has_new_data()) {
      auto sens_accel = imu->rpt.linear_accelerometer.get();
      accel = {sens_accel.x, sens_accel.y, sens_accel.z};

      int64_t time = esp_timer_get_time();
      if (last_time != -1) {
        float dt = (time - last_time) / 1000000.0;

        Vec3C init_vel = vel;
        Vec3C delta = apply_rot(&accel, &rot);

        multiply_vec_by(&delta, dt);
        add_to_vec(&vel, &delta);

        delta = init_vel;
        multiply_vec_by(&delta, dt);
        add_to_vec(&pos, &delta);
      }
      last_time = time;
    }
    if (imu->rpt.rv_game.has_new_data()) {
      auto sens_rot = imu->rpt.rv_game.get_quat();
      rot = {sens_rot.i, sens_rot.j, sens_rot.k, sens_rot.real};
    }
  });

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
