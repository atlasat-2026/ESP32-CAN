#include "BNO08x.hpp"
#include "BNO08xGlobalTypes.hpp"
#include "drone_controller.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdint>
#include <stdio.h>

static const constexpr char *TAG = "Main";

static struct Vec3C accel = {0, 0, 0};
static struct Vec3C last_accel = {0, 0, 0};
static struct QuatC rot = {0, 0, 0, 0};
static struct Vec3C pos = {0, 0, 0};
static struct Vec3C vel = {0, 0, 0};
static int64_t last_time = -1;

static struct Vec3C euler_ang = {0, 0, 0};

extern "C" void app_main(void) {
  BNO08x *imu = new BNO08x();

  if (!imu->initialize()) {
    ESP_LOGE(TAG, "Init failure");
    return;
  }

  imu->dynamic_calibration_autosave_enable();
  imu->dynamic_calibration_enable(BNO08xCalSel::all);

  // Linear accel at 100Hz (10000UL us)
  imu->rpt.rv_game.enable(2500UL);
  imu->rpt.linear_accelerometer.enable(2500UL);
  imu->rpt.rv_game.tare();

  imu->register_cb([imu]() {
    if (imu->rpt.linear_accelerometer.has_new_data()) {
      auto sens_accel = imu->rpt.linear_accelerometer.get();
      accel = {sens_accel.x, sens_accel.y, sens_accel.z};

      int64_t time = esp_timer_get_time();
      if (last_time != -1) {
        float dt = (time - last_time) / 1000000.0f;

        Vec3C delta = apply_rot(&accel, &rot);
        delta.x += last_accel.x;
        delta.y += last_accel.y;
        delta.z += last_accel.z;

        multiply_vec_by(&delta, dt * 0.5f);
        add_to_vec(&vel, &delta);
        last_accel = accel;
      }
      last_time = time;
    }

    if (imu->rpt.rv_game.has_new_data()) {
      auto sens_rot = imu->rpt.rv_game.get_quat();
      auto euler = imu->rpt.rv_game.get_euler();
      euler_ang = {euler.x, euler.y, euler.z};
      rot = {sens_rot.i, sens_rot.j, sens_rot.k, sens_rot.real};
    }
  });

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(100));
    // Print filtered data to see the difference
    printf("accel_filt - %f, %f, %f", accel.x, accel.y, accel.z);
    printf("; vel - %f, %f, %f", vel.x, vel.y, vel.z);
    printf("; euler - %f, %f, %f\n", euler_ang.x, euler_ang.y, euler_ang.z);
    // multiply_vec_by(&vel, 0.99);
  }
}
