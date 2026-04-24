#include "imu.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Esp.h"
#include "drone_controller.h"

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

#define IMU_CS GPIO_NUM_3   // Green
#define IMU_MOSI GPIO_NUM_2 // DI - White
#define IMU_RST GPIO_NUM_1  // Red

#define IMU_INT GPIO_NUM_4  // Yellow
#define IMU_MISO GPIO_NUM_5 // SDA - White
#define IMU_SCLK GPIO_NUM_6 // SCL - Green

BNO08x *setup_imu() {
  imu_state *local_state = new imu_state;
  imu_state_mutex = xSemaphoreCreateMutex();

  BNO08x *imu =
      new BNO08x(bno08x_config_t(SPI2_HOST, IMU_MOSI, IMU_MISO, IMU_SCLK,
                                 IMU_CS, IMU_INT, IMU_RST, 2000000, false));

  ESP_LOGI(TAG, "INITIALIZING IMU");
  if (!imu->initialize()) {
    ESP_LOGE(TAG, "BNO08x Init failure.");
    ESP.restart();
  }
  imu->print_product_ids();

  // imu->dynamic_calibration_run_routine();
  imu->dynamic_calibration_autosave_enable();
  imu->dynamic_calibration_enable(BNO08xCalSel::all);

  imu->rpt.rv.enable(10000UL);                   // 100Hz
  imu->rpt.linear_accelerometer.enable(10000UL); // 100Hz
  imu->rpt.cal_gyro.enable(2500UL);              // 400Hz

  imu->register_cb([imu, local_state]() {
    // ESP_LOGI("IMU", "CALLBACK");
    bool needs_updating = false;
    if (sens_fus_mutex == NULL || imu_state_mutex == NULL)
      return;

    if (imu->rpt.rv.has_new_data()) {

      needs_updating = true;

      auto sens_rot = imu->rpt.rv.get_quat();
      auto sens_euler = imu->rpt.rv.get_euler();
      local_state->rot =
          Eigen::Quaternionf(sens_rot.real, sens_rot.i, sens_rot.j, sens_rot.k);
      local_state->rot_euler =
          Eigen::Vector3f(sens_euler.x, sens_euler.y, sens_euler.z);

      // Eigen::Quaternionf q_global_yaw(
      //     Eigen::AngleAxisf(-M_PI / 2.0, Eigen::Vector3f::UnitZ()));
      // local_state->rot = q_global_yaw * local_state->rot;

      local_state->rot.normalize();

      // {.i = sens_rot.i, .j = sens_rot.j, .k = sens_rot.k, .w =
      // sens_rot.real}; local_state->rot_euler = {sens_euler.x, sens_euler.y,
      // sens_euler.z}; ESP_LOGI("IMU", "rot: roll %f, pitch %f, yaw %f",
      //          local_state->rot_euler.x(), local_state->rot_euler.y(),
      //          local_state->rot_euler.z());
    }

    if (imu->rpt.cal_gyro.has_new_data()) {

      needs_updating = true;

      auto cal_gyro = imu->rpt.cal_gyro.get();
      local_state->angvel = {cal_gyro.x, cal_gyro.y, cal_gyro.z};
      // ESP_LOGI("ROT", "angvel_z: %f", cal_gyro.z);
    }

    if (imu->rpt.linear_accelerometer.has_new_data()) {

      needs_updating = true;

      auto sens_accel_lin = imu->rpt.linear_accelerometer.get();
      local_state->lin_accel = {sens_accel_lin.x, sens_accel_lin.y,
                                sens_accel_lin.z};

      // ESP_LOGI("IMU-ac", "lin_accel %f,%f,%f", sens_accel_lin.x,
      //          sens_accel_lin.y, sens_accel_lin.z);

      int64_t current_time = esp_timer_get_time();

      if (local_state->last_time != -1) {

        float dt =
            ((float)(current_time - local_state->last_time)) / 1000000.0f;

        dcont::QuatC quatc{.i = local_state->rot.x(),
                           .j = local_state->rot.y(),
                           .k = local_state->rot.z(),
                           .w = local_state->rot.w()};

        dcont::Vec3C accel_global =
            dcont::apply_rot(&local_state->lin_accel, &quatc);

        local_state->lin_accel_global = accel_global;

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
  return imu;
}
