#include "servo.h"

#include "driver/ledc.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include <cstdint>
#include <limits>

#define SERVO_PIN 7
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 50

uint32_t us_to_duty(int us) {
  // 20000 is the period in microseconds (50Hz)
  // 8191 is the max duty cycle for 13-bit resolution
  return (uint32_t)((float)us / 20000.0f * 8191.0f);
}

ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                  .duty_resolution = LEDC_DUTY_RES,
                                  .timer_num = LEDC_TIMER,
                                  .freq_hz = LEDC_FREQUENCY,
                                  .clk_cfg = LEDC_AUTO_CLK};

ledc_channel_config_t ledc_channel = {.gpio_num = SERVO_PIN,
                                      .speed_mode = LEDC_MODE,
                                      .channel = LEDC_CHANNEL,
                                      .timer_sel = LEDC_TIMER,
                                      .duty = 0};
void servo_init() {
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
}

#define DUTY_CYCLE_DOWN 2000
#define DUTY_CYCLE_UP 850

void servo_set(SERVO_OPTIONS opt) {
  switch (opt) {
  case UP:
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, us_to_duty(DUTY_CYCLE_UP));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    break;
  case DOWN:
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, us_to_duty(DUTY_CYCLE_DOWN));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
  case OFF:
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
  }
}

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define LIGHT_ADC_CHANNEL ADC_CHANNEL_6

adc_cali_handle_t cali_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle;

void adc_init() {
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
  adc_oneshot_new_unit(&init_config, &adc1_handle);

  adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = ADC_ATTEN_DB_12, // 0-3.3V range
      .bitwidth = ADC_BITWIDTH_12,
  };
  adc_oneshot_config_channel(adc1_handle, LIGHT_ADC_CHANNEL, &chan_cfg);

  adc_cali_curve_fitting_config_t cali_config = {.unit_id = ADC_UNIT_1,
                                                 .atten = ADC_ATTEN_DB_12,
                                                 .bitwidth = ADC_BITWIDTH_12};
  adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
}

int adc_read() {
  int raw_val;
  int voltage_mv;

  adc_oneshot_read(adc1_handle, LIGHT_ADC_CHANNEL, &raw_val);
  adc_cali_raw_to_voltage(cali_handle, raw_val, &voltage_mv);

  return voltage_mv;
}

#define TIME_DARK_IN_ROCKET 30000
#define TIME_LIGHT_FOR_RELEASE 500
#define MAXIMUM_LIGHT_VOLTAGE 1000

bool has_been_in_rocket = false;
bool released = false;

constexpr uint64_t max_u64 = std::numeric_limits<uint64_t>::max();
uint64_t time_light_seen = max_u64;
uint64_t time_dark_seen = max_u64;
bool light_released() {
  bool is_light = adc_read() < MAXIMUM_LIGHT_VOLTAGE;

  if (is_light) {
    if (has_been_in_rocket) {
      if (time_light_seen == max_u64) {
        time_light_seen = millis();
      } else {
        if (millis() - time_light_seen > TIME_LIGHT_FOR_RELEASE) {
          released = true;
        }
      }
    }
    time_dark_seen = max_u64;
  } else {

    if (time_dark_seen == max_u64) {
      time_light_seen = millis();
    } else {
      if (millis() - time_dark_seen > TIME_DARK_IN_ROCKET) {
        has_been_in_rocket = true;
      }
    }
    time_light_seen = max_u64;
  }
}
