
#include "driver/ledc.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "esp_log.h"

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

enum SERVO_OPTIONS {
  UP,
  DOWN,
  OFF,
};

void set_servo(SERVO_OPTIONS opt) {
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
