#pragma once

enum SERVO_OPTIONS {
  UP,
  DOWN,
  OFF,
};

void servo_init();

void servo_set(SERVO_OPTIONS opt);
