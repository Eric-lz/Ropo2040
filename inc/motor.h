#pragma once
#include "pico/stdlib.h"

// Set PWM frequency and duty cycle
uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);

// Turn motor one full revolution
void oneturn_task(void *pvParameters);
void oneturnpid_task(void *pvParameters);

float calculate_pid(float current_pv);