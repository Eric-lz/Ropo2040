#pragma once
#include "pico/stdlib.h"

#define PULSES_PER_TURN 985.6

// PWM pin struct
typedef struct {
    uint slice_num;
    uint chan;
    uint freq;
    uint duty;
    uint32_t wrap;
} PWM_Pin_t;

// Initialize PWM pin
void pwm_init(PWM_Pin_t *pwm, uint pin, uint freq, uint duty);

// Set PWM duty cycle
void pwm_set_duty(PWM_Pin_t *pwm, uint duty);

// Turn wheel at a constant speed with PID controller
void const_speed_task(void *pvParameters);

// Turn motor one full revolution
void oneturnpid_task(void *pvParameters);

// Set PWM frequency and duty cycle
// uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t freq, int duty);

// Turn motor one full revolution
// void oneturn_task(void *pvParameters);

// PWM test
//void pwm_task(void *pvParameters);
