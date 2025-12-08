#pragma once
#include "pico/stdlib.h"

// Configuration for one PID controller
typedef struct {
    // Tuning Constants
    float Kp;  // Proportional Gain
    float Ki;  // Integral Gain
    float Kd;  // Derivative Gain

    // Memory Variables
    float prev_error;      // For calculating D term
    float integral_sum;    // For calculating I term

    // Output Limits
    float output_min;      // e.g., 0 (PWM off)
    float output_max;      // e.g., 65535 (PWM full speed)
    float max_integral;    // Anti-windup limit
    
} PID_Controller_t;

// Function Prototypes
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float min_pwm, float max_pwm);
void pid_reset(PID_Controller_t *pid);
uint16_t pid_calculate(PID_Controller_t *pid, float setpoint, float measurement);