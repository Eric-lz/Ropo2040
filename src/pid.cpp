#include "pid.h"

#include <stdio.h>

// Initialize the PID struct with safe defaults
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float min_pwm, float max_pwm) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->output_min = min_pwm;
    pid->output_max = max_pwm;
    
    // Set integral limit to something reasonable (e.g., 50-80% of max output)
    pid->max_integral = 1000.0f; 

    pid_reset(pid);
}

// Reset memory (Call this when the robot stops or changes direction!)
void pid_reset(PID_Controller_t *pid) {
    pid->prev_error = 0.0f;
    pid->integral_sum = 0.0f;
}

// The Main Calculation
uint16_t pid_calculate(PID_Controller_t *pid, float setpoint, float measurement) {
    // 1. Calculate Error
    float error = setpoint - measurement;

    // 2. Proportional Term (P)
    float p_out = pid->Kp * error;

    // 3. Integral Term (I)
    pid->integral_sum += error;

    // --- Anti-Windup: Clamp the integral sum ---
    if (pid->integral_sum > pid->max_integral) {
        pid->integral_sum = pid->max_integral;
    } else if (pid->integral_sum < -pid->max_integral) {
        pid->integral_sum = -pid->max_integral;
    }
    
    float i_out = pid->Ki * pid->integral_sum;

    // 4. Derivative Term (D)
    float derivative = error - pid->prev_error;
    float d_out = pid->Kd * derivative;

    // Save error for next time
    pid->prev_error = error;

    // 5. Total Output
    float output = p_out + i_out + d_out;

    // 6. Output Clamping (Ensure we stay within PWM limits)
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    // printf("set=%f   err=%f   p=%f   i=%f   d=%f    output=%f\n", setpoint, error, p_out, i_out, d_out, output);

    // Return as integer for the PWM hardware
    return (uint16_t)output;
}