#pragma once
#include "pico/stdlib.h"

// One DC motor on a BTS7960 (HW-039) H-bridge
typedef struct {
    uint fwd_slice, fwd_chan;   // RPWM
    int  rev_pin;               // LPWM, -1 if not wired
    uint rev_slice, rev_chan;
    uint16_t wrap;              // PWM counter top value (sets the resolution)
} Motor_t;

// Set up PWM pins and enable the driver. rev_pin = -1 for forward-only wiring.
void motor_init(Motor_t *m, uint fwd_pin, int rev_pin, uint en_pin, uint32_t freq_hz);

// duty in percent: +100 = full forward, -100 = full reverse, 0 = brake.
// Without a reverse pin, negative values brake instead.
void motor_set_duty(Motor_t *m, float duty);

// True if the reverse PWM pin is wired
bool motor_can_reverse(const Motor_t *m);
