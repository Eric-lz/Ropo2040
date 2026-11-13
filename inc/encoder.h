#pragma once
#include "pico/stdlib.h"
#include "config.h"

// Single-channel encoder counted in hardware by a PWM slice.
// The counter only counts up and cannot tell direction; the caller applies the sign.
typedef struct {
    uint slice;         // PWM slice used as the counter
    uint16_t last;      // Counter value at the previous encoder_take_delta() call
} Encoder_t;

// Set up the PWM slice of `pin` to count falling edges on that pin
void encoder_init(Encoder_t *enc, uint pin);

// Pulses counted since the previous call (handles 16-bit counter wrap-around).
// Must be called often enough that fewer than 65536 pulses happen between calls.
uint16_t encoder_take_delta(Encoder_t *enc);

// Exact position of `turns` wheel turns, in pulses, rounded to the nearest pulse.
// Uses integer math (turns * 4928 / 5) so the fraction never accumulates.
static inline int32_t wheel_turns_to_pulses(int32_t turns) {
    int64_t x = (int64_t)turns * WHEEL_PULSES_NUM;
    int64_t half = WHEEL_PULSES_DEN / 2;
    // Round half away from zero, symmetrically for negative values
    return (int32_t)(x >= 0 ? (x + half) / WHEEL_PULSES_DEN
                            : -((-x + half) / WHEEL_PULSES_DEN));
}
