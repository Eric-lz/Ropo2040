#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "config.h"

// Single-channel encoder counted by a PIO state machine.
// Counts both rising and falling edges (2 counts per slot) and ignores
// glitches shorter than ENCODER_GLITCH_FILTER_US.
// The count only goes up and cannot tell direction; the caller applies the sign.
typedef struct {
    PIO pio;            // PIO block used (pio0 or pio1)
    uint sm;            // State machine inside that block
    uint32_t last;      // Count at the previous encoder_take_delta() call
} Encoder_t;

// Load the PIO program and start counting edges on `pin` (any GPIO)
void encoder_init(Encoder_t *enc, uint pin);

// Total edges counted since encoder_init()
uint32_t encoder_get_count(Encoder_t *enc);

// Edges counted since the previous call
uint32_t encoder_take_delta(Encoder_t *enc);

// Exact position of `turns` wheel turns, in counts, rounded to the nearest count.
// Uses integer math (turns * 9856 / 5) so the fraction never accumulates.
static inline int32_t wheel_turns_to_counts(int32_t turns) {
    int64_t x = (int64_t)turns * WHEEL_COUNTS_NUM;
    int64_t half = WHEEL_COUNTS_DEN / 2;
    // Round half away from zero, symmetrically for negative values
    return (int32_t)(x >= 0 ? (x + half) / WHEEL_COUNTS_DEN
                            : -((-x + half) / WHEEL_COUNTS_DEN));
}
