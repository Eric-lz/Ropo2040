#include "encoder.h"

#include "hardware/clocks.h"
#include "edge_counter.pio.h"   // Generated from edge_counter.pio at build time

// SM cycles the glitch filter waits (32 checks x 2 instructions, see edge_counter.pio)
static const float FILTER_CYCLES = 64.0f;

void encoder_init(Encoder_t *enc, uint pin) {
    // Find a free state machine and load the program into its PIO block
    uint offset;
    bool ok = pio_claim_free_sm_and_add_program(&edge_counter_program, &enc->pio, &enc->sm, &offset);
    hard_assert(ok);    // Out of PIO state machines or instruction memory

    // Pin setup: input, owned by PIO. RP2040 pins start with a ~50k pull-down
    // enabled; the encoder circuit provides its own levels, so turn pulls off.
    pio_gpio_init(enc->pio, pin);
    gpio_disable_pulls(pin);
    gpio_set_input_hysteresis_enabled(pin, true);
    pio_sm_set_consecutive_pindirs(enc->pio, enc->sm, pin, 1, false);

    pio_sm_config c = edge_counter_program_get_default_config(offset);
    sm_config_set_jmp_pin(&c, pin);

    // Slow the SM down so the 64-cycle filter lasts ENCODER_GLITCH_FILTER_US.
    // E.g. 200 MHz, 5 us: 64 cycles in 5 us = 12.8 MHz SM clock, divider 15.6
    float div = (float)clock_get_hz(clk_sys) * ENCODER_GLITCH_FILTER_US / 1e6f / FILTER_CYCLES;
    if (div < 1.0f) div = 1.0f;
    sm_config_set_clkdiv(&c, div);

    // Start in the state that matches the pin right now, so starting up
    // doesn't count a fake edge
    uint start = gpio_get(pin) ? edge_counter_offset_high : edge_counter_offset_low;
    pio_sm_init(enc->pio, enc->sm, offset + start, &c);

    // Count starts at 0
    pio_sm_exec(enc->pio, enc->sm, pio_encode_mov(pio_x, pio_null));
    enc->last = 0;

    pio_sm_set_enabled(enc->pio, enc->sm, true);
}

uint32_t encoder_get_count(Encoder_t *enc) {
    // The SM pushes the count all the time, so the FIFO is usually full of
    // old values. Read out everything that's queued, plus one more: that last
    // one is pushed after we started reading, so it's the current count.
    // (Waits a few microseconds at most, while the filter is checking an edge.)
    uint n = pio_sm_get_rx_fifo_level(enc->pio, enc->sm) + 1;
    uint32_t x = 0;
    while (n > 0) {
        x = pio_sm_get_blocking(enc->pio, enc->sm);
        n--;
    }
    // X counts down from 0, so negate it (unsigned math wraps correctly)
    return 0u - x;
}

uint32_t encoder_take_delta(Encoder_t *enc) {
    uint32_t now = encoder_get_count(enc);
    uint32_t delta = now - enc->last;   // Unsigned subtraction handles wrap-around
    enc->last = now;
    return delta;
}
