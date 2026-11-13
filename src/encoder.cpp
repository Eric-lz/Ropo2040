#include "encoder.h"

#include "hardware/pwm.h"

void encoder_init(Encoder_t *enc, uint pin) {
    // A PWM slice can only count edges on its channel B pin
    assert(pwm_gpio_to_channel(pin) == PWM_CHAN_B);
    enc->slice = pwm_gpio_to_slice_num(pin);

    // Pin setup. RP2040 pins start with a ~50k pull-down enabled; the encoder
    // circuit provides its own levels, so turn the internal pulls off.
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_disable_pulls(pin);
    gpio_set_input_hysteresis_enabled(pin, true);   // Schmitt trigger (on by default)

    // Count every falling edge on the B pin (clock divider 1 = count all of them)
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_FALLING);
    pwm_config_set_clkdiv(&cfg, 1);
    pwm_init(enc->slice, &cfg, false);

    pwm_set_counter(enc->slice, 0);
    enc->last = 0;
    pwm_set_enabled(enc->slice, true);
}

uint16_t encoder_take_delta(Encoder_t *enc) {
    uint16_t now = pwm_get_counter(enc->slice);
    // Unsigned 16-bit subtraction gives the right answer even when the counter
    // wrapped from 65535 back to 0 since the last call
    uint16_t delta = now - enc->last;
    enc->last = now;
    return delta;
}
