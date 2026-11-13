#include "motor.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Configure one PWM pin for freq_hz, output low. Returns the wrap value.
static uint16_t pwm_pin_setup(uint pin, uint32_t freq_hz, uint *slice, uint *chan) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(pin);
    *chan = pwm_gpio_to_channel(pin);

    // Pick the smallest clock divider (in 1/16 steps) that keeps wrap <= 65535,
    // which gives the finest duty resolution for this frequency.
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = clock / freq_hz / 4096 + (clock % (freq_hz * 4096) != 0);
    if (divider16 / 16 == 0) divider16 = 16;
    uint16_t wrap = (uint16_t)((uint64_t)clock * 16 / divider16 / freq_hz - 1);

    pwm_set_clkdiv_int_frac4(*slice, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(*slice, wrap);
    pwm_set_chan_level(*slice, *chan, 0);
    pwm_set_enabled(*slice, true);
    return wrap;
}

void motor_init(Motor_t *m, uint fwd_pin, int rev_pin, uint en_pin, uint32_t freq_hz) {
    m->wrap = pwm_pin_setup(fwd_pin, freq_hz, &m->fwd_slice, &m->fwd_chan);

    m->rev_pin = rev_pin;
    if (rev_pin >= 0) {
        pwm_pin_setup((uint)rev_pin, freq_hz, &m->rev_slice, &m->rev_chan);
    }

    // Enable both half bridges. With EN high and both PWM inputs low,
    // both low-side switches are on: the motor terminals are shorted = brake.
    gpio_init(en_pin);
    gpio_set_dir(en_pin, GPIO_OUT);
    gpio_put(en_pin, 1);
}

bool motor_can_reverse(const Motor_t *m) {
    return m->rev_pin >= 0;
}

void motor_set_duty(Motor_t *m, float duty) {
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;
    if (duty < 0.0f && !motor_can_reverse(m)) duty = 0.0f;

    // Convert percent to PWM counter level (0..wrap), rounded
    float mag = duty >= 0.0f ? duty : -duty;
    uint16_t level = (uint16_t)(mag * m->wrap / 100.0f + 0.5f);

    // Drive one side, hold the other low. Both low = brake.
    uint16_t fwd = duty > 0.0f ? level : 0;
    uint16_t rev = duty < 0.0f ? level : 0;
    pwm_set_chan_level(m->fwd_slice, m->fwd_chan, fwd);
    if (motor_can_reverse(m)) {
        pwm_set_chan_level(m->rev_slice, m->rev_chan, rev);
    }
}
