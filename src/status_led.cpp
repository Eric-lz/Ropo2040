#include "status_led.h"
#include "config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"   // Generated from ws2812.pio at build time

static volatile StatusState_t current_status = STATUS_IDLE;

void status_led_init(StatusLed_t *led, uint pin) {
    uint offset;
    bool ok = pio_claim_free_sm_and_add_program(&ws2812_program, &led->pio, &led->sm, &offset);
    hard_assert(ok);    // Out of PIO state machines or instruction memory

    pio_gpio_init(led->pio, pin);
    pio_sm_set_consecutive_pindirs(led->pio, led->sm, pin, 1, true);

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = (float)clock_get_hz(clk_sys) / (800000.0f * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(led->pio, led->sm, offset, &c);
    pio_sm_set_enabled(led->pio, led->sm, true);
}

void status_led_set_rgb(StatusLed_t *led, uint8_t r, uint8_t g, uint8_t b) {
#if RGB_LED_IS_RGB
    uint32_t word = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
#else
    uint32_t word = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
#endif
    pio_sm_put_blocking(led->pio, led->sm, word << 8u);
}

void status_set(StatusState_t state) {
    current_status = state;
}

void status_led_task(void *pvParameters) {
    (void)pvParameters;

    StatusLed_t led;
    status_led_init(&led, RGB_LED_PIN);
    status_led_set_rgb(&led, 0, 0, 0);

    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t tick = 0;

    while (true) {
        const uint8_t b = RGB_LED_BRIGHTNESS;
        uint8_t r = 0, g = 0, bl = 0;

        switch (current_status) {
            case STATUS_IDLE:
                // Slow green blink (1 Hz: 500 ms on, 500 ms off)
                if ((tick % 20) < 10) {
                    g = b;
                }
                break;

            case STATUS_MOVING:
                // Solid blue
                bl = b;
                break;

            case STATUS_SETTLING:
                // Solid yellow
                r = b;
                g = b;
                break;

            case STATUS_ERROR:
                // Fast red blink (5 Hz: 100 ms on, 100 ms off)
                if ((tick % 4) < 2) {
                    r = b;
                }
                break;
        }

        status_led_set_rgb(&led, r, g, bl);
        tick++;
        vTaskDelayUntil(&last_wake, period);
    }
}
