#include "encoder.h"
#include "config.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "queue.h"

void encoder_task(void *pvParameters){
    // Initialise frequency pulse counter
    uint counter_slice;

    // Check if pulse pin is in PWM channel B (does not work in channel A)
    assert(pwm_gpio_to_channel(PULSE_PIN) == PWM_CHAN_B);
    counter_slice = pwm_gpio_to_slice_num(PULSE_PIN);

    // Get queue handle from parameter
    QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

    // Set up PWM hardware counter
    gpio_set_function(PULSE_PIN, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_FALLING);
    pwm_config_set_clkdiv(&cfg, 1);
    pwm_init(counter_slice, &cfg, false);

    // Enable Schmitt trigger and input buffer to smooth phototransistor signal
    // UNSURE
    gpio_set_input_hysteresis_enabled(PULSE_PIN, true);
    gpio_set_input_enabled(PULSE_PIN, true);

    // Start counter
    pwm_set_counter(counter_slice, 0);
    pwm_set_enabled(counter_slice, true);

    // Save pulse count of previous iteration
    uint16_t last_count = 0;

    TickType_t last_wake_time = xTaskGetTickCount();

    // Main loop
    while(1){
        // Get current pulse count
        uint16_t pulse_count = pwm_get_counter(counter_slice);
        // pwm_set_counter(counter_slice, 0);

        // Calculate how many pulses since last iteration
        uint16_t delta_count = pulse_count - last_count;

        // Send count to queue, wait at most 10 miliseconds as that is the sample rate
        int ret = xQueueSend(xQEncoder, &delta_count, 0);
        
        // If successful, update last_count. Otherwise, leave it as is.
        // This solves the problem of the queue potentially being full,
        // which means the pulses in this iteration of the loop would be lost.
        // This way, the last_count value was never updated and so the next
        // iteration would just send the pulses that missed plus however many
        // pulses happened since the last iteration.
        if (ret == pdPASS) {
            last_count = pulse_count;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}