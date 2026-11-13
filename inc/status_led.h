#pragma once
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// High-level firmware states displayed on the onboard RGB LED
typedef enum {
    STATUS_IDLE,        // Slow green blink (1 Hz): FreeRTOS alive, controller idle
    STATUS_MOVING,      // Solid blue: actively executing a motion profile
    STATUS_SETTLING,    // Solid yellow: profile complete, settling onto target
    STATUS_ERROR        // Fast red blink (5 Hz): move timed out outside tolerance
} StatusState_t;

// Low-level PIO WS2812 driver handle
typedef struct {
    PIO pio;
    uint sm;
} StatusLed_t;

// Load the WS2812 PIO program and configure `pin` for 800 kHz output
void status_led_init(StatusLed_t *led, uint pin);

// Set the RGB LED color (0..255 per channel)
void status_led_set_rgb(StatusLed_t *led, uint8_t r, uint8_t g, uint8_t b);

// Update the status displayed by status_led_task (safe to call from any task)
void status_set(StatusState_t state);

// FreeRTOS task driving the onboard RGB status LED
void status_led_task(void *pvParameters);
