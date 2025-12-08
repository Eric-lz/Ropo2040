#pragma once

// Blink an LED
void led_task(void *pvParameters);

// Every second, generate new number and add to queue
void rng_task(void *pvParameters);

// Every second, read from queue and print
void print_task(void *pvParameters);

// Print encoder pulse count
void print_speed(void *pvParameters);

// Pulse TEST_PIN to debug PWM counter
void pulse_pin(void *pvParameters);