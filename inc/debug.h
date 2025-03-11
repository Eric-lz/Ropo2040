#pragma once

// Blink an LED
void led_task(void *pvParameters);

// Every second, generate new number and add to queue
void rng_task(void *pvParameters);

// Every second, read from queue and print
void print_task(void *pvParameters);

// PWM test
void pwm_task(void *pvParameters);

// Print encoder pulse count
void print_speed(void *pvParameters);