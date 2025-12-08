#include "config.h"
#include "motor.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "queue.h"
#include <stdio.h>

// Set PWM frequency and duty cycle
uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d){
    uint32_t clock = SYS_CLK_HZ;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0) divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;

    pwm_set_clkdiv_int_frac4(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);

    return wrap;
}

void oneturn_task(void *pvParameters){
    printf("OneTurn_Task started!\n");

    // Get queue handle from parameter
    QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

    // Setup GPIO for PWM and Enable pin
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan = pwm_gpio_to_channel(PWM_PIN);
    uint freq = 1000;
    uint duty = 0;

    pwm_set_freq_duty(slice_num, chan, freq, duty);
    pwm_set_enabled(slice_num, true);

    uint16_t pulses;
    uint16_t total_pulses = 0;
    uint state = 0;
    uint start_duty = 50;

    while(1){
        if(state == 0){
            printf("Turning... %d\n", start_duty);

            // Start motor
            xQueueReset(xQEncoder);
            total_pulses = 0;
            duty = start_duty;
            pwm_set_freq_duty(slice_num, chan, freq, duty);
            gpio_put(EN_PIN, 1);
            state = 1;
        }
        else if(state == 1){
            int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
            total_pulses += pulses;
            
            if(total_pulses > 900){
                duty = 20;
                pwm_set_freq_duty(slice_num, chan, freq, duty);
                state = 2;
            }
        }
        else if(state == 2){
            if(total_pulses < 993){
                int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);    
                total_pulses += pulses;
            }
            else{
                state = 3;
            }
        }
        else if(state == 3){
            // Stop motor
            gpio_put(EN_PIN, 0);
            state = 0;

            int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
            total_pulses += pulses;

            printf("Done. Waiting 2 seconds... %d\n", total_pulses);
            vTaskDelay(1000);
        }
    }
}

void oneturnpid_task(void *pvParameters){
    printf("OneTurnPID_Task started!\n");

    // Get queue handle from parameter
    QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

    // Setup GPIO for PWM and Enable pin
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan = pwm_gpio_to_channel(PWM_PIN);
    uint freq = 1000;
    uint duty = 0;

    pwm_set_freq_duty(slice_num, chan, freq, duty);
    pwm_set_enabled(slice_num, true);

    uint16_t pulses;
    uint16_t total_pulses = 0;

}

float Kp = 1.0; // Proportional gain
float Ki = 0.1; // Integral gain
float Kd = 0.05; // Derivative gain

uint16_t setpoint = 50.0; // Desired value
uint16_t process_variable = 0.0; // Current measured value
uint16_t output = 0.0; // Control output

float error = 0.0;
float previous_error = 0.0;
float integral_term = 0.0;
float derivative_term = 0.0;

uint64_t last_time_us = 0; // For calculating delta_t

float calculate_pid(float current_pv) {
    uint64_t current_time_us = time_us_64();
    float delta_t = (float)(current_time_us - last_time_us) / 1000000.0f; // Convert to seconds
    last_time_us = current_time_us;

    error = setpoint - current_pv;

    integral_term += error * delta_t;
    derivative_term = (error - previous_error) / delta_t;

    output = (Kp * error) + (Ki * integral_term) + (Kd * derivative_term);

    previous_error = error;

    return output;
}