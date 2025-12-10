#include "config.h"
#include "motor.h"
#include "pid.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "queue.h"
#include <stdio.h>

// Initialize PWM pin
void pwm_init(PWM_Pin_t *pwm, uint pin, uint freq, uint duty){
    // GPIO Setup
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm->slice_num = pwm_gpio_to_slice_num(pin);
    pwm->chan = pwm_gpio_to_channel(pin);

    // Calculate wrap value
    uint32_t clock = SYS_CLK_HZ;
    uint32_t divider16 = clock / freq / 4096 + (clock % (freq * 4096) != 0);
    if (divider16 / 16 == 0) divider16 = 16;
    pwm->wrap = clock * 16 / divider16 / freq - 1;

    // Set up divider and wrap
    pwm_set_clkdiv_int_frac4(pwm->slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(pwm->slice_num, pwm->wrap);
    pwm_set_chan_level(pwm->slice_num, pwm->chan, pwm->wrap * duty / 100);

    // Enable PWM
    pwm_set_enabled(pwm->slice_num, true);
}

// Set PWM duty cycle
void pwm_set_duty(PWM_Pin_t *pwm, uint duty){
    pwm_set_chan_level(pwm->slice_num, pwm->chan, pwm->wrap * duty / 100);
}

// Turn motor one full revolution
void oneturnpid_task(void *pvParameters){
    printf("OneTurnPID_Task started!\n");

    // Get queue handle from parameter
    QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

    // Setup GPIO for Enable pin
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);
    
    // Setup PWM pin
    PWM_Pin_t Motor;
    pwm_init(&Motor, PWM_PIN, 1000, 0);

    // Initialize PID
    PID_Controller_t Motor_PID;
    pid_init(&Motor_PID,  0.8f, 0.2f, 0.1f, 0, 100);    // 20 Hz loop (50 ms): kp = 0.8; ki = 0.2; kd = 0.1;
    // pid_init(&Motor_PID,  0.1f, 0.3f, 0.0f, 0, 100);    // 10 Hz loop (100 ms): kp = 0.1; ki = 0.3; kd = 0; no filtering
    // pid_init(&Motor_PID,  2.0f, 0.3f, 0.2f, 0, 100);    // 100 Hz loop (10 ms): kp = ?; ki = ?; kd = ?;

    // Filter coefficient. E.g. a value of 0.2 means "20% new reading, 80% history"
    const float alpha = 0.2f;

    uint16_t pulses = 0;
    int16_t total_pulses = 0;
    int16_t extra_pulses = 0;
    uint16_t overshoot_pulses = 0;
    float target_speed = 10.0;
    float filtered_pulses = 0;

    // delay start
    vTaskDelay(5000);

    while(true) {
        // Wait for synchronized sensor data (should arrive every 100ms according to encoder_task)
        int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
        uint16_t pwm_val;

        if (ret == pdPASS) {
            // Increment odometer
            if (total_pulses >= 968) {
                target_speed = 0.0;
                pid_reset(&Motor_PID);
                pwm_set_duty(&Motor, 0);
                // overshoot_pulses += pulses;
                vTaskDelay(pdMS_TO_TICKS(2000));
                int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
                total_pulses += pulses;
                extra_pulses = PULSES_PER_TURN - total_pulses;
                total_pulses = -extra_pulses;
                target_speed = 10;
            } else {
                // Filter pulses
                filtered_pulses = (alpha * (float)pulses) + ((1.0 - alpha) * filtered_pulses);

                // Calculate PID Output
                pwm_val = pid_calculate(&Motor_PID, target_speed, filtered_pulses);
                
                // Apply to Motor
                pwm_set_duty(&Motor, pwm_val);
                total_pulses += pulses;
            }
            
            // Plot data to BSP
            printf("%f\t%f\t%d\t%d\t%d\n", target_speed, filtered_pulses, pwm_val, total_pulses, -extra_pulses);
        }
    }
}


// Constant speed task
void const_speed_task(void *pvParameters){
    printf("ConstSpeed_Task started!\n");

    // Get queue handle from parameter
    QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

    // Setup GPIO for Enable pin
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);
    
    // Setup PWM pin
    PWM_Pin_t Motor;
    pwm_init(&Motor, PWM_PIN, 1000, 0);

    // Initialize PID
    PID_Controller_t Motor_PID;
    pid_init(&Motor_PID,  0.8f, 0.2f, 0.1f, 0, 100);    // 20 Hz loop (50 ms): kp = 0.8; ki = 0.2; kd = 0.1;
    // pid_init(&Motor_PID,  0.1f, 0.3f, 0.0f, 0, 100);    // 10 Hz loop (100 ms): kp = 0.1; ki = 0.3; kd = 0; no filtering
    // pid_init(&Motor_PID,  2.0f, 0.3f, 0.2f, 0, 100);    // 100 Hz loop (10 ms): kp = ?; ki = ?; kd = ?;

    // Filter coefficient. E.g. a value of 0.2 means "20% new reading, 80% history"
    const float alpha = 0.2f;

    uint16_t pulses = 0;
    uint16_t total_pulses = 0;
    float target_speed = 5.0;
    float filtered_pulses = 0;

    // delay start
    vTaskDelay(5000);

    while(true) {
        // Wait for synchronized sensor data (should arrive every 100ms according to encoder_task)
        int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);

        if (ret == pdPASS) {
            // Filter pulses
            filtered_pulses = (alpha * (float)pulses) + ((1.0 - alpha) * filtered_pulses);

            // Calculate PID Output
            uint16_t pwm_val = pid_calculate(&Motor_PID, target_speed, filtered_pulses);
            
            // Apply to Motor
            pwm_set_duty(&Motor, pwm_val);

            // Increment odometer
            total_pulses += pulses;
            
            // Plot data to BSP
            printf("%f\t%f\t%d\t%d\n", target_speed, filtered_pulses, pwm_val, total_pulses);
        }
    }
}
