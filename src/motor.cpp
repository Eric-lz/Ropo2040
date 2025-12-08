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

void const_speed_task(void *pvParameters){
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
    pid_init(&Motor_PID,  0.1f, 0.3f, 0.0f, 0, 100);
    
    uint16_t target_speed_ticks = 60.0;
    uint16_t total_pulses = 0;
    uint16_t pulses;

    // delay start
    vTaskDelay(5000);

    while(true) {
        // Wait for synchronized sensor data
        int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);

        if (ret == pdPASS) {
            // Calculate PID Output
            uint16_t pwm_val = pid_calculate(&Motor_PID, (float)target_speed_ticks, (float)pulses);
            printf("%d\t%d\t%d\n", target_speed_ticks, pulses, pwm_val);

            // Apply to Motors
            // pwm_set_freq_duty(slice_num, chan, freq, pwm_val);
            pwm_set_duty(&Motor, pwm_val);
        }
    }
}


// Set PWM frequency and duty cycle
/* uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t freq, int duty){
    uint32_t clock = SYS_CLK_HZ;
    uint32_t divider16 = clock / freq / 4096 + (clock % (freq * 4096) != 0);
    if (divider16 / 16 == 0) divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / freq - 1;

    pwm_set_clkdiv_int_frac4(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * duty / 100);

    return wrap;
} */

// Perform one full turn of the wheel
/* void oneturn_task(void *pvParameters){
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
        if(state != 0){
            int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
            if (ret == pdPASS){
                total_pulses += pulses;
            }
        }

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
            if(total_pulses > 900){
                duty = 20;
                pwm_set_freq_duty(slice_num, chan, freq, duty);
                state = 2;
            }
        }
        else if(state == 2){
            if(total_pulses > 990){
                state = 3;
            }
        }
        else if(state == 3){
            // Stop motor
            gpio_put(EN_PIN, 0);
            vTaskDelay(10);
            state = 0;
            printf("Done. Waiting 2 seconds... %d\n", total_pulses);
            vTaskDelay(1000);
        }
    }
} */

// PWM test
/* void pwm_task(void *pvParameters){
    printf("PWM_Task started!\n");

    // Setup GPIO for PWM and Enable pin
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan = pwm_gpio_to_channel(PWM_PIN);
    uint freq = 1000;
    uint duty = 10;

    pwm_set_freq_duty(slice_num, chan, freq, duty);
    pwm_set_enabled(slice_num, true);

    while(1){
        // Increase duty cycle by 10% every seconds
        duty += 10;

        // Reset duty cycle to 0% after reaching 100%
        if(duty > 100){
            duty = 10;
        }

        printf("Setting duty cycle to %d\n", duty);
        pwm_set_freq_duty(slice_num, chan, freq, duty);

        vTaskDelay(5000);
    }
} */

