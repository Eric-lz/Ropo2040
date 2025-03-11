// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Pico
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

// C
#include <stdio.h>

#include "inc/rng_task.h"

// Core mask: 0b01 -> Core 0, 0b10 -> Core 1
#define CORE_0_MASK (1 << 0)
#define CORE_1_MASK (1 << 1)

// GPIO pinout
const uint LED_PIN = 14;
const uint PULSE_PIN = 3;
const uint PWM_PIN = 8;
const uint EN_PIN = 1;

// Create queue handle (used for communication between tasks)
static QueueHandle_t xQueue = NULL;
static QueueHandle_t xQEncoder = NULL;

void led_task(void *pvParameters){
    printf("LED_Task started!\n");

    // Setup GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Blink LED
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(100);
        gpio_put(LED_PIN, 0);
        vTaskDelay(100);
    }
}

// Every second, read from queue and print
void print_task(void *pvParameters){
    printf("Print_Task started!\n");

    int num;

    while(1){
        int ret = xQueueReceive(xQueue, &num, pdMS_TO_TICKS(1000));

        if(ret == pdPASS){
            printf("Number recv: %d, Core %d\n", num, get_core_num());
        }
        else{
            printf("Queue empty!\n");
        }

        vTaskDelay(1000);
    }
}

void pulse_task(void *pvParameters){
    // Initialise frequency pulse counter
    uint counter_slice;

    assert(pwm_gpio_to_channel(PULSE_PIN) == PWM_CHAN_B);
    counter_slice = pwm_gpio_to_slice_num(PULSE_PIN);

    gpio_set_function(PULSE_PIN, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1);
    pwm_init(counter_slice, &cfg, false);

    // Start counter
    pwm_set_counter(counter_slice, 0);
    pwm_set_enabled(counter_slice, true);

    uint16_t previous_count = 0;

    while(1){
        // Get pulse count and reset counter
        uint16_t pulse_count = pwm_get_counter(counter_slice);
        pwm_set_counter(counter_slice, 0);

        // Send count to queue
        int ret = xQueueSend(xQEncoder, &pulse_count, pdMS_TO_TICKS(10));
        
        // If queue is full, add current count to previous count and overwrite queue
        if(ret != pdPASS){
            //printf("Queue full, prev = %d, cur = %d\n", previous_count, pulse_count);
            pulse_count += previous_count;
            xQueueOverwrite(xQEncoder, &pulse_count);
        }
        
        previous_count = pulse_count;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void print_speed(void *pvParameters){
    printf("Print_Speed started!\n");

    uint16_t speed = 0;

    while(1){
        uint16_t count;
        int ret = xQueuePeek(xQEncoder, &count, pdMS_TO_TICKS(10));
        speed += count;
        if(ret == pdPASS){
            printf("Speed: %d\n", speed);
        }
        else{
            printf("Queue empty!\n");
        }

        vTaskDelay(100);
    }
}

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

void pwm_task(void *pvParameters){
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
        duty += 1;

        // Reset duty cycle to 0% after reaching 100%
        if(duty > 25){
            duty = 18;
        }

        printf("Setting duty cycle to %d\n", duty);
        pwm_set_freq_duty(slice_num, chan, freq, duty);

        vTaskDelay(2000);
    }
}

void oneturn_task(void *pvParameters){
    printf("OneTurn_Task started!\n");

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
    uint start_duty = 70;

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
            if(total_pulses < 990){
                int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);

                if(total_pulses > 900){
                    duty = 20;
                    pwm_set_freq_duty(slice_num, chan, freq, duty);
                }
                
                total_pulses += pulses;
                //printf("Running... %d/%d\n", pulses, total_pulses);
                vTaskDelay(10);
            }
            else{
                state = 2;
            }
        }
        else if(state == 2){
            // Stop motor
            gpio_put(EN_PIN, 0);
            state = 0;

            int ret = xQueueReceive(xQEncoder, &pulses, portMAX_DELAY);
            total_pulses += pulses;

            printf("Done. Waiting 5 seconds... %d\n", total_pulses);
            vTaskDelay(2000);
        }
    }
}

int main(){
    stdio_init_all();

    // Init queue
    xQueue = xQueueCreate(1, sizeof(int));
    xQEncoder = xQueueCreate(1, sizeof(int));

    // Task handles
    TaskHandle_t rng_task_handle;
    TaskHandle_t print_task_handle;

    // Start tasks
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    //xTaskCreate(rng_task, "RNG_Task", 256, (void*) xQueue, 1, &rng_task_handle);
    //xTaskCreate(print_task, "Print_Task", 256, NULL, 1, &print_task_handle);
    xTaskCreate(pulse_task, "Pulse_Task", 256, NULL, 5, NULL);
    //xTaskCreate(print_speed, "Print_Speed", 256, NULL, 4, NULL);
    //xTaskCreate(pwm_task, "PWM_Task", 256, NULL, 1, NULL);
    xTaskCreate(oneturn_task, "OneTurn_Task", 256, NULL, 5, NULL);

    // Pin task to core (has to be done before starting scheduler)
    // If task is not pinned, the scheduler will assign it to a core
    vTaskCoreAffinitySet(rng_task_handle, CORE_0_MASK);
    vTaskCoreAffinitySet(print_task_handle, CORE_1_MASK);

    // Start scheduler
    vTaskStartScheduler();
}
