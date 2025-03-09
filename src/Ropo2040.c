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

// LED pin
const uint LED_PIN = 14;
const uint PULSE_PIN = 3;

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
            printf("Queue full, prev = %d, cur = %d\n", previous_count, pulse_count);
            pulse_count += previous_count;
            xQueueOverwrite(xQEncoder, &pulse_count);
        }
        
        previous_count = pulse_count;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void print_speed(void *pvParameters){
    printf("Print_Speed started!\n");

    uint16_t speed;

    while(1){
        int ret = xQueueReceive(xQEncoder, &speed, pdMS_TO_TICKS(10));

        if(ret == pdPASS){
            printf("Speed: %d, Core %d\n", speed, get_core_num());
        }
        else{
            printf("Queue empty!\n");
        }

        vTaskDelay(1000);
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
    xTaskCreate(rng_task, "RNG_Task", 256, (void*) xQueue, 1, &rng_task_handle);
    xTaskCreate(print_task, "Print_Task", 256, NULL, 1, &print_task_handle);
    xTaskCreate(pulse_task, "Pulse_Task", 256, NULL, 5, NULL);
    xTaskCreate(print_speed, "Print_Speed", 256, NULL, 1, NULL);
    
    // Pin task to core (has to be done before starting scheduler)
    // If task is not pinned, the scheduler will assign it to a core
    vTaskCoreAffinitySet(rng_task_handle, CORE_0_MASK);
    vTaskCoreAffinitySet(print_task_handle, CORE_1_MASK);

    // Start scheduler
    vTaskStartScheduler();
}
