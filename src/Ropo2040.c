// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Pico
#include "pico/stdlib.h"
#include "pico/multicore.h"

// C
#include <stdio.h>

#include "inc/rng_task.h"

// Core mask: 0b01 -> Core 0, 0b10 -> Core 1
#define CORE_0_MASK (1 << 0)
#define CORE_1_MASK (1 << 1)

// LED pin
const uint LED_PIN = 14;

// Create queue handle (used for communication between tasks)
static QueueHandle_t xQueue = NULL;

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

int main(){
    stdio_init_all();

    // Init queue
    xQueue = xQueueCreate(1, sizeof(int));

    // Task handles
    TaskHandle_t rng_task_handle;
    TaskHandle_t print_task_handle;

    // Start tasks
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(rng_task, "RNG_Task", 256, (void*) xQueue, 1, &rng_task_handle);
    xTaskCreate(print_task, "Print_Task", 256, NULL, 1, &print_task_handle);
    
    // Pin task to core (has to be done before starting scheduler)
    // If task is not pinned, the scheduler will assign it to a core
    vTaskCoreAffinitySet(rng_task_handle, CORE_0_MASK);
    vTaskCoreAffinitySet(print_task_handle, CORE_1_MASK);

    // Start scheduler
    vTaskStartScheduler();
}
