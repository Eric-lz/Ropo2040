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

// Modules
#include "inc/debug.h"
#include "inc/motor.h"
#include "inc/encoder.h"
#include "inc/current.h"

// Core mask: 0b01 -> Core 0, 0b10 -> Core 1
#define CORE_0_MASK (1 << 0)
#define CORE_1_MASK (1 << 1)

// Create queue handle (used for communication between tasks)
static QueueHandle_t xQueue = NULL;
static QueueHandle_t xQEncoder = NULL;

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
    // xTaskCreate(print_current, "INA219_Task", 256, NULL, 1, NULL);
    //xTaskCreate(rng_task, "RNG_Task", 256, (void*) xQueue, 1, &rng_task_handle);
    //xTaskCreate(print_task, "Print_Task", 256, (void*) xQueue, 1, &print_task_handle);
    xTaskCreate(pulse_task, "Pulse_Task", 256, (void*) &xQEncoder, 4, NULL);
    // xTaskCreate(pulse_pin, "Pulse_Pin", 256, NULL, 4, NULL);
    // xTaskCreate(print_speed, "Print_Speed", 256, (void*) &xQEncoder, 5, NULL);
    
    // Motor
    // xTaskCreate(pwm_task, "PWM_Task", 256, NULL, 1, NULL);
    // xTaskCreate(oneturn_task, "OneTurn_Task", 256, (void*) &xQEncoder, 5, NULL);
    xTaskCreate(const_speed_task, "ConstSpeed_Task", 256, (void*) &xQEncoder, 5, NULL);

    // Pin task to core (has to be done before starting scheduler)
    // If task is not pinned, the scheduler will assign it to a core
    vTaskCoreAffinitySet(rng_task_handle, CORE_0_MASK);
    vTaskCoreAffinitySet(print_task_handle, CORE_1_MASK);

    // Start scheduler
    vTaskStartScheduler();
}
