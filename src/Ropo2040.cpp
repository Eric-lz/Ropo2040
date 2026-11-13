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
#include "inc/control.h"
#include "inc/tests.h"
#include "inc/current.h"

// Core mask: 0b01 -> Core 0, 0b10 -> Core 1
#define CORE_0_MASK (1 << 0)
#define CORE_1_MASK (1 << 1)

// Create queue handle (used for communication between tasks)
static QueueHandle_t xQueue = NULL;

int main(){
    stdio_init_all();

    // Create queues
    //xQueue = xQueueCreate(1, sizeof(int));
    control_init();

    // Create task handles
    //TaskHandle_t rng_task_handle;
    //TaskHandle_t print_task_handle;

    // Start tasks =======
    // Motor control loop: highest priority so it runs on time every 10 ms
    xTaskCreate(control_task, "Control_Task", 512, NULL, 6, NULL);
    xTaskCreate(telemetry_task, "Telemetry_Task", 1024, NULL, 2, NULL);

    // Test sequence: sends move commands to the control loop.
    // Commented out = hand test: the motor stays idle and the 2nd serial
    // column shows the live encoder count. 5 turns by hand should be 9856.
    // xTaskCreate(oneturn_test_task, "OneTurn_Test", 1024, NULL, 3, NULL);

    // INA219 Current sensor
    // xTaskCreate(print_current, "INA219_Task", 256, NULL, 1, NULL);
    
    // Debug tasks
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    // xTaskCreate(rng_task, "RNG_Task", 256, (void*) xQueue, 1, &rng_task_handle);
    // xTaskCreate(print_task, "Print_Task", 256, (void*) xQueue, 1, &print_task_handle);
    // xTaskCreate(pulse_pin, "Pulse_Pin", 256, NULL, 4, NULL);

    // Pin task to core (has to be done before starting scheduler)
    // If task is not pinned, the scheduler will assign it to a core
    // vTaskCoreAffinitySet(rng_task_handle, CORE_0_MASK);
    // vTaskCoreAffinitySet(print_task_handle, CORE_1_MASK);

    // Start scheduler
    vTaskStartScheduler();
}
