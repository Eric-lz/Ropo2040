#include "rng_task.h"
#include "config.h"
#include "debug.h"
#include "motor.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "hardware/pwm.h"

// Blink an LED
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

// Every second, generate new number and add to queue
void rng_task(void *pvParameters){
    printf("Print_Task started!\n");

    QueueHandle_t xQueue = (QueueHandle_t)pvParameters;
    int num = 0;

    while(1){
        num++;

        int ret = xQueueSend(xQueue, &num, pdMS_TO_TICKS(1000));
        printf("Number sent: %d, Core %d\n", num, get_core_num());

        if(ret != pdPASS){
            printf("Queue full!\n");
        }

        vTaskDelay(1000);
    }
}

// Every second, read from queue and print
void print_task(void *pvParameters){
    printf("Print_Task started!\n");

    QueueHandle_t xQueue = (QueueHandle_t)pvParameters;
    
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

// PWM test
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

// Print encoder pulse count
void print_speed(void *pvParameters){
  printf("Print_Speed started!\n");

  // Get queue handle from parameter
  QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

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
