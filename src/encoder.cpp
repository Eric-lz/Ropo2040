#include "encoder.h"
#include "config.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "queue.h"

void pulse_task(void *pvParameters){
  // Initialise frequency pulse counter
  uint counter_slice;

  assert(pwm_gpio_to_channel(PULSE_PIN) == PWM_CHAN_B);
  counter_slice = pwm_gpio_to_slice_num(PULSE_PIN);

  // Get queue handle from parameter
  QueueHandle_t xQEncoder = *(QueueHandle_t*) pvParameters;

  gpio_set_function(PULSE_PIN, GPIO_FUNC_PWM);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_FALLING);
  pwm_config_set_clkdiv(&cfg, 1);
  pwm_init(counter_slice, &cfg, false);

  // Start counter
  pwm_set_counter(counter_slice, 0);
  pwm_set_enabled(counter_slice, true);

  uint16_t failed_count = 0;
  uint16_t last_count = 0;
  uint16_t delta_count;

  while(1){
      // Get pulse count and reset counter
      uint16_t pulse_count = pwm_get_counter(counter_slice);
      // pwm_set_counter(counter_slice, 0);

      delta_count = pulse_count - last_count;
      last_count = pulse_count;

      // Send count to queue
      int ret = xQueueSend(xQEncoder, &delta_count, pdMS_TO_TICKS(10));
      
      // If queue is full, add current count to previous count and overwrite queue
      if(ret != pdPASS){
          //printf("Queue full, prev = %d, cur = %d\n", previous_count, pulse_count);
          delta_count += failed_count;
          xQueueOverwrite(xQEncoder, &delta_count);
      }
      
      failed_count = delta_count;

      vTaskDelay(pdMS_TO_TICKS(10));
  }
}