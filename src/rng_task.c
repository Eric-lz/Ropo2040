#include "rng_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

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