#include "current.h"
#include "config.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "INA219.h"

// Blink an LED
void print_current(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(4000));
  printf("INA219 task started!\n");
  //i2c bus, i2c address
  INA219 ina219(i2c1, INA219_ADDR);
  ina219.I2C_START(PIN_SDA, PIN_SCL, I2C_FREQ_KHZ);

  // Calibrate for 0.1 Ohm shunt resistor and 3.2A max expected current
  ina219.calibrate(0.1, 3.2);

  while (1) {
    float voltage = ina219.read_voltage();
    float shunt_voltage = ina219.read_shunt_voltage();
    float current = ina219.read_current();
    float power = ina219.read_power();

    printf("Voltage: %.4f V\n", voltage);
    printf("Shunt Voltage: %.4f mV\n", shunt_voltage);
    printf("Current: %.4f A\n", current);
    printf("Power: %.4f W\n", power);
    printf("--------------------\n");

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}