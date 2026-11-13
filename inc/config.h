// Configuration file
#pragma once

// Pin definitions
#define LED_PIN 14
#define TEST_PIN 28

// Encoder (counted by PIO, any GPIO works)
#define ENCODER_PIN 3
#define ENCODER_GLITCH_FILTER_US 5   // Edges must hold this long to count. Max real
                                     // edge rate ~6 kHz (~160 us apart), so 5 us is safe.

// Motor driver (BTS7960 / HW-039)
#define MOTOR_PWM_FWD_PIN 8     // RPWM
#define MOTOR_PWM_REV_PIN -1    // LPWM. -1 = not wired (forward only). Set to 9 when wired.
#define MOTOR_EN_PIN 1          // R_EN and L_EN tied together
#define MOTOR_PWM_FREQ_HZ 20000

// Wheel geometry
// 16-slot encoder disc on the motor shaft x 61.6:1 gearbox = 985.6 slots per wheel turn.
// Both edges of each slot are counted: 1971.2 counts per wheel turn.
// Stored as a fraction so it stays exact: 9856 counts every 5 turns.
#define WHEEL_COUNTS_NUM 9856
#define WHEEL_COUNTS_DEN 5

// Control loop
#define CONTROL_PERIOD_MS 10    // 100 Hz

// I2C
#define I2C_FREQ_KHZ 400
#define PIN_SDA 14
#define PIN_SCL 15

// INA219
#define INA219_ADDR 0x40
