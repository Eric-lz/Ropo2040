// Configuration file
#pragma once

// Pin definitions
// Onboard WS2812 RGB status LED (Waveshare RP2040-Zero = GPIO 16)
#define RGB_LED_PIN 16
#define RGB_LED_BRIGHTNESS 20   // Max channel level (0..255). 20 is comfortable on the desk.
#define RGB_LED_IS_RGB 0        // 0 = GRB byte order (standard WS2812), 1 = RGB byte order
#define TEST_PIN 28

// Encoder (counted by PIO, any GPIO works)
#define ENCODER_PIN 3
// Glitch filter: a new pin level must hold this long before the edge is counted.
// Why 25 us: while the motor is driven, the encoder counted extra edges (the wheel
// came up short every turn), while turning it by hand counted exactly 9856 per
// 5 turns. The likely cause is the motor current shifting the ground for the whole
// PWM on-time (up to ~50 us at 20 kHz, ~20 us at 40% duty), which a 5 us filter
// let through. At 25 us the one-turn test showed no drift over 10, 20 and 30 turns.
// Upper limit: real edges are ~160 us apart at full motor speed (~6 kHz),
// so keep this <= 60 us or real edges start being rejected.
#define ENCODER_GLITCH_FILTER_US 25

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
