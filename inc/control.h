#pragma once
#include <stdint.h>

// A move request: go to an absolute wheel position
typedef struct {
    int32_t target;     // Absolute target position [pulses]
    float max_vel;      // Cruise speed [pulses/s]
    float accel;        // Acceleration and deceleration [pulses/s^2]
} MotionCmd_t;

// Reported when a move finishes
typedef struct {
    int32_t target;     // Where it was asked to go [pulses]
    int32_t final_pos;  // Where the wheel actually stopped [pulses]
    bool timed_out;     // True if it gave up settling before reaching the tolerance
} MotionResult_t;

// Create the queues. Call once in main(), before the scheduler starts.
void control_init(void);

// The 100 Hz control loop: reads the encoder, runs the profile and controller, drives the motor
void control_task(void *pvParameters);

// Prints the latest control loop state for Better Serial Plotter (50 Hz)
void telemetry_task(void *pvParameters);

// Start a move. Returns false if a previous command hasn't been picked up yet.
bool motion_move_to(const MotionCmd_t *cmd);

// Wait until the current move finishes. Returns false on timeout.
bool motion_wait_done(MotionResult_t *result, uint32_t timeout_ms);
