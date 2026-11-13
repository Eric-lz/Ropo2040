#include "control.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <math.h>
#include <stdio.h>

// ===================== Tuning =====================
// Duty is in percent (-100..100), positions in counts, speeds in counts/s.
// 1 count = one encoder edge = 1/1971.2 of a wheel turn (both edges counted).
// These are starting guesses: tune them on the real robot.

// Feedforward: the duty we expect to need, before looking at any error
static const float KS = 8.0f;     // Friction: duty added whenever the setpoint is moving [%]
static const float KV = 0.03f;    // Speed: duty per count/s of setpoint speed [%/(count/s)]
                                  // Measured: ~46% duty holds 1200 counts/s -> (46 - KS) / 1200

// Feedback: corrections based on how far off we are
static const float KP = 0.25f;    // Per count of position error [%/count]
static const float KI = 0.25f;    // Per count*second of accumulated error [%/(count*s)]
static const float KD = 0.005f;   // Per count/s of speed error [%/(count/s)]
static const float I_LIMIT = 10.0f;   // Max duty the integral term may contribute [%]

// Measured speed smoothing: 0.3 = 30% new reading, 70% history
static const float VEL_ALPHA = 0.3f;

// End of move
static const int32_t POS_TOLERANCE = 2;       // Close enough [counts] (~0.37 deg of wheel)
static const uint32_t STOPPED_TICKS = 5;      // No counts for this many ticks = wheel stopped
static const uint32_t SETTLE_TIMEOUT_MS = 1000;   // Give up settling after this long
// ==================================================

static const float DT = CONTROL_PERIOD_MS / 1000.0f;   // Loop period [s]

typedef enum { STATE_IDLE, STATE_MOVING, STATE_SETTLING } ControlState_t;

// One snapshot of the loop, sent to the telemetry task
typedef struct {
    float sp_pos;       // Setpoint position [counts]
    int32_t pos;        // Measured position [counts]
    float sp_vel;       // Setpoint speed [counts/s]
    float vel;          // Measured speed, filtered [counts/s]
    float duty;         // Motor output [%]
} Telemetry_t;

static QueueHandle_t cmd_queue;         // Commands into the loop (length 1)
static QueueHandle_t result_queue;      // Results out of the loop (length 1)
static QueueHandle_t telemetry_queue;   // Latest snapshot (length 1, overwritten every tick)

void control_init(void) {
    cmd_queue = xQueueCreate(1, sizeof(MotionCmd_t));
    result_queue = xQueueCreate(1, sizeof(MotionResult_t));
    telemetry_queue = xQueueCreate(1, sizeof(Telemetry_t));
}

bool motion_move_to(const MotionCmd_t *cmd) {
    xQueueReset(result_queue);   // Forget the result of any earlier move
    return xQueueSend(cmd_queue, cmd, 0) == pdPASS;
}

bool motion_wait_done(MotionResult_t *result, uint32_t timeout_ms) {
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(result_queue, result, ticks) == pdPASS;
}

// Advance the trapezoidal motion profile by one tick.
// Moves *pos toward target, accelerating up to max_vel and braking in time
// to arrive with zero speed. Returns true when the setpoint has reached target.
static bool profile_step(float *pos, float *vel, float target, float max_vel, float accel) {
    float remaining = target - *pos;
    float dir = remaining >= 0.0f ? 1.0f : -1.0f;
    float dist = fabsf(remaining);

    // Speed in the direction of the target (negative if currently moving away)
    float v = *vel * dir;

    // Three limits on the new speed; the smallest one wins:
    v += accel * DT;                            // 1. can only speed up by accel*dt
    if (v > max_vel) v = max_vel;               // 2. cruise speed
    float v_brake = sqrtf(2.0f * accel * dist); // 3. fastest speed that can still stop
    if (v > v_brake) v = v_brake;               //    in `dist` with deceleration `accel`

    float step = v * DT;
    if (v >= 0.0f && step >= dist) {
        // This tick would reach (or pass) the target: land exactly on it
        *pos = target;
        *vel = 0.0f;
        return true;
    }
    *pos += dir * step;
    *vel = dir * v;
    return false;
}

void control_task(void *pvParameters) {
    printf("Control_Task started!\n");

    Encoder_t enc;
    encoder_init(&enc, ENCODER_PIN);

    Motor_t motor;
    motor_init(&motor, MOTOR_PWM_FWD_PIN, MOTOR_PWM_REV_PIN, MOTOR_EN_PIN, MOTOR_PWM_FREQ_HZ);
    motor_set_duty(&motor, 0.0f);

    // Measured state
    int32_t pos = 0;                // Wheel position [counts]. Never reset.
    float vel = 0.0f;               // Filtered speed [counts/s]
    int dir = 1;                    // Direction the wheel is assumed to turn (+1/-1)
    uint32_t ticks_since_edge = STOPPED_TICKS;

    // Profile / controller state
    ControlState_t state = STATE_IDLE;
    MotionCmd_t cmd = {0, 0.0f, 0.0f};
    float sp_pos = 0.0f;            // Setpoint position [counts]
    float sp_vel = 0.0f;            // Setpoint speed [counts/s]
    float i_term = 0.0f;            // Integral contribution [%]
    TickType_t settle_start = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // ---------- 1. Measure ----------
        // The encoder can't tell direction, so the counts are signed with the
        // direction we have been driving the motor in.
        uint32_t delta = encoder_take_delta(&enc);
        pos += dir * (int32_t)delta;
        vel = VEL_ALPHA * (dir * (int32_t)delta / DT) + (1.0f - VEL_ALPHA) * vel;
        if (delta > 0) {
            ticks_since_edge = 0;
        } else if (ticks_since_edge < STOPPED_TICKS) {
            ticks_since_edge++;
        }
        bool stopped = ticks_since_edge >= STOPPED_TICKS;

        // ---------- 2. New command? ----------
        MotionCmd_t new_cmd;
        if (xQueueReceive(cmd_queue, &new_cmd, 0) == pdPASS) {
            cmd = new_cmd;
            if (state == STATE_IDLE) {
                // Start the profile from where the wheel really is, at rest
                sp_pos = (float)pos;
                sp_vel = 0.0f;
            }
            i_term = 0.0f;
            state = STATE_MOVING;
        }

        // ---------- 3. Profile + controller ----------
        float duty = 0.0f;
        if (state != STATE_IDLE) {
            if (state == STATE_MOVING) {
                if (profile_step(&sp_pos, &sp_vel, (float)cmd.target, cmd.max_vel, cmd.accel)) {
                    state = STATE_SETTLING;
                    settle_start = xTaskGetTickCount();
                }
            }

            float err = sp_pos - (float)pos;    // Position error [counts]
            float vel_err = sp_vel - vel;       // Speed error [counts/s]

            // Feedforward
            float ff = KV * sp_vel;
            if (sp_vel > 0.0f) ff += KS;
            else if (sp_vel < 0.0f) ff -= KS;

            // Feedback
            float u = ff + KP * err + i_term + KD * vel_err;

            // Integrate only if the output isn't already maxed out in the
            // direction the error is pushing (anti-windup)
            float u_max = 100.0f;
            float u_min = motor_can_reverse(&motor) ? -100.0f : 0.0f;
            bool saturated = (u >= u_max && err > 0.0f) || (u <= u_min && err < 0.0f);
            if (!saturated) {
                i_term += KI * err * DT;
                if (i_term > I_LIMIT) i_term = I_LIMIT;
                if (i_term < -I_LIMIT) i_term = -I_LIMIT;
            }

            duty = u;
            if (duty > u_max) duty = u_max;
            if (duty < u_min) duty = u_min;

            // ---------- 4. Finished? ----------
            if (state == STATE_SETTLING) {
                int32_t final_err = cmd.target - pos;
                bool on_target = final_err <= POS_TOLERANCE && final_err >= -POS_TOLERANCE;
                bool timed_out = (xTaskGetTickCount() - settle_start) >= pdMS_TO_TICKS(SETTLE_TIMEOUT_MS);
                if ((on_target && stopped) || timed_out) {
                    state = STATE_IDLE;
                    i_term = 0.0f;
                    duty = 0.0f;
                    MotionResult_t res = {cmd.target, pos, timed_out && !on_target};
                    xQueueOverwrite(result_queue, &res);
                }
            }
        }

        // ---------- 5. Direction change guard ----------
        // Only allowed once the wheel has stopped; otherwise counts from the
        // wheel still turning the old way would be counted in the new direction.
        int want_dir = duty > 0.0f ? 1 : (duty < 0.0f ? -1 : 0);
        if (want_dir != 0 && want_dir != dir) {
            if (stopped) {
                dir = want_dir;
            } else {
                duty = 0.0f;    // Brake until it stops
            }
        }

        // ---------- 6. Output ----------
        motor_set_duty(&motor, duty);   // 0 = brake (also while idle)

        Telemetry_t t = {sp_pos, pos, sp_vel, vel, duty};
        xQueueOverwrite(telemetry_queue, &t);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void telemetry_task(void *pvParameters) {
    Telemetry_t t;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // Print the newest snapshot, if there is one we haven't printed yet.
        // Printing here instead of in control_task keeps slow USB output
        // from delaying the control loop.
        if (xQueueReceive(telemetry_queue, &t, 0) == pdPASS) {
            // Columns: setpoint pos, pos, setpoint speed, speed, duty
            printf("%.1f\t%d\t%.0f\t%.0f\t%.1f\n",
                   t.sp_pos, (int)t.pos, t.sp_vel, t.vel, t.duty);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}
