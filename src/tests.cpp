#include "tests.h"
#include "control.h"
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

void oneturn_test_task(void *pvParameters) {
    // Give time to open the serial monitor
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("OneTurn_Test started!\n");

    for (int32_t turn = 1; ; turn++) {
        // Target is always computed from the start (turn N = N * 985.6 pulses,
        // rounded), never by adding to the previous target, so the 0.6 never
        // accumulates into drift.
        MotionCmd_t cmd;
        cmd.target = wheel_turns_to_pulses(turn);
        cmd.max_vel = 600.0f;     // pulses/s (~0.6 turns/s)
        cmd.accel = 1500.0f;      // pulses/s^2 (reaches 600 in 0.4 s)
        motion_move_to(&cmd);

        MotionResult_t res;
        motion_wait_done(&res, portMAX_DELAY);
        printf("# turn %d: target %d, final %d, error %d%s\n",
               (int)turn, (int)res.target, (int)res.final_pos,
               (int)(res.final_pos - res.target), res.timed_out ? " (timed out)" : "");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
