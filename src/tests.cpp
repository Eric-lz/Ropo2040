#include "tests.h"
#include "control.h"
#include "encoder.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

// Number of turns before stopping (0 = run forever)
static const int32_t TEST_TURNS = 10;

void oneturn_test_task(void *pvParameters) {
    // Give time to open the serial monitor
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("OneTurn_Test started! Filter %d us\n", ENCODER_GLITCH_FILTER_US);

    for (int32_t turn = 1; TEST_TURNS == 0 || turn <= TEST_TURNS; turn++) {
        // Target is always computed from the start (turn N = N * 1971.2 counts,
        // rounded), never by adding to the previous target, so the 0.2 never
        // accumulates into drift.
        MotionCmd_t cmd;
        cmd.target = wheel_turns_to_counts(turn);
        cmd.max_vel = 1200.0f;    // counts/s (~0.6 turns/s)
        cmd.accel = 3000.0f;      // counts/s^2 (reaches 1200 in 0.4 s)
        motion_move_to(&cmd);

        MotionResult_t res;
        motion_wait_done(&res, portMAX_DELAY);
        printf("# turn %d: target %d, final %d, error %d%s\n",
               (int)turn, (int)res.target, (int)res.final_pos,
               (int)(res.final_pos - res.target), res.timed_out ? " (timed out)" : "");

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Done: the control loop stays idle (braked) and keeps printing the count
    printf("# test done: %d turns\n", (int)TEST_TURNS);
    vTaskDelete(NULL);
}
