#include "lock_system.h"

static Gesture key[3]     = {NONE, NONE, NONE};
static Gesture attempt[3] = {NONE, NONE, NONE};
static int     key_recorded = 0;

/* Record a 3-gesture unlock sequence at startup */
void record_key(void) {
    record_mode_signal();

    for (int i = 0; i < 3; i++) {
        key[i] = wait_for_valid_gesture(i == 0);
        wait_for_stillness();
    }

    key_recorded = 1;
    blink_led(2, 100);
}

/* Compare a 3-gesture attempt against the recorded key */
int unlock_attempt(void) {
    if (!key_recorded)
        return 0;

    unlock_mode_signal();

    int      matched      = 1;
    uint32_t attempt_start = HAL_GetTick();

    for (int i = 0; i < 3; i++) {
        attempt[i] = wait_for_valid_gesture_with_timeout(i == 0, attempt_start);

        if (attempt[i] == NONE ||
            HAL_GetTick() - attempt_start >= UNLOCK_ATTEMPT_TIMEOUT_MS)
            return 0;

        if (attempt[i] != key[i])
            matched = 0;

        wait_for_stillness();
        HAL_Delay(40);

        if (HAL_GetTick() - attempt_start >= UNLOCK_ATTEMPT_TIMEOUT_MS)
            return 0;
    }

    if (matched) {
        success_signal();
        return 1;
    }

    fail_signal();
    return 0;
}
