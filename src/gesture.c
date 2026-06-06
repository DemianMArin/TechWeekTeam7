#include "gesture.h"
#include <math.h>

const char *gesture_name(Gesture g) {
    switch (g) {
        case GX_POS: return "GX_POS";
        case GX_NEG: return "GX_NEG";
        case GY_POS: return "GY_POS";
        case GY_NEG: return "GY_NEG";
        case GZ_POS: return "GZ_POS";
        case GZ_NEG: return "GZ_NEG";
        default:     return "NONE";
    }
}

/* Pick the dominant gyro axis and sign */
Gesture classify_gyro_peak(float gx_peak, float gy_peak, float gz_peak) {
    float abs_gx = fabsf(gx_peak);
    float abs_gy = fabsf(gy_peak);
    float abs_gz = fabsf(gz_peak);
    const float threshold = 150.0f;

    if (abs_gx < threshold && abs_gy < threshold && abs_gz < threshold)
        return NONE;
    if (abs_gx > 1.3f * abs_gy && abs_gx > 1.3f * abs_gz)
        return gx_peak > 0 ? GX_POS : GX_NEG;
    if (abs_gy > 1.3f * abs_gx && abs_gy > 1.3f * abs_gz)
        return gy_peak > 0 ? GY_POS : GY_NEG;
    if (abs_gz > 1.3f * abs_gx && abs_gz > 1.3f * abs_gy)
        return gz_peak > 0 ? GZ_POS : GZ_NEG;

    return NONE;
}

/* Check if board is flat and upright (required before first gesture of each attempt) */
int is_home_orientation(float ax_g, float ay_g, float az_g) {
    return fabsf(az_g) > HOME_AZ_MIN_G &&
           fabsf(ax_g) < HOME_AX_MAX_G &&
           fabsf(ay_g) < HOME_AY_MAX_G;
}

/* Block until board is flat, still, and stable for HOME_STABLE_TIME_MS */
void wait_for_home_ready(void) {
    uint32_t stable_start    = 0;
    int      timing_stability = 0;

    while (1) {
        float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
        read_imu(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);

        int orientation_ok = is_home_orientation(ax_g, ay_g, az_g);
        int still_ok = fabsf(gx_dps) < HOME_STILL_GYRO_MAX_DPS &&
                       fabsf(gy_dps) < HOME_STILL_GYRO_MAX_DPS &&
                       fabsf(gz_dps) < HOME_STILL_GYRO_MAX_DPS;

        if (orientation_ok && still_ok) {
            if (!timing_stability) {
                stable_start     = HAL_GetTick();
                timing_stability = 1;
            }
            if (HAL_GetTick() - stable_start >= HOME_STABLE_TIME_MS)
                return;
        } else {
            timing_stability = 0;
        }

        update_mode_leds();
        HAL_Delay(SAMPLE_DELAY_MS);
    }
}

/* Wait for motion onset, capture a gesture window, classify dominant axis */
Gesture detect_gesture(int require_home_orientation) {
    float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;

    if (require_home_orientation)
        wait_for_home_ready();

    uint32_t wait_start = HAL_GetTick();

    while (HAL_GetTick() - wait_start < MOTION_WAIT_TIMEOUT_MS) {
        read_imu(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);

        if (fabsf(gx_dps) > MOTION_START_THRESHOLD_DPS ||
            fabsf(gy_dps) > MOTION_START_THRESHOLD_DPS ||
            fabsf(gz_dps) > MOTION_START_THRESHOLD_DPS)
            break;

        update_mode_leds();
        HAL_Delay(SAMPLE_DELAY_MS);
    }

    if (HAL_GetTick() - wait_start >= MOTION_WAIT_TIMEOUT_MS)
        return NONE;

    float best_gx = 0.0f, best_gy = 0.0f, best_gz = 0.0f;
    uint32_t capture_start = HAL_GetTick();

    while (HAL_GetTick() - capture_start < GESTURE_WINDOW_MS) {
        read_imu(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);

        if (fabsf(gx_dps) > fabsf(best_gx)) best_gx = gx_dps;
        if (fabsf(gy_dps) > fabsf(best_gy)) best_gy = gy_dps;
        if (fabsf(gz_dps) > fabsf(best_gz)) best_gz = gz_dps;

        update_mode_leds();
        HAL_Delay(SAMPLE_DELAY_MS);
    }

    return classify_gyro_peak(best_gx, best_gy, best_gz);
}

/* Retry until a valid gesture is detected */
Gesture wait_for_valid_gesture(int require_home_orientation) {
    Gesture g = NONE;
    while (g == NONE) {
        g = detect_gesture(require_home_orientation);
        if (g == NONE)
            HAL_Delay(150);
    }
    return g;
}

/* Like wait_for_valid_gesture but abandons if the overall attempt has timed out */
Gesture wait_for_valid_gesture_with_timeout(int require_home_orientation,
                                            uint32_t attempt_start_tick) {
    Gesture g = NONE;
    while (g == NONE) {
        if (HAL_GetTick() - attempt_start_tick >= UNLOCK_ATTEMPT_TIMEOUT_MS)
            return NONE;

        g = detect_gesture(require_home_orientation);

        if (g == NONE) {
            if (HAL_GetTick() - attempt_start_tick >= UNLOCK_ATTEMPT_TIMEOUT_MS)
                return NONE;
            HAL_Delay(150);
        }
    }
    return g;
}

/* Wait for all gyro axes to settle before accepting the next gesture */
void wait_for_stillness(void) {
    while (1) {
        float ax, ay, az, gx, gy, gz;
        read_imu(&ax, &ay, &az, &gx, &gy, &gz);
        if (fabsf(gx) < 40.0f && fabsf(gy) < 40.0f && fabsf(gz) < 40.0f)
            break;
        update_mode_leds();
        HAL_Delay(20);
    }
    HAL_Delay(80);
}
