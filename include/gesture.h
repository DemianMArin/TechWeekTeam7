#pragma once
#include "imu.h"
#include "ui.h"
#include <stdint.h>

#define GESTURE_WINDOW_MS           400
#define SAMPLE_DELAY_MS             10
#define MOTION_START_THRESHOLD_DPS  80.0f
#define MOTION_WAIT_TIMEOUT_MS      5000
#define HOME_AZ_MIN_G               0.70f
#define HOME_AY_MAX_G               0.50f
#define HOME_AX_MAX_G               0.50f
#define HOME_STILL_GYRO_MAX_DPS     35.0f
#define HOME_STABLE_TIME_MS         300

typedef enum {
    NONE,
    GX_POS, GX_NEG,
    GY_POS, GY_NEG,
    GZ_POS, GZ_NEG
} Gesture;

const char *gesture_name(Gesture g);
Gesture     classify_gyro_peak(float gx_peak, float gy_peak, float gz_peak);
int         is_home_orientation(float ax_g, float ay_g, float az_g);
void        wait_for_home_ready(void);
Gesture     detect_gesture(int require_home_orientation);
Gesture     wait_for_valid_gesture(int require_home_orientation);
Gesture     wait_for_valid_gesture_with_timeout(int require_home_orientation,
                                                uint32_t attempt_start_tick);
void        wait_for_stillness(void);
