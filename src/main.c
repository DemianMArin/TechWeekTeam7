/*
 * main.c — Gesture Lock (refactored from charbelhad/embedded-gesture-lock)
 *
 * Target  : NUCLEO-G474RE
 * IMU     : MPU-6050  — I2C1, PB8=SCL / PB9=SDA
 * Servo   : SG90      — TIM3 CH1, PA6 (D12)
 * Speaker : STEMMA    — TIM1 CH2, PA9 (D8)
 * Button  : B1        — PC13 (active low)
 *
 * CubeMX setup:
 *   I2C1  : Fast Mode 400 kHz, PB8/PB9
 *   TIM3  : CH1 PWM on PA6,  PSC=169, ARR=19999  (50 Hz servo)
 *   TIM1  : CH2 PWM on PA9,  PSC=169, ARR=999    (audio, 1 MHz clock)
 *   GPIO  : PA5 output (LD2), PC13 input no-pull (B1)
 *   Link  : add -lm to linker flags
 *
 * Startup flow:
 *   1. Wait for B1 press (system silent)
 *   2. Record a 3-gesture key sequence (B1 starts recording)
 *   3. Loop: B1 press → unlock attempt → servo opens 10 s if correct
 */

#include "imu.h"
#include "ui.h"
#include "gesture.h"
#include "lock_system.h"

/* HAL handles declared by CubeMX-generated code */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim3;

int app_main(void) {
    leds_off();

    /* Verify MPU-6050 is present before proceeding */
    if (read_register(WHO_AM_I) != WHO_AM_I_VAL) {
        while (1) {
            fail_signal();
            HAL_Delay(1000);
        }
    }

    init_imu();

    /* Wait for user to press B1, then record the unlock key */
    wait_mode();
    record_key();

    while (1) {
        lock_mode_signal();
        wait_for_button_press();

        int unlocked = unlock_attempt();

        if (unlocked)
            unlocked_mode_wait();

        HAL_Delay(300);
    }
}
