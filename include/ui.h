#pragma once
#include "stm32g4xx_hal.h"

#define UNLOCKED_TIMEOUT_MS       10000
#define UNLOCK_ATTEMPT_TIMEOUT_MS 30000

/* ── Pin definitions ────────────────────────────────────────────── *
 * LD2 (onboard LED) : PA5  — active high                           *
 * B1  (user button) : PC13 — active low                            *
 * SG90 servo PWM    : PA6  — TIM3 CH1                              *
 * STEMMA speaker    : PA9  — TIM1 CH2                              *
 */
#define LED_PORT            GPIOA
#define LED_PIN             GPIO_PIN_5
#define BTN_PORT            GPIOC
#define BTN_PIN             GPIO_PIN_13

/* Servo (TIM3 CH1, 1 MHz timer clock, 50 Hz) */
#define SERVO_CENTER_US     1450u
#define SERVO_US_PER_DEG    10.56f

/* Audio (TIM1 CH2, 1 MHz timer clock) */
#define AUDIO_CLK_HZ        1000000UL
#define AUDIO_READY_HZ      1000u
#define AUDIO_ERROR_HZ      300u
#define AUDIO_SWEEP_LO_HZ   500u
#define AUDIO_SWEEP_HI_HZ   1500u
#define AUDIO_SWEEP_STEPS   50u
#define AUDIO_SWEEP_STEP_MS 10u

typedef enum {
    UI_WAIT,
    UI_RECORD,
    UI_LOCKED,
    UI_UNLOCK,
    UI_UNLOCKED
} UiMode;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void leds_off(void);
void blink_led(int times, int delay_ms);
void update_mode_leds(void);
void success_signal(void);
void fail_signal(void);
int  button_pressed(void);
void wait_for_button_press(void);
void wait_mode(void);
void record_mode_signal(void);
void lock_mode_signal(void);
void unlock_mode_signal(void);
void unlocked_mode_wait(void);
