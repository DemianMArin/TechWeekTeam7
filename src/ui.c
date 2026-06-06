#include "ui.h"

/*
 * CubeMX prerequisites:
 *   GPIO  : PA5  output (LD2 onboard LED)
 *   GPIO  : PC13 input, no pull (B1 user button, active low)
 *   TIM3  : CH1 PWM on PA6, PSC=169, ARR=19999  (servo, 50 Hz)
 *   TIM1  : CH2 PWM on PA9, PSC=169, ARR=999    (speaker, 1 MHz clock)
 *   Link  : add -lm to linker flags
 */

static UiMode current_ui_mode = UI_WAIT;

/* ── LED ─────────────────────────────────────────────────────────── */

void leds_off(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);
    }
}

/* Non-blocking LED animation for current UI mode — call inside polling loops */
void update_mode_leds(void) {
    static uint32_t last_tick = 0;
    static int      phase     = 0;

    int interval_ms = (current_ui_mode == UI_UNLOCKED) ? 350 : 180;

    if (HAL_GetTick() - last_tick < (uint32_t)interval_ms)
        return;

    last_tick = HAL_GetTick();
    phase     = !phase;

    switch (current_ui_mode) {
        case UI_WAIT:
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case UI_RECORD:
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case UI_LOCKED:
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            break;
        case UI_UNLOCK:
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case UI_UNLOCKED:
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            break;
    }
}

/* ── Servo ───────────────────────────────────────────────────────── */

static void servo_set(float deg) {
    int32_t pulse = (int32_t)(SERVO_CENTER_US + deg * SERVO_US_PER_DEG);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pulse);
}

static void servo_error_sweep(void) {
    servo_set(0.0f);
    HAL_Delay(150);
    servo_set(-30.0f);
    HAL_Delay(400);
    servo_set(0.0f);
}

/* ── Audio ───────────────────────────────────────────────────────── */

static void audio_set_freq(uint32_t freq_hz) {
    uint32_t arr = (AUDIO_CLK_HZ / freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, arr / 2);
}

static void audio_tone(uint32_t freq_hz, uint32_t duration_ms) {
    audio_set_freq(freq_hz);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

static void audio_ready_beeps(void) {
    audio_tone(AUDIO_READY_HZ, 80);
    HAL_Delay(80);
    audio_tone(AUDIO_READY_HZ, 80);
}

static void audio_error_beeps(void) {
    for (int i = 0; i < 3; i++) {
        audio_tone(AUDIO_ERROR_HZ, 150);
        if (i < 2) HAL_Delay(100);
    }
}

static void audio_unlock_sweep(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    for (uint32_t i = 0; i <= AUDIO_SWEEP_STEPS; i++) {
        audio_set_freq(AUDIO_SWEEP_LO_HZ +
                       (i * (AUDIO_SWEEP_HI_HZ - AUDIO_SWEEP_LO_HZ) / AUDIO_SWEEP_STEPS));
        HAL_Delay(AUDIO_SWEEP_STEP_MS);
    }
    for (uint32_t i = AUDIO_SWEEP_STEPS; i > 0; i--) {
        audio_set_freq(AUDIO_SWEEP_LO_HZ +
                       (i * (AUDIO_SWEEP_HI_HZ - AUDIO_SWEEP_LO_HZ) / AUDIO_SWEEP_STEPS));
        HAL_Delay(AUDIO_SWEEP_STEP_MS);
    }
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

/* ── Public UI functions ─────────────────────────────────────────── */

/* Brief confirmation before entering unlocked state */
void success_signal(void) {
    blink_led(2, 120);
    audio_unlock_sweep();
}

/* Wrong sequence — audio beeps then servo sweep, both fast */
void fail_signal(void) {
    audio_error_beeps();
    servo_error_sweep();
}

int button_pressed(void) {
    return HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET;
}

void wait_for_button_press(void) {
    while (!button_pressed())
        HAL_Delay(20);
    HAL_Delay(50);
    while (button_pressed())
        HAL_Delay(20);
    HAL_Delay(50);
}

/* Startup gate — silent until B1 pressed */
void wait_mode(void) {
    leds_off();
    current_ui_mode = UI_WAIT;
    while (!button_pressed())
        update_mode_leds();
    wait_for_button_press();
    audio_ready_beeps();
}

void record_mode_signal(void) {
    current_ui_mode = UI_RECORD;
    leds_off();
    audio_ready_beeps();
}

void lock_mode_signal(void) {
    current_ui_mode = UI_LOCKED;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void unlock_mode_signal(void) {
    current_ui_mode = UI_UNLOCK;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/* Hold servo open for UNLOCKED_TIMEOUT_MS or until B1 pressed again */
void unlocked_mode_wait(void) {
    current_ui_mode = UI_UNLOCKED;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    servo_set(90.0f);

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < UNLOCKED_TIMEOUT_MS) {
        update_mode_leds();
        if (button_pressed()) {
            wait_for_button_press();
            break;
        }
        HAL_Delay(20);
    }

    servo_set(0.0f);
    HAL_Delay(500);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    leds_off();
    audio_ready_beeps();
}
