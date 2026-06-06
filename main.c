/* main.c — Gesture Lock Application
 * Target: NUCLEO-G474RE | IMU: MPU-6050 (I2C1) | Servo: SG90 (TIM3 CH1)
 *         Speaker: Adafruit STEMMA (TIM2 CH1)
 *
 * CubeMX prerequisites:
 *   I2C1  : PB8=SCL, PB9=SDA, Standard 100 kHz
 *   TIM3  : CH1 PWM on PA6, PSC=169, ARR=19999 (50 Hz, 1 us resolution)
 *   TIM2  : CH2 PWM on PA1, PSC=169, ARR=999  (1 MHz clock, freq set at runtime)
 *   Link  : add -lm to linker flags
 *
 * Pin connections:
 *   MPU-6050      SCL -> PB8 (D15), SDA -> PB9 (D14), VCC -> 3.3V, AD0 -> GND
 *   SG90          PWM -> PA6 (D12), VCC -> 5V,   GND -> GND
 *   STEMMA Speaker SIG -> PA1 (A1),  VCC -> 3.3V, GND -> GND
 */

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ── Gesture tuning ─────────────────────────────────────────────── */
#define GESTURE_THRESHOLD_DEG   30.0f   /* tilt angle to register a gesture           */
#define GESTURE_HOLD_MS         500u    /* ms gesture must be sustained to confirm     */
#define NEUTRAL_THRESHOLD_DEG   5.0f    /* degrees within which neutral is confirmed   */
#define NEUTRAL_DEBOUNCE_COUNT  5       /* consecutive in-range samples required       */
#define POST_STEP_COOLDOWN_MS   500u    /* ms pause after correct step, before neutral */
#define ERROR_TIMEOUT_MS        2000u   /* ms no-input window after wrong gesture      */
#define UNLOCK_HOLD_MS          10000u  /* ms servo holds at 90 deg on successful unlock */
#define SEQUENCE_LEN            3

/* ── Servo pulse widths (us) — SG90 on 50 Hz / 1 us timer ──────── */
#define SERVO_CENTER_US         1450u
#define SERVO_US_PER_DEG        10.56f  /* (2400-500)/180 */

/* ── Audio — STEMMA Speaker (TIM2 CH1, PA0, 1 MHz timer clock) ─── */
#define AUDIO_TIMER_CLK_HZ      1000000UL
#define AUDIO_READY_FREQ_HZ     1000u   /* two quick high beeps                  */
#define AUDIO_ERROR_FREQ_HZ     300u    /* three low beeps                        */
#define AUDIO_SWEEP_LO_HZ       500u    /* unlock sweep low end                  */
#define AUDIO_SWEEP_HI_HZ       1500u   /* unlock sweep high end                 */
#define AUDIO_SWEEP_STEP_MS     10u     /* ms per frequency step in sweep        */
#define AUDIO_SWEEP_STEPS       50u     /* steps per sweep direction             */

/* ── MPU-6050 ───────────────────────────────────────────────────── */
#define MPU_ADDR                (0x68 << 1)
#define MPU_REG_PWR_MGMT_1      0x6Bu
#define MPU_REG_ACCEL_XOUT_H    0x3Bu
#define ACCEL_LSB_PER_G         16384.0f

/* USER CODE END PD */

/* USER CODE BEGIN PTD */
typedef enum { NONE=0, PITCH_FWD, PITCH_BWD, ROLL_LEFT, ROLL_RIGHT } Gesture_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD - button */
#define B1_PRESSED()  (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
/* USER CODE END PD - button */

/* USER CODE BEGIN PV */

/* ── Unlock combo — edit these three to change the combination ───── */
static const Gesture_t UNLOCK_SEQ[SEQUENCE_LEN] = {
    PITCH_FWD,
    ROLL_RIGHT,
    PITCH_BWD,
};

/* USER CODE END PV */

/* USER CODE BEGIN 0 */

extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;

/* ── Button ─────────────────────────────────────────────────────── */

static void wait_for_b1(void)
{
    while (!B1_PRESSED());    /* wait for press   */
    HAL_Delay(50);            /* debounce         */
    while (B1_PRESSED());     /* wait for release */
    HAL_Delay(50);
}

/* ── Servo ──────────────────────────────────────────────────────── */

static void servo_set(float deg)
{
    int32_t pulse = (int32_t)(SERVO_CENTER_US + deg * SERVO_US_PER_DEG);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pulse);
}

static void servo_error_feedback(void)
{
    servo_set(0.0f);
    HAL_Delay(150);
    servo_set(-30.0f);
    HAL_Delay(400);
    servo_set(0.0f);
}

static void servo_unlock(void)
{
    servo_set(90.0f);
    audio_unlock_sweep();                          /* sweep plays as servo opens    */
    HAL_Delay(UNLOCK_HOLD_MS - (AUDIO_SWEEP_STEPS * 2 * AUDIO_SWEEP_STEP_MS));
    servo_set(0.0f);
}

/* ── Audio ──────────────────────────────────────────────────────── */

static void audio_set_freq(uint32_t freq_hz)
{
    uint32_t arr = (AUDIO_TIMER_CLK_HZ / freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 2);
}

static void audio_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    audio_set_freq(freq_hz);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

static void audio_ready(void)
{
    audio_tone(AUDIO_READY_FREQ_HZ, 80);
    HAL_Delay(80);
    audio_tone(AUDIO_READY_FREQ_HZ, 80);
}

static void audio_error(void)
{
    for (int i = 0; i < 3; i++) {
        audio_tone(AUDIO_ERROR_FREQ_HZ, 150);
        if (i < 2) HAL_Delay(100);
    }
}

static void audio_unlock_sweep(void)
{
    /* 500 Hz -> 1500 Hz -> 500 Hz, one step every AUDIO_SWEEP_STEP_MS */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    for (uint32_t i = 0; i <= AUDIO_SWEEP_STEPS; i++) {
        audio_set_freq(AUDIO_SWEEP_LO_HZ + (i * (AUDIO_SWEEP_HI_HZ - AUDIO_SWEEP_LO_HZ) / AUDIO_SWEEP_STEPS));
        HAL_Delay(AUDIO_SWEEP_STEP_MS);
    }
    for (uint32_t i = AUDIO_SWEEP_STEPS; i > 0; i--) {
        audio_set_freq(AUDIO_SWEEP_LO_HZ + (i * (AUDIO_SWEEP_HI_HZ - AUDIO_SWEEP_LO_HZ) / AUDIO_SWEEP_STEPS));
        HAL_Delay(AUDIO_SWEEP_STEP_MS);
    }
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

/* ── MPU-6050 ───────────────────────────────────────────────────── */

static void mpu_init(void)
{
    uint8_t wake = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &wake, 1, HAL_MAX_DELAY);
}

static void mpu_read_accel(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, raw, 6, HAL_MAX_DELAY);

    *ax = (int16_t)((raw[0] << 8) | raw[1]) / ACCEL_LSB_PER_G;
    *ay = (int16_t)((raw[2] << 8) | raw[3]) / ACCEL_LSB_PER_G;
    *az = (int16_t)((raw[4] << 8) | raw[5]) / ACCEL_LSB_PER_G;
}

static void compute_tilt(float *pitch, float *roll)
{
    float ax, ay, az;
    mpu_read_accel(&ax, &ay, &az);
    *pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * (180.0f / (float)M_PI);
    *roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * (180.0f / (float)M_PI);
}

/* ── Gesture detection ──────────────────────────────────────────── *
 * Returns a confirmed gesture only after it has been held for
 * GESTURE_HOLD_MS. Resets pending state when user returns to neutral.
 */
static Gesture_t detect_gesture(void)
{
    static Gesture_t  pending    = NONE;
    static uint32_t   pending_ts = 0;

    float pitch, roll;
    compute_tilt(&pitch, &roll);

    Gesture_t current = NONE;
    if      (pitch >  GESTURE_THRESHOLD_DEG) current = PITCH_FWD;
    else if (pitch < -GESTURE_THRESHOLD_DEG) current = PITCH_BWD;
    else if (roll  >  GESTURE_THRESHOLD_DEG) current = ROLL_RIGHT;
    else if (roll  < -GESTURE_THRESHOLD_DEG) current = ROLL_LEFT;

    if (current == NONE) {
        pending = NONE;
        return NONE;
    }

    if (current != pending) {
        pending    = current;
        pending_ts = HAL_GetTick();
        return NONE;
    }

    if ((HAL_GetTick() - pending_ts) >= GESTURE_HOLD_MS) {
        Gesture_t confirmed = pending;
        pending = NONE;
        return confirmed;
    }

    return NONE;
}

/* ── Neutral-return gate ────────────────────────────────────────── *
 * Blocks until NEUTRAL_DEBOUNCE_COUNT consecutive samples are within
 * NEUTRAL_THRESHOLD_DEG on both axes. Any out-of-range sample resets
 * the counter.
 */
static void wait_for_neutral(void)
{
    uint8_t count = 0;
    while (count < NEUTRAL_DEBOUNCE_COUNT) {
        HAL_Delay(10);
        float pitch, roll;
        compute_tilt(&pitch, &roll);
        if (fabsf(pitch) < NEUTRAL_THRESHOLD_DEG && fabsf(roll) < NEUTRAL_THRESHOLD_DEG)
            count++;
        else
            count = 0;
    }
}

/* USER CODE END 0 */

/*
 * Call lock_run() from main() after all MX_xxx_Init() calls complete.
 *
 * In the generated main():
 *   1. Replace the while(1) loop contents with a call to lock_run(), or
 *   2. Paste the USER CODE BEGIN 2 and USER CODE BEGIN WHILE blocks below
 *      into the corresponding sections of the generated main.c.
 */

/* USER CODE BEGIN 2 */
/*
    mpu_init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    servo_set(0.0f);
    wait_for_b1();    /* system is silent until B1 pressed    */
    audio_ready();    /* two beeps: now ready for gesture input */
    int step = 0;
*/
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
/*
    while (1)
    {
        HAL_Delay(10);

        if (B1_PRESSED()) {
            HAL_Delay(50);          /* debounce         */
            while (B1_PRESSED());   /* wait for release */
            step = 0;
            audio_ready();          /* two beeps: reset confirmed */
            continue;
        }

        Gesture_t g = detect_gesture();
        if (g == NONE) continue;

        if (g == UNLOCK_SEQ[step]) {
            step++;
            if (step == SEQUENCE_LEN) {
                servo_unlock();       /* sweep plays simultaneously with open  */
                step = 0;
                audio_ready();        /* two beeps: back to idle, ready again  */
            } else {
                HAL_Delay(POST_STEP_COOLDOWN_MS);
                wait_for_neutral();
            }
        } else {
            audio_error();            /* three low beeps before servo feedback */
            servo_error_feedback();
            HAL_Delay(ERROR_TIMEOUT_MS);
            step = 0;
        }
    }
*/
/* USER CODE END WHILE */
