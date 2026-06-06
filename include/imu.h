#pragma once
#include "stm32g4xx_hal.h"
#include <stdint.h>

/* MPU-6050 I2C address and register map */
#define MPU6050_ADDR    (0x68 << 1)   /* HAL 8-bit format           */
#define WHO_AM_I        0x75
#define WHO_AM_I_VAL    0x68
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG_R  0x1C
#define OUTX_H_G        0x43          /* GYRO_XOUT_H  (big-endian)  */
#define OUTX_H_XL       0x3B          /* ACCEL_XOUT_H (big-endian)  */

extern I2C_HandleTypeDef hi2c1;

void     write_register(uint8_t reg_addr, uint8_t value);
uint8_t  read_register(uint8_t reg_addr);
void     read_registers(uint8_t start_reg, uint8_t *buffer, int length);
void     init_imu(void);
void     read_imu(float *ax_g, float *ay_g, float *az_g,
                  float *gx_dps, float *gy_dps, float *gz_dps);
