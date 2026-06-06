#include "imu.h"

/*
 * CubeMX prerequisites:
 *   I2C1: PB8=SCL, PB9=SDA, Fast Mode 400 kHz
 */

void write_register(uint8_t reg_addr, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

uint8_t read_register(uint8_t reg_addr) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr,
                     I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

void read_registers(uint8_t start_reg, uint8_t *buffer, int length) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, start_reg,
                     I2C_MEMADD_SIZE_8BIT, buffer, length, HAL_MAX_DELAY);
}

void init_imu(void) {
    write_register(PWR_MGMT_1,   0x00);  /* wake from sleep                   */
    write_register(SMPLRT_DIV,   0x09);  /* 100 Hz sample rate (DLPF on)      */
    write_register(MPU_CONFIG,   0x03);  /* DLPF cfg 3 — 44 Hz bandwidth      */
    write_register(GYRO_CONFIG,  0x10);  /* ±1000 dps full scale               */
    write_register(ACCEL_CONFIG_R, 0x08); /* ±4g full scale                   */
}

void read_imu(float *ax_g, float *ay_g, float *az_g,
              float *gx_dps, float *gy_dps, float *gz_dps) {
    /* MPU-6050 registers are big-endian (H byte first) */
    const float GYRO_SENS  = 32.8f;    /* LSB/dps at ±1000 dps               */
    const float ACCEL_SENS = 8192.0f;  /* LSB/g   at ±4g                     */

    uint8_t gyro_data[6];
    uint8_t accel_data[6];

    read_registers(OUTX_H_G,  gyro_data,  6);
    read_registers(OUTX_H_XL, accel_data, 6);

    *gx_dps = (int16_t)((gyro_data[0]  << 8) | gyro_data[1])  / GYRO_SENS;
    *gy_dps = (int16_t)((gyro_data[2]  << 8) | gyro_data[3])  / GYRO_SENS;
    *gz_dps = (int16_t)((gyro_data[4]  << 8) | gyro_data[5])  / GYRO_SENS;

    *ax_g   = (int16_t)((accel_data[0] << 8) | accel_data[1]) / ACCEL_SENS;
    *ay_g   = (int16_t)((accel_data[2] << 8) | accel_data[3]) / ACCEL_SENS;
    *az_g   = (int16_t)((accel_data[4] << 8) | accel_data[5]) / ACCEL_SENS;
}
