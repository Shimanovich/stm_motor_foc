#pragma once
#include "stm32f1xx_hal.h"   // или ваш MCU


#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_INT_PIN_CFG   0x37
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_GYRO_XOUT_H   0x43
#define MPU6050_TEMP_OUT_H    0x41

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} MPU6050_RawData_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c,uint8_t addr);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c,uint8_t addr, MPU6050_RawData_t *data);
