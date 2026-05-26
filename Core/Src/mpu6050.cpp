#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c,uint8_t addr)
{
    uint8_t check = 0;
    HAL_StatusTypeDef ret;

    // Проверка WHO_AM_I
    ret = HAL_I2C_Mem_Read(hi2c, addr, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    if (ret != HAL_OK || check != 0x68) return HAL_ERROR;

    // 1. Wake up + PLL from gyro
    uint8_t data = 0x01;
    ret = HAL_I2C_Mem_Write(hi2c, addr, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(100);

    // 2. Sample rate 1000 Гц
    data = 0x00;  // SMPLRT_DIV = 0
    ret = HAL_I2C_Mem_Write(hi2c, addr, MPU6050_SMPLRT_DIV, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    // 3. DLPF = 98 Гц gyro / 94 Гц accel
    data = 0x02;
    ret = HAL_I2C_Mem_Write(hi2c, addr, MPU6050_CONFIG, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    // 4. Gyro ±2000 °/с
    data = 0x18;  // FS_SEL = 3
    ret = HAL_I2C_Mem_Write(hi2c, addr, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    // 5. Accel ±8 g
    data = 0x10;  // AFS_SEL = 2
    ret = HAL_I2C_Mem_Write(hi2c, addr, MPU6050_ACCEL_CONFIG, 1, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c,uint8_t addr, MPU6050_RawData_t *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, addr, MPU6050_ACCEL_XOUT_H,
                                             1, buf, 14, 100);
    if (ret != HAL_OK) return ret;

    data->ax   = (int16_t)(buf[0] << 8 | buf[1]);
    data->ay   = (int16_t)(buf[2] << 8 | buf[3]);
    data->az   = (int16_t)(buf[4] << 8 | buf[5]);
    data->temp = (int16_t)(buf[6] << 8 | buf[7]);
    data->gx   = (int16_t)(buf[8] << 8 | buf[9]);
    data->gy   = (int16_t)(buf[10] << 8 | buf[11]);
    data->gz   = (int16_t)(buf[12] << 8 | buf[13]);

    return HAL_OK;
}
