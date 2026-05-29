/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body для STorM32-BGC v1.30 + SimpleFOC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
//#include "stdio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"

#include "mpu6050.h"
#include "notch_filter.h"
#include "fft.h"
#include "common/pid.h"   // <-- Добавлено PID из проекта

#include "FreeRTOS.h"
#include "queue.h"

#define SAMPLES 128U          // должно быть степенью 2

typedef struct {
    float    data[SAMPLES];   // значения акселерометра
    float    fs;              // реальная частота дискретизации (Гц)
    uint32_t timestamp;       // метка времени (например, HAL_GetTick())
    uint32_t reserved;        // выравнивание
} AccelPacket_t;

// ... (остальной код без изменений до MotorTask) 

// Вставьте это перед определением MotorTask или глобально
static PIDController pid_pitch(35.0f, 12.0f, 1.5f, 5000.0f, 35.0f);
static PIDController pid_yaw(28.0f, 8.0f, 1.0f, 5000.0f, 35.0f);

void MotorTask(void *argument)
{
    // ... (весь код инициализации MPU, моторов, калибровки sum1/sum2 остаётся)

    motor0.enable();
    motor1.enable();

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

    float gyro1_y;
    float gyro2_z;

    uint32_t tick = osKernelGetTickCount();           
    const uint32_t PERIOD_MS = 2;            

    for (;;)
    {
        MPU6050_ReadRaw(&hi2c2, 0xd0, &mpu1_data);
        gyro1_y = mpu1_data.gy * (2000.0f / 32768.0f) * (3.1415926535f / 180.0f) ; // pitch
        // gyro1_y = Notch_Update(&gyro_notch, gyro1_y);   // можно оставить

        float error_pitch = 0.0f - (gyro1_y - sum1);  // setpoint = 0
        float vel_cmd_pitch = pid_pitch(error_pitch);
        motor0.move(vel_cmd_pitch);

        MPU6050_ReadRaw(&hi2c1, 0xd2, &mpu2_data);
        gyro2_z = mpu2_data.gz * (2000.0f / 32768.0f) * (3.1415926535f / 180.0f) ; // yaw
        float error_yaw = 0.0f - (gyro2_z + sum2);
        float vel_cmd_yaw = pid_yaw(error_yaw);
        motor1.move(vel_cmd_yaw);  // знак может потребовать корректировки

        // остальной код (zero crossing, delay и т.д.)

        tick += PERIOD_MS;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        osDelayUntil(tick);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }
}

// ... (остальной код main и функций)
