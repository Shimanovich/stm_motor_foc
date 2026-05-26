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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"

#include <stdio.h>

/* USER CODE END Includes */

/* === EXTERN "C" + ВСЕ НЕОБХОДИМЫЕ ФУНКЦИИ === */
#ifdef __cplusplus
extern "C" {
#endif

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

/* Полная реализация SystemClock_Config (HSE 8MHz → 72MHz) */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_FREERTOS_Init(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_I2C2_Init(void);
void MX_USART3_UART_Init(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
/* ============================================== */

/* Private variables ---------------------------------------------------------*/
osThreadId_t motorTaskHandle;

/* USER CODE BEGIN PV */
// Драйверы
BLDCDriver3PWM driverMot0(&htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_4);
BLDCDriver3PWM driverMot1(&htim3, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4);
BLDCDriver3PWM driverMot2(&htim2, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_3, &htim4, TIM_CHANNEL_4);

// Моторы DC-2813C (7 pole pairs)
BLDCMotor motor0 = BLDCMotor(7);
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MotorTask(void *argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void MotorTask(void *argument)
{
    // === НАСТРОЙКИ ДЛЯ DC-2813C + 7 В ===
    float vm = 7.0f;
    driverMot0.voltage_power_supply = vm;

    driverMot0.voltage_limit = 4.0f;
    motor0.pole_pairs     = 14;

    motor0.voltage_limit  = 3.0f;
    motor0.velocity_limit = 60.0f;       // можно выше
    motor0.controller     = ControlType::velocity_openloop;

    driverMot0.init();
    motor0.linkDriver(&driverMot0);
    motor0.init();
    motor0.sensor = nullptr;

    motor0.enable();

    float target_vel = 0.0f;   // ← начинаем с комфортной скорости

    //printf("Started\n");

    float inc = 0.002;

    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

        motor0.move(target_vel);

        target_vel +=inc;

        if (target_vel>50) inc=-inc;

        if (target_vel<-50) inc=-inc;


        //osDelay(1);

    }
}



void Test_Mot0_PWM(void)
{
    // Запускаем PWM (на всякий случай)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7  = TIM3_CH2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0  = TIM3_CH3
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1  = TIM3_CH4

    const uint16_t period = htim3.Init.Period;           // обычно 65535
    uint16_t duty = (uint16_t)(period * 0.30f);          // 30% — начинаем с этого

    while (1)
    {
        // Фиксированная позиция (ротор должен сильно дёрнуться и удерживаться)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);   // Phase A
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);      // Phase B
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);      // Phase C

        HAL_Delay(2000);   // держим 2 секунды

        // Меняем позицию (чтобы увидеть, что мотор реагирует)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

        HAL_Delay(2000);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);

        HAL_Delay(2000);
    }
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C2_Init();
    MX_USART3_UART_Init();


    /* === FreeRTOS === */
    MX_FREERTOS_Init();               // инициализация объектов
    osKernelInitialize();             // ← ЯВНЫЙ вызов

    //MotorTask(NULL);


    /* Создаём задачу с умеренным стеком */
    const osThreadAttr_t motorTask_attributes = {
        .name       = "MotorTask",
        .stack_size = 3072,                    // 3 КБ — достаточно
        .priority   = (osPriority_t) osPriorityAboveNormal
    };

    motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);

    if (motorTaskHandle == NULL) {
        // быстрый красный — задача не создана
        while(1) { HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13); osDelay(80); }
    }

    osStatus_t status = osKernelStart();

    // Если дошли сюда — osKernelStart() вернул osError
    while(1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);   // медленный красный
        osDelay(500);
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
