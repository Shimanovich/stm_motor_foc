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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
/* USER CODE END Includes */

/* === EXTERN "C" + ВСЕ НЕОБХОДИМЫЕ ФУНКЦИИ === */
#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

/* Полная реализация SystemClock_Config (8MHz HSE → 72MHz) */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
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

    /** Initializes the CPU, AHB and APB buses clocks
    */
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

// Моторы DC-2813C
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
    driverMot0.init();
    driverMot1.init();
    driverMot2.init();

    motor0.pole_pairs     = 7;
    motor0.voltage_limit  = 4.0f;
    motor0.velocity_limit = 30.0f;
    motor0.controller     = ControlType::velocity_openloop;

    motor1.pole_pairs     = 7;
    motor1.voltage_limit  = 4.0f;
    motor1.velocity_limit = 30.0f;
    motor1.controller     = ControlType::velocity_openloop;

    motor2.pole_pairs     = 7;
    motor2.voltage_limit  = 4.0f;
    motor2.velocity_limit = 30.0f;
    motor2.controller     = ControlType::velocity_openloop;

    motor0.linkDriver(&driverMot0);
    motor1.linkDriver(&driverMot1);
    motor2.linkDriver(&driverMot2);

    motor0.init();
    motor1.init();
    motor2.init();

    motor0.initFOC();
    motor1.initFOC();
    motor2.initFOC();

    float target_vel = 0.0f;

    for(;;)
    {
        target_vel = 15.0f;
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(4000);

        target_vel = -15.0f;
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(4000);

        target_vel = 0.0f;
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(2000);
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

    /* USER CODE BEGIN 2 */
    MX_FREERTOS_Init();

    const osThreadAttr_t motorTask_attributes = {
        .name       = "MotorTask",
        .stack_size = 4096,
        .priority   = (osPriority_t) osPriorityAboveNormal
    };

    motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);
    /* USER CODE END 2 */

    osKernelStart();

    while (1) {}
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
