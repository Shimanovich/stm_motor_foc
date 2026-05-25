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
#include "foc_utils.h"      // если требуется
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId_t motorTaskHandle;

/* USER CODE BEGIN PV */

// === Драйверы моторов (3PWM) ===
BLDCDriver3PWM driverMot0(&htim3, TIM_CHANNEL_2,   // Mot0: PA7  TIM3_CH2
                          &htim3, TIM_CHANNEL_3,   //       PB0  TIM3_CH3
                          &htim3, TIM_CHANNEL_4);  //       PB1  TIM3_CH4

BLDCDriver3PWM driverMot1(&htim3, TIM_CHANNEL_1,   // Mot1: PA6  TIM3_CH1
                          &htim2, TIM_CHANNEL_3,   //       PA2  TIM2_CH3
                          &htim2, TIM_CHANNEL_4);  //       PA3  TIM2_CH4

BLDCDriver3PWM driverMot2(&htim2, TIM_CHANNEL_2,   // Mot2: PA1  TIM2_CH2
                          &htim4, TIM_CHANNEL_3,   //       PB8  TIM4_CH3
                          &htim4, TIM_CHANNEL_4);  //       PB9  TIM4_CH4

// === Моторы (DC-2813C — 7 пар полюсов) ===
BLDCMotor motor0 = BLDCMotor(7);   // Mot0
BLDCMotor motor1 = BLDCMotor(7);   // Mot1
BLDCMotor motor2 = BLDCMotor(7);   // Mot2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
void MotorTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MotorTask(void *argument)
{
    /* Инициализация драйверов */
    driverMot0.init();
    driverMot1.init();
    driverMot2.init();

    /* Настройка моторов (DC-2813C) */
    motor0.pole_pairs     = 7;
    motor0.voltage_limit  = 4.0f;      // безопасное значение на старте
    motor0.velocity_limit = 30.0f;
    motor0.controller     = MotionControlType::velocity_openloop;

    motor1.pole_pairs     = 7;
    motor1.voltage_limit  = 4.0f;
    motor1.velocity_limit = 30.0f;
    motor1.controller     = MotionControlType::velocity_openloop;

    motor2.pole_pairs     = 7;
    motor2.voltage_limit  = 4.0f;
    motor2.velocity_limit = 30.0f;
    motor2.controller     = MotionControlType::velocity_openloop;

    /* Привязка драйверов */
    motor0.linkDriver(&driverMot0);
    motor1.linkDriver(&driverMot1);
    motor2.linkDriver(&driverMot2);

    /* Инициализация FOC (даже в open-loop режиме) */
    motor0.init();
    motor1.init();
    motor2.init();

    motor0.initFOC();
    motor1.initFOC();
    motor2.initFOC();

    float target_vel = 0.0f;

    for(;;)
    {
        // Тест: плавное вращение вперёд-назад на всех трёх моторах одновременно
        target_vel = 15.0f;                     // rad/s ≈ 143 об/мин
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(4000);                          // 4 секунды вперёд

        target_vel = -15.0f;
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(4000);                          // 4 секунды назад

        target_vel = 0.0f;
        motor0.move(target_vel);
        motor1.move(target_vel);
        motor2.move(target_vel);
        osDelay(2000);                          // пауза 2 секунды
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C2_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */

    MX_FREERTOS_Init();

    /* Создаём задачу управления моторами */
    const osThreadAttr_t motorTask_attributes = {
        .name = "MotorTask",
        .priority = (osPriority_t) osPriorityAboveNormal,
        .stack_size = 4096     // 4 КБ стек (достаточно для SimpleFOC)
    };

    motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);

    /* USER CODE END 2 */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
