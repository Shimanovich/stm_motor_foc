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

#include "FreeRTOS.h"
#include "queue.h"
#include "common/pid.h"

#define SAMPLES 128U          // должно быть степенью 2

typedef struct {
    float    data[SAMPLES];   // значения акселерометра
    float    fs;              // реальная частота дискретизации (Гц)
    uint32_t timestamp;       // метка времени (например, HAL_GetTick())
    uint32_t reserved;        // выравнивание
} AccelPacket_t;

// Пример начальных коэффициентов (подбирайте экспериментально!)
static PIDController pid_pitch(0.1f, 0.0f, 0.0f, 500.0f, 25.0f);  // P,I,D,ramp,limit
static PIDController   pid_yaw(1.0f, 0.0f, 0.0f, 500.0f, 25.0f);


//void init(void) {
//    zc_sliding_init(&zc, sample_buffer, BUFFER_SIZE, WINDOW_DURATION, 0.08f, SAMPLE_RATE);
//}
//
//void process_sample(float adc_value) {
//    float freq = zc_sliding_update(&zc, adc_value, SAMPLE_RATE);
//
//    if (freq > 0.1f) {
//        // используем freq
//    }
//}

typedef struct {
    // Кольцевой буфер для хранения отсчётов
    float* buffer;              // указатель на массив
    uint32_t buffer_size;       // размер буфера (должен быть >= samples_in_window)
    uint32_t head;              // индекс для записи нового значения

    uint32_t sample_count;      // общее количество принятых отсчётов
    uint32_t window_samples;    // количество отсчётов в окне (фиксированная длительность)

    uint32_t zero_crossings;    // текущее количество пересечений в окне
    float prev_sample;          // предыдущее значение (для детекции)

    float last_frequency;       // последняя рассчитанная частота
    float hysteresis;           // гистерезис

    float window_duration;      // длительность окна в секундах (например 0.5f)
} ZeroCrossingSlidingWindow;


#define WINDOW_DURATION  1.0f     // 0.5 секунды
#define SAMPLE_RATE     500.0f
#define BUFFER_SIZE     1024      // должно быть больше чем WINDOW_DURATION * SAMPLE_RATE

float sample_buffer[BUFFER_SIZE];
ZeroCrossingSlidingWindow zc = {0};


void zc_sliding_init(ZeroCrossingSlidingWindow* state,
                     float* buffer,
                     uint32_t buffer_size,
                     float window_duration_sec,
                     float hysteresis,
                     float sample_rate)
{
    state->buffer = buffer;
    state->buffer_size = buffer_size;
    state->window_samples = (uint32_t)(window_duration_sec * sample_rate + 0.5f);

    // Защита от слишком большого окна
    if (state->window_samples > buffer_size) {
        state->window_samples = buffer_size;
    }

    state->head = 0;
    state->sample_count = 0;
    state->zero_crossings = 0;
    state->prev_sample = 0.0f;
    state->last_frequency = 0.0f;
    state->hysteresis = hysteresis;
    state->window_duration = window_duration_sec;

    // Очистка буфера
    for (uint32_t i = 0; i < buffer_size; i++) {
        state->buffer[i] = 0.0f;
    }
}

/**
 * Обновление по одному новому отсчёту
 *
 * @param state       - состояние
 * @param new_sample  - новое значение сигнала
 * @param sample_rate - частота дискретизации
 * @return            - рассчитанная частота в Гц
 */
float zc_sliding_update(ZeroCrossingSlidingWindow* state, float new_sample, float sample_rate)
{
    if (sample_rate <= 0.0f || state->window_samples < 10) {
        return 0.0f;
    }

    // Добавляем новое значение в кольцевой буфер
    uint32_t old_index = state->head;
    float old_sample = state->buffer[old_index];        // значение, которое вытесняем

    state->buffer[state->head] = new_sample;
    state->head = (state->head + 1) % state->buffer_size;
    state->sample_count++;

    // === Обработка пересечений при добавлении нового отсчёта ===
    if ((state->prev_sample <= -state->hysteresis && new_sample > state->hysteresis) ||
        (state->prev_sample >= state->hysteresis && new_sample < -state->hysteresis)) {
        state->zero_crossings++;
    }

    // === Удаляем пересечения, которые вышли за пределы окна ===
    if (state->sample_count > state->window_samples) {
        // Проверяем, было ли пересечение на вытесненном значении
        float next_sample = state->buffer[(old_index + 1) % state->buffer_size]; // следующее после вытесненного

        if ((old_sample <= -state->hysteresis && next_sample > state->hysteresis) ||
            (old_sample >= state->hysteresis && next_sample < -state->hysteresis)) {
            if (state->zero_crossings > 0) {
                state->zero_crossings--;
            }
        }
    }

    state->prev_sample = new_sample;

    // Расчёт частоты (только когда окно заполнено)
    if (state->sample_count >= state->window_samples) {
        float periods = state->zero_crossings / 2.0f;
        float frequency = periods / state->window_duration;

        state->last_frequency = frequency;
    }

    return state->last_frequency;
}


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


// Глобальные/статические переменные
NotchFilter gyro_notch;
NotchFilter common_notch;
float gyro_rate_filtered_rad_s = 0.0f;

#define FREQ_BUF_CNT  2


static uint8_t QueueStorageBufferPtrs[FREQ_BUF_CNT];
static AccelPacket_t QueueStorageBuffer[FREQ_BUF_CNT];


//
//static uint8_t datato_calc_stora[FREQ_BUF_CNT * 4];
//static StaticQueue_t accelQueueControlBlock;                      // control block
//
//static const osMessageQueueAttr_t accelQueueAttr = {
//    .name      = "accelQueue",
//    .attr_bits = 0U,
//    .cb_mem    = &accelQueueControlBlock,      // ← ЗАПОЛНЕНО СТАТИЧЕСКИ
//    .cb_size   = sizeof(StaticQueue_t),        // ← ЗАПОЛНЕНО СТАТИЧЕСКИ
//    .mq_mem    = accelQueueStorage,            // данные пакетов
//    .mq_size   = sizeof(accelQueueStorage)
//};

osMessageQueueId_t data_to_calc_QueueHandle;
osMessageQueueId_t data_from_calc_QueueHandle;



// В init (например, в main() или task init)
void Stabilization_Init(void)
{
    // fs — частота вызова фильтра (например 1000 Гц)
    // f_notch — частота вашего резонанса (измерьте осциллографом/FFT)
    // Q — добротность (обычно 5…30, чем выше — уже режекция)
    Notch_Init(&gyro_notch, 500.0f, 104.0f, 30.0f);   // пример: 85 Гц, Q=10
    Notch_Init(&common_notch, 1000.0f, 140.8f, 30.0f);   // пример: 85 Гц, Q=10

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


extern I2C_HandleTypeDef hi2c2;
MPU6050_RawData_t mpu1_data;
MPU6050_RawData_t mpu2_data;


void I2C_ScanExternalBus(I2C_HandleTypeDef *hi2c);


//// Задача в стиле CMSIS-RTOS2
//void MPU6050_Task(void *argument)
//{
//
//    I2C_ScanExternalBus(&hi2c2);
//    I2C_ScanExternalBus(&hi2c1);
//
//
//    if (MPU6050_Init(&hi2c2,0xd0) != HAL_OK) {
//        printf("MPU6050: init error!\r\n");
//        for(;;);                    // остановка задачи
//    }
//    printf("MPU6050: init Ok  (adr 0x69)\r\n");
//
//
//    if (MPU6050_Init(&hi2c1,0xd2) != HAL_OK) {
//            printf("MPU6050: init error!\r\n");
//            for(;;);                    // остановка задачи
//        }
//        printf("MPU6050: init Ok  (adr 0x69)\r\n");
//
//
//
//    uint32_t tick = osKernelGetTickCount();   // для точного периодического вызова
//    uint32_t print_counter = 0;
//
//    for (;;) {
//        MPU6050_ReadRaw(&hi2c2,0xd0, &mpu1_data);
//        MPU6050_ReadRaw(&hi2c1,0xd2, &mpu2_data);
//
//
//        // Конвертация в физические единицы (оптимально для стабилизации)
////        float accel_x = mpu1_data.ax * (8.0f / 32768.0f);   // ±8 g
////        float accel_y = mpu1_data.ay * (8.0f / 32768.0f);
////        float accel_z = mpu1_data.az * (8.0f / 32768.0f);
//
//        float gyro1_x = mpu1_data.gx * (2000.0f / 32768.0f); // ±2000 °/с
//        float gyro1_y = mpu1_data.gy * (2000.0f / 32768.0f);
//        float gyro1_z = mpu1_data.gz * (2000.0f / 32768.0f);
//
//        float gyro2_x = mpu2_data.gx * (2000.0f / 32768.0f); // ±2000 °/с
//        float gyro2_y = mpu2_data.gy * (2000.0f / 32768.0f);
//        float gyro2_z = mpu2_data.gz * (2000.0f / 32768.0f);
//
//        float temperature = (mpu1_data.temp / 340.0f) + 36.53f;
//
//        // Вывод каждые ~100 мс
//        if (++print_counter >= 100) {
//            print_counter = 0;
//            //printf("MPU6050 | Acc: %.3f %.3f %.3f g | Gyro: %.2f %.2f %.2f °/s | Temp: %.2f °C\r\n",
//            printf(">g1x:%f\n",gyro1_x);
//            printf(">g1y:%f\n",gyro1_y);
//            printf(">g1z:%f\n",gyro1_z);
//
//            printf(">g2x:%f\n",gyro2_x);
//            printf(">g2y:%f\n",gyro2_y);
//            printf(">g2z:%f\n",gyro2_z);
////                   accel_x, accel_y, accel_z,
////                   gyro_x, gyro_y, gyro_z,
////                   temperature);
//        }
//
//        tick += 1;                          // 1 мс (1000 Гц)
//        osDelayUntil(tick);
//    }
//}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MotorTask(void *argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

float vReal[SAMPLES];
float vImag[SAMPLES];


AccelPacket_t* pkt ;
void StartFFT_Task(void *argument)
{
	pkt=NULL;

	uint8_t id;
    printf("\r\n=== FFT Task started (CMSIS-RTOS2) ===\r\n");



    for (;;) {
        // Ждём заполненный буфер
        osStatus_t status = osMessageQueueGet(data_to_calc_QueueHandle,
                                              &id,
                                              NULL,
                                              osWaitForever);
    	pkt = &QueueStorageBuffer[id];
        if (status == osOK) {
            // === Обработка FFT ===
            for (uint16_t i = 0; i < SAMPLES; i++) {
                vReal[i] = pkt->data[i];
                vImag[i] = 0.0f;
            }


            dc_removal(vReal, SAMPLES);
            window_hamming(vReal, SAMPLES);
            fft(vReal, vImag, SAMPLES);
            complex_to_magnitude(vReal, SAMPLES);

            float peak_freq = find_peak_frequency(vReal, pkt->fs, SAMPLES);

            printf("[%lu] Peak frequency: %.2f Hz (fs=%.1f Hz)\r\n",
                   pkt->timestamp, peak_freq, pkt->fs);


            osStatus_t put_status = osMessageQueuePut(data_from_calc_QueueHandle, &id, 0U, 0U);

            if (put_status != osOK) {
                printf("FFT: failed to return buffer to pool!\r\n");
            }

            pkt = NULL;   // очищаем локальную переменную
        }
    }
}


void MotorTask(void *argument)
{
    I2C_ScanExternalBus(&hi2c2);
    I2C_ScanExternalBus(&hi2c1);

    static AccelPacket_t * pkt = NULL;   // static + обнуление
    static uint16_t pkt_pos = 0;

    if (MPU6050_Init(&hi2c2, 0xd0) != HAL_OK) {
        printf("MPU6050: init error!\r\n");
        for (;;); // остановка задачи
    }
    printf("MPU6050: init Ok (adr 0x69)\r\n");

    while (MPU6050_Init(&hi2c1, 0xd2) != HAL_OK) {
    	 I2C_ScanExternalBus(&hi2c1);
        printf("MPU6050: init error!\r\n");
       // for (;;); // остановка задачи
    }
    printf("MPU6050: init Ok (adr 0x69)\r\n");

    // === НАСТРОЙКИ ДЛЯ DC-2813C + 7 В ===
    float vm = 7.0f;
    driverMot0.voltage_power_supply = vm;
    driverMot0.voltage_limit = 7.0f;
    motor0.pole_pairs = 7;
    motor0.voltage_limit = 3.0f;
    motor0.velocity_limit = 30.1f;
    motor0.controller = ControlType::velocity_openloop;

    driverMot1.voltage_power_supply = vm;
    driverMot1.voltage_limit = 7.0f;
    motor1.pole_pairs = 7;
    motor1.voltage_limit = 3.0f;
    motor1.velocity_limit = 30.0f;
    motor1.controller = ControlType::velocity_openloop;

    driverMot2.voltage_power_supply = vm;
    driverMot2.voltage_limit = 7.0f;
    motor2.pole_pairs = 7;
    motor2.voltage_limit = 4.0f;
    motor2.velocity_limit = 30.0f;
    motor2.controller = ControlType::velocity_openloop;

    driverMot0.init();
    motor0.linkDriver(&driverMot0);
    motor0.init();
    motor0.sensor = nullptr;

    driverMot1.init();
    motor1.linkDriver(&driverMot1);
    motor1.init();
    motor1.sensor = nullptr;

    // driverMot2.init();
    // motor2.linkDriver(&driverMot2);
    // motor2.init();
    // motor2.sensor = nullptr;

    motor0.disable();
    motor1.disable();
    osDelay(5000);

    const float DEG_TO_RAD = 3.1415926535f / 180.0f; // π/180

    // === Калибровка offset'ов гироскопов ===
    double sum1 = 0.0;
    double sum2 = 0.0;
    int cnt = 20000;

    uint32_t tick = osKernelGetTickCount();
//    for (int i = 0; i < cnt; i++) {
//        MPU6050_ReadRaw(&hi2c2, 0xd0, &mpu1_data);
//        MPU6050_ReadRaw(&hi2c1, 0xd2, &mpu2_data);
//
//        sum1 += (double)(mpu1_data.gy * (2000.0f / 32768.0f) * DEG_TO_RAD);
//        sum2 += (double)(mpu2_data.gz * (2000.0f / 32768.0f) * DEG_TO_RAD);
//
//        tick += 1;               // период 1 мс
//        osDelayUntil(tick);
//    }
//
//    sum1 = sum1 / cnt;
//    sum2 = sum2 / cnt;
//
//    printf(">sum1:%f\n", (float)sum1);
//    printf(">sum2:%f\n", (float)sum2);

    motor0.enable();
    motor1.enable();
    //motor2.enable();

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

    // === Главный цикл с ГАРАНТИРОВАННЫМ периодом ===
    float target_vel = 0.0f; // ← оставлено как в оригинале
    float inc = 0.002;

    float gyro1_y;
    float gyro2_z;

	uint8_t id;

    tick = osKernelGetTickCount();           // начальная метка времени
    const uint32_t PERIOD_MS = 2;            // желаемый период 2 мс


    osStatus_t status;

    for (;;)
    {
//    	if (pkt == NULL)
//    	{
//
//    		status =  osMessageQueueGet(data_from_calc_QueueHandle, &id, NULL, 0); // no wait
//    		if (status == osOK)
//    		{
//    			pkt_pos = 0;
//    		}
//    		else
//    		{
//    			pkt_pos = 0;
//    			pkt = &QueueStorageBuffer[id];
//    		}
//    	}


        MPU6050_ReadRaw(&hi2c2, 0xd0, &mpu1_data);
        gyro1_y = mpu1_data.gy * (2000.0f / 32768.0f) * DEG_TO_RAD ; // pitch
        // Для pitch (мотор0)
        float error_pitch = 0.0f - gyro1_y;           // setpoint = 0 (стабилизация скорости)
        float vel_cmd_pitch = pid_pitch(error_pitch); // ← через PID
        motor0.move(vel_cmd_pitch);


        MPU6050_ReadRaw(&hi2c1, 0xd2, &mpu2_data);
        gyro2_z = mpu2_data.gz * (2000.0f / 32768.0f) * DEG_TO_RAD ; // yaw
        float error_yaw = 0.0f - gyro2_z;
        float vel_cmd_yaw = pid_yaw(error_yaw);
        motor1.move(-vel_cmd_yaw);

//        static int cnt =0;
//
//
//        float freq = zc_sliding_update(&zc, vel_cmd, SAMPLE_RATE);
//
//            if (freq > 0.1f) {
//            	cnt++;
//
//            	if (cnt>128)
//            	{
//
//            		printf("%.2f\r\n",freq);
//            		cnt= 0;
//            	}
//            	//printf("freq %.2f\r\n",freq);
//            	//printf(">g1:%f\n", freq);
//            }

//        if (pkt!=NULL)
//        {
//
//        	 if (pkt_pos >= SAMPLES)
//        	 {
//        		 pkt_pos = 0;
//        		 pkt->timestamp = HAL_GetTick();
//        		 pkt->fs = 500.0f;
//
//        		 osStatus_t put_status = osMessageQueuePut(data_to_calc_QueueHandle, &id, 0U, 0U);
//
//        		 if (put_status != osOK) {
//					printf("Queue put failed: %d (full?)\r\n", put_status);
//
//				} else {
//					printf("Queue put OK\r\n");
//				}
//				pkt = NULL;
//        	 }
//        	 else
//        	 {
//            	 pkt->data[pkt_pos]=gyro1_y;
//            	 pkt_pos++;
//        	 }
//        }





        //printf(">g1:%f\n", gyro1_y);
        //printf(">g2:%f\n", gyro2_z);

        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

        tick += PERIOD_MS;

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        osDelayUntil(tick);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);


    }
}

/**
 * @brief Сканирует I2C-шину и выводит все найденные устройства
 * @param hi2c  - указатель на обработчик I2C (например, &hi2c1 или &hi2c2)
 */
void I2C_ScanExternalBus(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        printf("Error: hi2c == NULL\r\n");
        return;
    }

    printf("=== SCAN I2C BUS ===\r\n");
    printf("Devs addrs (7-bit):\r\n");

    uint8_t found = 0;
    uint32_t start_tick = HAL_GetTick();

    for (uint8_t addr = 1; addr < 128; addr++)   // 0x01 .. 0x7F
    {
        // Проверяем наличие устройства (3 попытки, таймаут 100 мс)
        if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 3, 100) == HAL_OK)
        {
            printf("  find: 0x%02X  (0x%02X)\r\n", addr, (addr << 1));
            found++;
        }

        // Небольшая пауза, чтобы не забивать шину
        osDelay(1);
    }

    uint32_t duration_ms = HAL_GetTick() - start_tick;

    if (found == 0)
        printf("  No devs find!\r\n");
    else
        printf("  find devs : %d\r\n", found);

    printf("end of scan  %lu ms\r\n", duration_ms);
    printf("====================================\r\n\r\n");
}


void I2CScanTask(void *argument)
{
    while (1)
    {
        I2C_ScanExternalBus(&hi2c1);   // ваш hi2c
        I2C_ScanExternalBus(&hi2c2);   // ваш hi2c
        osDelay(10000);                // повтор каждые 10 сек (или удалите цикл)
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
    MX_I2C1_Init();
    MX_USART3_UART_Init();


    /* === FreeRTOS === */
    MX_FREERTOS_Init();               // инициализация объектов
    osKernelInitialize();             // ← ЯВНЫЙ вызов
    Stabilization_Init();
    //MotorTask(NULL);


//    data_to_calc_QueueHandle = osMessageQueueNew(FREQ_BUF_CNT, sizeof(AccelPacket_t*), NULL);
//    data_from_calc_QueueHandle = osMessageQueueNew(FREQ_BUF_CNT, sizeof(AccelPacket_t*), NULL);


//    // put all ptrs
//    for (int i=0;i<FREQ_BUF_CNT;i++)
//    {
//    	QueueStorageBufferPtrs[i] = i;
//    	osMessageQueuePut(data_from_calc_QueueHandle, &QueueStorageBufferPtrs[i], 0U, 0U);  // ← &p !
//    }


    zc_sliding_init(&zc, sample_buffer, BUFFER_SIZE, WINDOW_DURATION, 0.08f, SAMPLE_RATE);


	const osThreadAttr_t motorTask_attributes = {
			.name = "MotorTask",
			.stack_size = 8000,
			.priority = (osPriority_t) osPriorityNormal };

	osThreadNew(MotorTask, NULL, &motorTask_attributes);


//	// Создаём задачу анализа
//	const osThreadAttr_t fftTask_attributes = {
//			.name = "FFT_Task",
//			.stack_size =5000,
//			.priority = (osPriority_t) osPriorityNormal };
//
//	osThreadNew(StartFFT_Task, NULL, &fftTask_attributes);



	motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);
	if (motorTaskHandle == NULL) {
		// быстрый красный — задача не создана
		while (1) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			osDelay(80);
		}
	}


//    // Атрибуты задачи (стиль CMSIS-RTOS2)
//    const osThreadAttr_t MPU6050Task_attributes = {
//      .name       = "MPU6050",
//      .stack_size = 512 * 4,          // 2 КБ стек (printf любит память)
//      .priority   = osPriorityHigh,   // или osPriorityNormal / osPriorityRealtime — выберите под вашу систему
//    };
//
//    // Создание задачи (RTOS-стиль)
//    osThreadId_t MPU6050TaskHandle = osThreadNew(MPU6050_Task, NULL, &MPU6050Task_attributes);

//    // === НОВАЯ ЗАДАЧА ДЛЯ СКАНИРОВАНИЯ I2C ===
//        osThreadId_t I2CScanTaskHandle;
//        const osThreadAttr_t I2CScanTask_attributes = {
//            .name = "I2CScanTask",
//            .stack_size = 512,        // 0.5 КБ стек (достаточно)
//            .priority = (osPriority_t) osPriorityLow,   // низкий приоритет, чтобы не мешать мотору
//        };
//
//    I2CScanTaskHandle = osThreadNew(I2CScanTask, NULL, &I2CScanTask_attributes);




    osStatus_t status = osKernelStart();

    // Если дошли сюда — osKernelStart() вернул osError
    while(1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);   // медленный красный
        osDelay(500);
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
