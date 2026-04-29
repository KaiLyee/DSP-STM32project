/******************************************************************************
 * @file    main.c
 * @author  Junkai Li
 * @date    2026-04-28
 * @brief   Bluetooth DSP Speaker - 8-Band Parametric EQ
 * @mail	ssyjl10@outlook.com
 *
 * Real-time audio processing system with 8-band peaking EQ,
 * 1024-point FFT spectrum analysis, and Python GUI control.
 *
 * Hardware: STM32F407VG Discovery + ESP32 + ADAU1761 + TPA3116D2
 *
 * Georgia Institute of Technology
 *
 * GitHub: https://github.com/KaiLyee/bluetooth-dsp-speaker
 ******************************************************************************/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t band;
    float b0, b1, b2, a1, a2;
} EqBandCoeffs_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUF_SIZE 16
#define EQ_BANDS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int16_t rxBuf[AUDIO_BUF_SIZE];
int16_t txBuf[AUDIO_BUF_SIZE];

/* FFT collection buffers - 512 points */
float fftBufPre[1024];
float fftBufPost[1024];
volatile uint16_t fftBufIdx = 0;
volatile uint8_t fftReady = 0;

/* 8-band Peaking EQ - initialized to all-pass (gain=0) */
static volatile float eq_b0[EQ_BANDS] = {1,1,1,1,1,1,1,1};
static volatile float eq_b1[EQ_BANDS] = {0,0,0,0,0,0,0,0};
static volatile float eq_b2[EQ_BANDS] = {0,0,0,0,0,0,0,0};
static volatile float eq_a1[EQ_BANDS] = {0,0,0,0,0,0,0,0};
static volatile float eq_a2[EQ_BANDS] = {0,0,0,0,0,0,0,0};

static volatile float eq_x1[EQ_BANDS] = {0};
static volatile float eq_x2[EQ_BANDS] = {0};
static volatile float eq_y1[EQ_BANDS] = {0};
static volatile float eq_y2[EQ_BANDS] = {0};

extern QueueHandle_t xCoeffQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  for (int i = 0; i < 6; i++) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    HAL_Delay(150);
  }
  /* USER CODE END 2 */

  osKernelInitialize();
  MX_FREERTOS_Init();
  osKernelStart();

  while (1) {}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

/* USER CODE BEGIN 4 */

static float biquad(float in,
                    volatile float *b0, volatile float *b1, volatile float *b2,
                    volatile float *a1, volatile float *a2,
                    volatile float *x1, volatile float *x2,
                    volatile float *y1, volatile float *y2)
{
    float out = *b0 * in + *b1 * (*x1) + *b2 * (*x2)
                         - *a1 * (*y1) - *a2 * (*y2);
    *x2 = *x1;
    *x1 = in;
    *y2 = *y1;
    *y1 = out;
    return out;
}

static void process_frame(int16_t in_L, int16_t in_R,
                          int16_t *out_L, int16_t *out_R)
{
    float sample = ((float)in_L + (float)in_R) * 0.5f;

    for (int i = 0; i < EQ_BANDS; i++)
    {
        sample = biquad(sample,
                        &eq_b0[i], &eq_b1[i], &eq_b2[i],
                        &eq_a1[i], &eq_a2[i],
                        &eq_x1[i], &eq_x2[i],
                        &eq_y1[i], &eq_y2[i]);
    }

    if (sample >  32767.0f) sample =  32767.0f;
    if (sample < -32768.0f) sample = -32768.0f;

    *out_L = (int16_t)sample;
    *out_R = (int16_t)sample;
}

void apply_eq_band(int band, float b0, float b1, float b2, float a1, float a2)
{
    if (band < 0 || band >= EQ_BANDS) return;
    eq_b0[band] = b0;
    eq_b1[band] = b1;
    eq_b2[band] = b2;
    eq_a1[band] = a1;
    eq_a2[band] = a2;
    eq_x1[band] = 0; eq_x2[band] = 0;
    eq_y1[band] = 0; eq_y2[band] = 0;
}

static void process_half(int16_t *rx, int16_t *tx)
{
    int16_t outL, outR;

    process_frame(rx[0], rx[1], &outL, &outR);
    tx[0] = outL;
    tx[1] = outR;
    tx[2] = 0;
    tx[3] = 0;

    if (!fftReady && fftBufIdx < 1024)
    {
        fftBufPre[fftBufIdx]  = ((float)rx[0] + (float)rx[1]) * 0.5f;
        fftBufPost[fftBufIdx] = (float)outL;
        fftBufIdx++;
    }

    process_frame(rx[4], rx[5], &outL, &outR);
    tx[4] = outL;
    tx[5] = outR;
    tx[6] = 0;
    tx[7] = 0;

    if (!fftReady && fftBufIdx < 1024)
    {
        fftBufPre[fftBufIdx]  = ((float)rx[4] + (float)rx[5]) * 0.5f;
        fftBufPost[fftBufIdx] = (float)outL;
        fftBufIdx++;
        if (fftBufIdx >= 1024) fftReady = 1;
    }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    process_half(&rxBuf[0], &txBuf[0]);
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    process_half(&rxBuf[8], &txBuf[8]);
}

/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
    HAL_IncTick();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
