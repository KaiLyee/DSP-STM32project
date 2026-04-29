/******************************************************************************
 * @file    freertos.c
 * @author  Junkai Li
 * @date    2026-04-28
 * @brief   FreeRTOS tasks: UART command parser, FFT spectrum, 8-band EQ
 * @mail	ssyjl10@outlook.com
 *
 * Georgia Institute of Technology
 * GitHub: https://github.com/KaiLyee/bluetooth-dsp-speaker
 ******************************************************************************/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "i2s.h"
#include "usart.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
#define FS 44100.0f
#define FFT_SIZE 1024
#define FFT_NUM_BINS (FFT_SIZE / 2)
#define DISPLAY_BINS 64
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t xCoeffQueue = NULL;

/* FFT working buffers - STATIC globals */
static float fftIn[FFT_SIZE];
static float fftOut[FFT_SIZE];
static float magPre[FFT_NUM_BINS];
static float magPost[FFT_NUM_BINS];

/* FFTTask large locals moved to static to avoid stack overflow */
static uint16_t dispPre[DISPLAY_BINS];
static uint16_t dispPost[DISPLAY_BINS];
static char fftLine[600];
/* USER CODE END Variables */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN FunctionPrototypes */
void ButtonTask(void *argument);
void UartTask(void *argument);
void FFTTask(void *argument);
static void compute_peaking_eq(float fc, float gain_dB, float Q, EqBandCoeffs_t *c, uint8_t band);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void) {
  xCoeffQueue = xQueueCreate(1, sizeof(EqBandCoeffs_t));

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  const osThreadAttr_t buttonTask_attributes = {
    .name = "ButtonTask", .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(ButtonTask, NULL, &buttonTask_attributes);

  const osThreadAttr_t uartTask_attributes = {
    .name = "UartTask", .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(UartTask, NULL, &uartTask_attributes);

  const osThreadAttr_t fftTask_attributes = {
    .name = "FFTTask", .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(FFTTask, NULL, &fftTask_attributes);
}

void StartDefaultTask(void *argument)
{
  MX_USB_HOST_Init();

  extern int16_t rxBuf[];
  extern int16_t txBuf[];

  if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2,
        (uint16_t *)txBuf, (uint16_t *)rxBuf, 16) != HAL_OK)
  {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      for (;;) { osDelay(1000); }
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  for(;;) { osDelay(1000); }
}

/* USER CODE BEGIN Application */

static void compute_peaking_eq(float fc, float gain_dB, float Q, EqBandCoeffs_t *c, uint8_t band)
{
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * PI * fc / FS;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);
    float a0 = 1.0f + alpha / A;

    c->band = band;
    c->b0 = (1.0f + alpha * A) / a0;
    c->b1 = (-2.0f * cosw0) / a0;
    c->b2 = (1.0f - alpha * A) / a0;
    c->a1 = (-2.0f * cosw0) / a0;
    c->a2 = (1.0f - alpha / A) / a0;
}

void ButtonTask(void *argument)
{
    for (;;) { osDelay(500); }
}

void UartTask(void *argument)
{
    extern UART_HandleTypeDef huart2;
    char rxLine[64];
    uint8_t rxByte;
    uint8_t idx = 0;

    static const float band_freqs[8] = {63, 125, 250, 500, 1000, 2000, 4000, 8000};
    static const float band_Q = 0.7f;

    const char *hello = "STM32 DSP Ready\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)hello, strlen(hello), 100);

    osDelay(3000);
    { uint8_t dummy; while (HAL_UART_Receive(&huart2, &dummy, 1, 10) == HAL_OK) {} }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    for (;;)
    {
        if (HAL_UART_Receive(&huart2, &rxByte, 1, 50) == HAL_OK)
        {
            if (rxByte == '\n' || rxByte == '\r')
            {
                if (idx > 0)
                {
                    rxLine[idx] = '\0';

                    if (strcmp(rxLine, "PING") == 0)
                    {
                        const char *msg = "OK:PONG\r\n";
                        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
                    }
                    else if (strncmp(rxLine, "EQ:", 3) == 0)
                    {
                        char *comma = strchr(rxLine + 3, ',');
                        if (comma)
                        {
                            *comma = '\0';
                            int band = atoi(rxLine + 3);
                            int gain = atoi(comma + 1);

                            if (band >= 0 && band < 8 && gain >= -12 && gain <= 12)
                            {
                                EqBandCoeffs_t coeffs;
                                compute_peaking_eq(band_freqs[band], (float)gain, band_Q, &coeffs, (uint8_t)band);

                                extern void apply_eq_band(int band, float b0, float b1, float b2, float a1, float a2);
                                apply_eq_band(coeffs.band, coeffs.b0, coeffs.b1, coeffs.b2, coeffs.a1, coeffs.a2);

                                const char *msg = "OK:EQ\r\n";
                                HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
                            }
                            else
                            {
                                const char *msg = "ERR:range\r\n";
                                HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
                            }
                        }
                    }
                    idx = 0;
                }
            }
            else
            {
                if (rxByte >= 0x20 && rxByte <= 0x7E)
                {
                    if (idx < sizeof(rxLine) - 1)
                        rxLine[idx++] = (char)rxByte;
                }
                else
                    idx = 0;
            }
        }
    }
}

void FFTTask(void *argument)
{
    extern UART_HandleTypeDef huart2;
    extern float fftBufPre[];
    extern float fftBufPost[];
    extern volatile uint16_t fftBufIdx;
    extern volatile uint8_t fftReady;

    arm_rfft_fast_instance_f32 fftInst;
    arm_rfft_fast_init_f32(&fftInst, FFT_SIZE);

    /* Log-spaced bin mapping: 512 FFT bins -> 64 display bins */
    static const uint16_t bin_map[65] = {
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
            33, 34, 35, 36, 37, 38, 39, 42, 46, 51, 56, 62, 68, 75, 83, 91,
            100, 110, 121, 133, 147, 162, 178, 196, 215, 237, 261, 287, 316, 348, 383, 422,
            464
        };

    osDelay(5000);

    for (;;)
    {
        if (fftReady)
        {
            for (int i = 0; i < FFT_SIZE; i++)
                fftIn[i] = fftBufPre[i];
            arm_rfft_fast_f32(&fftInst, fftIn, fftOut, 0);
            arm_cmplx_mag_f32(fftOut, magPre, FFT_NUM_BINS);

            for (int i = 0; i < FFT_SIZE; i++)
                fftIn[i] = fftBufPost[i];
            arm_rfft_fast_f32(&fftInst, fftIn, fftOut, 0);
            arm_cmplx_mag_f32(fftOut, magPost, FFT_NUM_BINS);

            for (int i = 0; i < DISPLAY_BINS; i++)
            {
                float maxPre = 0, maxPost = 0;
                for (int k = bin_map[i]; k < bin_map[i + 1]; k++)
                {
                    if (k < FFT_NUM_BINS)
                    {
                        if (magPre[k] > maxPre) maxPre = magPre[k];
                        if (magPost[k] > maxPost) maxPost = magPost[k];
                    }
                }
                dispPre[i]  = (uint16_t)(maxPre / 32768.0f * 999.0f);
                dispPost[i] = (uint16_t)(maxPost / 32768.0f * 999.0f);
                if (dispPre[i] > 999) dispPre[i] = 999;
                if (dispPost[i] > 999) dispPost[i] = 999;
            }

            int pos = 0;
            fftLine[pos++] = 'F'; fftLine[pos++] = 'F'; fftLine[pos++] = 'T'; fftLine[pos++] = ':';

            for (int i = 0; i < DISPLAY_BINS; i++)
            {
                uint16_t val = dispPre[i];
                if (val >= 100) { fftLine[pos++] = '0' + val / 100; val %= 100; fftLine[pos++] = '0' + val / 10; fftLine[pos++] = '0' + val % 10; }
                else if (val >= 10) { fftLine[pos++] = '0' + val / 10; fftLine[pos++] = '0' + val % 10; }
                else { fftLine[pos++] = '0' + val; }
                if (i < DISPLAY_BINS - 1) fftLine[pos++] = ',';
            }

            fftLine[pos++] = ';';

            for (int i = 0; i < DISPLAY_BINS; i++)
            {
                uint16_t val = dispPost[i];
                if (val >= 100) { fftLine[pos++] = '0' + val / 100; val %= 100; fftLine[pos++] = '0' + val / 10; fftLine[pos++] = '0' + val % 10; }
                else if (val >= 10) { fftLine[pos++] = '0' + val / 10; fftLine[pos++] = '0' + val % 10; }
                else { fftLine[pos++] = '0' + val; }
                if (i < DISPLAY_BINS - 1) fftLine[pos++] = ',';
            }

            fftLine[pos++] = '\r';
            fftLine[pos++] = '\n';

            HAL_UART_Transmit(&huart2, (uint8_t *)fftLine, pos, 500);

            fftBufIdx = 0;
            fftReady = 0;

            osDelay(500);
        }
        else
        {
            osDelay(50);
        }
    }
}

/* USER CODE END Application */
