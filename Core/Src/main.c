/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
<<<<<<< Updated upstream
=======
#include <stdbool.h>
#include "debug.h"
#include "bts7960.h"
>>>>>>> Stashed changes
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPWM_CHANNEL TIM_CHANNEL_1
#define LPWM_CHANNEL TIM_CHANNEL_2
#define R_EN_PIN GPIO_PIN_0
#define L_EN_PIN GPIO_PIN_1
#define EN_PORT GPIOB

#define VREF 3.3f
#define ADC_MAX 4095
#define SENSOR_ZERO 1.670f 
#define MV_PER_AMP 0.185f
<<<<<<< Updated upstream
#define TARGET_CURRENT 7.5f

// Note: If you pass percentage (0-100) to BTS7960_Forward, 
// your PID limits should ideally be 100.0f and 0.0f.
#define PWM_MAX 100.0f 
#define PWM_MIN 0.0f
=======
#define TARGET_CURRENT 6.4f //5.9f
#define HYST_MAX 0.8f
#define HYST_MIN 0.5f

>>>>>>> Stashed changes
#define DT 0.00005f

#define ADC_RAW_TO_VOLTAGE 0.0008058608f // 3.3 / 4095

#define GATE_WIDTH_M     0.005f
#define GATE_TO_COIL_M  0.01f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- Light Gate Variables ---- */
volatile uint32_t gate_start = 0;
volatile uint32_t gate_time_us = 0;
volatile uint8_t  gate_active = 0;
volatile uint8_t  gate_done = 0;

/* ---- Physics & Timing Variables ---- */
volatile float    ball_speed = 0.0f;
volatile uint32_t coil_hold_time_ms = 0;

/* ---- Speed PID Controller ---- */
float target_speed = 3.0f;    // m/s
float speed_error = 0.0f;
float speed_error_prev = 0.0f;
float speed_integral = 0.0f;
float Kp = 7.0f;
float Ki = 0.04f;
float Kd = 0.7f;
static uint32_t last_speed_tick = 0;

/* ---- Current PID Controller ---- */
float Kp_coil = 8.7f;
float Ki_coil = 150.0f;
float integral = 0.0f;
float filtered_current = 0.0f;
float alpha = 0.03f;
volatile float measured_current = 0.0f;
uint16_t current_value;

/* ---- System State ---- */
volatile uint8_t mode_auto = 1;
volatile uint8_t auto_running = 1;

typedef enum {
    PHASE_IDLE, 
    PHASE1_SPINUP, 
    PHASE2_HOLD, 
    PHASE3_REVERSE
} AutoPhase_t;

static AutoPhase_t autoPhase = PHASE_IDLE;
static uint32_t phase_timer = 0;
static float pwm_forward = 0.0f;
static float pwm_reverse = 0.0f;

/* ---- USB Variables ---- */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

<<<<<<< Updated upstream
/* Stop both PWMs (coast) */
void BTS7960_Stop(void)
{
    HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, 0);
}

/* Drive forward: RPWM = pwm_percent, LPWM = 0 */
void BTS7960_Forward(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * 1800.0f); // 1800 is the timer period

    HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, val);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, 0);
}

/* Drive reverse: LPWM = pwm_percent, RPWM = 0 */
void BTS7960_Reverse(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * 1800.0f); // 1800 is the timer period

    HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, val);
=======
void process_adc_buffer(void)
{
	uint16_t raw = adc_buf[ADC_BUF_LEN-1];
	float voltage = ((float)raw * VREF) / ADC_MAX;
	measured_current = (voltage - SENSOR_ZERO) * 1000.0f / MV_PER_AMP;
}

static inline uint16_t adc_to_millivolts(uint16_t adc)
{
    return (uint16_t)((adc * 3300UL) / 4095UL);
>>>>>>> Stashed changes
}

void usbPrint(const char* msg)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
        return;

    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void runAutoStateMachine(void)
{
    if (!auto_running) { autoPhase = PHASE_IDLE; return; }

    uint32_t current_time = HAL_GetTick();

    switch (autoPhase)
    {
        case PHASE_IDLE:
            BTS7960_Stop();
            pwm_reverse = 0.0f;
            break;

        case PHASE1_SPINUP:
            // Handled dynamically inside ADC interrupt for tight current control
            break;

        case PHASE2_HOLD:
            if (current_time - phase_timer >= coil_hold_time_ms)
            {
                autoPhase = PHASE3_REVERSE;
            }
            break;

        case PHASE3_REVERSE:
            pwm_reverse = 100.0f;
            BTS7960_Reverse(pwm_reverse);

            if (measured_current <= 1.75f) 
            {
                BTS7960_Stop();
                phase_timer = current_time;
                autoPhase = PHASE_IDLE;
            }
            break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_1)
        return;

    uint32_t timer_now = __HAL_TIM_GET_COUNTER(&htim2);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        // Beam blocked
        gate_start = timer_now;
        gate_active = 1;
    }
    else if (gate_active)
    {
        // Beam released
        if (timer_now >= gate_start)
            gate_time_us = timer_now - gate_start;
        else
            gate_time_us = (0xFFFF - gate_start) + timer_now + 1;

        gate_active = 0;
        gate_done = 1;
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Start ADC
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
     Error_Handler();
  }

  // Start PWM main counter
  HAL_TIM_Base_Start(&htim1);

  // Start the Light gate timer
  HAL_TIM_Base_Start(&htim2);

  // Trigger the ADC when TIM1 counter reaches 2
  htim1.Instance->CCR3 = 2;

  BTS7960_Stop();

  // Start PWM on both channels used by BTS7960
  if (HAL_TIM_PWM_Start(&htim1, RPWM_CHANNEL) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Start(&htim1, LPWM_CHANNEL) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  mode_auto = 1;
  auto_running = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Check if a ball was detected and start auto trigger
      if(mode_auto && gate_done && auto_running)
      {
          gate_done = 0;
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

          ball_speed = 0.0f;
          speed_integral = 0.0f;
          speed_error = 0.0f;

          if(gate_time_us > 0)
          {
              /* ---- Calculate speed ---- */
              float gate_time_s = gate_time_us * 1e-6f;
              ball_speed = GATE_WIDTH_M / gate_time_s;

              /* ---- Calculate travel time to coil ---- */
              float travel_time_ms = (GATE_TO_COIL_M / ball_speed) * 1000.0f;

              /* ---- PID timing correction ---- */
              uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);
              uint32_t dt_us;

              if (now_us >= last_speed_tick)
                  dt_us = now_us - last_speed_tick;
              else
                  dt_us = (0xFFFF - last_speed_tick) + now_us + 1;

              last_speed_tick = now_us;
              float dt_s = dt_us * 1e-6f;

              speed_error = target_speed - ball_speed;
              speed_integral += speed_error * dt_s;

              float derivative = (speed_error - speed_error_prev) / dt_s;
              speed_error_prev = speed_error;

              float pid_output = (Kp*speed_error + Ki*speed_integral + Kd*derivative);

              coil_hold_time_ms = (uint32_t)(travel_time_ms + pid_output);

              /* ---- Reset integral for coil PID & Turn coil ON immediately ---- */
              integral = 0.0f; 
              autoPhase = PHASE1_SPINUP;
              phase_timer = HAL_GetTick();

              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

              char buf[96];
              snprintf(buf, sizeof(buf),
              "Pulse_delay=%lu ms | speed=%.2f m/s | speed_error=%f \r\n",
              coil_hold_time_ms, ball_speed, speed_error_prev);
              usbPrint(buf);
          }
      }

      // Run auto state machine to handle timing transitions
      if(mode_auto)
      {
          runAutoStateMachine();
      }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        current_value = HAL_ADC_GetValue(hadc);
        float voltage_diff = (current_value * ADC_RAW_TO_VOLTAGE) - SENSOR_ZERO;
        measured_current = voltage_diff / MV_PER_AMP;

        if(autoPhase == PHASE1_SPINUP)
        {
            BTS7960_Forward(100.0f);

            if(measured_current >= (TARGET_CURRENT - 2.5f))
            {
                phase_timer = HAL_GetTick();
                autoPhase = PHASE2_HOLD;
            }
        }
        else if(autoPhase == PHASE2_HOLD)
        {
            filtered_current += alpha * (measured_current - filtered_current);
            float error_c = TARGET_CURRENT - filtered_current;
            
            float pwm_unsat = Kp_coil * error_c + Ki_coil * integral;

            // Anti-windup clamping
            if ( (pwm_unsat < PWM_MAX && pwm_unsat > PWM_MIN) ||
                 (pwm_unsat >= PWM_MAX && error_c < 0) ||
                 (pwm_unsat <= PWM_MIN && error_c > 0) )
            {
                integral += error_c * DT;
            }

            pwm_forward = Kp_coil * error_c + Ki_coil * integral;
            BTS7960_Forward(pwm_forward);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */