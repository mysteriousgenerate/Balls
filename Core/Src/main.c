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
#define SENSOR_ZERO 1.670f //1.745f
#define MV_PER_AMP 0.185f
#define TARGET_CURRENT 7.5f
#define HYST_MAX 0.8f
#define HYST_MIN 0.5f
#define PWM_MAX 1800U
#define PWM_MIN 0U
#define DT 0.00005f

#define ADC_BUF_LEN 16
#define ADC_RAW_TO_VOLTAGE 0.0008058608059
#define PACKET_SIZE 128

#define GATE_WIDTH_M     0.005f
#define GATE_TO_COIL_M  0.01f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Light Gate test variables
volatile uint32_t gate_start = 0;
volatile uint32_t gate_time_us = 0;
volatile uint8_t  gate_active = 0;
volatile uint8_t  gate_done = 0;

volatile float    ball_speed = 0.0f;
volatile uint32_t coil_delay_ms = 0;
volatile uint8_t  auto_trigger_pending = 0;
volatile uint8_t  delay_active = 0;
volatile uint32_t delay_start_time = 0;

volatile uint32_t coil_arrival_delay_ms = 0;
volatile uint32_t coil_hold_time_ms = 0;

// Speed control
float target_speed = 3.0f;    // m/s (you choose)
float speed_error = 0.0f;
float speed_error_prev = 0.0f;
float speed_integral = 0.0f;

// PID gains (start conservative)
float Kp = 7.0f;
float Ki = 0.05f;
float Kd = 0.7f;

// PID output
float hold_time_scale = 1.0f;
static uint32_t last_speed_tick = 0; // keeps track of last tick for PID dt

// USB buffer
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint32_t UserRxLengthFS;
extern USBD_HandleTypeDef hUsbDeviceFS;
char inputBuffer[APP_RX_DATA_SIZE];
uint32_t inputIndex = 0;

// DMA setup
volatile uint16_t adc_buf[ADC_BUF_LEN];
volatile float currentVoltage[ADC_BUF_LEN];
char uart_msg[16];
static uint16_t last_mv = 0;
volatile uint8_t dma_buffer_full = 0;

// Current sensor
volatile float measured_current = 0.0f;

// State
volatile uint8_t mode_auto = 0;
volatile uint8_t stop_requested = 0;
uint8_t stopCheck(void);
void runAutoPhases(void);

// Auto mode state
typedef enum {PHASE_IDLE, PHASE1, PHASE2_HOLD, PHASE3} AutoPhase_t;
static AutoPhase_t autoPhase = PHASE_IDLE;
static uint32_t phase_timer = 0;
static float pwm_percent = 0.0f;
static float pwm_forward = 0.0f;
static float last_pwm_forward = 0.0f;
static float pwm_reverse = 0.0f;
static uint8_t phase_msg_sent = 0; // flag to print USB messages once per phase
volatile uint8_t auto_running = 0;

// Debug options
volatile uint32_t debug_timer_ms = 0;
volatile float debug_pwm = 0.0f;
uint32_t now;
uint32_t last_dma_time = 0;
uint32_t dma_interval_ms = 10000;
uint32_t dma_interval_max = 11000;

uint16_t current_value;
float error = 0;

// ---- Tunable gains ----
float Kp_coil = 0.6f;
float Ki_coil = 150.0f;

// ---- Internal states ----
float integral = 0.0f;
float filtered_current = 0.0f;
float alpha = 0.03f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------- BTS7960 helper functions ---------------- */

/* Stop both PWMs (coast) */
void BTS7960_Stop(void)
{
	HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, 0);
}

/* Drive forward: RPWM = pwm_percent, LPWM = 0, enable pins high */
void BTS7960_Forward(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * (float)PWM_MAX);

    HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, val);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, 0);
}

/* Drive reverse: LPWM = pwm_percent, RPWM = 0 */
void BTS7960_Reverse(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * (float)PWM_MAX);

    HAL_GPIO_WritePin(EN_PORT, R_EN_PIN | L_EN_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim1, RPWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LPWM_CHANNEL, val);
}

/*void process_adc_buffer(void)
{
	uint16_t raw = adc_buf[ADC_BUF_LEN-1];
	float voltage = ((float)raw * VREF) / ADC_MAX;
	measured_current = (voltage - SENSOR_ZERO) * 1000.0f / MV_PER_AMP;
}*/

static inline uint16_t adc_to_millivolts(uint16_t adc)
{
    return (uint16_t)((adc * 3300UL) / 4095UL);
}

void usbPrint(const char* msg)
{
    uint8_t res;
    do {
        res = CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        HAL_Delay(1);
    } while(res == USBD_BUSY);
}


void runAutoStateMachine(void)
{
    if (!auto_running) { autoPhase = PHASE_IDLE; return; }

    uint32_t now = HAL_GetTick();
    float current = measured_current;

    switch (autoPhase)
    {
        case PHASE_IDLE:
            BTS7960_Stop();
            //pwm_percent = 0.0f;
            //pwm_forward = 0.0f;
            pwm_reverse = 0.0f;
            break;

        case PHASE1:
            //pwm_forward = 100.0f;
        	/*if(last_pwm_forward > 0.0f)
        	{
        		BTS7960_Forward(last_pwm_forward);
        	}
        	else
        	{
        		BTS7960_Forward(pwm_forward);
        	}*/

            pwm_reverse = 0.0f;
            //BTS7960_Reverse(pwm_reverse);

            //autoPhase = PHASE2_HOLD;
            //if(phase_timer == 0) phase_timer = now;

            /*if (current >= TARGET_CURRENT || (now - phase_timer) >= 4)
            {
            	phase_timer = now;
                autoPhase = PHASE2_HOLD;
            }*/
            break;

        case PHASE2_HOLD:
        {
        	//float error = TARGET_CURRENT - current;
        	//pwm_forward += error * 50.0f;
        	//if (pwm_forward > 100) pwm_forward = 100;
        	//if (pwm_forward < 0) pwm_forward = 0;
        	//BTS7960_Forward(pwm_forward);

        	/*if(current <= TARGET_CURRENT + HYST_MIN)
        	{
        		float error = (TARGET_CURRENT + HYST_MIN) - current;
        		pwm_forward += error * 250.0f;
        		BTS7960_Forward(pwm_forward);
        	}

        	if(current >= TARGET_CURRENT + HYST_MAX)
        	{
        		float error = (TARGET_CURRENT + HYST_MAX) - current;
        		pwm_forward += error * 250.0f;
        		BTS7960_Forward(pwm_forward);
        	}*/

            if (now - phase_timer >= coil_hold_time_ms)
            {
            	//last_pwm_forward = pwm_forward;
                autoPhase = PHASE3;
            }
        }
        break;

        case PHASE3:
            pwm_reverse = 100.0f;
            BTS7960_Reverse(pwm_reverse);

            //pwm_forward = 0.0f;
            //BTS7960_Forward(pwm_forward);

            if (current <= 1.75f) //All fixed variables must change under different curcumpstances
            {
                BTS7960_Stop();
                phase_timer = now;
                autoPhase = PHASE_IDLE;
            }
            break;

        /*case PHASE3_WAIT:
            if (fabs(current) <= 0.05f)
            {
                BTS7960_Stop();
                autoPhase = PHASE_IDLE;
                auto_running = 0; // optional: leave 1 if you want continuous cycling
            }
            break;*/
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_1)
        return;

    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        // Beam blocked
        gate_start = now;
        gate_active = 1;
    }
    else if (gate_active)
    {
        // Beam released
        if (now >= gate_start)
            gate_time_us = now - gate_start;
        else
            gate_time_us = (0xFFFF - gate_start) + now + 1;

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

  while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
            HAL_Delay(10);

  // Start ADC
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
   	 Error_Handler();
  }

  // DMA ADC setup
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  // Start PWM main counter
  HAL_TIM_Base_Start(&htim1);

  // Start the Light gate timer
  HAL_TIM_Base_Start(&htim2);

  //Trigger the ADC when TIM1 counter reaches 2
  htim1.Instance->CCR3 = 2;
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, TIM1->ARR / 2); !Needs testing

  BTS7960_Stop();

  // Start PWM on both channels used by BTS7960
  if (HAL_TIM_PWM_Start(&htim1, RPWM_CHANNEL) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, LPWM_CHANNEL) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
      Error_Handler();
  }

  //__HAL_TIM_MOE_ENABLE(&htim1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  mode_auto = 1;
  auto_running = 1;

  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Drive motor forward at fixed PWM
	      //BTS7960_Forward(debug_pwm);
	      //usbPrint("Starting live current read. Press 'e' to stop.\r\n");

	      //process_adc_buffer();

	  	  	  	//error = TARGET_CURRENT - measured_current;
	  	  	  	//debug_pwm += error * 50.0f;
	          	//BTS7960_Forward(debug_pwm);

	          	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET); // Toggle PB9 to measure the sampling of the ADC
	          	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);

	  now = HAL_GetTick();


	          // Check if a ball was detected and start auto trigger
	          if(mode_auto && gate_done && auto_running)
	          {
	              gate_done = 0;
	              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	              ball_speed = 0.0f;
	              speed_integral = 0.0f;
	              speed_error = 0.0f;

	              if(gate_time_us == 0) return;

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

	                  /* ---- Safety clamp ---- */

	                  //if(coil_hold_time_ms < 1) coil_hold_time_ms = 1;
	                  //if(coil_hold_time_ms > 50) coil_hold_time_ms = 50;

	                  /* ---- Turn coil ON immediately ---- */

	                  autoPhase = PHASE1;
	                  phase_timer = HAL_GetTick();

	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	              char buf[96];
	              snprintf(buf, sizeof(buf),
	              "Pulse_delay=%lu ms | speed=%.2f m/s | speed_error=%f \r\n",
				  coil_hold_time_ms, ball_speed, speed_error_prev);
	              usbPrint(buf);
	          }

	          // Trigger coil after delay
	          /*if(mode_auto && auto_trigger_pending && delay_active)
	          {
	              if(HAL_GetTick() - delay_start_time >= coil_delay_ms)
	              {
	                  delay_active = 0;
	                  auto_trigger_pending = 0;
	                  auto_running = 1;
	                  autoPhase = PHASE1;
	                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	                  phase_timer = HAL_GetTick();
	              }
	          }*/

	          // Run auto state machine to drive coil based on measured_current
	          if(mode_auto)
	              runAutoStateMachine();


	      /*if(dma_buffer_full)
	      {
	          for(int i = 0; i < ADC_BUF_LEN; i++)
	          {
	              uint16_t mv = adc_to_millivolts(adc_buf[i]);
	              char buf[16];
	              snprintf(buf, sizeof(buf), "C%u\r\n", mv);
	              usbPrint(buf);

	              HAL_Delay(1); // slow USB transfer
	          }

	          dma_buffer_full = 0; // reset the flag
	      }*/

	      /*if(now - last_dma_time >= 100)   // print every 100 ms
	      {
	    	  last_dma_time = now;

	          char buf[32];
	          snprintf(buf, sizeof(buf), "C%.3f\r\n", measured_current);
	          //snprintf(buf, sizeof(buf), "C%u\r\n", current_value);
	          usbPrint(buf);
	      }*/
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*void process_adc(uint16_t adc_val)
{
    static uint16_t last_mv = 0;
    uint16_t mv = adc_to_millivolts(adc_val);

    if (mv != last_mv)
    {
    	char buf[16];
        snprintf(buf, sizeof(buf), "C%u\r\n", mv);
        usbPrint(buf);
        last_mv = mv;
    }
}*/


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
    	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET); // Toggle PB9 to measure the sampling of the ADC
    	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
    	current_value = HAL_ADC_GetValue(hadc);
    	float voltage_diff = (current_value * ADC_RAW_TO_VOLTAGE) - SENSOR_ZERO;
    	measured_current = voltage_diff / MV_PER_AMP;

    	if(autoPhase == PHASE1)
    	{

    		BTS7960_Forward(100.0f);

    		if(measured_current >= (TARGET_CURRENT - 2.5f))
    			{
    		       phase_timer = now;
    		       autoPhase = PHASE2_HOLD;
    			}
    	}

    	if(autoPhase == PHASE2_HOLD)
    	{
    		filtered_current += alpha * (measured_current - filtered_current);

    		float error = TARGET_CURRENT - filtered_current;

    		float pwm_unsat = Kp_coil * error + Ki_coil * integral;

    		    if (
    		        (pwm_unsat < PWM_MAX && pwm_unsat > PWM_MIN) ||
    		        (pwm_unsat >= PWM_MAX && error < 0) ||
    		        (pwm_unsat <= PWM_MIN && error > 0)
    		       )
    		    {
    		        integral += error * DT;
    		    }

    		    pwm_forward = Kp_coil * error + Ki_coil * integral;

    		    BTS7960_Forward(pwm_forward);

    		/*float error = TARGET_CURRENT - measured_current;
    		pwm_forward += error * 1.0f;
    		BTS7960_Forward(pwm_forward);*/

    		/*if(last_pwm_forward > 0.0f)
    		{
    			BTS7960_Forward(last_pwm_forward);
    		}
    		else
    		{
    			float error = TARGET_CURRENT - measured_current;
    			pwm_forward += error * 1.0f;
    			BTS7960_Forward(pwm_forward);
    		}*/

    	    /*if(current <= TARGET_CURRENT + HYST_MIN)
    	    {
    	    	float error = (TARGET_CURRENT + HYST_MIN) - current;
        		pwm_forward += error * 1.0f;
        		__HAL_TIM_SET_COMPARE(&htim1,RPWM_CHANNEL, (uint32_t)((pwm_forward / 100.0f) * PWM_MAX));
    	    }

    	    if(current >= TARGET_CURRENT + HYST_MAX)
    	    {
    	    	float error = (TARGET_CURRENT + HYST_MAX) - current;
    	    	pwm_forward += error * 1.0f;
    	        __HAL_TIM_SET_COMPARE(&htim1,RPWM_CHANNEL, (uint32_t)((pwm_forward / 100.0f) * PWM_MAX));
    	    }*/

    	}

    	/*if(now - last_dma_time >= dma_interval_ms)
    	{
    		last_dma_time = now;

    		for (uint8_t index = 0; index < ADC_BUF_LEN; index++)
    		{
    			currentVoltage[index] = ADC_RAW_TO_VOLTAGE * (float) adc_buf[index];
    			//float voltage_diff = currentVoltage[ADC_BUF_LEN-1] - SENSOR_ZERO;
    			//measured_current = (voltage_diff * 1000.0f) / MV_PER_AMP;

    			// DMA buffer is completely filled
    			dma_buffer_full = 1;
    			HAL_ADC_Stop_DMA(&hadc1);
    			HAL_ADC_Start_IT(&hadc1);
    			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
    		}
    	}*/
    }
}



void HAL_SYSTICK_Callback(void)
{
    debug_timer_ms++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
