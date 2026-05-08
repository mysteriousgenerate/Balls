#include "bts7960.h"
#include <cstdint>



void BTS7960_Init(TIM_HandleTypeDef *param_coil_tim, 
                  uint8_t param_r_en_port, 
                  uint8_t param_r_en_pin, 
                  uint8_t param_rpwm_channel,
                  uint8_t param_l_en_port,
                  uint8_t param_l_en_pin,
                  uint8_t param_lpwm_channel)
{
    coil_tim = param_coil_tim;

    r_en_port = param_r_en_port;
    r_en_pin = param_r_en_pin;
    rpwm_channel = param_rpwm_channel;

    l_en_port = param_l_en_port;
    l_en_pin = param_l_en_pin;
    lpwm_channel = param_lpwm_channel;

}

void BTS7960_Stop()
{
	HAL_GPIO_WritePin(r_en_port, r_en_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(l_en_port, l_en_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&coil_tim, rpwm_channel, 0);
    __HAL_TIM_SET_COMPARE(&coil_tim, lpwm_channel, 0);
}

void BTS7960_Forward(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * (float)PWM_MAX);

    HAL_GPIO_WritePin(r_en_port, r_en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(l_en_port, l_en_pin, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&coil_tim, rpwm_channel, val);
    __HAL_TIM_SET_COMPARE(&coil_tim, lpwm_channel, 0);
}

void BTS7960_Reverse(float percent)
{
    if(percent < 0.0f) percent = 0.0f;
    if(percent > 100.0f) percent = 100.0f;

    uint32_t val = (uint32_t)((percent / 100.0f) * (float)PWM_MAX);

    HAL_GPIO_WritePin(r_en_port, r_en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(l_en_port, l_en_pin, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&coil_tim, rpwm_channel, 0);
    __HAL_TIM_SET_COMPARE(&coil_tim, lpwm_channel, val);
}