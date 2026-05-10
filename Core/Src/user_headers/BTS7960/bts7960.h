#ifndef BTS7960_H
#define BTS7960_H

#include "stdint.h"
#include "main.h"

#define PWM_MAX 1800U
#define PWM_MIN 0U

static TIM_HandleTypeDef *coil_tim;
static uint8_t r_en_port, l_en_port;
static uint8_t r_en_pin, l_en_pin;
static uint8_t rpwm_channel, lpwm_channel;

void BTS7960_Init(TIM_HandleTypeDef *param_coil_tim, 
                  uint8_t param_r_en_port, 
                  uint8_t param_r_en_pin, 
                  uint8_t param_rpwm_channel,
                  uint8_t param_l_en_port,
                  uint8_t param_l_en_pin,
                  uint8_t param_lpwm_channel);
void BTS7960_Stop(void);
void BTS7960_Forward(float percent);
void BTS7960_Reverse(float percent);

#endif