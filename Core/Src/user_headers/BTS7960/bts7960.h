#ifndef BTS7960_H
#define BTS7960_H

#include "stdint.h"
#include "main.h"

#define PWM_MAX 1800U
#define PWM_MIN 0U



extern TIM_HandleTypeDef *coil_tim;
extern GPIO_TypeDef *r_en_port, *l_en_port;
extern short unsigned int r_en_pin, l_en_pin;
extern short unsigned int rpwm_channel, lpwm_channel;

void BTS7960_Init(TIM_HandleTypeDef *param_coil_tim, 
                  GPIO_TypeDef* param_r_en_port, 
                  short unsigned int param_r_en_pin, 
                  short unsigned int param_rpwm_channel,
                  GPIO_TypeDef* param_l_en_port,
                  short unsigned int param_l_en_pin,
                  short unsigned int param_lpwm_channel);
void BTS7960_Stop(void);
void BTS7960_Forward(float percent);
void BTS7960_Reverse(float percent);


#endif