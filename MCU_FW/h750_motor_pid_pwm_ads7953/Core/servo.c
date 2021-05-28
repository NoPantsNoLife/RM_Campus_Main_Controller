#include "servo.h"



typedef void(*TIM_OC_SET_COMP)(TIM_TypeDef*, uint32_t);

TIM_OC_SET_COMP SERVO_PWM_UPDATE[4] = {LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH4, 
																				LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4};

TIM_TypeDef* SERVO_PWM_TIM[4] = {TIM3, TIM4, TIM4, TIM3};

uint16_t SERVO_PWM_MIN[4] = {0};
uint16_t SERVO_PWM_MAX[4] = {100};


void servo_update(uint8_t id, float angle)
{
	uint16_t pwm;
	pwm = SERVO_PWM_MIN[id] + angle * (SERVO_PWM_MAX[id] - SERVO_PWM_MIN[id]) / 180;
	(*SERVO_PWM_UPDATE[id])(SERVO_PWM_TIM[id], pwm);
}




