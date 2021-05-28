#include "motor.h"
#include "pid.h"
#include "stdlib.h"

uint32_t motor_speed1[4];
uint32_t motor_speed2[4];

uint32_t aaaaa;

int32_t motor_target[4];

PID_Typedef MOTOR_PID[4];

float motor_get_speed(motor_id id);
void motor_update_pwm(motor_id id, float pwm_value);

typedef void(*TIM_OC_SET_COMP)(TIM_TypeDef*, uint32_t);
TIM_OC_SET_COMP MOTOR_PWM_UPDATE[4] = {LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2, LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4};
TIM_TypeDef* MOTOR_PWM_TIM[4] = {TIM1, TIM1, TIM1, TIM1};

GPIO_TypeDef* MOTOR_DIR_GPIO_PORT[4] = {DIR_FL_GPIO_Port, DIR_FR_GPIO_Port, DIR_RL_GPIO_Port, DIR_RR_GPIO_Port};
uint32_t MOTOR_DIR_PIN[4] = {DIR_FL_Pin, DIR_FR_Pin, DIR_RL_Pin, DIR_RR_Pin};

void motor_pid_init(float kP, float kI, float kD)
{
	pid_init(&MOTOR_PID[FL], kP, kI, kD, PWM_MAX, PWM_MIN);
	pid_init(&MOTOR_PID[FR], kP, kI, kD, PWM_MAX, PWM_MIN);
	pid_init(&MOTOR_PID[RL], kP, kI, kD, PWM_MAX, PWM_MIN);
	pid_init(&MOTOR_PID[RR], kP, kI, kD, PWM_MAX, PWM_MIN);
}

void motor_pid_tune(float kP, float kI, float kD)
{
	pid_tune(&MOTOR_PID[FL], kP, kI, kD);
	pid_tune(&MOTOR_PID[FR], kP, kI, kD);
	pid_tune(&MOTOR_PID[RL], kP, kI, kD);
	pid_tune(&MOTOR_PID[RR], kP, kI, kD);
}



void motor_update_all(void)
{
	
	motor_update_pwm(FL, pid(&MOTOR_PID[FL], motor_get_speed(FL), abs(motor_target[FL])) * fsign(motor_target[FL]));
	motor_update_pwm(FR, pid(&MOTOR_PID[FR], motor_get_speed(FR), abs(motor_target[FR])) * fsign(motor_target[FR]));
	motor_update_pwm(RL, pid(&MOTOR_PID[RL], motor_get_speed(RL), abs(motor_target[RL])) * fsign(motor_target[RL]));
	motor_update_pwm(RR, pid(&MOTOR_PID[RR], motor_get_speed(RR), abs(motor_target[RR])) * fsign(motor_target[RR]));
}

void motor_update_pwm(motor_id id, float pwm_value)
{

	if (fabs(pwm_value) <= PWM_MIN + 300)
		pwm_value = 0;
	
	if (id == FL || id == RL)
		pwm_value = -pwm_value;
		
	if(pwm_value >=0)
		LL_GPIO_SetOutputPin(MOTOR_DIR_GPIO_PORT[id], MOTOR_DIR_PIN[id]);
	else
		LL_GPIO_ResetOutputPin(MOTOR_DIR_GPIO_PORT[id], MOTOR_DIR_PIN[id]);
		
	(*MOTOR_PWM_UPDATE[id])(MOTOR_PWM_TIM[id], fabs(pwm_value));
}


float motor_get_speed(motor_id id)
{
	static float Speed2_Last[4][4];
	uint32_t speed;
	
	speed = Speed2_Last[0][id] == motor_speed2[id] ? 0 : SPEED_CONST / ((motor_speed2[id]+MOTOR_TIM_ARR-motor_speed1[id])%MOTOR_TIM_ARR);
	Speed2_Last[0][id] = Speed2_Last[1][id];
	Speed2_Last[1][id] = Speed2_Last[2][id];
	Speed2_Last[2][id] = Speed2_Last[3][id];
	Speed2_Last[3][id] = motor_speed2[id];
	
	return speed;
}


