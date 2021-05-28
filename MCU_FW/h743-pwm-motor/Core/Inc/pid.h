#ifndef __PID_H__
#define __PID_H__


#include "main.h"


void pid_update_all(void);


extern uint32_t FL_Speed1;
extern uint32_t FL_Speed2;

extern uint32_t FR_Speed1;
extern uint32_t FR_Speed2;

extern uint32_t RL_Speed1;
extern uint32_t RL_Speed2;

extern uint32_t RR_Speed1;
extern uint32_t RR_Speed2;

extern float FL_Target;
extern float FR_Target;
extern float RL_Target;
extern float RR_Target;

extern float kP, kI, kD;


#define SPEED_CONST 20000

#define SPEED_MAX SPEED_CONST / 115
#define SPEED_MIN SPEED_CONST / 1600

#define PWM_MIN 8000
#define PWM_MAX 23000

#define TIM2_ARR 2147483647


#endif