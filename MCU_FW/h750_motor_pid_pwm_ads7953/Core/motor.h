#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "main.h"
#include "pid.h"

extern uint32_t aaaaa;

extern uint32_t motor_speed1[4];
extern uint32_t motor_speed2[4];

extern int32_t motor_target[4];


#define SPEED_CONST 20000

#define SPEED_MAX SPEED_CONST / 115
#define SPEED_MIN SPEED_CONST / 1600

#define PWM_MIN 8000
#define PWM_MAX 23000

#define MOTOR_TIM_ARR 2147483647


//typedef enum {FL=0, FR=1, RL=2, RR=3} motor_id;
#define FL 0
#define FR 1
#define RL 2
#define RR 3
#define motor_id uint8_t





void motor_update_all(void);
void motor_pid_init(float kP, float kI, float kD);
void motor_pid_tune(float kP, float kI, float kD);



#endif