#ifndef __PID_H__
#define __PID_H__


#include "main.h"


typedef struct
{
	float kP, kI, kD;
	float error, last_error, last_last_error, out;
	float out_max, out_min;
} PID_Typedef;


float pid(PID_Typedef* PID, float input, float target);
void pid_tune(PID_Typedef* PID, float kP, float kI, float kD);
void pid_init(PID_Typedef* PID, float kP, float kI, float kD, float out_max, float out_min);

#endif