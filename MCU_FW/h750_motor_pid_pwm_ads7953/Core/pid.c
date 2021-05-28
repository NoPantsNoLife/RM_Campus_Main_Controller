#include "pid.h"
#include "stdlib.h"
#include "math.h"




void pid_init(PID_Typedef* PID, float kP, float kI, float kD, float out_max, float out_min)
{
	PID->kP = kP;
	PID->kI = kI;
	PID->kD = kD;
	PID->out_max = out_max;
	PID->out_min = out_min;
	
	PID->out = 0;
	PID->error = 0;
	PID->last_error = 0;
	PID->last_last_error = 0;
}


void pid_tune(PID_Typedef* PID, float kP, float kI, float kD)
{
	PID->kP = kP;
	PID->kI = kI;
	PID->kD = kD;
}




float pid(PID_Typedef* PID, float input, float target)
{
	

	PID->error = target - input;
	
	PID->out += PID->kP * (PID->error - PID->last_error)
							+PID->kI * PID->error
							+PID->kD * (PID->error - 2 * PID->last_error + PID->last_last_error);
	
	PID->last_last_error = PID->last_error;
	PID->last_error = PID->error;
	
	if(PID->out > PID->out_max)
		PID->out = PID->out_max;
	if(PID->out < PID->out_min)
		PID->out = PID->out_min;
	
	//if(input!=0)printf("%d, %d, %f, %f,\r\n", input, target, PID->error, PID->out);
	
	return PID->out;
}


