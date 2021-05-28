#include "seek_line.h"
#include "pid.h"
#include "motor.h"


#define SENS_LEFT_ID 0
#define SENS_RIGHT_ID 15

#define LINE_CENTER_TARGET 7

#define LINE_WIDTH_TH 3
#define LINE_WIDTH_MAX 6



#define WINDOW_W 4

PID_Typedef PIDx, PIDy, PIDr;


void seek_line_pid_init(float kPx, float kIx, float kDx, float kPy, float kIy, float kDy, float kPr, float kIr, float kDr)
{
	pid_init(&PIDx, kPx, kIx, kDx, SPEED_MAX, -SPEED_MAX);
	pid_init(&PIDy, kPy, kIy, kDy, SPEED_MAX, -SPEED_MAX);
	pid_init(&PIDr, kPr, kIr, kDr, SPEED_MAX, -SPEED_MAX);
}

void seek_line_pid_tune(float kPx, float kIx, float kDx, float kPy, float kIy, float kDy, float kPr, float kIr, float kDr)
{
	pid_tune(&PIDx, kPx, kIx, kDx);
	pid_tune(&PIDy, kPy, kIy, kDy);
	pid_tune(&PIDr, kPr, kIr, kDr);
}


uint16_t auto_th(uint16_t dat[16])
{
	uint16_t th;
	uint32_t sum=0, wsum;
	uint8_t i, j;
	for(i = SENS_LEFT_ID; i <= SENS_RIGHT_ID - WINDOW_W + 1; i++)
	{
		wsum = 0;
		for(j=0; j<WINDOW_W; j++)
		{
			wsum += dat[i + j];
		}
		sum += wsum;
	}
	th = sum / WINDOW_W / (SENS_RIGHT_ID - WINDOW_W + 1);
	
	return th;
}


void detect_abov_th(uint16_t dat[16], uint16_t th, uint8_t out[17])
{
	uint8_t i, abov_th_cnt = 0;
	
	for (i = SENS_LEFT_ID; i <= SENS_RIGHT_ID; i++)
	{
		if(dat[i] > th)
		{
			abov_th_cnt++;
			out[abov_th_cnt] = i;
		}
	}
	out[0] = abov_th_cnt;
}


void seek_line(uint16_t F[16], uint16_t R[16], int32_t* speed_y, int32_t* speed_x, int32_t* speed_r)
{
	uint8_t F_abov_th[17]= {0}, R_abov_th[17]= {0};
	uint8_t F_center, R_center;
	uint8_t i;
	uint16_t sum;
	
	detect_abov_th(F, auto_th(F), F_abov_th);
	detect_abov_th(R, auto_th(R), R_abov_th);
	
	
	if(F_abov_th[0])
	{
		sum = 0;
		for(i=1; i<=F_abov_th[0]; i++)
		{
			sum += F_abov_th[i];
		}
		F_center = sum / F_abov_th[0];
	}
	
	if(R_abov_th[0])
	{
		sum = 0;
		for(i=1; i<=R_abov_th[0]; i++)
		{
			sum += R_abov_th[i];
		}
		R_center = sum / R_abov_th[0];
	}
	
	
	if(F_abov_th[0] >= LINE_WIDTH_TH && F_abov_th[0] <= LINE_WIDTH_MAX &&
			R_abov_th[0] >= LINE_WIDTH_TH && R_abov_th[0] <= LINE_WIDTH_MAX)
	{
		*speed_r = pid(&PIDr, F_center-R_center, LINE_CENTER_TARGET);
		*speed_x = pid(&PIDx, R_center, LINE_CENTER_TARGET);
	}
	else if (F_abov_th[0] >= LINE_WIDTH_TH && F_abov_th[0] <= LINE_WIDTH_MAX)
	{
		*speed_r = pid(&PIDr, F_center, LINE_CENTER_TARGET);
	}
	else if (R_abov_th[0] >= LINE_WIDTH_TH && R_abov_th[0] <= LINE_WIDTH_MAX)
	{
		*speed_r = pid(&PIDr, R_center, LINE_CENTER_TARGET);
	}
	else
	{
		*speed_y = 0;
		*speed_x = 0;
		*speed_r = 0;
	}
	
}
