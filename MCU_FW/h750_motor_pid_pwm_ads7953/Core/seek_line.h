#ifndef __SEEK_LINE_H_
#define __SEEK_LINE_H_

#include "main.h"

void seek_line(uint16_t F[16], uint16_t R[16], int32_t* speed_y, int32_t* speed_x, int32_t* speed_r); 
void seek_line_pid_init(float kPx, float kIx, float kDx, float kPy, float kIy, float kDy, float kPr, float kIr, float kDr);
void seek_line_pid_tune(float kPx, float kIx, float kDx, float kPy, float kIy, float kDy, float kPr, float kIr, float kDr);

#endif