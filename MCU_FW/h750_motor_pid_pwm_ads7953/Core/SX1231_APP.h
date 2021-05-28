#ifndef __SX1231_APP_H__
#define __SX1231_APP_H__

#include "SX1231_Driver.h"
#include "main.h"

#define SX1231_PayloadLength 0x30

extern SX1231_Typedef SX1231_Secondary;

void SX1231_Init_Secondary();
void SX1231_Secondary_Reset();
uint8_t Checksum(uint8_t bfr[], uint32_t length);

uint8_t SX1231_Secondary_ReceivePacket(uint8_t bfr[SX1231_PayloadLength]);


#endif