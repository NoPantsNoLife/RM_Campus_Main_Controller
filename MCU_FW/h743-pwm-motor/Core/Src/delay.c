#include "delay.h"

void delayms(uint32_t t)
{
	LL_mDelay(t);
}
void delay(uint32_t t)
{
	while(t--);
}

void delayus(uint32_t t)
{
	///
}
