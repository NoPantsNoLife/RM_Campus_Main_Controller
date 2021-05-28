#include "SX1231_APP.h"
#include "main.h"
#include "delay.h"

#include "stdio.h"

SX1231_Typedef SX1231_Secondary;

#define SX1231_SECONDARY_INVALID_THRESHOLD 25000


uint8_t Checksum(uint8_t bfr[], uint32_t length)
{
	uint32_t i;
	uint8_t s = 0;
	for(i=0; i<length-1; i++)
		s += bfr[i];
	return s;
}


uint8_t SX1231_Secondary_ReceivePacket(uint8_t bfr[SX1231_PayloadLength])
{
	static uint32_t SX1231_Secondary_InvalidTimes;
	
	
	if(SX1231_Secondary_InvalidTimes > SX1231_SECONDARY_INVALID_THRESHOLD)
	{
		SX1231_Secondary_InvalidTimes = 0;
		if(SX1231_IsActiveFlag_Timeout(&SX1231_Secondary) 
				|| ! SX1231_IsActiveFlag_RxReady(&SX1231_Secondary) 
				|| (SX1231_IsActiveFlag_Rssi(&SX1231_Secondary) && !SX1231_IsActiveFlag_FifoNotEmpty(&SX1231_Secondary))
				|| (SX1231_IsActiveFlag_FifoNotEmpty(&SX1231_Secondary) && ! SX1231_IsActiveFlag_PayloadReady(&SX1231_Secondary))
			) 
		{
			printf("AFC!");
			SX1231_Secondary_InvalidTimes = 0;
			SX1231_ClearFifo(&SX1231_Secondary);
			SX1231_SetAfcStart(&SX1231_Secondary);
			SX1231_SetRestartRx(&SX1231_Secondary);  
		}
		else if(! SX1231_IsActiveFlag_ModeReady(&SX1231_Secondary) || ! SX1231_IsActiveFlag_PllLock(&SX1231_Secondary))
		{
			printf("Rst!\r\n");
			SX1231_Init_Secondary();
		}
		else
		{
			printf("Rr!");
			SX1231_SetRestartRx(&SX1231_Secondary);  
		}
	}
	else
	{
		if(SX1231_IsActiveFlag_PayloadReady(&SX1231_Secondary))
		{
			//printf("rx");
			SX1231_ReadFifo(&SX1231_Secondary, bfr, SX1231_PayloadLength);
			SX1231_ClearFifo(&SX1231_Secondary);
			if(bfr[SX1231_PayloadLength - 1] == Checksum(bfr, SX1231_PayloadLength))
			{
				SX1231_Secondary_InvalidTimes = 0;
				return 1;
			}
			else
				printf("chkE!");
		}	
	}
	SX1231_Secondary_InvalidTimes++;
	return 0;
}


void SX1231_Init_Secondary()
{
	SX1231_Typedef_Init(&SX1231_Secondary);

	SX1231_Secondary.SPI_Port = SPI4;
	SX1231_Secondary.NSS_GPIO_Pin = SPI_NE_Pin;
	SX1231_Secondary.NSS_GPIO_Port = SPI_NE_GPIO_Port;
	SX1231_Secondary.DMA_Enable = 0;
	
	SX1231_Secondary.Common.Mode = SX1231_MODE_RX;
	SX1231_Secondary.Common.BitRate = 9600;
	SX1231_Secondary.Common.Fdev = 20000;
	SX1231_Secondary.Common.Frf = 913456789;

	//SX1231_Secondary.IRQ_PinMapping.ClkOut = 5;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[0] = 1;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[1] = 3;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[2] = 0;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[3] = 2;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[4] = 1;
	SX1231_Secondary.IRQ_PinMapping.DioMapping[5] = 3;

	SX1231_Secondary.IRQ_PinMapping.RssiThreshold = 0xff;
	SX1231_Secondary.IRQ_PinMapping.TimeoutRxStart = 0x80;
	SX1231_Secondary.IRQ_PinMapping.TimeoutRssiThresh = 0x80;


	SX1231_Secondary.Receiver.LnaZin = SX1231_LNA_ZIN_200_OHMS;
	SX1231_Secondary.Receiver.RxBwMant = SX1231_RX_BW_MANT_20;
	SX1231_Secondary.Receiver.RxBwExp = 2;

	SX1231_Secondary.PacketEngine.PreambleSize = 4;
	SX1231_Secondary.PacketEngine.SyncSize = 2;
	SX1231_Secondary.PacketEngine.SyncValue[0] = 0x66;
	SX1231_Secondary.PacketEngine.SyncValue[1] = 0xcc;
	SX1231_Secondary.PacketEngine.SyncValue[2] = 0xff;
	SX1231_Secondary.PacketEngine.PacketFormat = SX1231_PACKET_FORMAT_FIXED_LENGTH;
	SX1231_Secondary.PacketEngine.AddressFiltering = SX1231_ADDRESS_FILTERING_NONE;// MATCH_NODE_ADDRESS;
	SX1231_Secondary.PacketEngine.PayloadLength = SX1231_PayloadLength;
	SX1231_Secondary.PacketEngine.NodeAddress = 0x23;
	SX1231_Secondary.PacketEngine.AutoRxRestartOn = SX1231_AUTO_RX_RESTART_ENABLED;
	SX1231_Secondary.PacketEngine.DcFree = SX1231_DC_FREE_MANCHESTER;
	
	SX1231_Secondary_Reset();
	SX1231_Init(&SX1231_Secondary);
}


void SX1231_Secondary_Reset()
{
	LL_GPIO_SetOutputPin(RF_RST_GPIO_Port, RF_RST_Pin);
	delayms(2);
	LL_GPIO_ResetOutputPin(RF_RST_GPIO_Port, RF_RST_Pin);
	delayms(10);
}