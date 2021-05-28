#include "SX1231_Driver.h"




__STATIC_INLINE void SX1231_SPI_Tx(SX1231_Typedef* SX1231, uint8_t dat);
__STATIC_INLINE uint8_t SX1231_SPI_Rx(SX1231_Typedef* SX1231);
__STATIC_INLINE void SX1231_SPI_Enable(SX1231_Typedef* SX1231);
__STATIC_INLINE void SX1231_SPI_Disable(SX1231_Typedef* SX1231);


void SX1231_Typedef_Init(SX1231_Typedef* SX1231)
{
	SX1231->DMA_Enable = 0;
	SX1231->Common.SequencerOff = SX1231_SEQUENCER_OFF_DISABLED;
	SX1231->Common.ListenOn = SX1231_LISTEN_DISABLED;
	SX1231->Common.Mode = SX1231_MODE_STDBY;
	SX1231->Common.DataMode = SX1231_DATA_MODE_PKT;
	SX1231->Common.ModulationType = SX1231_MODULATION_TYPE_FSK;
	SX1231->Common.BitRate = 4800;
	SX1231->Common.Fdev = 5000;
	SX1231->Common.Frf = 915000000;
	SX1231->Common.LowBatOn = SX1231_LOW_BAT_OFF;
	SX1231->Common.LowBatTrim = SX1231_LOW_BAT_TH_1V835;
	SX1231->Common.ListenResol = SX1231_LISTEN_RESOLUTION_4_1ms;
	SX1231->Common.ListenCriteria = SX1231_LISTEN_CRITERIA_SIGNAL_STRENGTH_ABOVE_RSSI_THRESHOLD;
	SX1231->Common.ListenEnd = SX1231_LISTEN_END_STAY_IN_RX_UNTIL_INTERRUPT;
	SX1231->Common.ListenCoefIdle = 0xf5;
	SX1231->Common.ListenCoefRx = 0x20;

	SX1231->Transmitter.PaOn = SX1231_PA0_ON;
	SX1231->Transmitter.OutputPower = 0x1f;
	SX1231->Transmitter.PaRamp = SX1231_PA_RAMP_40_us;
	SX1231->Transmitter.OcpOn = SX1231_OCP_ENABLE;
	SX1231->Transmitter.OcpTrim = 0xb;
	
	SX1231->Receiver.AgcAutoReferenceOn = SX1231_AGC_REFERENCE_AUTO;
	SX1231->Receiver.AgcReferenceLevel = 0;
	SX1231->Receiver.AgcSnrMargin = 5;
	SX1231->Receiver.AgcStep[0] = 16;
	SX1231->Receiver.AgcStep[1] = 7;
	SX1231->Receiver.AgcStep[2] = 11;
	SX1231->Receiver.AgcStep[3] = 9;
	SX1231->Receiver.AgcStep[4] = 11;
	SX1231->Receiver.LnaZin = SX1231_LNA_ZIN_200_OHMS;
	SX1231->Receiver.LnaLowPowerOn = SX1231_LNA_LOW_POWER_NORMAL_MODE;
	SX1231->Receiver.LnaGainSelect = SX1231_LNA_GAIN_SET_BY_AGC;
	SX1231->Receiver.DccFreq = 2;
	SX1231->Receiver.RxBwMant = SX1231_RX_BW_MANT_24;
	SX1231->Receiver.RxBwExp = 5;
	SX1231->Receiver.DccFreqAfc = 4;
	SX1231->Receiver.RxBwMantAfc = SX1231_RX_BW_MANT_20;
	SX1231->Receiver.RxBwExpAfc = 3;
	SX1231->Receiver.OokThreshType = SX1231_OOK_THRESH_PEAK;
	SX1231->Receiver.OokPeakTheshStep = SX1231_OOK_PEAK_THRESH_STEP_0_5_dB;
	SX1231->Receiver.OokAverageThreshFilt = 2;
	SX1231->Receiver.OokFixedThresh = 6;
	SX1231->Receiver.AfcAutoclearOn = SX1231_AFC_AUTOCLEAR_DISABLED;
	SX1231->Receiver.AfcAutoOn = SX1231_AFC_AUTO_DISABLED;
	SX1231->Receiver.AfcValue = 0x0;
	SX1231->Receiver.FastRx = SX1231_FAST_RX_2_RSSI_SAMPLE;

	SX1231->IRQ_PinMapping.DioMapping[0] = 0;
	SX1231->IRQ_PinMapping.DioMapping[1] = 0;
	SX1231->IRQ_PinMapping.DioMapping[2] = 0;
	SX1231->IRQ_PinMapping.DioMapping[3] = 0;
	SX1231->IRQ_PinMapping.DioMapping[4] = 0;
	SX1231->IRQ_PinMapping.DioMapping[5] = 0;
	SX1231->IRQ_PinMapping.ClkOut = 7;
	SX1231->IRQ_PinMapping.RssiThreshold = 0xe4;
	SX1231->IRQ_PinMapping.TimeoutRxStart = 0x00;
	SX1231->IRQ_PinMapping.TimeoutRssiThresh = 0x00;

	SX1231->PacketEngine.PreambleSize = 3;
	SX1231->PacketEngine.SyncOn = SX1231_SYNC_ENABLED;
	SX1231->PacketEngine.FifoFillCondition = SX1231_FIFO_FILL_CONDITION_SYNC_ADDRESS_INTERRUPT;
	SX1231->PacketEngine.SyncSize = 3;
	SX1231->PacketEngine.SyncTol = 0;
	SX1231->PacketEngine.SyncValue[0] = 0x00;
	SX1231->PacketEngine.SyncValue[1] = 0x01;
	SX1231->PacketEngine.SyncValue[2] = 0x01;
	SX1231->PacketEngine.SyncValue[3] = 0x01;
	SX1231->PacketEngine.SyncValue[4] = 0x01;
	SX1231->PacketEngine.SyncValue[5] = 0x01;
	SX1231->PacketEngine.SyncValue[6] = 0x01;
	SX1231->PacketEngine.SyncValue[7] = 0x01;
	SX1231->PacketEngine.PacketFormat = SX1231_PACKET_FORMAT_FIXED_LENGTH;
	SX1231->PacketEngine.DcFree = SX1231_DC_FREE_OFF;
	SX1231->PacketEngine.CrcOn = SX1231_CRC_ON;
	SX1231->PacketEngine.CrcAutoClearOff = SX1231_CRC_AUTO_CLEAR_OFF;
	SX1231->PacketEngine.AddressFiltering = SX1231_ADDRESS_FILTERING_NONE;
	SX1231->PacketEngine.PayloadLength = 0x40;
	SX1231->PacketEngine.NodeAddress = 0x00;
	SX1231->PacketEngine.BroadcastAddress = 0x00;
	SX1231->PacketEngine.IntermediateEnterCondition = SX1231_INTERMEDIATE_ENTER_CONDITION_NONE;
	SX1231->PacketEngine.IntermediateExitCondition = SX1231_INTERMEDIATE_EXIT_CONDITION_NONE;
	SX1231->PacketEngine.IntermediateMode = SX1231_INTERMEDIATE_MODE_SLEEP;
	SX1231->PacketEngine.TxStartCondition = SX1231_TX_START_CONDITION_FIFI_NOT_EMPTY;
	SX1231->PacketEngine.FifoThreshold = 0x0f;
	SX1231->PacketEngine.InterPacketRxDelay = 0x0;
	SX1231->PacketEngine.AutoRxRestartOn = SX1231_AUTO_RX_RESTART_ENABLED;
	SX1231->PacketEngine.AesOn = SX1231_AES_DISABLED;
	SX1231->PacketEngine.AesKey[0] = 0x00;
	SX1231->PacketEngine.AesKey[1] = 0x00;
	SX1231->PacketEngine.AesKey[2] = 0x00;
	SX1231->PacketEngine.AesKey[3] = 0x00;
	SX1231->PacketEngine.AesKey[4] = 0x00;
	SX1231->PacketEngine.AesKey[5] = 0x00;
	SX1231->PacketEngine.AesKey[6] = 0x00;
	SX1231->PacketEngine.AesKey[7] = 0x00;
	SX1231->PacketEngine.AesKey[8] = 0x00;
	SX1231->PacketEngine.AesKey[9] = 0x00;
	SX1231->PacketEngine.AesKey[10] = 0x00;
	SX1231->PacketEngine.AesKey[11] = 0x00;
	SX1231->PacketEngine.AesKey[12] = 0x00;
	SX1231->PacketEngine.AesKey[13] = 0x00;
	SX1231->PacketEngine.AesKey[14] = 0x00;
	SX1231->PacketEngine.AesKey[15] = 0x00;

	SX1231->TempSensor.AdcLowPowerOn = SX1231_TEMP_ADC_LOW_POWER_ENABLED;
	
	SX1231->Test.SensitivityBoost = SX1231_SENSITIVITY_BOOST_NORMAL;
	SX1231->Test.OokDeltaThreshold = 0x0c;
	
}


void SX1231_Init(SX1231_Typedef* SX1231)
{
	SX1231_SetSequencerOff(SX1231, SX1231->Common.SequencerOff);
	SX1231_SetListenOn(SX1231, SX1231->Common.ListenOn);
	SX1231_SetMode(SX1231, SX1231->Common.Mode);
	SX1231_SetDataMode(SX1231, SX1231->Common.DataMode);
	SX1231_SetModulationType(SX1231, SX1231->Common.ModulationType);
	SX1231_SetBitRate(SX1231, SX1231->Common.BitRate);
	SX1231_SetFreqDev(SX1231, SX1231->Common.Fdev);
	SX1231_SetCarrierFreq(SX1231, SX1231->Common.Frf);
	SX1231_SetLowBatOn(SX1231, SX1231->Common.LowBatOn);
	SX1231_SetLowBatTrim(SX1231, SX1231->Common.LowBatTrim);
	SX1231_SetListenResol(SX1231, SX1231->Common.ListenResol);
	SX1231_SetListenCriteria(SX1231, SX1231->Common.ListenCriteria);
	SX1231_SetListenEnd(SX1231, SX1231->Common.ListenEnd);
	SX1231_SetListenCoefIdle(SX1231, SX1231->Common.ListenCoefIdle);
	SX1231_SetListenCoefRx(SX1231, SX1231->Common.ListenCoefRx);

	SX1231_SetPaOn(SX1231, SX1231->Transmitter.PaOn);
	SX1231_SetOutputPower(SX1231, SX1231->Transmitter.OutputPower);
	SX1231_SetPaRamp(SX1231, SX1231->Transmitter.PaRamp);
	SX1231_SetOcpOn(SX1231, SX1231->Transmitter.OcpOn);
	SX1231_SetOcpTrim(SX1231, SX1231->Transmitter.OcpTrim);

	SX1231_SetAgcAutoReferenceOn(SX1231, SX1231->Receiver.AgcAutoReferenceOn);
	SX1231_SetAgcReferenceLevel(SX1231, SX1231->Receiver.AgcReferenceLevel);
	SX1231_SetAgcSnrMargin(SX1231, SX1231->Receiver.AgcSnrMargin);
	SX1231_SetAgcStep(SX1231, SX1231->Receiver.AgcStep);
	SX1231_SetLnaZin(SX1231, SX1231->Receiver.LnaZin);
	SX1231_SetLnaLowPowerOn(SX1231, SX1231->Receiver.LnaLowPowerOn);
	SX1231_SetLnaGainSelect(SX1231, SX1231->Receiver.LnaGainSelect);
	SX1231_SetDccFreq(SX1231, SX1231->Receiver.DccFreq);
	SX1231_SetRxBwMant(SX1231, SX1231->Receiver.RxBwMant);
	SX1231_SetRxBwExp(SX1231, SX1231->Receiver.RxBwExp);
	SX1231_SetDccFreqAfc(SX1231, SX1231->Receiver.DccFreqAfc);
	SX1231_SetRxBwMantAfc(SX1231, SX1231->Receiver.RxBwMantAfc);
	SX1231_SetRxBwExpAfc(SX1231, SX1231->Receiver.RxBwExpAfc);
	SX1231_SetOokThreshType(SX1231, SX1231->Receiver.OokThreshType);
	SX1231_SetOokPeakTheshStep(SX1231, SX1231->Receiver.OokPeakTheshStep);
	SX1231_SetOokAverageThreshFilt(SX1231, SX1231->Receiver.OokAverageThreshFilt);
	SX1231_SetOokFixedThresh(SX1231, SX1231->Receiver.OokFixedThresh);
	SX1231_SetAfcAutoclearOn(SX1231, SX1231->Receiver.AfcAutoclearOn);
	SX1231_SetAfcAutoOn(SX1231, SX1231->Receiver.AfcAutoOn);
	SX1231_SetAfcValue(SX1231, SX1231->Receiver.AfcValue);
	SX1231_SetFastRx(SX1231, SX1231->Receiver.FastRx);

	SX1231_SetDioMapping(SX1231, 0, SX1231->IRQ_PinMapping.DioMapping[0]);
	SX1231_SetDioMapping(SX1231, 1, SX1231->IRQ_PinMapping.DioMapping[1]);
	SX1231_SetDioMapping(SX1231, 2, SX1231->IRQ_PinMapping.DioMapping[2]);
	SX1231_SetDioMapping(SX1231, 3, SX1231->IRQ_PinMapping.DioMapping[3]);
	SX1231_SetDioMapping(SX1231, 4, SX1231->IRQ_PinMapping.DioMapping[4]);
	SX1231_SetDioMapping(SX1231, 5, SX1231->IRQ_PinMapping.DioMapping[5]);
	SX1231_SetClkOut(SX1231, SX1231->IRQ_PinMapping.ClkOut);
	SX1231_SetRssiThreshold(SX1231, SX1231->IRQ_PinMapping.RssiThreshold);
	SX1231_SetTimeoutRxStart(SX1231, SX1231->IRQ_PinMapping.TimeoutRxStart);
	SX1231_SetTimeoutRssiThresh(SX1231, SX1231->IRQ_PinMapping.TimeoutRssiThresh);

	SX1231_SetPreambleSize(SX1231, SX1231->PacketEngine.PreambleSize);
	SX1231_SetSyncOn(SX1231, SX1231->PacketEngine.SyncOn);
	SX1231_SetFifoFillCondition(SX1231, SX1231->PacketEngine.FifoFillCondition);
	SX1231_SetSyncSize(SX1231, SX1231->PacketEngine.SyncSize);
	SX1231_SetSyncTol(SX1231, SX1231->PacketEngine.SyncTol);
	SX1231_SetSyncValue(SX1231, SX1231->PacketEngine.SyncValue);
	SX1231_SetPacketFormat(SX1231, SX1231->PacketEngine.PacketFormat);
	SX1231_SetDcFree(SX1231, SX1231->PacketEngine.DcFree);
	SX1231_SetCrcOn(SX1231, SX1231->PacketEngine.CrcOn);
	SX1231_SetCrcAutoClearOff(SX1231, SX1231->PacketEngine.CrcAutoClearOff);
	SX1231_SetAddressFiltering(SX1231, SX1231->PacketEngine.AddressFiltering);
	SX1231_SetPayloadLength(SX1231, SX1231->PacketEngine.PayloadLength);
	SX1231_SetNodeAddress(SX1231, SX1231->PacketEngine.NodeAddress);
	SX1231_SetBroadcastAddress(SX1231, SX1231->PacketEngine.BroadcastAddress);
	SX1231_SetIntermediateEnterCondition(SX1231, SX1231->PacketEngine.IntermediateEnterCondition);
	SX1231_SetIntermediateExitCondition(SX1231, SX1231->PacketEngine.IntermediateExitCondition);
	SX1231_SetIntermediateMode(SX1231, SX1231->PacketEngine.IntermediateMode);
	SX1231_SetTxStartCondition(SX1231, SX1231->PacketEngine.TxStartCondition);
	SX1231_SetFifoThreshold(SX1231, SX1231->PacketEngine.FifoThreshold);
	SX1231_SetInterPacketRxDelay(SX1231, SX1231->PacketEngine.InterPacketRxDelay);
	SX1231_SetAutoRxRestartOn(SX1231, SX1231->PacketEngine.AutoRxRestartOn);
	SX1231_SetAesOn(SX1231, SX1231->PacketEngine.AesOn);
	SX1231_SetAesKey(SX1231, SX1231->PacketEngine.AesKey);

	SX1231_SetTempAdcLowPowerOn(SX1231, SX1231->TempSensor.AdcLowPowerOn);

	SX1231_SetSensitivityBoost(SX1231, SX1231->Test.SensitivityBoost);
	SX1231_SetOokDeltaThreshold(SX1231, SX1231->Test.OokDeltaThreshold);

}


void SX1231_ClearFifo(SX1231_Typedef* SX1231)
{
	while(SX1231_IsActiveFlag_FifoNotEmpty(SX1231))
		SX1231_ReadReg(SX1231, _SX1231_RegFifo);
}


void SX1231_WriteFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length)
{
	uint32_t i;
	
	if (SX1231->DMA_Enable)
	{
		// xie ni ma ne !!
	}
	else
	{
		SX1231_SPI_Enable(SX1231);
		SX1231_SPI_Tx(SX1231, _SX1231_RegFifo | _SX1231_WRITE_ADDR_MASK);
		for(i=0; i<length; i++)
			SX1231_SPI_Tx(SX1231, bfr[i]);
		SX1231_SPI_Disable(SX1231);
	}
}


void SX1231_ReadFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length)
{
	uint32_t i;
	
	if (SX1231->DMA_Enable)
	{
		// xie ni ma ne !!
	}
	else
	{
		SX1231_SPI_Enable(SX1231);
		SX1231_SPI_Tx(SX1231, _SX1231_RegFifo);
		for(i=0; i<length; i++)
			bfr[i] = SX1231_SPI_Rx(SX1231);
		SX1231_SPI_Disable(SX1231);
	}
}


void SX1231_SetOokDeltaThreshold(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegTestOok, val);
}


uint8_t SX1231_GetOokDeltaThreshold(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegTestOok);
}


void SX1231_SetSensitivityBoost(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegTestLna, val);
}


uint8_t SX1231_GetSensitivityBoost(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegTestLna);
}


uint8_t SX1231_GetTempValue(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegTemp2);
}


uint8_t SX1231_GetTempAdcLowPowerOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegTemp1) & _SX1231_ADC_LOW_POWER_ON_MASK) >> _SX1231_ADC_LOW_POWER_ON_BIT;
}


void SX1231_SetTempAdcLowPowerOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegTemp1, (SX1231_ReadReg(SX1231, _SX1231_RegTemp1) & ~_SX1231_ADC_LOW_POWER_ON_MASK) | (val << _SX1231_ADC_LOW_POWER_ON_BIT));
}


uint8_t SX1231_GetTempMeasRunning(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegTemp1) & _SX1231_TEMP_MEAS_RUNNING_MASK) >> _SX1231_TEMP_MEAS_RUNNING_BIT;
}


void SX1231_SetTempMeasStart(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegTemp1, (SX1231_ReadReg(SX1231, _SX1231_RegTemp1) & ~_SX1231_TEMP_MEAS_START_MASK) | (1 << _SX1231_TEMP_MEAS_START_BIT));
}


void SX1231_SetAesKey(SX1231_Typedef* SX1231, uint8_t AesKey[16])
{
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[0]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey2, AesKey[1]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey3, AesKey[2]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[3]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[4]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[5]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[6]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[7]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[8]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[9]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[10]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[11]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[12]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[13]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[14]);
	SX1231_WriteReg(SX1231, _SX1231_RegAesKey1, AesKey[15]);
}

uint8_t SX1231_GetAesOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & _SX1231_AES_ON_MASK) >> _SX1231_AES_ON_BIT;
}


void SX1231_SetAesOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig2, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & ~_SX1231_AES_ON_MASK) | (val << _SX1231_AES_ON_BIT));
}


uint8_t SX1231_GetAutoRxRestartOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & _SX1231_AUTO_RX_RESTART_ON_MASK) >> _SX1231_AUTO_RX_RESTART_ON_BIT;
}


void SX1231_SetAutoRxRestartOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig2, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & ~_SX1231_AUTO_RX_RESTART_ON_MASK) | (val << _SX1231_AUTO_RX_RESTART_ON_BIT));
}


void SX1231_SetRestartRx(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig2, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & ~_SX1231_RESTART_RX_MASK) | (1 << _SX1231_RESTART_RX_BIT));
}


uint8_t SX1231_GetInterPacketRxDelay(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & _SX1231_INTER_PACKET_RX_DELAY_MASK) >> _SX1231_INTER_PACKET_RX_DELAY_BIT;
}


void SX1231_SetInterPacketRxDelay(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig2, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig2) & ~_SX1231_INTER_PACKET_RX_DELAY_MASK) | (val << _SX1231_INTER_PACKET_RX_DELAY_BIT));
}


uint8_t SX1231_GetFifoThreshold(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegFifoThresh) & _SX1231_FIFO_THRESHOLD_MASK) >> _SX1231_FIFO_THRESHOLD_BIT;
}


void SX1231_SetFifoThreshold(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegFifoThresh, (SX1231_ReadReg(SX1231, _SX1231_RegFifoThresh) & ~_SX1231_FIFO_THRESHOLD_MASK) | (val << _SX1231_FIFO_THRESHOLD_BIT));
}


uint8_t SX1231_GetTxStartCondition(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegFifoThresh) & _SX1231_TX_START_CONDITION_MASK) >> _SX1231_TX_START_CONDITION_BIT;
}


void SX1231_SetTxStartCondition(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegFifoThresh, (SX1231_ReadReg(SX1231, _SX1231_RegFifoThresh) & ~_SX1231_TX_START_CONDITION_MASK) | (val << _SX1231_TX_START_CONDITION_BIT));
}


uint8_t SX1231_GetIntermediateMode(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & _SX1231_INTERMEDIATE_MODE_MASK) >> _SX1231_INTERMEDIATE_MODE_BIT;
}


void SX1231_SetIntermediateMode(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAutoModes, (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & ~_SX1231_INTERMEDIATE_MODE_MASK) | (val << _SX1231_INTERMEDIATE_MODE_BIT));
}


uint8_t SX1231_GetIntermediateExitCondition(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & _SX1231_EXIT_CONDITION_MASK) >> _SX1231_EXIT_CONDITION_BIT;
}


void SX1231_SetIntermediateExitCondition(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAutoModes, (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & ~_SX1231_EXIT_CONDITION_MASK) | (val << _SX1231_EXIT_CONDITION_BIT));
}


uint8_t SX1231_GetIntermediateEnterCondition(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & _SX1231_ENTER_CONDITION_MASK) >> _SX1231_ENTER_CONDITION_BIT;
}


void SX1231_SetIntermediateEnterCondition(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAutoModes, (SX1231_ReadReg(SX1231, _SX1231_RegAutoModes) & ~_SX1231_ENTER_CONDITION_MASK) | (val << _SX1231_ENTER_CONDITION_BIT));
}


void SX1231_SetBroadcastAddress(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegBroadcastAdrs, val);
}


uint8_t SX1231_GetBroadcastAddress(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegBroadcastAdrs);
}


void SX1231_SetNodeAddress(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegNodeAdrs, val);
}


uint8_t SX1231_GetNodeAddress(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegNodeAdrs);
}


void SX1231_SetPayloadLength(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPayloadLength, val);
}


uint8_t SX1231_GetPayloadLength(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegPayloadLength);
}


uint8_t SX1231_GetPacketFormat(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & _SX1231_PACKET_FORMAT_MASK) >> _SX1231_PACKET_FORMAT_BIT;
}


void SX1231_SetPacketFormat(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig1, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & ~_SX1231_PACKET_FORMAT_MASK) | (val << _SX1231_PACKET_FORMAT_BIT));
}


uint8_t SX1231_GetDcFree(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & _SX1231_DC_FREE_MASK) >> _SX1231_DC_FREE_BIT;
}


void SX1231_SetDcFree(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig1, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & ~_SX1231_DC_FREE_MASK) | (val << _SX1231_DC_FREE_BIT));
}


uint8_t SX1231_GetCrcOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & _SX1231_CRC_ON_MASK) >> _SX1231_CRC_ON_BIT;
}


void SX1231_SetCrcOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig1, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & ~_SX1231_CRC_ON_MASK) | (val << _SX1231_CRC_ON_BIT));
}


uint8_t SX1231_GetCrcAutoClearOff(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & _SX1231_CRC_AUTO_CLEAR_OFF_MASK) >> _SX1231_CRC_AUTO_CLEAR_OFF_BIT;
}


void SX1231_SetCrcAutoClearOff(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig1, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & ~_SX1231_CRC_AUTO_CLEAR_OFF_MASK) | (val << _SX1231_CRC_AUTO_CLEAR_OFF_BIT));
}


uint8_t SX1231_GetAddressFiltering(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & _SX1231_ADDRESS_FILTERING_MASK) >> _SX1231_ADDRESS_FILTERING_BIT;
}


void SX1231_SetAddressFiltering(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPacketConfig1, (SX1231_ReadReg(SX1231, _SX1231_RegPacketConfig1) & ~_SX1231_ADDRESS_FILTERING_MASK) | (val << _SX1231_ADDRESS_FILTERING_BIT));
}


void SX1231_GetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8])
{
	SyncValue[0] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue1);
	SyncValue[1] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue2);
	SyncValue[2] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue3);
	SyncValue[3] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue4);
	SyncValue[4] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue5);
	SyncValue[5] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue6);
	SyncValue[6] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue7);
	SyncValue[7] = SX1231_ReadReg(SX1231, _SX1231_RegSyncValue8);
}


void SX1231_SetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8])
{
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue1, SyncValue[0]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue2, SyncValue[1]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue3, SyncValue[2]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue4, SyncValue[3]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue5, SyncValue[4]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue6, SyncValue[5]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue7, SyncValue[6]);
	SX1231_WriteReg(SX1231, _SX1231_RegSyncValue8, SyncValue[7]);
}


uint8_t SX1231_GetSyncTol(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & _SX1231_SYNC_TOL_MASK) >> _SX1231_SYNC_TOL_BIT;
}


void SX1231_SetSyncTol(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegSyncConfig, (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & ~_SX1231_SYNC_TOL_MASK) | (val << _SX1231_SYNC_TOL_BIT));
}


uint8_t SX1231_GetSyncSize(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & _SX1231_SYNC_SIZE_MASK) >> _SX1231_SYNC_SIZE_BIT;
}


void SX1231_SetSyncSize(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegSyncConfig, (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & ~_SX1231_SYNC_SIZE_MASK) | (val << _SX1231_SYNC_SIZE_BIT));
}


uint8_t SX1231_GetFifoFillCondition(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & _SX1231_FIFO_FILL_CONDITION_MASK) >> _SX1231_FIFO_FILL_CONDITION_BIT;
}


void SX1231_SetFifoFillCondition(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegSyncConfig, (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & ~_SX1231_FIFO_FILL_CONDITION_MASK) | (val << _SX1231_FIFO_FILL_CONDITION_BIT));
}


uint8_t SX1231_GetSyncOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & _SX1231_SYNC_ON_MASK) >> _SX1231_SYNC_ON_BIT;
}


void SX1231_SetSyncOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegSyncConfig, (SX1231_ReadReg(SX1231, _SX1231_RegSyncConfig) & ~_SX1231_SYNC_ON_MASK) | (val << _SX1231_SYNC_ON_BIT));
}


uint16_t SX1231_GetPreambleSize(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegPreambleMsb) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegPreambleLsb);
}


void SX1231_SetPreambleSize(SX1231_Typedef* SX1231, uint16_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPreambleMsb, val >> 8);
	SX1231_WriteReg(SX1231, _SX1231_RegPreambleLsb, val & 0xff);
}


void SX1231_SetTimeoutRssiThresh(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRxTimeout2, val);
}


uint8_t SX1231_GetTimeoutRssiThresh(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegRxTimeout2);
}


void SX1231_SetTimeoutRxStart(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRxTimeout1, val);
}


uint8_t SX1231_GetTimeoutRxStart(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegRxTimeout1);
}


void SX1231_SetRssiThreshold(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRssiThresh, val);
}


uint8_t SX1231_GetRssiThreshold(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegRssiThresh);
}


uint8_t SX1231_IsActiveFlag_FifoFull(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_FIFO_FULL_MASK) >> _SX1231_FIFO_FULL_BIT;
}


uint8_t SX1231_IsActiveFlag_FifoNotEmpty(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_FIFO_NOT_EMPTY_MASK) >> _SX1231_FIFO_NOT_EMPTY_BIT;
}


uint8_t SX1231_IsActiveFlag_FifoLevel(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_FIFO_LEVEL_MASK) >> _SX1231_FIFO_LEVEL_BIT;
}


uint8_t SX1231_IsActiveFlag_FifoOverrun(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_FIFO_OVERRUN_MASK) >> _SX1231_FIFO_OVERRUN_BIT;
}


uint8_t SX1231_IsActiveFlag_PacketSent(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_PACKET_SENT_MASK) >> _SX1231_PACKET_SENT_BIT;
}


uint8_t SX1231_IsActiveFlag_PayloadReady(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_PAY_LOAD_READY_MASK) >> _SX1231_PAY_LOAD_READY_BIT;
}


uint8_t SX1231_IsActiveFlag_CrcOk(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_CRC_OK_MASK) >> _SX1231_CRC_OK_BIT;
}


uint8_t SX1231_IsActiveFlag_LowBat(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags2) & _SX1231_LOW_BAT_MASK) >> _SX1231_LOW_BAT_BIT;
}


uint8_t SX1231_IsActiveFlag_ModeReady(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_MODE_READY_MASK) >> _SX1231_FLAG_MODE_READY_BIT;
}


uint8_t SX1231_IsActiveFlag_RxReady(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_RX_READY_MASK) >> _SX1231_FLAG_RX_READY_BIT;
}


uint8_t SX1231_IsActiveFlag_TxReady(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_TX_READY_MASK) >> _SX1231_FLAG_TX_READY_BIT;
}


uint8_t SX1231_IsActiveFlag_PllLock(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_PLL_LOCK_MASK) >> _SX1231_FLAG_PLL_LOCK_BIT;
}


uint8_t SX1231_IsActiveFlag_Rssi(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_RSSI_MASK) >> _SX1231_FLAG_RSSI_BIT;
}


uint8_t SX1231_IsActiveFlag_Timeout(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_TIMEOUT_MASK) >> _SX1231_FLAG_TIMEOUT_BIT;
}


uint8_t SX1231_IsActiveFlag_AutoMode(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_AUTO_MODE_MASK) >> _SX1231_FLAG_AUTO_MODE_BIT;
}


uint8_t SX1231_IsActiveFlag_SyncAddressMatch(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegIrqFlags1) & _SX1231_FLAG_SYNC_ADDRESS_MATCH_MASK) >> _SX1231_FLAG_SYNC_ADDRESS_MATCH_BIT;
}


uint8_t SX1231_GetClkOut(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegDioMapping2) & _SX1231_CLK_OUT_MASK) >> _SX1231_CLK_OUT_BIT;
}


void SX1231_SetClkOut(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegDioMapping2, (SX1231_ReadReg(SX1231, _SX1231_RegDioMapping2) & ~_SX1231_CLK_OUT_MASK) | (val << _SX1231_CLK_OUT_BIT));
}


uint8_t SX1231_GetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID)
{
	uint8_t reg, mask, bit;
	if(DioID <= 3)
	{
		reg = _SX1231_RegDioMapping1;
		bit = (3 - DioID) * 2;
	}
	else
	{
		reg = _SX1231_RegDioMapping2;
		bit = (7 - DioID) * 2;
	}
	mask = 0x03 << bit;
	return (SX1231_ReadReg(SX1231, reg) & mask) >> bit;
}


void SX1231_SetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID, uint8_t Mapping)
{
	uint8_t reg, mask, bit;
	if(DioID <= 3)
	{
		reg = _SX1231_RegDioMapping1;
		bit = (3 - DioID) * 2;
	}
	else
	{
		reg = _SX1231_RegDioMapping2;
		bit = (7 - DioID) * 2;
	}
	mask = 0x03 << bit;
	SX1231_WriteReg(SX1231, reg, (SX1231_ReadReg(SX1231, reg) & ~mask) | (Mapping << bit));
}


uint8_t SX1231_GetRssiValue(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegRssiValue);
}


void SX1231_SetRssiStart(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRssiConfig, (SX1231_ReadReg(SX1231, _SX1231_RegRssiConfig) & ~_SX1231_RSSI_START_MASK) | (val << _SX1231_RSSI_START_BIT));
}


uint8_t SX1231_GetRssiDone(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegRssiConfig) & _SX1231_RSSI_DONE_MASK) >> _SX1231_RSSI_DONE_BIT;
}


uint8_t SX1231_GetFastRx(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegRssiConfig) & _SX1231_FAST_RX_MASK) >> _SX1231_FAST_RX_BIT;
}


void SX1231_SetFastRx(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRssiConfig, (SX1231_ReadReg(SX1231, _SX1231_RegRssiConfig) & ~_SX1231_FAST_RX_MASK) | (val << _SX1231_FAST_RX_BIT));
}


uint16_t SX1231_GetFeiValue(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegFeiMsb) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegFeiLsb);
}


uint16_t SX1231_GetAfcValue(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegAfcMsb) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegAfcLsb);
}


void SX1231_SetAfcValue(SX1231_Typedef* SX1231, uint16_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcMsb, val >> 8);
	SX1231_WriteReg(SX1231, _SX1231_RegAfcLsb, val & 0xff);
}


uint8_t SX1231_GetAfcAutoOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & _SX1231_AFC_AUTO_ON_MASK) >> _SX1231_AFC_AUTO_ON_BIT;
}


void SX1231_SetAfcAutoOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcFei, (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & ~_SX1231_AFC_AUTO_ON_MASK) | (val << _SX1231_AFC_AUTO_ON_BIT));
}


uint8_t SX1231_GetAfcAutoclearOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & _SX1231_AFC_AUTO_CLEAR_ON_MASK) >> _SX1231_AFC_AUTO_CLEAR_ON_BIT;
}


void SX1231_SetAfcAutoclearOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcFei, (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & ~_SX1231_AFC_AUTO_CLEAR_ON_MASK) | (val << _SX1231_AFC_AUTO_CLEAR_ON_BIT));
}


uint8_t SX1231_GetAfcClear(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & _SX1231_AFC_CLEAR_MASK) >> _SX1231_AFC_CLEAR_BIT;
}


uint8_t SX1231_GetAfcDone(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & _SX1231_AFC_DONE_MASK) >> _SX1231_AFC_DONE_BIT;
}


void SX1231_SetAfcStart(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcFei, (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & ~_SX1231_AFC_START_MASK) | (1 << _SX1231_AFC_START_BIT));
}


void SX1231_SetAfcClear(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcFei, (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & ~_SX1231_AFC_CLEAR_MASK) | (1 << _SX1231_AFC_CLEAR_BIT));
}

uint8_t SX1231_GetFeiDone(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & _SX1231_FEI_DONE_MASK) >> _SX1231_FEI_DONE_BIT;
}


void SX1231_SetFeiStart(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcFei, (SX1231_ReadReg(SX1231, _SX1231_RegAfcFei) & ~_SX1231_FEI_START_MASK) | (1 << _SX1231_FEI_START_BIT));
}


void SX1231_SetOokFixedThresh(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOokFix, val);
}


uint8_t SX1231_GetOokFixedThresh(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegOokFix);
}


uint8_t SX1231_GetOokAverageThreshFilt(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOokAvg) & _SX1231_OOK_AVERAGE_THRESH_FILT_MASK) >> _SX1231_OOK_AVERAGE_THRESH_FILT_BIT;
}


void SX1231_SetOokAverageThreshFilt(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOokAvg, (SX1231_ReadReg(SX1231, _SX1231_RegOokAvg) & ~_SX1231_OOK_AVERAGE_THRESH_FILT_MASK) | (val << _SX1231_OOK_AVERAGE_THRESH_FILT_BIT));
}


uint8_t SX1231_GetOokPeakThreshDec(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & _SX1231_OOK_PEAK_THRESH_DEC_MASK) >> _SX1231_OOK_PEAK_THRESH_DEC_BIT;
}


void SX1231_SetOokPeakThreshDec(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOokPeak, (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & ~_SX1231_OOK_PEAK_THRESH_DEC_MASK) | (val << _SX1231_OOK_PEAK_THRESH_DEC_BIT));
}


uint8_t SX1231_GetOokPeakTheshStep(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & _SX1231_OOK_PEAK_THESH_STEP_MASK) >> _SX1231_OOK_PEAK_THESH_STEP_BIT;
}


void SX1231_SetOokPeakTheshStep(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOokPeak, (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & ~_SX1231_OOK_PEAK_THESH_STEP_MASK) | (val << _SX1231_OOK_PEAK_THESH_STEP_BIT));
}

uint8_t SX1231_GetOokThreshType(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & _SX1231_OOK_THRESH_TYPE_MASK) >> _SX1231_OOK_THRESH_TYPE_BIT;
}


void SX1231_SetOokThreshType(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOokPeak, (SX1231_ReadReg(SX1231, _SX1231_RegOokPeak) & ~_SX1231_OOK_THRESH_TYPE_MASK) | (val << _SX1231_OOK_THRESH_TYPE_BIT));
}


uint8_t SX1231_GetRxBwExpAfc(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & _SX1231_RX_BW_EXP_AFC_MASK) >> _SX1231_RX_BW_EXP_AFC_BIT;
}


void SX1231_SetRxBwExpAfc(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcBw, (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & ~_SX1231_RX_BW_EXP_AFC_MASK) | (val << _SX1231_RX_BW_EXP_AFC_BIT));
}


uint8_t SX1231_GetRxBwMantAfc(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & _SX1231_RX_BW_MANT_AFC_MASK) >> _SX1231_RX_BW_MANT_AFC_BIT;
}


void SX1231_SetRxBwMantAfc(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcBw, (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & ~_SX1231_RX_BW_MANT_AFC_MASK) | (val << _SX1231_RX_BW_MANT_AFC_BIT));
}


uint8_t SX1231_GetDccFreqAfc(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & _SX1231_DCC_FREQ_AFC_MASK) >> _SX1231_DCC_FREQ_AFC_BIT;
}


void SX1231_SetDccFreqAfc(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAfcBw, (SX1231_ReadReg(SX1231, _SX1231_RegAfcBw) & ~_SX1231_DCC_FREQ_AFC_MASK) | (val << _SX1231_DCC_FREQ_AFC_BIT));
}


uint8_t SX1231_GetRxBwExp(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & _SX1231_RX_BW_EXP_MASK) >> _SX1231_RX_BW_EXP_BIT;
}


void SX1231_SetRxBwExp(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRxBw, (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & ~_SX1231_RX_BW_EXP_MASK) | (val << _SX1231_RX_BW_EXP_BIT));
}


uint8_t SX1231_GetRxBwMant(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & _SX1231_RX_BW_MANT_MASK) >> _SX1231_RX_BW_MANT_BIT;
}


void SX1231_SetRxBwMant(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRxBw, (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & ~_SX1231_RX_BW_MANT_MASK) | (val << _SX1231_RX_BW_MANT_BIT));
}


uint8_t SX1231_GetDccFreq(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & _SX1231_DCC_FREQ_MASK) >> _SX1231_DCC_FREQ_BIT;
}


void SX1231_SetDccFreq(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegRxBw, (SX1231_ReadReg(SX1231, _SX1231_RegRxBw) & ~_SX1231_DCC_FREQ_MASK) | (val << _SX1231_DCC_FREQ_BIT));
}


uint8_t SX1231_GetLnaCurrentGain(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLna) & _SX1231_LNA_CURRENT_GAIN_MASK) >> _SX1231_LNA_CURRENT_GAIN_BIT;
}


uint8_t SX1231_GetLnaGainSelect(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLna) & _SX1231_LNA_GAIN_SELECT_MASK) >> _SX1231_LNA_GAIN_SELECT_BIT;
}


void SX1231_SetLnaGainSelect(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegLna, (SX1231_ReadReg(SX1231, _SX1231_RegLna) & ~_SX1231_LNA_GAIN_SELECT_MASK) | (val << _SX1231_LNA_GAIN_SELECT_BIT));
}

uint8_t SX1231_GetLnaLowPowerOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLna) & _SX1231_LNA_LOW_POWER_ON_MASK) >> _SX1231_LNA_LOW_POWER_ON_BIT;
}


void SX1231_SetLnaLowPowerOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegLna, (SX1231_ReadReg(SX1231, _SX1231_RegLna) & ~_SX1231_LNA_LOW_POWER_ON_MASK) | (val << _SX1231_LNA_LOW_POWER_ON_BIT));
}

uint8_t SX1231_GetLnaZin(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLna) & _SX1231_LNA_ZIN_MASK) >> _SX1231_LNA_ZIN_BIT;
}


void SX1231_SetLnaZin(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegLna, (SX1231_ReadReg(SX1231, _SX1231_RegLna) & ~_SX1231_LNA_ZIN_MASK) | (val << _SX1231_LNA_ZIN_BIT));
}


void SX1231_GetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5])
{
	AgcStep[0] = (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh1) & _SX1231_AGC_STEP_1_MASK) >> _SX1231_AGC_STEP_1_BIT;
	AgcStep[1] = (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh2) & _SX1231_AGC_STEP_2_MASK) >> _SX1231_AGC_STEP_2_BIT;
	AgcStep[2] = (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh2) & _SX1231_AGC_STEP_3_MASK) >> _SX1231_AGC_STEP_3_BIT;
	AgcStep[3] = (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh3) & _SX1231_AGC_STEP_4_MASK) >> _SX1231_AGC_STEP_4_BIT;
	AgcStep[4] = (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh3) & _SX1231_AGC_STEP_5_MASK) >> _SX1231_AGC_STEP_5_BIT;
}


void SX1231_SetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5])
{
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh1, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh1) & ~_SX1231_AGC_STEP_1_MASK) | (AgcStep[0] << _SX1231_AGC_STEP_1_BIT));
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh2, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh2) & ~_SX1231_AGC_STEP_2_MASK) | (AgcStep[0] << _SX1231_AGC_STEP_2_BIT));
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh2, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh2) & ~_SX1231_AGC_STEP_3_MASK) | (AgcStep[0] << _SX1231_AGC_STEP_3_BIT));
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh3, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh3) & ~_SX1231_AGC_STEP_4_MASK) | (AgcStep[0] << _SX1231_AGC_STEP_4_BIT));
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh3, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh3) & ~_SX1231_AGC_STEP_5_MASK) | (AgcStep[0] << _SX1231_AGC_STEP_5_BIT));
}


uint8_t SX1231_GetAgcSnrMargin(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh1) & _SX1231_AGC_SNR_MARGIN_MASK) >> _SX1231_AGC_SNR_MARGIN_BIT;
}


void SX1231_SetAgcSnrMargin(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAgcThresh1, (SX1231_ReadReg(SX1231, _SX1231_RegAgcThresh1) & ~_SX1231_AGC_SNR_MARGIN_MASK) | (val << _SX1231_AGC_SNR_MARGIN_BIT));
}


uint8_t SX1231_GetAgcReferenceLevel(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAgcRef) & _SX1231_AGC_REFERENCE_LEVEL_MASK) >> _SX1231_AGC_REFERENCE_LEVEL_BIT;
}


void SX1231_SetAgcReferenceLevel(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAgcRef, (SX1231_ReadReg(SX1231, _SX1231_RegAgcRef) & ~_SX1231_AGC_REFERENCE_LEVEL_MASK) | (val << _SX1231_AGC_REFERENCE_LEVEL_BIT));
}


uint8_t SX1231_GetAgcAutoReferenceOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegAgcRef) & _SX1231_AGC_AUTO_REFERENCE_ON_MASK) >> _SX1231_AGC_AUTO_REFERENCE_ON_BIT;
}


void SX1231_SetAgcAutoReferenceOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegAgcRef, (SX1231_ReadReg(SX1231, _SX1231_RegAgcRef) & ~_SX1231_AGC_AUTO_REFERENCE_ON_MASK) | (val << _SX1231_AGC_AUTO_REFERENCE_ON_BIT));
}


uint8_t SX1231_GetOcpTrim(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOcp) & _SX1231_OCP_TRIM_MASK) >> _SX1231_OCP_TRIM_BIT;
}


void SX1231_SetOcpTrim(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOcp, (SX1231_ReadReg(SX1231, _SX1231_RegOcp) & ~_SX1231_OCP_TRIM_MASK) | (val << _SX1231_OCP_TRIM_BIT));
}


uint8_t SX1231_GetOcpOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOcp) & _SX1231_OCP_ON_MASK) >> _SX1231_OCP_ON_BIT;
}


void SX1231_SetOcpOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOcp, (SX1231_ReadReg(SX1231, _SX1231_RegOcp) & ~_SX1231_OCP_ON_MASK) | (val << _SX1231_OCP_ON_BIT));
}

uint8_t SX1231_GetPaRamp(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPaRamp) & _SX1231_PA_RAMP_MASK) >> _SX1231_PA_RAMP_BIT;
}


void SX1231_SetPaRamp(SX1231_Typedef* SX1231, uint8_t mask)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPaRamp, (SX1231_ReadReg(SX1231, _SX1231_RegPaRamp) & ~_SX1231_PA_RAMP_MASK) | (mask << _SX1231_PA_RAMP_BIT));
}


uint8_t SX1231_GetPaOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPaLevel) & _SX1231_PA_MASK) >> _SX1231_PA_BIT;
}


void SX1231_SetPaOn(SX1231_Typedef* SX1231, uint8_t mask)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPaLevel, (SX1231_ReadReg(SX1231, _SX1231_RegPaLevel) & ~_SX1231_PA_MASK) | (mask << _SX1231_PA_BIT));
}


uint8_t SX1231_GetOutputPower(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegPaLevel) & _SX1231_OUTPUT_POWER_MASK) >> _SX1231_OUTPUT_POWER_BIT;
}


void SX1231_SetOutputPower(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegPaLevel, (SX1231_ReadReg(SX1231, _SX1231_RegPaLevel) & ~_SX1231_OUTPUT_POWER_MASK) | (val << _SX1231_OUTPUT_POWER_BIT));
}


uint8_t SX1231_GetVersion(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegVersion);
}


void SX1231_SetListenCoefRx(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegListen3, val);
}


uint8_t SX1231_GetListenCoefRx(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegListen3);
}


void SX1231_SetListenCoefIdle(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegListen2, val);
}


uint8_t SX1231_GetListenCoefIdle(SX1231_Typedef* SX1231)
{
	return SX1231_ReadReg(SX1231, _SX1231_RegListen2);
}


uint8_t SX1231_GetListenEnd(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & _SX1231_LISTEN_END_MASK) >> _SX1231_LISTEN_END_BIT;
}


void SX1231_SetListenEnd(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegListen1, (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & ~_SX1231_LISTEN_END_MASK) | (val << _SX1231_LISTEN_END_BIT));
}


uint8_t SX1231_GetListenCriteria(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & _SX1231_LISTEN_CRITERIA_MASK) >> _SX1231_LISTEN_CRITERIA_BIT;
}


void SX1231_SetListenCriteria(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegListen1, (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & ~_SX1231_LISTEN_CRITERIA_MASK) | (val << _SX1231_LISTEN_CRITERIA_BIT));
}


uint8_t SX1231_GetListenResol(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & _SX1231_LISTEN_RESOL_MASK) >> _SX1231_LISTEN_RESOL_BIT;
}


void SX1231_SetListenResol(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegListen1, (SX1231_ReadReg(SX1231, _SX1231_RegListen1) & ~_SX1231_LISTEN_RESOL_MASK) | (val << _SX1231_LISTEN_RESOL_BIT));
}



uint8_t SX1231_GetLowBatTrim(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLowBat) & _SX1231_LOW_BAT_TRIM_MASK) >> _SX1231_LOW_BAT_TRIM_BIT;
}


void SX1231_SetLowBatTrim(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegLowBat, (SX1231_ReadReg(SX1231, _SX1231_RegLowBat) & ~_SX1231_LOW_BAT_TRIM_MASK) | (val << _SX1231_LOW_BAT_TRIM_BIT));
}


uint8_t SX1231_GetLowBatOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLowBat) & _SX1231_LOW_BAT_ON_MASK) >> _SX1231_LOW_BAT_ON_BIT;
}


void SX1231_SetLowBatOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegLowBat, (SX1231_ReadReg(SX1231, _SX1231_RegLowBat) & ~_SX1231_LOW_BAT_ON_MASK) | (val << _SX1231_LOW_BAT_ON_BIT));
}


uint8_t SX1231_GetLowBatMonitor(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegLowBat) & _SX1231_LOW_BAT_MONITOR_MASK) >> _SX1231_LOW_BAT_MONITOR_BIT;
}


void SX1231_SetRcCalStart(SX1231_Typedef* SX1231)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOsc1, 0x80);
}


uint8_t SX1231_GetRcCalDone(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOsc1) & _SX1231_RC_CAL_DONE_MASK ) >> _SX1231_RC_CAL_DONE_BIT;
}


uint32_t SX1231_GetCarrierFreq(SX1231_Typedef* SX1231)
{
	return FSTEP * (SX1231_ReadReg(SX1231, _SX1231_RegFrfMsb) >> 16 | SX1231_ReadReg(SX1231, _SX1231_RegFrfMid) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegFrfLsb));
}


void SX1231_SetCarrierFreq(SX1231_Typedef* SX1231, uint32_t freq)
{
	freq = freq / FSTEP;
	SX1231_WriteReg(SX1231, _SX1231_RegFrfMsb, (freq >> 16) & 0xff);
	SX1231_WriteReg(SX1231, _SX1231_RegFrfMid, (freq >> 8) & 0xff);
	SX1231_WriteReg(SX1231, _SX1231_RegFrfLsb, freq & 0xff);
}


uint32_t SX1231_GetFreqDev(SX1231_Typedef* SX1231)
{
	return FSTEP * (SX1231_ReadReg(SX1231, _SX1231_RegFdevMsb) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegFdevLsb));
}


void SX1231_SetFreqDev(SX1231_Typedef* SX1231, uint32_t FreqDev)
{
	FreqDev = FreqDev / FSTEP;
	SX1231_WriteReg(SX1231, _SX1231_RegFdevMsb, FreqDev >> 8);
	SX1231_WriteReg(SX1231, _SX1231_RegFdevLsb, FreqDev & 0xff);
}


void SX1231_SetBitRate(SX1231_Typedef* SX1231, uint32_t bitRate)
{
	bitRate = FXOSC / bitRate;
	SX1231_WriteReg(SX1231, _SX1231_RegBitrateMsb, bitRate >> 8);
	SX1231_WriteReg(SX1231, _SX1231_RegBitrateLsb, bitRate & 0xff);
}


uint32_t SX1231_GetBitRate(SX1231_Typedef* SX1231)
{
	return FXOSC / (SX1231_ReadReg(SX1231, _SX1231_RegBitrateMsb) << 8 | SX1231_ReadReg(SX1231, _SX1231_RegBitrateLsb));
}


void SX1231_SetModulationShaping(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegDataModul, (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & ~_SX1231_MODULATION_SHAPING_MASK) | (val << _SX1231_MODULATION_SHAPING_BIT));
}


uint8_t SX1231_GetModulationShaping(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & _SX1231_MODULATION_SHAPING_MASK) >> _SX1231_MODULATION_SHAPING_BIT;
}


void SX1231_SetModulationType(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegDataModul, (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & ~_SX1231_MODULATION_TYPE_MASK) | (val << _SX1231_MODULATION_TYPE_BIT));
}


uint8_t SX1231_GetModulationType(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & _SX1231_MODULATION_TYPE_MASK) >> _SX1231_MODULATION_TYPE_BIT;
}


void SX1231_SetDataMode(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegDataModul, (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & ~_SX1231_DATA_MODE_MASK) | (val << _SX1231_DATA_MODE_BIT));
}


uint8_t SX1231_GetDataMode(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegDataModul) & _SX1231_DATA_MODE_MASK) >> _SX1231_DATA_MODE_BIT;
}


void SX1231_SetListenAbort(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOpMode, (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & ~_SX1231_LISTEN_ABORT_MASK) | (val << _SX1231_LISTEN_ABORT_BIT));
}


void SX1231_SetSequencerOff(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOpMode, (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & ~_SX1231_SEQUENCER_OFF_MASK) | (val << _SX1231_SEQUENCER_OFF_BIT));
}


uint8_t SX1231_GetSequencerOff(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & _SX1231_SEQUENCER_OFF_MASK) >> _SX1231_SEQUENCER_OFF_BIT;
}


void SX1231_SetListenOn(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOpMode, (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & ~_SX1231_LISTEN_ON_MASK) | (val << _SX1231_LISTEN_ON_BIT));
}


uint8_t SX1231_GetListenOn(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & _SX1231_LISTEN_ON_MASK) >> _SX1231_LISTEN_ON_BIT;
}


void SX1231_SetMode(SX1231_Typedef* SX1231, uint8_t val)
{
	SX1231_WriteReg(SX1231, _SX1231_RegOpMode, (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & ~_SX1231_MODE_MASK) | (val << _SX1231_MODE_BIT));
}


uint8_t SX1231_GetMode(SX1231_Typedef* SX1231)
{
	return (SX1231_ReadReg(SX1231, _SX1231_RegOpMode) & _SX1231_MODE_MASK) >> _SX1231_MODE_BIT;
}


void SX1231_WriteReg(SX1231_Typedef* SX1231, uint8_t addr, uint8_t val)
{
	SX1231_SPI_Enable(SX1231);
	SX1231_SPI_Tx(SX1231, addr | _SX1231_WRITE_ADDR_MASK);
	SX1231_SPI_Tx(SX1231, val);
	SX1231_SPI_Disable(SX1231);
}


uint8_t SX1231_ReadReg(SX1231_Typedef* SX1231, uint8_t addr)
{
	uint8_t dat;
	SX1231_SPI_Enable(SX1231);
	SX1231_SPI_Tx(SX1231, addr & _SX1231_READ_ADDR_MASK);
	dat = SX1231_SPI_Rx(SX1231);
	SX1231_SPI_Disable(SX1231);
	return dat;
}


__STATIC_INLINE void SX1231_SPI_Tx(SX1231_Typedef* SX1231, uint8_t dat)
{
	SPI_ReadWriteByte(SX1231->SPI_Port, dat);
}


__STATIC_INLINE uint8_t SX1231_SPI_Rx(SX1231_Typedef* SX1231)
{
	return SPI_ReadWriteByte(SX1231->SPI_Port, 0xff);
}


__STATIC_INLINE void SX1231_SPI_Enable(SX1231_Typedef* SX1231)
{
	LL_GPIO_ResetOutputPin(SX1231->NSS_GPIO_Port, SX1231->NSS_GPIO_Pin);
}


__STATIC_INLINE void SX1231_SPI_Disable(SX1231_Typedef* SX1231)
{
	LL_GPIO_SetOutputPin(SX1231->NSS_GPIO_Port, SX1231->NSS_GPIO_Pin);
}

