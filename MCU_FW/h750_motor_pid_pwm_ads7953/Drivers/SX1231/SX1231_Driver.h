#ifndef __SX1231_DRIVER_H__
#define __SX1231_DRIVER_H__

#include "main.h"
#include "spi.h"

#define FXOSC 32000000UL

#define FSTEP (FXOSC / 524288UL)

typedef struct
{
	SPI_TypeDef* SPI_Port;
	GPIO_TypeDef* NSS_GPIO_Port;
	uint32_t NSS_GPIO_Pin;
	uint8_t DMA_Enable;

	struct
	{
		uint8_t SequencerOff;
		uint8_t ListenOn;
		uint8_t Mode;
		uint8_t DataMode;
		uint8_t ModulationType;
		uint8_t ModulationShaping;
		uint32_t BitRate;
		uint32_t Fdev;
		uint32_t Frf;
		uint8_t LowBatOn;
		uint8_t LowBatTrim;
		uint8_t ListenResol;
		uint8_t ListenCriteria;
		uint8_t ListenEnd;
		uint8_t ListenCoefIdle;
		uint8_t ListenCoefRx;
	} Common;

	struct
	{
		uint8_t PaOn;
		uint8_t OutputPower;
		uint8_t PaRamp;
		uint8_t OcpOn;
		uint8_t OcpTrim;
	} Transmitter;

	struct
	{
		uint8_t AgcAutoReferenceOn;
		uint8_t AgcReferenceLevel;
		uint8_t AgcSnrMargin;
		uint8_t AgcStep[5];
		uint8_t LnaZin;
		uint8_t LnaLowPowerOn;
		uint8_t LnaGainSelect;
		uint8_t DccFreq;
		uint8_t RxBwMant;
		uint8_t RxBwExp;
		uint8_t DccFreqAfc;
		uint8_t RxBwMantAfc;
		uint8_t RxBwExpAfc;
		uint8_t OokThreshType;
		uint8_t OokPeakTheshStep;
		uint8_t OokPeakThreshDec;
		uint8_t OokAverageThreshFilt;
		uint8_t OokFixedThresh;
		uint8_t AfcAutoclearOn;
		uint8_t AfcAutoOn;
		uint16_t AfcValue;
		uint16_t FeiValue;
		uint8_t FastRx;
	} Receiver;

	struct
	{
		uint8_t DioMapping[6];
		uint8_t ClkOut;
		uint8_t RssiThreshold;
		uint8_t TimeoutRxStart;
		uint8_t TimeoutRssiThresh;
	} IRQ_PinMapping;

	struct
	{
		uint16_t PreambleSize;
		uint8_t SyncOn;
		uint8_t FifoFillCondition;
		uint8_t SyncSize;
		uint8_t SyncTol;
		uint8_t SyncValue[8];
		uint8_t PacketFormat;
		uint8_t DcFree;
		uint8_t CrcOn;
		uint8_t CrcAutoClearOff;
		uint8_t AddressFiltering;
		uint8_t PayloadLength;
		uint8_t NodeAddress;
		uint8_t BroadcastAddress;
		uint8_t IntermediateEnterCondition;
		uint8_t IntermediateExitCondition;
		uint8_t IntermediateMode;
		uint8_t TxStartCondition;
		uint8_t FifoThreshold;
		uint8_t InterPacketRxDelay;
		uint8_t RestartRx;
		uint8_t AutoRxRestartOn;
		uint8_t AesOn;
		uint8_t AesKey[16];
	} PacketEngine;

	struct
	{
		uint8_t AdcLowPowerOn;
	} TempSensor;

	struct
	{
		uint8_t SensitivityBoost;
		uint8_t OokDeltaThreshold;
	} Test;

} SX1231_Typedef;



#define _SX1231_RegFifo						0x00
#define _SX1231_RegOpMode					0x01
#define _SX1231_RegDataModul			0x02
#define _SX1231_RegBitrateMsb			0x03
#define _SX1231_RegBitrateLsb			0x04
#define _SX1231_RegFdevMsb				0x05
#define _SX1231_RegFdevLsb				0x06
#define _SX1231_RegFrfMsb					0x07
#define _SX1231_RegFrfMid					0x08
#define _SX1231_RegFrfLsb					0x09
#define _SX1231_RegOsc1						0x0A
#define _SX1231_RegOsc2						0x0B
#define _SX1231_RegLowBat					0x0C
#define _SX1231_RegListen1				0x0D
#define _SX1231_RegListen2				0x0E
#define _SX1231_RegListen3				0x0F
#define _SX1231_RegVersion				0x10
#define _SX1231_RegPaLevel				0x11
#define _SX1231_RegPaRamp					0x12
#define _SX1231_RegOcp						0x13
#define _SX1231_RegAgcRef					0x14
#define _SX1231_RegAgcThresh1			0x15
#define _SX1231_RegAgcThresh2			0x16
#define _SX1231_RegAgcThresh3			0x17
#define _SX1231_RegLna						0x18
#define _SX1231_RegRxBw						0x19
#define _SX1231_RegAfcBw					0x1A
#define _SX1231_RegOokPeak				0x1B
#define _SX1231_RegOokAvg					0x1C
#define _SX1231_RegOokFix					0x1D
#define _SX1231_RegAfcFei					0x1E
#define _SX1231_RegAfcMsb					0x1F
#define _SX1231_RegAfcLsb					0x20
#define _SX1231_RegFeiMsb					0x21
#define _SX1231_RegFeiLsb					0x22
#define _SX1231_RegRssiConfig			0x23
#define _SX1231_RegRssiValue			0x24
#define _SX1231_RegDioMapping1		0x25
#define _SX1231_RegDioMapping2		0x26
#define _SX1231_RegIrqFlags1			0x27
#define _SX1231_RegIrqFlags2			0x28
#define _SX1231_RegRssiThresh			0x29
#define _SX1231_RegRxTimeout1			0x2A
#define _SX1231_RegRxTimeout2			0x2B
#define _SX1231_RegPreambleMsb		0x2C
#define _SX1231_RegPreambleLsb		0x2D
#define _SX1231_RegSyncConfig			0x2E
#define _SX1231_RegSyncValue1			0x2F
#define _SX1231_RegSyncValue2			0x30
#define _SX1231_RegSyncValue3			0x31
#define _SX1231_RegSyncValue4			0x32
#define _SX1231_RegSyncValue5			0x33
#define _SX1231_RegSyncValue6			0x34
#define _SX1231_RegSyncValue7			0x35
#define _SX1231_RegSyncValue8			0x36
#define _SX1231_RegPacketConfig1	0x37
#define _SX1231_RegPayloadLength	0x38
#define _SX1231_RegNodeAdrs				0x39
#define _SX1231_RegBroadcastAdrs	0x3A
#define _SX1231_RegAutoModes			0x3B
#define _SX1231_RegFifoThresh			0x3C
#define _SX1231_RegPacketConfig2	0x3D
#define _SX1231_RegAesKey1				0x3E
#define _SX1231_RegAesKey2				0x3F
#define _SX1231_RegAesKey3				0x40
#define _SX1231_RegAesKey4				0x41
#define _SX1231_RegAesKey5				0x42
#define _SX1231_RegAesKey6				0x43
#define _SX1231_RegAesKey7				0x44
#define _SX1231_RegAesKey8				0x45
#define _SX1231_RegAesKey9				0x46
#define _SX1231_RegAesKey10				0x47
#define _SX1231_RegAesKey11				0x48
#define _SX1231_RegAesKey12				0x49
#define _SX1231_RegAesKey13				0x4A
#define _SX1231_RegAesKey14				0x4B
#define _SX1231_RegAesKey15				0x4C
#define _SX1231_RegAesKey16				0x4D
#define _SX1231_RegTemp1					0x4E
#define _SX1231_RegTemp2					0x4F
#define _SX1231_RegTestLna				0x58
#define _SX1231_RegTestOok				0x6E

#define _SX1231_READ_ADDR_MASK 0x7F
#define _SX1231_WRITE_ADDR_MASK 0x80

#define _SX1231_MODE_MASK  0x1c
#define _SX1231_MODE_BIT  2

#define _SX1231_LISTEN_ABORT_MASK  0x20
#define _SX1231_LISTEN_ABORT_BIT  5

#define _SX1231_LISTEN_ON_MASK  0x40
#define _SX1231_LISTEN_ON_BIT  6

#define _SX1231_SEQUENCER_OFF_MASK  0x80
#define _SX1231_SEQUENCER_OFF_BIT  7

#define _SX1231_MODULATION_SHAPING_MASK 0x03
#define _SX1231_MODULATION_SHAPING_BIT 0

#define _SX1231_MODULATION_TYPE_MASK 0x18
#define _SX1231_MODULATION_TYPE_BIT 3

#define _SX1231_DATA_MODE_MASK 0x60
#define _SX1231_DATA_MODE_BIT 5

#define _SX1231_LOW_BAT_ON_MASK 0x08
#define _SX1231_LOW_BAT_ON_BIT 3

#define _SX1231_RC_CAL_DONE_MASK 0x40
#define _SX1231_RC_CAL_DONE_BIT 6

#define _SX1231_LOW_BAT_MONITOR_MASK 0x10
#define _SX1231_LOW_BAT_MONITOR_BIT 4

#define _SX1231_LOW_BAT_TRIM_MASK 0x10
#define _SX1231_LOW_BAT_TRIM_BIT 4

#define _SX1231_LISTEN_RESOL_MASK 0xf0
#define _SX1231_LISTEN_RESOL_BIT 4

#define _SX1231_LISTEN_CRITERIA_MASK 0x08
#define _SX1231_LISTEN_CRITERIA_BIT 3

#define _SX1231_LISTEN_END_MASK 0x06
#define _SX1231_LISTEN_END_BIT 1

#define _SX1231_OUTPUT_POWER_BIT 0
#define _SX1231_OUTPUT_POWER_MASK 0x1f

#define _SX1231_PA_BIT 5
#define _SX1231_PA_MASK 0xe0

#define _SX1231_PA_RAMP_BIT 0
#define _SX1231_PA_RAMP_MASK 0x0f

#define _SX1231_OCP_ON_BIT 4
#define _SX1231_OCP_ON_MASK 0x10

#define _SX1231_OCP_TRIM_BIT 0
#define _SX1231_OCP_TRIM_MASK 0x0f

#define _SX1231_AGC_AUTO_REFERENCE_ON_BIT 6
#define _SX1231_AGC_AUTO_REFERENCE_ON_MASK 0x40

#define _SX1231_AGC_REFERENCE_LEVEL_BIT 0
#define _SX1231_AGC_REFERENCE_LEVEL_MASK 0x3f

#define _SX1231_AGC_SNR_MARGIN_MASK 0xe0
#define _SX1231_AGC_SNR_MARGIN_BIT 5

#define _SX1231_AGC_STEP_1_MASK 0x1f
#define _SX1231_AGC_STEP_1_BIT 0

#define _SX1231_AGC_STEP_2_MASK 0xf0
#define _SX1231_AGC_STEP_2_BIT 4

#define _SX1231_AGC_STEP_3_MASK 0x0f
#define _SX1231_AGC_STEP_3_BIT 0

#define _SX1231_AGC_STEP_4_MASK 0xf0
#define _SX1231_AGC_STEP_4_BIT 4

#define _SX1231_AGC_STEP_5_MASK 0x0f
#define _SX1231_AGC_STEP_5_BIT 0

#define _SX1231_LNA_ZIN_MASK 0x80
#define _SX1231_LNA_ZIN_BIT 7

#define _SX1231_LNA_LOW_POWER_ON_MASK 0x80
#define _SX1231_LNA_LOW_POWER_ON_BIT 7

#define _SX1231_LNA_GAIN_SELECT_MASK 0x07
#define _SX1231_LNA_GAIN_SELECT_BIT 0

#define _SX1231_LNA_CURRENT_GAIN_MASK 0x38
#define _SX1231_LNA_CURRENT_GAIN_BIT 3

#define _SX1231_DCC_FREQ_MASK 0xe0
#define _SX1231_DCC_FREQ_BIT 5

#define _SX1231_RX_BW_MANT_MASK 0x18
#define _SX1231_RX_BW_MANT_BIT 3

#define _SX1231_RX_BW_EXP_MASK 0x07
#define _SX1231_RX_BW_EXP_BIT 0

#define _SX1231_DCC_FREQ_AFC_MASK 0xe0
#define _SX1231_DCC_FREQ_AFC_BIT 5

#define _SX1231_RX_BW_MANT_AFC_MASK 0x18
#define _SX1231_RX_BW_MANT_AFC_BIT 3

#define _SX1231_RX_BW_EXP_AFC_MASK 0x07
#define _SX1231_RX_BW_EXP_AFC_BIT 0

#define _SX1231_OOK_PEAK_THRESH_DEC_MASK 0x03
#define _SX1231_OOK_PEAK_THRESH_DEC_BIT 0

#define _SX1231_OOK_PEAK_THESH_STEP_MASK 0x38
#define _SX1231_OOK_PEAK_THESH_STEP_BIT 3

#define _SX1231_OOK_THRESH_TYPE_MASK 0xC0
#define _SX1231_OOK_THRESH_TYPE_BIT 6

#define _SX1231_OOK_AVERAGE_THRESH_FILT_MASK 0xc0
#define _SX1231_OOK_AVERAGE_THRESH_FILT_BIT 6

#define _SX1231_FEI_START_MASK 0X20
#define _SX1231_FEI_START_BIT 5

#define _SX1231_FEI_DONE_MASK 0X40
#define _SX1231_FEI_DONE_BIT 6

#define _SX1231_AFC_START_MASK 0X01
#define _SX1231_AFC_START_BIT 0

#define _SX1231_AFC_CLEAR_MASK 0X02
#define _SX1231_AFC_CLEAR_BIT 1

#define _SX1231_AFC_DONE_MASK 0X10
#define _SX1231_AFC_DONE_BIT 4

#define _SX1231_AFC_CLEAR_MASK 0X02
#define _SX1231_AFC_CLEAR_BIT 1

#define _SX1231_AFC_AUTO_CLEAR_ON_MASK 0X08
#define _SX1231_AFC_AUTO_CLEAR_ON_BIT 3

#define _SX1231_AFC_AUTO_ON_MASK 0X04
#define _SX1231_AFC_AUTO_ON_BIT 2

#define _SX1231_FAST_RX_MASK 0X08
#define _SX1231_FAST_RX_BIT 3

#define _SX1231_RSSI_DONE_MASK 0X02
#define _SX1231_RSSI_DONE_BIT 1

#define _SX1231_RSSI_START_MASK 0X01
#define _SX1231_RSSI_START_BIT 0

#define _SX1231_CLK_OUT_MASK 0x07
#define _SX1231_CLK_OUT_BIT 0

#define _SX1231_FLAG_MODE_READY_MASK 0x80
#define _SX1231_FLAG_MODE_READY_BIT 7

#define _SX1231_FLAG_RX_READY_MASK 0x40
#define _SX1231_FLAG_RX_READY_BIT 6

#define _SX1231_FLAG_TX_READY_MASK 0x20
#define _SX1231_FLAG_TX_READY_BIT 5

#define _SX1231_FLAG_PLL_LOCK_MASK 0x10
#define _SX1231_FLAG_PLL_LOCK_BIT 4

#define _SX1231_FLAG_RSSI_MASK 0x08
#define _SX1231_FLAG_RSSI_BIT 3

#define _SX1231_FLAG_TIMEOUT_MASK 0x04
#define _SX1231_FLAG_TIMEOUT_BIT 2

#define _SX1231_FLAG_AUTO_MODE_MASK 0x02
#define _SX1231_FLAG_AUTO_MODE_BIT 1

#define _SX1231_FLAG_SYNC_ADDRESS_MATCH_MASK 0x01
#define _SX1231_FLAG_SYNC_ADDRESS_MATCH_BIT 0

#define _SX1231_FIFO_FULL_MASK 0x80
#define _SX1231_FIFO_FULL_BIT 7

#define _SX1231_FIFO_NOT_EMPTY_MASK 0x40
#define _SX1231_FIFO_NOT_EMPTY_BIT 6

#define _SX1231_FIFO_LEVEL_MASK 0x20
#define _SX1231_FIFO_LEVEL_BIT 5

#define _SX1231_FIFO_OVERRUN_MASK 0x10
#define _SX1231_FIFO_OVERRUN_BIT 4

#define _SX1231_PACKET_SENT_MASK 0x08
#define _SX1231_PACKET_SENT_BIT 3

#define _SX1231_PAY_LOAD_READY_MASK 0x04
#define _SX1231_PAY_LOAD_READY_BIT 2

#define _SX1231_CRC_OK_MASK 0x02
#define _SX1231_CRC_OK_BIT 1

#define _SX1231_LOW_BAT_MASK 0x01
#define _SX1231_LOW_BAT_BIT 0

#define _SX1231_SYNC_TOL_MASK 0x07
#define _SX1231_SYNC_TOL_BIT 0

#define _SX1231_SYNC_SIZE_MASK 0x38
#define _SX1231_SYNC_SIZE_BIT 3

#define _SX1231_FIFO_FILL_CONDITION_MASK 0x40
#define _SX1231_FIFO_FILL_CONDITION_BIT 6

#define _SX1231_SYNC_ON_MASK 0x80
#define _SX1231_SYNC_ON_BIT 7

#define _SX1231_ADDRESS_FILTERING_MASK 0x06
#define _SX1231_ADDRESS_FILTERING_BIT 1

#define _SX1231_CRC_AUTO_CLEAR_OFF_MASK 0x80
#define _SX1231_CRC_AUTO_CLEAR_OFF_BIT 3

#define _SX1231_CRC_ON_MASK 0x10
#define _SX1231_CRC_ON_BIT 4

#define _SX1231_DC_FREE_MASK 0x60
#define _SX1231_DC_FREE_BIT 5

#define _SX1231_PACKET_FORMAT_MASK 0x80
#define _SX1231_PACKET_FORMAT_BIT 7

#define _SX1231_INTERMEDIATE_MODE_MASK 0x03
#define _SX1231_INTERMEDIATE_MODE_BIT 0

#define _SX1231_EXIT_CONDITION_MASK 0x1C
#define _SX1231_EXIT_CONDITION_BIT 2

#define _SX1231_ENTER_CONDITION_MASK 0xe0
#define _SX1231_ENTER_CONDITION_BIT 5

#define _SX1231_TX_START_CONDITION_MASK 0x80
#define _SX1231_TX_START_CONDITION_BIT 7

#define _SX1231_FIFO_THRESHOLD_MASK 0x7f
#define _SX1231_FIFO_THRESHOLD_BIT 0

#define _SX1231_AES_ON_MASK 0x01
#define _SX1231_AES_ON_BIT 0

#define _SX1231_AUTO_RX_RESTART_ON_MASK 0x02
#define _SX1231_AUTO_RX_RESTART_ON_BIT 1

#define _SX1231_RESTART_RX_MASK 0x04
#define _SX1231_RESTART_RX_BIT 2

#define _SX1231_INTER_PACKET_RX_DELAY_MASK 0xf0
#define _SX1231_INTER_PACKET_RX_DELAY_BIT 4

#define _SX1231_ADC_LOW_POWER_ON_MASK 0x01
#define _SX1231_ADC_LOW_POWER_ON_BIT 0

#define  _SX1231_TEMP_MEAS_RUNNING_MASK 0x04
#define  _SX1231_TEMP_MEAS_RUNNING_BIT 2

#define _SX1231_TEMP_MEAS_START_MASK 0x08
#define _SX1231_TEMP_MEAS_START_BIT 3



















#define SX1231_MODE_SLEEP 0x0
#define SX1231_MODE_STDBY 0x1
#define SX1231_MODE_FS 0x2
#define SX1231_MODE_TX 0x3
#define SX1231_MODE_RX 0x4

#define SX1231_DATA_MODE_PKT 0
#define SX1231_DATA_MODE_CONT_W_SYN 2
#define SX1231_DATA_MODE_CONT_WO_SYN 3

#define SX1231_MODULATION_TYPE_FSK 0
#define SX1231_MODULATION_TYPE_OOK 1

#define SX1231_MODULATION_SHAPING_NO 0
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_1_0 1
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_0_5 2
#define SX1231_MODULATION_SHAPING_GAUSS_FIL_BT_0_3 3
#define SX1231_MODULATION_SHAPING_FIL_BR 1
#define SX1231_MODULATION_SHAPING_FIL_2BR 2

#define SX1231_RC_CAL_IN_PROGRESS 0
#define SX1231_RC_CAL_DONE 1

#define SX1231_LOW_BAT_ON 1
#define SX1231_LOW_BAT_OFF 0

#define SX1231_LOW_BAT_TH_1V695 0x0
#define SX1231_LOW_BAT_TH_1V764 0x1
#define SX1231_LOW_BAT_TH_1V835 0x2
#define SX1231_LOW_BAT_TH_1V905 0x3
#define SX1231_LOW_BAT_TH_1V976 0x4
#define SX1231_LOW_BAT_TH_2V045 0x5
#define SX1231_LOW_BAT_TH_2V116 0x6
#define SX1231_LOW_BAT_TH_2V185 0x7

#define SX1231_LISTEN_RESOLUTION_64us 0x5
#define SX1231_LISTEN_RESOLUTION_4_1ms 0xa
#define SX1231_LISTEN_RESOLUTION_262ms 0xf

#define SX1231_LISTEN_CRITERIA_SIGNAL_STRENGTH_ABOVE_RSSI_THRESHOLD 0
#define SX1231_LISTEN_CRITERIA_SIGNAL_STRENGTH_ABOVE_RSSI_THRESHOLD_AND_SYNC_ADDRESS_MATCHED 1

#define SX1231_LISTEN_END_STAY_IN_RX_AND_LISTEN_STOPS_AND_MUST_BE_DISABLED 0
#define SX1231_LISTEN_END_STAY_IN_RX_UNTIL_INTERRUPT 1
#define SX1231_LISTEN_END_STAY_IN_RX_UNTIL_INTERRUPT_THEN_RESUME 2

#define SX1231_PA0_ON 0x04
#define SX1231_PA1_ON 0x02
#define SX1231_PA2_ON 0x01

#define SX1231_PA_RAMP_3_4_ms 0x0
#define SX1231_PA_RAMP_2_ms 0x1
#define SX1231_PA_RAMP_1_ms 0x2
#define SX1231_PA_RAMP_500_us 0x3
#define SX1231_PA_RAMP_250_us 0x4
#define SX1231_PA_RAMP_125_us 0x5
#define SX1231_PA_RAMP_100_us 0x6
#define SX1231_PA_RAMP_62_us 0x7
#define SX1231_PA_RAMP_50_us 0x8
#define SX1231_PA_RAMP_40_us 0x9
#define SX1231_PA_RAMP_31_us 0xa
#define SX1231_PA_RAMP_25_us 0xb
#define SX1231_PA_RAMP_20_us 0xc
#define SX1231_PA_RAMP_15_us 0xd
#define SX1231_PA_RAMP_12_us 0xe
#define SX1231_PA_RAMP_10_us 0xf

#define SX1231_OCP_ENABLE 1
#define SX1231_OCP_DISABLE 0

#define SX1231_AGC_REFERENCE_FORCED 0
#define SX1231_AGC_REFERENCE_AUTO 1

#define SX1231_LNA_ZIN_50_OHMS 0
#define SX1231_LNA_ZIN_200_OHMS 1

#define SX1231_LNA_LOW_POWER_NORMAL_MODE 0
#define SX1231_LNA_LOW_POWER_MODE_ENABLED 1

#define SX1231_LNA_GAIN_SET_BY_AGC 0x0
#define SX1231_LNA_GAIN_G1 0x1
#define SX1231_LNA_GAIN_G2 0x2
#define SX1231_LNA_GAIN_G3 0x3
#define SX1231_LNA_GAIN_G4 0x4
#define SX1231_LNA_GAIN_G5 0x5
#define SX1231_LNA_GAIN_G6 0x6
#define SX1231_LNA_GAIN_G7 0x7

#define SX1231_RX_BW_MANT_16 0x0
#define SX1231_RX_BW_MANT_20 0x1
#define SX1231_RX_BW_MANT_24 0x2

#define SX1231_OOK_THRESH_FIXED 0x0
#define SX1231_OOK_THRESH_PEAK 0x1
#define SX1231_OOK_THRESH_AVERAGE 0x2

#define SX1231_OOK_PEAK_THRESH_STEP_0_5_dB 0x0
#define SX1231_OOK_PEAK_THRESH_STEP_1_0_dB 0x1
#define SX1231_OOK_PEAK_THRESH_STEP_1_5_dB 0x2
#define SX1231_OOK_PEAK_THRESH_STEP_2_0_dB 0x3
#define SX1231_OOK_PEAK_THRESH_STEP_3_0_dB 0x4
#define SX1231_OOK_PEAK_THRESH_STEP_4_0_dB 0x5
#define SX1231_OOK_PEAK_THRESH_STEP_5_0_dB 0x6
#define SX1231_OOK_PEAK_THRESH_STEP_6_0_dB 0x7

#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_PER_CHIP 0x0
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_2_CHIPS 0x1
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_4_CHIPS 0x2
#define SX1231_OOK_PEAK_THRESH_DEC_ONCE_EVERY_8_CHIPS 0x3
#define SX1231_OOK_PEAK_THRESH_DEC_TWICE_IN_EACH_CHIP 0x4
#define SX1231_OOK_PEAK_THRESH_DEC_4_TIMES_IN_EACH_CHIP 0x5
#define SX1231_OOK_PEAK_THRESH_DEC_8_TIMES_IN_EACH_CHIP 0x6
#define SX1231_OOK_PEAK_THRESH_DEC_16_TIMES_IN_EACH_CHIP 0x7

#define SX1231_FEI_ONGOING 0
#define SX1231_FEI_FINISHED 1

#define SX1231_AFC_ONGOING 0
#define SX1231_AFC_FINISHED 1

#define SX1231_AFC_AUTOCLEAR_DISABLED 0
#define SX1231_AFC_AUTOCLEAR_ENABLED 1

#define SX1231_AFC_AUTO_DISABLED 0
#define SX1231_AFC_AUTO_ENABLED 1

#define SX1231_FAST_RX_2_RSSI_SAMPLE 0
#define SX1231_FAST_RX_1_RSSI_SAMPLE 1

#define SX1231_RSSI_ONGOING 0
#define SX1231_RSSI_FINISHED 1

#define SX1231_FIFO_FILL_CONDITION_SYNC_ADDRESS_INTERRUPT 0
#define SX1231_FIFO_FILL_CONDITION_FIFO_FILL_CONDITION_IS_SET 1

#define SX1231_PACKET_FORMAT_FIXED_LENGTH 0
#define SX1231_PACKET_FORMAT_VARIABLE_LENGTH 1

#define SX1231_DC_FREE_OFF 0
#define SX1231_DC_FREE_MANCHESTER 1
#define SX1231_DC_FREE_WHITENING 2

#define SX1231_CRC_OFF 0
#define SX1231_CRC_ON 1

#define SX1231_CRC_AUTO_CLEAR_OFF 0
#define SX1231_CRC_AUTO_CLEAR_ON 1

#define SX1231_ADDRESS_FILTERING_NONE 0
#define SX1231_ADDRESS_FILTERING_MATCH_NODE_ADDRESS 1
#define SX1231_ADDRESS_FILTERING_MATCH_NODE_OR_BROADCAST_ADDRESS 1

#define SX1231_INTERMEDIATE_ENTER_CONDITION_NONE 0x0
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_NOT_EMPTY_SET 0x1
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_LEVEL_SET 0x2
#define SX1231_INTERMEDIATE_ENTER_CONDITION_CRC_OK_SET 0x3
#define SX1231_INTERMEDIATE_ENTER_CONDITION_PAY_LOAD_READY_SET 0x4
#define SX1231_INTERMEDIATE_ENTER_CONDITION_SYNC_ADDRESS_SET 0x5
#define SX1231_INTERMEDIATE_ENTER_CONDITION_PACKET_SENT_SET 0x6
#define SX1231_INTERMEDIATE_ENTER_CONDITION_FIFO_NOT_EMPTY_RESET 0x7

#define SX1231_INTERMEDIATE_EXIT_CONDITION_NONE 0x0
#define SX1231_INTERMEDIATE_EXIT_CONDITION_FIFO_NOT_EMPTY_RESET 0x1
#define SX1231_INTERMEDIATE_EXIT_CONDITION_FIFO_LEVEL_OR_TIMEOUT_SET 0x2
#define SX1231_INTERMEDIATE_EXIT_CONDITION_CRC_OK_OR_TIMEOUT_SET 0x3
#define SX1231_INTERMEDIATE_EXIT_CONDITION_PAY_LOAD_READY_OR_TIMEOUT_SET 0x4
#define SX1231_INTERMEDIATE_EXIT_CONDITION_SYNC_ADDRESS_OR_TIMEOUT_SET 0x5
#define SX1231_INTERMEDIATE_EXIT_CONDITION_PACKET_SENT_SET 0x6
#define SX1231_INTERMEDIATE_EXIT_CONDITION_TIMEOUT_SET 0x7

#define SX1231_INTERMEDIATE_MODE_SLEEP 0x0
#define SX1231_INTERMEDIATE_MODE_STAND_BY 0x1
#define SX1231_INTERMEDIATE_MODE_RECEIVER 0x2
#define SX1231_INTERMEDIATE_MODE_TRANSMITTER 0x3

#define SX1231_TX_START_CONDITION_FIFO_LEVEL 0x0
#define SX1231_TX_START_CONDITION_FIFI_NOT_EMPTY 0x1

#define SX1231_AUTO_RX_RESTART_ENABLED 1
#define SX1231_AUTO_RX_RESTART_DISABLED 0

#define SX1231_AES_ENABLED 1
#define SX1231_AES_DISABLED 0

#define SX1231_TEMP_ADC_LOW_POWER_ENABLED 1
#define SX1231_TEMP_ADC_LOW_POWER_DISABLED 0

#define SX1231_SENSITIVITY_BOOST_NORMAL 0x1B
#define SX1231_SENSITIVITY_BOOST_HIGH_SENSITIVITY 0x2D

#define SX1231_SYNC_ENABLED 1
#define SX1231_SYNC_DISABLED 0

#define SX1231_SEQUENCER_OFF_ENABLED 1
#define SX1231_SEQUENCER_OFF_DISABLED 0

#define SX1231_LISTEN_ENABLED 1;
#define SX1231_LISTEN_DISABLED 0;










void SX1231_WriteReg(SX1231_Typedef* SX1231, uint8_t addr, uint8_t val);
uint8_t SX1231_ReadReg(SX1231_Typedef* SX1231, uint8_t addr);

uint8_t SX1231_GetOokDeltaThreshold(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSensitivityBoost(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempValue(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempAdcLowPowerOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTempMeasRunning(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAesOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAutoRxRestartOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetInterPacketRxDelay(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFifoThreshold(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTxStartCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateMode(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateExitCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetIntermediateEnterCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetBroadcastAddress(SX1231_Typedef* SX1231);
uint8_t SX1231_GetNodeAddress(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPayloadLength(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPacketFormat(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDcFree(SX1231_Typedef* SX1231);
uint8_t SX1231_GetCrcOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetCrcAutoClearOff(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAddressFiltering(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncTol(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncSize(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFifoFillCondition(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSyncOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTimeoutRssiThresh(SX1231_Typedef* SX1231);
uint8_t SX1231_GetTimeoutRxStart(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRssiThreshold(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoFull(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoNotEmpty(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoLevel(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_FifoOverrun(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PacketSent(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PayloadReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_CrcOk(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_LowBat(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_ModeReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_RxReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_TxReady(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_PllLock(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_Rssi(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_Timeout(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_AutoMode(SX1231_Typedef* SX1231);
uint8_t SX1231_IsActiveFlag_SyncAddressMatch(SX1231_Typedef* SX1231);
uint8_t SX1231_GetClkOut(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID);
uint8_t SX1231_GetRssiValue(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRssiDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFastRx(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcAutoOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcAutoclearOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcClear(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAfcDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetFeiDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokFixedThresh(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokAverageThreshFilt(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokPeakThreshDec(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokPeakTheshStep(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOokThreshType(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwExpAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwMantAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDccFreqAfc(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwExp(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRxBwMant(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDccFreq(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaCurrentGain(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaGainSelect(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaLowPowerOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLnaZin(SX1231_Typedef* SX1231);
void SX1231_GetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5]);
uint8_t SX1231_GetAgcSnrMargin(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAgcReferenceLevel(SX1231_Typedef* SX1231);
uint8_t SX1231_GetAgcAutoReferenceOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOcpTrim(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOcpOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPaRamp(SX1231_Typedef* SX1231);
uint8_t SX1231_GetPaOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetOutputPower(SX1231_Typedef* SX1231);
uint8_t SX1231_GetVersion(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCoefRx(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCoefIdle(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenEnd(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenCriteria(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenResol(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatTrim(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetLowBatMonitor(SX1231_Typedef* SX1231);
uint8_t SX1231_GetRcCalDone(SX1231_Typedef* SX1231);
uint8_t SX1231_GetModulationShaping(SX1231_Typedef* SX1231);
uint8_t SX1231_GetModulationType(SX1231_Typedef* SX1231);
uint8_t SX1231_GetDataMode(SX1231_Typedef* SX1231);
uint8_t SX1231_GetSequencerOff(SX1231_Typedef* SX1231);
uint8_t SX1231_GetListenOn(SX1231_Typedef* SX1231);
uint8_t SX1231_GetMode(SX1231_Typedef* SX1231);
uint16_t SX1231_GetPreambleSize(SX1231_Typedef* SX1231);
uint16_t SX1231_GetFeiValue(SX1231_Typedef* SX1231);
uint16_t SX1231_GetAfcValue(SX1231_Typedef* SX1231);
uint32_t SX1231_GetFreqDev(SX1231_Typedef* SX1231);
uint32_t SX1231_GetBitRate(SX1231_Typedef* SX1231);
void SX1231_SetSensitivityBoost(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTempAdcLowPowerOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTempMeasStart(SX1231_Typedef* SX1231);
void SX1231_SetAesKey(SX1231_Typedef* SX1231, uint8_t AesKey[16]);
void SX1231_SetAesOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAutoRxRestartOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRestartRx(SX1231_Typedef* SX1231);
void SX1231_SetInterPacketRxDelay(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFifoThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTxStartCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateMode(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateExitCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetIntermediateEnterCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetBroadcastAddress(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetNodeAddress(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPayloadLength(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPacketFormat(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDcFree(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetCrcOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetCrcAutoClearOff(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAddressFiltering(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_GetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8]);
void SX1231_SetSyncValue(SX1231_Typedef* SX1231, uint8_t SyncValue[8]);
void SX1231_SetSyncTol(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSyncSize(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFifoFillCondition(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSyncOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPreambleSize(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetTimeoutRssiThresh(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetTimeoutRxStart(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRssiThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetClkOut(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDioMapping(SX1231_Typedef* SX1231, uint8_t DioID, uint8_t Mapping);
void SX1231_SetRssiStart(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFastRx(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetFeiValue(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetAfcValue(SX1231_Typedef* SX1231, uint16_t val);
void SX1231_SetAfcAutoOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAfcAutoclearOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAfcStart(SX1231_Typedef* SX1231l);
void SX1231_SetFeiStart(SX1231_Typedef* SX1231);
void SX1231_SetOokFixedThresh(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokAverageThreshFilt(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokPeakThreshDec(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokPeakTheshStep(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokThreshType(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwExpAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwMantAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDccFreqAfc(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwExp(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRxBwMant(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDccFreq(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaGainSelect(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaLowPowerOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLnaZin(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOokDeltaThreshold(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcStep(SX1231_Typedef* SX1231, uint8_t AgcStep[5]);
void SX1231_SetAgcSnrMargin(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcReferenceLevel(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetAgcAutoReferenceOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOcpTrim(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetOcpOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetPaRamp(SX1231_Typedef* SX1231, uint8_t mask);
void SX1231_SetPaOn(SX1231_Typedef* SX1231, uint8_t mask);
void SX1231_SetOutputPower(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCoefRx(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCoefIdle(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenEnd(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenCriteria(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenResol(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLowBatTrim(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetLowBatOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetRcCalStart(SX1231_Typedef* SX1231);
void SX1231_SetCarrierFreq(SX1231_Typedef* SX1231, uint32_t freq);
void SX1231_SetFreqDev(SX1231_Typedef* SX1231, uint32_t FreqDev);
void SX1231_SetBitRate(SX1231_Typedef* SX1231, uint32_t bitRate);
void SX1231_SetModulationShaping(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetModulationType(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetDataMode(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenAbort(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetSequencerOff(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetListenOn(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_SetMode(SX1231_Typedef* SX1231, uint8_t val);
void SX1231_ClearFifo(SX1231_Typedef* SX1231);
uint32_t SX1231_GetCarrierFreq(SX1231_Typedef* SX1231);

void SX1231_WriteFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length);
void SX1231_ReadFifo(SX1231_Typedef* SX1231, uint8_t bfr[], uint32_t length);

void SX1231_Init(SX1231_Typedef* SX1231);
void SX1231_Typedef_Init(SX1231_Typedef* SX1231);

#endif