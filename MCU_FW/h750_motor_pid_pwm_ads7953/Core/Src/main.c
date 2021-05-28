/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1231_APP.h"
#include "ADS7953.h"
#include "motor.h"
#include "delay.h"
#include "stdlib.h"
#include "stdio.h"
#include "seek_line.h"
#include "servo.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RMT_XY_SCALE 4100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t bfr[SX1231_PayloadLength];
	uint8_t id, ack, dat[6], i, rx_Valid;
	uint8_t seek_line_enable = 0;
	uint32_t invalid_cnt = 0;
	int32_t rmt_LY[2], rmt_LX[2], rmt_RY[2], rmt_RX[2];
	int32_t rmt_LY_o, rmt_LX_o, rmt_RY_o, rmt_RX_o;
	uint16_t rmt_keys;
	int32_t speed_X, speed_Y, speed_R;
	int32_t speed_FL, speed_FR, speed_RL, speed_RR;
	
	float kPm = 9.3, kIm = 2.7, kDm = 10.4;
	float kPx, kIx, kDx, kPy, kIy, kDy, kPr, kIr, kDr;
	
	float servo_angle[4] = {90, 90, 90, 90};
	
	uint16_t ADS7953_F_bfr[16];
	uint16_t ADS7953_R_bfr[16];
	ADS7953_Typedef ADS7953_F, ADS7953_R;
	
	ADS7953_F.ADC_Range = ADS7953_Range_VREF;
	ADS7953_F.SPI_Port = SPI6;
	ADS7953_F.NSS_GPIO_Pin = SPI6_NE1_Pin;
	ADS7953_F.NSS_GPIO_Port = SPI6_NE1_GPIO_Port;
	
	ADS7953_R.ADC_Range = ADS7953_Range_VREF;
	ADS7953_R.SPI_Port = SPI6;
	ADS7953_R.NSS_GPIO_Pin = SPI6_NE2_Pin;
	ADS7953_R.NSS_GPIO_Port = SPI6_NE2_GPIO_Port;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  MX_SPI6_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	LL_SPI_Enable(SPI3);
	LL_SPI_Enable(SPI6);
	
	

	
	//motor pwm
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);
	
	
	//servo pwm
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableCounter(TIM4);
	
	
	//speed detect
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableIT_CC2(TIM2);
	LL_TIM_EnableIT_CC3(TIM2);
	LL_TIM_EnableIT_CC4(TIM2);
	LL_TIM_EnableCounter(TIM2);
	
	
	motor_pid_init(kPm, kIm, kDm);
	//motor pid timer
	LL_TIM_EnableIT_UPDATE(TIM6);
	LL_TIM_EnableCounter(TIM6);
	
	
  SX1231_Init_Secondary();
  while (!SX1231_IsActiveFlag_ModeReady(&SX1231_Secondary))
  {
    printf("wait");
  }
	
	printf("aaaa\r\n");
	
	
	rmt_LX[0] = 0x810;
	rmt_LY[0] = 0x820;
	rmt_RX[0] = 0x820;
	rmt_RY[0] = 0x820;
	
	
	servo_angle[0] = 90;
	servo_angle[1] = 90;
	servo_angle[2] = 90;
	servo_angle[3] = 90;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ADS7953_ReadChannels(&ADS7953_F, ADS7953_F_bfr, ADS7953_Channel_All);
		ADS7953_ReadChannels(&ADS7953_R, ADS7953_R_bfr, ADS7953_Channel_All);
/*
		for(i=0; i<16; i++)
			printf("F%03x ", ADS7953_F_bfr[i]);
		//printf("\r\n");
		
		for(i=0; i<16; i++)
			printf("R%03x ", ADS7953_R_bfr[i]);
		printf("\r\n");
*/
		LL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		rx_Valid = 0;
		if(SX1231_Secondary_ReceivePacket(bfr))
			if(bfr[0] == 0x23 && bfr[1] == 0x33)
			{
				/*
				for(i=0; i<SX1231_PayloadLength; i++)
					printf("%x ", bfr[i]);
				printf("\r\n");
				*/
				rx_Valid = 1;
				invalid_cnt = 0;
				LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				
				if(seek_line_enable)
				{
					seek_line(ADS7953_F_bfr, ADS7953_R_bfr, &speed_Y, &speed_X, &speed_R);
				}
				else
				{
					
					rmt_keys  = bfr[2] << 8 | bfr[3];
					
					if(rmt_keys & 1)
					{
						
					}					
						
					
					for(i=0; i<4; i++)
						servo_update(i,servo_angle[i]);
					
					rmt_LX[1] = rmt_LX[0];
					rmt_LY[1] = rmt_LY[0];
					rmt_RX[1] = rmt_RX[0];
					rmt_RY[1] = rmt_RY[0];
					
					rmt_LX[0] = bfr[8] << 8 | bfr[9];
					rmt_LY[0] = bfr[10] << 8 | bfr[11];
					rmt_RX[0] = bfr[4] << 8 | bfr[5];
					rmt_RY[0] = bfr[6] << 8 | bfr[7];
					
					rmt_LX_o = (rmt_LX[0] + rmt_LX[1]) / 2;
					rmt_LY_o = (rmt_LY[0] + rmt_LY[1]) / 2;
					rmt_RX_o = (rmt_RX[0] + rmt_RX[1]) / 2;
					rmt_RY_o = (rmt_RY[0] + rmt_RY[1]) / 2;
					
					rmt_LX_o -= 0x810;
					rmt_LY_o -= 0x820;
					rmt_RX_o -= 0x820;
					rmt_RY_o -= 0x820;
					
					rmt_LX_o = sign(rmt_LX_o) * rmt_LX_o * rmt_LX_o / 2000;
					rmt_LY_o = sign(rmt_LY_o) * rmt_LY_o * rmt_LY_o / 2000;
					rmt_RX_o = sign(rmt_RX_o) * rmt_RX_o * rmt_RX_o / 2000;
					rmt_RY_o = sign(rmt_RY_o) * rmt_RY_o * rmt_RY_o / 2000;
					
					if(labs(rmt_LX_o) < 30)
						rmt_LX_o = 0;
					else
						rmt_LX_o -= 30;
					if(labs(rmt_LY_o) < 30)
						rmt_LY_o = 0;
					else
						rmt_LY_o -= 30;
					if(labs(rmt_RX_o) < 30)
						rmt_RX_o = 0;
					else
						rmt_RX_o -= 30;
					if(labs(rmt_RY_o) < 30)
						rmt_RY_o = 0;
					else
						rmt_RY_o -= 30;
					
					speed_Y = (rmt_LY_o + rmt_RY_o);
					speed_R = rmt_LX_o;
					speed_X = rmt_RX_o;
				}
				
				speed_FL = speed_Y - speed_X + speed_R;
				speed_FR = speed_Y + speed_X - speed_R;
				speed_RL = speed_Y - speed_X - speed_R;
				speed_RR = speed_Y + speed_X + speed_R;


				motor_target[FL] = sign(speed_FL) * SPEED_MIN + speed_FL * (SPEED_MAX - SPEED_MIN) / 4100;
				motor_target[FR] = sign(speed_FR) * SPEED_MIN + speed_FR * (SPEED_MAX - SPEED_MIN) / 4100;
				motor_target[RL] = sign(speed_RL) * SPEED_MIN + speed_RL * (SPEED_MAX - SPEED_MIN) / 4100;
				motor_target[RR] = sign(speed_RR) * SPEED_MIN + speed_RR * (SPEED_MAX - SPEED_MIN) / 4100;
				
				
				for(i=0; i<4; i++)				
					if(labs(motor_target[FL]) > SPEED_MAX)
						motor_target[FL] = SPEED_MAX * sign(motor_target[FL]);
					
				for(i=0; i<4; i++)				
					if(labs(motor_target[FR]) > SPEED_MAX)
						motor_target[FR] = SPEED_MAX * sign(motor_target[FR]);
						
				for(i=0; i<4; i++)				
					if(labs(motor_target[RL]) > SPEED_MAX)
						motor_target[RL] = SPEED_MAX * sign(motor_target[RL]);
						
				for(i=0; i<4; i++)				
					if(labs(motor_target[RR]) > SPEED_MAX)
						motor_target[RR] = SPEED_MAX * sign(motor_target[RR]);
	
				for(i=0; i<4; i++)
					if(labs(motor_target[FL]) < SPEED_MIN + 1)
						motor_target[FL] = 0;
					
				for(i=0; i<4; i++)
					if(labs(motor_target[FR]) < SPEED_MIN + 1)
						motor_target[FR] = 0;
					
				for(i=0; i<4; i++)
					if(labs(motor_target[RL]) < SPEED_MIN + 1)
						motor_target[RL] = 0;
					
				for(i=0; i<4; i++)
					if(labs(motor_target[RR]) < SPEED_MIN + 1)
						motor_target[RR] = 0;


				aaaaa = bfr[5];
					
				kPm = (float)bfr[12] + (float)bfr[13] / 10;
				kIm = (float)bfr[14] + (float)bfr[15] / 10;
				kDm = (float)bfr[16] + (float)bfr[17] / 10;
				
				kPx = (float)bfr[18] + (float)bfr[19] / 10;
				kIx = (float)bfr[20] + (float)bfr[21] / 10;
				kDx = (float)bfr[22] + (float)bfr[23] / 10;
				
				kPy = (float)bfr[24] + (float)bfr[25] / 10;
				kIy = (float)bfr[26] + (float)bfr[27] / 10;
				kDy = (float)bfr[28] + (float)bfr[29] / 10;
					
				kPr = (float)bfr[30] + (float)bfr[31] / 10;
				kIr = (float)bfr[32] + (float)bfr[33] / 10;
				kDr = (float)bfr[34] + (float)bfr[35] / 10;
				
				motor_pid_tune(kPm, kIm, kDm);
				seek_line_pid_tune(kPx, kIx, kDx, kPy, kIy, kDy, kPr, kIr, kDr);

				
			}
		if(! rx_Valid)
		{
			invalid_cnt++;
			if(invalid_cnt > 20000)
			{
				invalid_cnt = 20000+1;
				motor_target[FL] = 0;
				motor_target[FR] = 0;
				motor_target[RL] = 0;
				motor_target[RR] = 0;
			}
		}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1R_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(1);
  LL_RCC_PLL1_SetN(120);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(1);
  LL_RCC_PLL2_SetN(64);
  LL_RCC_PLL2_SetP(2);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(480000000);

  LL_SetSystemCoreClock(480000000);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PCLK4);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE* f)
{
	LL_USART_TransmitData8(USART1, (unsigned char)ch);
	while(!LL_USART_IsActiveFlag_TXE(USART1));
	return ch;
}

int32_t sign(int32_t x)
{
	return x > 0 ? 1 : -1;
}

float fsign(float x)
{
	return x > 0 ? 1 : -1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
