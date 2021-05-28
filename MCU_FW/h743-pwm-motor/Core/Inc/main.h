/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t FL_Speed1;
extern uint32_t FL_Speed2;

extern uint32_t FR_Speed1;
extern uint32_t FR_Speed2;

extern uint32_t RL_Speed1;
extern uint32_t RL_Speed2;

extern uint32_t RR_Speed1;
extern uint32_t RR_Speed2;

extern int32_t FL_target;
extern int32_t FR_target;
extern int32_t RL_target;
extern int32_t RR_target;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int32_t sign(int32_t x);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_NE_Pin LL_GPIO_PIN_3
#define SPI_NE_GPIO_Port GPIOE
#define RF_RST_Pin LL_GPIO_PIN_4
#define RF_RST_GPIO_Port GPIOE
#define PWM_FL_Pin LL_GPIO_PIN_0
#define PWM_FL_GPIO_Port GPIOA
#define PWM_FR_Pin LL_GPIO_PIN_1
#define PWM_FR_GPIO_Port GPIOA
#define PWM_RL_Pin LL_GPIO_PIN_2
#define PWM_RL_GPIO_Port GPIOA
#define PWM_RR_Pin LL_GPIO_PIN_3
#define PWM_RR_GPIO_Port GPIOA
#define SPI1_EN_Pin LL_GPIO_PIN_4
#define SPI1_EN_GPIO_Port GPIOA
#define SPEED_RL_Pin LL_GPIO_PIN_10
#define SPEED_RL_GPIO_Port GPIOB
#define SPEED_RR_Pin LL_GPIO_PIN_11
#define SPEED_RR_GPIO_Port GPIOB
#define DIR_RR_Pin LL_GPIO_PIN_6
#define DIR_RR_GPIO_Port GPIOC
#define DIR_RL_Pin LL_GPIO_PIN_7
#define DIR_RL_GPIO_Port GPIOC
#define DIR_FR_Pin LL_GPIO_PIN_8
#define DIR_FR_GPIO_Port GPIOC
#define DIR_FL_Pin LL_GPIO_PIN_9
#define DIR_FL_GPIO_Port GPIOC
#define SPEED_FL_Pin LL_GPIO_PIN_15
#define SPEED_FL_GPIO_Port GPIOA
#define SPI3_NE_Pin LL_GPIO_PIN_0
#define SPI3_NE_GPIO_Port GPIOD
#define SPEED_FR_Pin LL_GPIO_PIN_3
#define SPEED_FR_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define SERVO_PWM_MIN 10000
#define SERVO_PWM_MAX 10000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
