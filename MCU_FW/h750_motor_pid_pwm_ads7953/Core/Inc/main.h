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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPEED_FL_Pin LL_GPIO_PIN_0
#define SPEED_FL_GPIO_Port GPIOA
#define SPEED_FR_Pin LL_GPIO_PIN_1
#define SPEED_FR_GPIO_Port GPIOA
#define SPEED_RL_Pin LL_GPIO_PIN_2
#define SPEED_RL_GPIO_Port GPIOA
#define SPEED_RR_Pin LL_GPIO_PIN_3
#define SPEED_RR_GPIO_Port GPIOA
#define SPI1_NE1_Pin LL_GPIO_PIN_4
#define SPI1_NE1_GPIO_Port GPIOC
#define SPI1_NE2_Pin LL_GPIO_PIN_5
#define SPI1_NE2_GPIO_Port GPIOC
#define PWM_FL_Pin LL_GPIO_PIN_9
#define PWM_FL_GPIO_Port GPIOE
#define DIR_FR_Pin LL_GPIO_PIN_10
#define DIR_FR_GPIO_Port GPIOE
#define PWM_FR_Pin LL_GPIO_PIN_11
#define PWM_FR_GPIO_Port GPIOE
#define DIR_RL_Pin LL_GPIO_PIN_12
#define DIR_RL_GPIO_Port GPIOE
#define PWM_RL_Pin LL_GPIO_PIN_13
#define PWM_RL_GPIO_Port GPIOE
#define PWM_RR_Pin LL_GPIO_PIN_14
#define PWM_RR_GPIO_Port GPIOE
#define DIR_RR_Pin LL_GPIO_PIN_15
#define DIR_RR_GPIO_Port GPIOE
#define DIR_FL_Pin LL_GPIO_PIN_12
#define DIR_FL_GPIO_Port GPIOB
#define SERVO_PWM6_Pin LL_GPIO_PIN_14
#define SERVO_PWM6_GPIO_Port GPIOD
#define SERVO_PWM5_Pin LL_GPIO_PIN_15
#define SERVO_PWM5_GPIO_Port GPIOD
#define SERVO_PWM4_Pin LL_GPIO_PIN_6
#define SERVO_PWM4_GPIO_Port GPIOC
#define SERVO_PWM3_Pin LL_GPIO_PIN_7
#define SERVO_PWM3_GPIO_Port GPIOC
#define SERVO_PWM2_Pin LL_GPIO_PIN_8
#define SERVO_PWM2_GPIO_Port GPIOC
#define SERVO_PWM1_Pin LL_GPIO_PIN_9
#define SERVO_PWM1_GPIO_Port GPIOC
#define RF_NE_Pin LL_GPIO_PIN_0
#define RF_NE_GPIO_Port GPIOD
#define RF_RST_Pin LL_GPIO_PIN_1
#define RF_RST_GPIO_Port GPIOD
#define LED1_Pin LL_GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define LED2_Pin LL_GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define LED3_Pin LL_GPIO_PIN_4
#define LED3_GPIO_Port GPIOD
#define LED4_Pin LL_GPIO_PIN_5
#define LED4_GPIO_Port GPIOD
#define SPI6_NE2_Pin LL_GPIO_PIN_6
#define SPI6_NE2_GPIO_Port GPIOD
#define SPI6_NE1_Pin LL_GPIO_PIN_7
#define SPI6_NE1_GPIO_Port GPIOD
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
int32_t sign(int32_t x);
float fsign(float x);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
