/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_EI_Pin GPIO_PIN_13
#define IMU_EI_GPIO_Port GPIOC
#define LM_1_Pin GPIO_PIN_2
#define LM_1_GPIO_Port GPIOA
#define LM2_Pin GPIO_PIN_3
#define LM2_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_4
#define IR2_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_6
#define IR3_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_4
#define IR1_GPIO_Port GPIOC
#define PT3_Pin GPIO_PIN_5
#define PT3_GPIO_Port GPIOC
#define PT1_Pin GPIO_PIN_0
#define PT1_GPIO_Port GPIOB
#define PT2_Pin GPIO_PIN_1
#define PT2_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_13
#define MCU_LED_GPIO_Port GPIOB
#define LE_B_Pin GPIO_PIN_6
#define LE_B_GPIO_Port GPIOC
#define RM_2_Pin GPIO_PIN_8
#define RM_2_GPIO_Port GPIOA
#define RM_1_Pin GPIO_PIN_9
#define RM_1_GPIO_Port GPIOA
#define SPI3_SS_Pin GPIO_PIN_10
#define SPI3_SS_GPIO_Port GPIOC
#define LE_A_Pin GPIO_PIN_5
#define LE_A_GPIO_Port GPIOB
#define RE_A_Pin GPIO_PIN_6
#define RE_A_GPIO_Port GPIOB
#define RE_B_Pin GPIO_PIN_7
#define RE_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
