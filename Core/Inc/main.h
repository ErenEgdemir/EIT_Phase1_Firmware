/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DDS_CS_Pin GPIO_PIN_13
#define DDS_CS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_14
#define SD_CS_GPIO_Port GPIOC
#define MUX2_EN_Pin GPIO_PIN_15
#define MUX2_EN_GPIO_Port GPIOC
#define MUX4_S0_Pin GPIO_PIN_4
#define MUX4_S0_GPIO_Port GPIOA
#define MUX1_S0_Pin GPIO_PIN_0
#define MUX1_S0_GPIO_Port GPIOB
#define MUX1_S1_Pin GPIO_PIN_1
#define MUX1_S1_GPIO_Port GPIOB
#define MUX1_S2_Pin GPIO_PIN_2
#define MUX1_S2_GPIO_Port GPIOB
#define MUX1_S3_Pin GPIO_PIN_10
#define MUX1_S3_GPIO_Port GPIOB
#define MUX2_S0_Pin GPIO_PIN_12
#define MUX2_S0_GPIO_Port GPIOB
#define MUX2_S1_Pin GPIO_PIN_13
#define MUX2_S1_GPIO_Port GPIOB
#define MUX2_S2_Pin GPIO_PIN_14
#define MUX2_S2_GPIO_Port GPIOB
#define MUX2_S3_Pin GPIO_PIN_15
#define MUX2_S3_GPIO_Port GPIOB
#define MUX4_S1_Pin GPIO_PIN_8
#define MUX4_S1_GPIO_Port GPIOA
#define MUX4_S2_Pin GPIO_PIN_9
#define MUX4_S2_GPIO_Port GPIOA
#define MUX4_S3_Pin GPIO_PIN_10
#define MUX4_S3_GPIO_Port GPIOA
#define MUX1_EN_Pin GPIO_PIN_15
#define MUX1_EN_GPIO_Port GPIOA
#define MUX3_S0_Pin GPIO_PIN_3
#define MUX3_S0_GPIO_Port GPIOB
#define MUX3_S1_Pin GPIO_PIN_4
#define MUX3_S1_GPIO_Port GPIOB
#define MUX3_S2_Pin GPIO_PIN_5
#define MUX3_S2_GPIO_Port GPIOB
#define MUX3_S3_Pin GPIO_PIN_6
#define MUX3_S3_GPIO_Port GPIOB
#define COMPARATOR_Pin GPIO_PIN_7
#define COMPARATOR_GPIO_Port GPIOB
#define MUX3_EN_Pin GPIO_PIN_8
#define MUX3_EN_GPIO_Port GPIOB
#define MUX4_EN_Pin GPIO_PIN_9
#define MUX4_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
