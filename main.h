/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define RightSW_Pin GPIO_PIN_14
#define RightSW_GPIO_Port GPIOC
#define LeftSW_Pin GPIO_PIN_15
#define LeftSW_GPIO_Port GPIOC
#define IZMIK_Pin GPIO_PIN_4
#define IZMIK_GPIO_Port GPIOA
#define M2INB_Pin GPIO_PIN_14
#define M2INB_GPIO_Port GPIOB
#define M2INA_Pin GPIO_PIN_15
#define M2INA_GPIO_Port GPIOB
#define M2EN_Pin GPIO_PIN_8
#define M2EN_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_10
#define PWM1_GPIO_Port GPIOA
#define M1INB_Pin GPIO_PIN_11
#define M1INB_GPIO_Port GPIOA
#define M1INA_Pin GPIO_PIN_12
#define M1INA_GPIO_Port GPIOA
#define M1EN_Pin GPIO_PIN_4
#define M1EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
