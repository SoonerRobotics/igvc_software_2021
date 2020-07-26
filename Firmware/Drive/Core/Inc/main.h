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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define LeftA_Pin GPIO_PIN_0
#define LeftA_GPIO_Port GPIOA
#define LeftB_Pin GPIO_PIN_1
#define LeftB_GPIO_Port GPIOA
#define RightA_Pin GPIO_PIN_2
#define RightA_GPIO_Port GPIOA
#define RightB_Pin GPIO_PIN_3
#define RightB_GPIO_Port GPIOA
#define LeftMotorPWM_Pin GPIO_PIN_0
#define LeftMotorPWM_GPIO_Port GPIOB
#define RightMotorPWM_Pin GPIO_PIN_1
#define RightMotorPWM_GPIO_Port GPIOB
#define RightEncoderA_Pin GPIO_PIN_6
#define RightEncoderA_GPIO_Port GPIOC
#define RighEncoderB_Pin GPIO_PIN_7
#define RighEncoderB_GPIO_Port GPIOC
#define LeftEncoderA_Pin GPIO_PIN_8
#define LeftEncoderA_GPIO_Port GPIOA
#define LeftEncoderB_Pin GPIO_PIN_9
#define LeftEncoderB_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
