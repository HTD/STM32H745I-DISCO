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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "console.h"
#include "tests.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern char* debugMessage;
extern uint8_t debugMessageChanged;

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
#define D10_Pin GPIO_PIN_4
#define D10_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_2
#define D12_GPIO_Port GPIOI
#define D09_Pin GPIO_PIN_15
#define D09_GPIO_Port GPIOH
#define D08_Pin GPIO_PIN_3
#define D08_GPIO_Port GPIOE
#define VBUS_OTG_FS_Pin GPIO_PIN_9
#define VBUS_OTG_FS_GPIO_Port GPIOA
#define D07_Pin GPIO_PIN_8
#define D07_GPIO_Port GPIOI
#define D06_Pin GPIO_PIN_6
#define D06_GPIO_Port GPIOE
#define D05_Pin GPIO_PIN_8
#define D05_GPIO_Port GPIOA
#define D02_Pin GPIO_PIN_3
#define D02_GPIO_Port GPIOG
#define D04_Pin GPIO_PIN_1
#define D04_GPIO_Port GPIOK
#define D03_Pin GPIO_PIN_6
#define D03_GPIO_Port GPIOA
#define D15_Pin GPIO_PIN_12
#define D15_GPIO_Port GPIOD
#define D14_Pin GPIO_PIN_13
#define D14_GPIO_Port GPIOD
#define D11_Pin GPIO_PIN_15
#define D11_GPIO_Port GPIOB
void   MX_SDMMC1_MMC_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
