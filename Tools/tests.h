/**
 * @file      tests.h
 * @author    CodeDog
 * @brief     Contains some tests for the STM32H745I-DISCO board.
 *
 * @copyright (c)2022 CodeDog, All rights reserved.
 */

#pragma once

#include "main.h"

// From main.c:

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

/**
 * @struct
 * @typedef TestGPIOPin_TypeDef
 * @brief Test GPIO pin definition.
 */
typedef struct
{
  GPIO_TypeDef* port; ///< GPIO port pointer.
  uint16_t bit; ///< GPIO pin bit mask.
  char* ARD; ///< ARDUINO pin name.
  GPIO_PinState state; ///< Pin state storage.
} TestGPIOPin_TypeDef;

void PWM_Test();
void GPIO_Test();
void UART_Test(const char* message);
void USB_Disk_Test();
