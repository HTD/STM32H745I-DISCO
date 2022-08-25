/**
 * @file      tests.h
 * @author    CodeDog
 * @brief     Contains some tests for the STM32H745I-DISCO board.
 *
 * @copyright (c)2022 CodeDog, All rights reserved.
 */

#pragma once

#include "main.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

typedef struct
{
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t ARD;
  GPIO_PinState state;
} TestGPIOPin_TypeDef;

void PWM_Test();
void GPIO_Out_Test();
void GPIO_In_Test();
void UART_Test(const char* message);
void USB_Disk_Test();
