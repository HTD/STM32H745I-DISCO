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

void printTest(const char* message);
void USBDiskTest();
