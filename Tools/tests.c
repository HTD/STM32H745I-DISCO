/**
 * @file		  tests.c
 * @author		CodeDog
 * @brief		  Contains some tests for the STM32H745I-DISCO board.
 *
 * @copyright	(c)2022 CodeDog, All rights reserved.
 */

#include "tests.h"
#include "fatfs.h"

static TestGPIOPin_TypeDef pins[10] =
{
    (TestGPIOPin_TypeDef){ D02_GPIO_Port, D02_Pin, 2, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D04_GPIO_Port, D04_Pin, 4, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D07_GPIO_Port, D07_Pin, 7, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D08_GPIO_Port, D08_Pin, 8, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D09_GPIO_Port, D09_Pin, 9, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D10_GPIO_Port, D10_Pin, 10, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D11_GPIO_Port, D11_Pin, 11, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D12_GPIO_Port, D12_Pin, 12, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ A0_GPIO_Port, A0_Pin, 0, GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ A1_GPIO_Port, A1_Pin, 1, GPIO_PIN_RESET }
};

static void write_all_ARD_pins(GPIO_PinState pin_state)
{
  for (int i = 0; i < 10; i++)
    HAL_GPIO_WritePin(pins[i].port, pins[i].pin, pin_state);
}

void GPIO_Out_Test()
{
  debug("GPIO OUT test in progress...");
  for (;;)
  {
    write_all_ARD_pins(GPIO_PIN_SET);
    osDelay(1);
    write_all_ARD_pins(GPIO_PIN_RESET);
    osDelay(1);
  }
}

void GPIO_In_Test()
{
  debug("GPIO IN test in progress...");
  for (;;)
  {
    for (int i = 0; i < 10; i++)
    {
      GPIO_PinState lastState = pins[i].state;
      GPIO_PinState currentState = HAL_GPIO_ReadPin(pins[i].port, pins[i].pin);
      if (currentState != lastState)
      {
        if (currentState == GPIO_PIN_SET)
          debug_i("PIN D%i is SET.", pins[i].ARD);
        else
          debug_i("PIN D%i is RESET.", pins[i].ARD);
      }
      pins[i].state = currentState;
    }
    osDelay(1);
  }
}

/**
 * @fn void PWMTest()
 * @brief Tests the 3 PWM generators and 10 I/O pins.
 */
void PWM_Test()
{
  console_write("100kHz PWM on D3, D5 and D6...");
  htim1.Instance->CCR1 = (htim1.Instance->ARR + 1) / 2 - 1;
  htim3.Instance->CCR1 = (htim3.Instance->ARR + 1) / 2 - 1;
  htim15.Instance->CCR2 = (htim15.Instance->ARR + 1) / 2 - 1;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void UART_Test(const char* message)
{
  console_write("Printing...");
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, strlen(message));
  console_writeln("OK.");
}

/**
 * @fn void USBDiskTest()
 * @brief Tests the USB disk support.
 */
void USB_Disk_Test()
{
  debug("Mounting USB disk...");
  FRESULT fr = f_mount(&USBHFatFS, USBHPath, 0x1);
  if (fr == FR_OK)
    debug("USB mounted successfully.");
  else
    debug("ERROR: USB f_mount().");
  const TCHAR* fileName = "STM32H745I-DISCO-TEST.txt";
  const TCHAR* content = "It seems like the USB communication works just fine.";
  TCHAR buffer[128];
  const UINT contentLength = strlen(content);
  uint8_t error = 0;
  UINT bytesRead = 0, bytesWritten = 0;
  fr = f_open(&USBHFile, fileName, FA_WRITE | FA_CREATE_ALWAYS);
  if (fr == FR_OK)
  {
    debug("File created successfully.");
    fr = f_write(&USBHFile, content, contentLength, &bytesWritten);
    if (fr == FR_OK)
    {
      debug("File content written.");
      fr = f_close(&USBHFile);
      if (fr == FR_OK)
      {
        fr = f_open(&USBHFile, fileName, FA_READ);
        if (fr == FR_OK)
        {
          debug("File opnened successfully.");
          fr = f_read(&USBHFile, buffer, contentLength, &bytesRead);
          if (fr == FR_OK)
          {
            if (bytesRead == bytesWritten)
            {
              for (uint8_t i = 0; i < contentLength; i++)
                if (buffer[i] != content[i])
                {
                  error = 1;
                  break;
                }
              if (error)
                debug("File contents differ!");
              else
                debug("Test file content matches.");
            }
            else
              debug("File size mismatch!");
            fr = f_close(&USBHFile);
            if (fr == FR_OK)
              debug("USB host test SUCCESS!");
            else
              debug("Could not close the file read.");
          }
          else
            debug("Could not read the file.");
        }
        else
          debug("Could not open the file.");
      }
      else
        debug("Could not close the file written.");
    }
    else
      debug("Could not write data.");
  }
  else
    debug("Could not create the file.");
}
