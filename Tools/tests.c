/**
 * @file		  tests.c
 * @author		CodeDog
 * @brief		  Contains some tests for the STM32H745I-DISCO board.
 *
 * @copyright	(c)2022 CodeDog, All rights reserved.
 */

#include "tests.h"
#include "fatfs.h"

/**
 * @brief Pins defined in hardware configuration as GPIO.
 */
static TestGPIOPin_TypeDef pins[10] =
{
    (TestGPIOPin_TypeDef){ D02_GPIO_Port, D02_Pin, "D2", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D04_GPIO_Port, D04_Pin, "D4", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D07_GPIO_Port, D07_Pin, "D7", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D08_GPIO_Port, D08_Pin, "D8", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D09_GPIO_Port, D09_Pin, "D9", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D10_GPIO_Port, D10_Pin, "D10", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D11_GPIO_Port, D11_Pin, "D11", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ D12_GPIO_Port, D12_Pin, "D12", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ A0_GPIO_Port, A0_Pin, "A0", GPIO_PIN_RESET },
    (TestGPIOPin_TypeDef){ A1_GPIO_Port, A1_Pin, "A1", GPIO_PIN_RESET }
};

/**
 * @fn void write_all_ARD_pins(GPIO_PinState)
 * @brief Writes specified state to all GPIO pins configured as outputs.
 * @param pin_state Pin state to set.
 */
static void write_all_ARD_pins(GPIO_PinState pin_state)
{
  for (int i = 0; i < 10; i++)
  {
    if (pins[i].port->MODER & pins[i].pin) // the pin is output
      HAL_GPIO_WritePin(pins[i].port, pins[i].pin, pin_state);
  }
}

/**
 * @fn void scan_inputs()
 * @brief Scans all GPIO pins configured as inputs and outputs changes in states to the console.
 */
static void scan_inputs()
{
  for (int i = 0; i < 10; i++)
  {
    if (pins[i].port->MODER & pins[i].pin) continue; // ignore outputs
    GPIO_PinState lastState = pins[i].state;
    GPIO_PinState currentState = HAL_GPIO_ReadPin(pins[i].port, pins[i].pin);
    if (currentState != lastState)
    {
      if (currentState == GPIO_PIN_SET)
        debug_s("PIN %s is SET.", pins[i].ARD);
      else
        debug_s("PIN %s is RESET.", pins[i].ARD);
    }
    pins[i].state = currentState;
  }
}

/**
 * @fn void GPIO_Test()
 * @brief Tests GPIO by setting 500Hz pulse to all GPIO outputs, and reporting change of state of all GPIO inputs.
 */
void GPIO_Test()
{
  debug("GPIO test in progress...");
  for (;;)
  {
    scan_inputs();
    write_all_ARD_pins(GPIO_PIN_SET);
    osDelay(1);
    write_all_ARD_pins(GPIO_PIN_RESET);
    osDelay(1);
  }
}

/**
 * @fn void PWMTest()
 * @brief Tests the 3 PWM generators by applying 100kHz 50% PWM to the channel outputs.
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

/**
 * @fn void UART_Test(const char*)
 * @brief Tests the UART by sending a message to the UART port.
 * @param message Message to send.
 */
void UART_Test(const char* message)
{
  console_write("UART TX...");
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, strlen(message));
  console_writeln("OK.");
}

/**
 * @fn void USBDiskTest()
 * @brief Tests the USB disk support by mounting, writing and reading a test file.
 * @remarks Call from USBH_Application(), it's called whenever USB disk is connected and detected as MSC device.
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
