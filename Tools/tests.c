/**
 * @file		  tests.c
 * @author		CodeDog
 * @brief		  Contains some tests for the STM32H745I-DISCO board.
 *
 * @copyright	(c)2022 CodeDog, All rights reserved.
 */

#include "tests.h"
#include "fatfs.h"

void printTest(const char* message)
{
  console_write("Printing...");
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, strlen(message));
  console_writeln("OK.");
}

/**
 * @fn void USBH_Test()
 * @brief Tests the USB disk support.
 */
void USBDiskTest()
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
