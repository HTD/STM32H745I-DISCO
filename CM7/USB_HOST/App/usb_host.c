/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */
#include "fatfs.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

static osThreadId_t usbhApplicationTaskHandle;
static const osThreadAttr_t usbhApplication_attributes = {
  .name = "USB Host Application",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/**
 * @fn void USBH_Test()
 * @brief Tests the USB disk support.
 */
static void USBH_Test()
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

/**
 * @fn void USBH_Application(void*)
 * @brief The application run as the separate OS thread when the USB disk is connected.
 * @param argument
 */
void USBH_Application(USBH_HandleTypeDef *phost)
{
  USBH_Test(); // TODO: Provide proper USB disk initialization.
  for (;;) osDelay(1);
}

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
    debug("USB: SELECT_CONFIGURATION");
    break;

  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_DISCONNECT;
    debug("USB: DISCONNECTION");
    osThreadTerminate(usbhApplicationTaskHandle);
    break;

  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_READY;
    debug("USB: CLASS_ACTIVE");
    usbhApplicationTaskHandle = osThreadNew((osThreadFunc_t)USBH_Application, phost, &usbhApplication_attributes);
    break;

  case HOST_USER_CONNECTION:
    Appli_state = APPLICATION_START;
    debug("USB: CONNECTION");
    break;

  default:
    break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

