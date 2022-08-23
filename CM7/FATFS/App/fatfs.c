/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"

uint8_t retUSBH;    /* Return value for USBH */
char USBHPath[4];   /* USBH logical drive path */
FATFS USBHFatFS;    /* File system object for USBH logical drive */
FIL USBHFile;       /* File object for USBH */

/* USER CODE BEGIN Variables */
uint8_t retMMC; /* Return value for eMMC */
TCHAR MMCPath[4]; /* eMMC logical drive path */
FATFS MMCFatFS; /* File system object for eMMC logical drive */
FIL MMCFile; /* File object for eMMC */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USBH driver ###########################*/
  retUSBH = FATFS_LinkDriver(&USBH_Driver, USBHPath);

  /* USER CODE BEGIN Init */
  /*## FatFS: Link the MMC driver ###########################*/
  retMMC = FATFS_LinkDriver(&MMC_Driver, MMCPath);
  /* additional user code for init */
  if (retUSBH) debug("ERROR linking the USBH driver.");
  if (retMMC) debug("ERROR linking the MMC driver.");
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

void formatMMC()
{
  debug("Formatting MMC...");
  osDelay(20);
  static TCHAR workBuffer[_MIN_SS];
  FRESULT fr = f_mkfs(MMCPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
  if (fr == FR_OK)
    debug("eMMC formatted successfully.");
  else
    debug("ERROR: eMMC f_mkfs().");
}

void testUSB()
{
//  osDelay(1000);
  debug("Mounting USB disk...");
  osDelay(20);
  FRESULT fr = f_mount(&USBHFatFS, USBHPath, 0x1);
  if (fr == FR_OK)
    debug("USB mounted successfully.");
  else
    debug("ERROR: USB f_mount().");
}

/* USER CODE END Application */
