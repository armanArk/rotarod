/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

uint8_t retUSER;      /* Return value for USER */
uint8_t retUSER2;     /* Return value for second logical drive */
char USERPath[4] = "0:";   /* USER logical drive path */
char USERPath2[4] = "1:";  /* Secondary logical drive path */
FATFS USERFatFS;      /* File system object for USER logical drive */
FATFS USERFatFS2;     /* File system object for secondary logical drive */
FIL USERFile;         /* File object for USER */

/* USER CODE BEGIN Variables */

#if _MULTI_PARTITION
PARTITION VolToPart[_VOLUMES] = {
    {0, 0},   /* 0: auto-detect (SFD or first MBR partition) */
    {0, 2}    /* 1: (unused) */
};
#endif

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);
  retUSER2 = FATFS_LinkDriver(&USER_Driver, USERPath2);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
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

/* USER CODE END Application */
