/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer for W25Q128 Flash (16 MB)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"
#include "w25q64.h"          // <-- Driver SPI Flash

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_STORAGE
  * @{
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @{
  */

#define STORAGE_LUN_NBR                  1
// Kapasitas W25Q128 = 16 MB = 16*1024*1024 byte
#define STORAGE_BLK_SIZ                  512
#define STORAGE_BLK_NBR                  (16 * 1024 * 1024 / STORAGE_BLK_SIZ) // 32768

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
const int8_t STORAGE_Inquirydata_FS[] = {
  0x00, 0x80, 0x02, 0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00, 0x00, 0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ',
  'W', '2', '5', 'Q', '1', '2', '8', ' ',  /* Product: W25Q128 */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '1', '.', '0', '0'
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS = {
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/

int8_t STORAGE_Init_FS(uint8_t lun)
{
  UNUSED(lun);
  // Flash sudah diinisialisasi di main.c
  return USBD_OK;
}

int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  UNUSED(lun);
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return USBD_OK;
}

int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  UNUSED(lun);
  return USBD_OK;
}

int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  UNUSED(lun);
  return USBD_OK;
}

int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  UNUSED(lun);
  uint32_t addr = blk_addr * STORAGE_BLK_SIZ;
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;

  W25Q64_ReadData(addr, buf, len);
  return USBD_OK;
}

int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  UNUSED(lun);
  uint32_t addr = blk_addr * STORAGE_BLK_SIZ;
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;

  // Buffer 4KB untuk menangani erase-before-write
  static uint8_t sector_buf[4096];
  uint32_t sector_start = (addr / 4096) * 4096;
  uint32_t sector_end   = ((addr + len + 4095) / 4096) * 4096;

  for (uint32_t se = sector_start; se < sector_end; se += 4096) {
    // Baca seluruh 4KB ke buffer
    W25Q64_ReadData(se, sector_buf, 4096);
    // Salin data baru ke posisi yang sesuai
    uint32_t offset = (addr > se) ? (addr - se) : 0;
    uint32_t copy_len = (len > (4096 - offset)) ? (4096 - offset) : len;
    if (copy_len > 0 && addr >= se) {
      memcpy(sector_buf + offset, buf + (addr - se), copy_len);
    }
    // Hapus sektor 4KB
    W25Q64_SectorErase(se);
    // Tulis ulang per halaman 256 byte
    for (uint32_t i = 0; i < 4096; i += 256) {
      W25Q64_PageProgram(se + i, sector_buf + i, 256);
    }
  }
  return USBD_OK;
}

int8_t STORAGE_GetMaxLun_FS(void)
{
  return (STORAGE_LUN_NBR - 1);
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
