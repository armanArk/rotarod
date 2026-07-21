/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @brief          : Memory management layer for W25Q Flash (8/16/32 MB)
  *                   - Auto-detects flash capacity from ID
  *                   - Sector size: 4096 bytes (W25Q erase sector)
  *                   - Block size: 512 bytes (USB MSC standard)
  *                   - Read-modify-write for 512B blocks within 4KB sectors
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"
#include "w25q64.h"

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_SIZ                  512     // USB MSC standard block size

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
const int8_t STORAGE_Inquirydata_FS[] = {
  0x00, 0x80, 0x02, 0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00, 0x00, 0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ',
  'W', '2', '5', 'Q', 'F', 'l', 'a', 's',  /* Product */
  'h', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '1', '.', '0', '0'
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

// Auto-detected flash capacity
static uint32_t s_flash_capacity_bytes = 0;
static uint32_t s_block_count = 0;

// Static buffer for read-modify-write (one W25Q sector = 4KB)
static uint8_t sector_buf[4096];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
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

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
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

/**
  * @brief  Initializes the storage unit (medium) over USB FS IP
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  UNUSED(lun);

  // Detect flash capacity from ID
  uint32_t flash_id = W25Q64_ReadID();
  uint8_t cap_byte = flash_id & 0xFF;

  if (cap_byte == 0x17)      s_flash_capacity_bytes = 8  * 1024 * 1024;   // W25Q64
  else if (cap_byte == 0x18) s_flash_capacity_bytes = 16 * 1024 * 1024;   // W25Q128
  else if (cap_byte == 0x19) s_flash_capacity_bytes = 32 * 1024 * 1024;   // W25Q256
  else                       s_flash_capacity_bytes = 8  * 1024 * 1024;   // Default

  s_block_count = s_flash_capacity_bytes / STORAGE_BLK_SIZ;

  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number.
  * @param  block_num: Number of total block number.
  * @param  block_size: Block size.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  UNUSED(lun);

  *block_num  = s_block_count;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief   Checks whether the medium is ready.
  * @param  lun:  Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  UNUSED(lun);

  return (USBD_OK);  // 0 = NOT write protected
  /* USER CODE END 5 */
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
  UNUSED(lun);

  // Bounds check
  if ((blk_addr + blk_len) > s_block_count) {
    return USBD_FAIL;
  }

  uint32_t addr = blk_addr * STORAGE_BLK_SIZ;
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;

  W25Q64_ReadData(addr, buf, len);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
  UNUSED(lun);

  // Bounds check
  if ((blk_addr + blk_len) > s_block_count) {
    return USBD_FAIL;
  }

  uint32_t addr = blk_addr * STORAGE_BLK_SIZ;
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;
  uint32_t end_addr = addr + len;

  // Process each 4KB W25Q sector that overlaps the write range
  for (uint32_t se = (addr / 4096) * 4096; se < end_addr; se += 4096) {

    // 1. Read existing sector content
    W25Q64_ReadData(se, sector_buf, 4096);

    // 2. Calculate overlap between write data and this sector
    uint32_t write_start_in_sector = (addr > se) ? (addr - se) : 0;
    uint32_t write_end_in_sector = (end_addr < (se + 4096)) ? (end_addr - se) : 4096;
    uint32_t copy_len = write_end_in_sector - write_start_in_sector;
    uint32_t buf_offset = (addr > se) ? 0 : (se - addr);

    // 3. Merge new data into sector buffer
    memcpy(sector_buf + write_start_in_sector, buf + buf_offset, copy_len);

    // 4. Erase the sector (required before write)
    W25Q64_SectorErase(se);

    // 5. Write back sector in 256-byte pages
    for (uint32_t pg = 0; pg < 4096; pg += 256) {
      W25Q64_PageProgram(se + pg, sector_buf + pg, 256);
    }
  }

  return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  Returns the Max Supported LUNs.
  * @param  None
  * @retval Lun(s) number.
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
