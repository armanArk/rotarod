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

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

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

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

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
static uint32_t s_partition_offset_bytes = 0;
// Media ready flag set at init to avoid slow checks in IsReady
static volatile uint8_t s_media_ready = 0;

// Sector Cache for read-modify-write (one W25Q sector = 4KB)
#define SECTOR_SIZE 4096
#define INVALID_SECTOR 0xFFFFFFFF
static uint8_t s_sector_buf[SECTOR_SIZE];
static uint32_t s_cached_sector = INVALID_SECTOR;
static uint8_t s_cache_dirty = 0;

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

  // Prefer using flash size detected by FATFS diskio layer if available
  extern uint32_t USER_GetFlashTotalSize(void);
  uint32_t fsz = USER_GetFlashTotalSize();
  if (fsz > 0) {
    s_flash_capacity_bytes = fsz;
  } else {
    // Fallback: detect flash capacity from ID
    uint32_t flash_id = W25Q64_ReadID();
    uint8_t cap_byte = flash_id & 0xFF;

    if (cap_byte == 0x17)      s_flash_capacity_bytes = 8  * 1024 * 1024;   // W25Q64
    else if (cap_byte == 0x18) s_flash_capacity_bytes = 16 * 1024 * 1024;   // W25Q128
    else if (cap_byte == 0x19) s_flash_capacity_bytes = 32 * 1024 * 1024;   // W25Q256
    else                       s_flash_capacity_bytes = 8  * 1024 * 1024;   // Default
  }

  s_block_count = s_flash_capacity_bytes / STORAGE_BLK_SIZ;
  /* Avoid heavy/blocking debug I/O in USB callbacks (log from main loop instead) */

  // Mark media ready for fast IsReady responses
  s_media_ready = (s_block_count > 0) ? 1 : 0;

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
  // USB MSC expects the last valid LBA (block count - 1)
  if (s_block_count == 0) return USBD_FAIL;
  *block_num  = s_block_count - 1;
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
  // Small delay to allow flash / SPI to settle when host probes
  // Return cached readiness set during STORAGE_Init_FS to avoid blocking USB stack
  if (s_media_ready) return USBD_OK;
  return USBD_FAIL;
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

  return 1;  // 1 = Write Protected (Read-Only)
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

  uint32_t addr = s_partition_offset_bytes + (blk_addr * STORAGE_BLK_SIZ);
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;

  if ((addr + len) > (s_partition_offset_bytes + s_flash_capacity_bytes)) {
    return USBD_FAIL;
  }

  for (uint32_t offset = 0; offset < len; offset += STORAGE_BLK_SIZ) {
    uint32_t curr_addr = addr + offset;
    uint32_t sector_addr = (curr_addr / SECTOR_SIZE) * SECTOR_SIZE;
    uint32_t offset_in_sector = curr_addr % SECTOR_SIZE;

    if (s_cached_sector == sector_addr) {
      // Read from cache
      memcpy(buf + offset, s_sector_buf + offset_in_sector, STORAGE_BLK_SIZ);
    } else {
      // Read from flash
      W25Q64_ReadData(curr_addr, buf + offset, STORAGE_BLK_SIZ);
    }
  }

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

  uint32_t addr = s_partition_offset_bytes + (blk_addr * STORAGE_BLK_SIZ);
  uint32_t len  = blk_len * STORAGE_BLK_SIZ;

  if ((addr + len) > (s_partition_offset_bytes + s_flash_capacity_bytes)) {
    return USBD_FAIL;
  }

  for (uint32_t offset = 0; offset < len; offset += STORAGE_BLK_SIZ) {
    uint32_t curr_addr = addr + offset;
    uint32_t sector_addr = (curr_addr / SECTOR_SIZE) * SECTOR_SIZE;
    uint32_t offset_in_sector = curr_addr % SECTOR_SIZE;

    if (s_cached_sector != sector_addr) {
      // Flush old sector if dirty
      STORAGE_Flush();
      
      // Load new sector
      s_cached_sector = sector_addr;
      W25Q64_ReadData(s_cached_sector, s_sector_buf, SECTOR_SIZE);
    }

    // Modify cache
    memcpy(s_sector_buf + offset_in_sector, buf + offset, STORAGE_BLK_SIZ);
    s_cache_dirty = 1;
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
/* Allow application to control media ready flag and capacity */
void STORAGE_SetMediaReady(uint8_t ready)
{
  s_media_ready = ready ? 1 : 0;
}

void STORAGE_UpdateCapacity(uint32_t bytes)
{
  s_flash_capacity_bytes = bytes;
  s_block_count = s_flash_capacity_bytes / STORAGE_BLK_SIZ;
}

void STORAGE_SetPartitionOffset(uint32_t byte_offset)
{
  s_partition_offset_bytes = byte_offset;
}

void STORAGE_Flush(void){if(s_cache_dirty&&s_cached_sector!=INVALID_SECTOR){W25Q64_SectorErase(s_cached_sector);for(uint32_t pg=0;pg<SECTOR_SIZE;pg+=256){W25Q64_PageProgram(s_cached_sector+pg,s_sector_buf+pg,256);}s_cache_dirty=0;}}

void STORAGE_Invalidate(void)
{
  s_cached_sector = INVALID_SECTOR;
  s_cache_dirty = 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */


