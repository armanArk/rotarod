/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   User Disk I/O driver for W25Q Flash (FatFs)
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "ff_gen_drv.h"
#include "w25q64.h"
#include "cli.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_BLOCK_SIZE 512
#define FLASH_SECTOR_SIZE 4096

/* Private variables ---------------------------------------------------------*/
uint32_t flash_total_size = 0;
uint8_t sector_buffer[4096];

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

uint32_t USER_GetFlashTotalSize(void) {
    return flash_total_size;
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  UNUSED(pdrv);

  // Init W25Q and detect capacity
  W25Q64_Init();
  uint32_t flash_id = W25Q64_ReadID();
  uint8_t cap_byte = flash_id & 0xFF;

  if (cap_byte == 0x17)      flash_total_size = 8  * 1024 * 1024;   // W25Q64
  else if (cap_byte == 0x18) flash_total_size = 16 * 1024 * 1024;   // W25Q128
  else if (cap_byte == 0x19) flash_total_size = 32 * 1024 * 1024;   // W25Q256
  else                       flash_total_size = 8  * 1024 * 1024;   // Default

  // Clear status register write protection bits
  uint8_t sr = W25Q64_ReadStatusRegister1();
  if (sr != 0x00) {
      W25Q64_WriteStatusRegister1(0x00);
  }

  // Debug: report detected flash size
  {
    char dbg[64];
    sprintf(dbg, "user_diskio: flash_total_size=%lu bytes\r\n", flash_total_size);
    UART_Print(dbg);
  }

  Stat = 0;  // STA_OK
  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  UNUSED(pdrv);
  if (flash_total_size == 0) return STA_NOINIT;
  return 0; // STA_OK
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  UNUSED(pdrv);

  uint32_t addr = sector * FLASH_BLOCK_SIZE;
  uint32_t len  = count * FLASH_BLOCK_SIZE;

  // Bounds check
  if ((addr + len) > flash_total_size) return RES_PARERR;

  W25Q64_ReadData(addr, buff, len);
  return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  UNUSED(pdrv);

  uint32_t addr = sector * FLASH_BLOCK_SIZE;
  uint32_t len  = count * FLASH_BLOCK_SIZE;
  uint32_t end_addr = addr + len;

  // Bounds check
  if (end_addr > flash_total_size) return RES_PARERR;

  // Process each 4KB W25Q sector that overlaps the write range
  for (uint32_t se = (addr / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE; se < end_addr; se += FLASH_SECTOR_SIZE) {

    // 1. Read existing sector content
    W25Q64_ReadData(se, sector_buffer, FLASH_SECTOR_SIZE);

    // 2. Calculate overlap between write data and this sector
    uint32_t write_start_in_sector = (addr > se) ? (addr - se) : 0;
    uint32_t write_end_in_sector   = (end_addr < (se + FLASH_SECTOR_SIZE)) ? (end_addr - se) : FLASH_SECTOR_SIZE;
    uint32_t copy_len = write_end_in_sector - write_start_in_sector;
    uint32_t buf_offset = (addr > se) ? 0 : (se - addr);

    // 3. Merge new data into sector buffer
    memcpy(sector_buffer + write_start_in_sector, buff + buf_offset, copy_len);

    // 4. Erase the sector (required before write)
    W25Q64_SectorErase(se);

    // 5. Write back sector in 256-byte pages
    for (uint32_t pg = 0; pg < FLASH_SECTOR_SIZE; pg += 256) {
      W25Q64_PageProgram(se + pg, sector_buffer + pg, 256);
    }
  }

  return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  UNUSED(pdrv);
  DRESULT res = RES_OK;

  switch (cmd) {
    case CTRL_SYNC:
      // Nothing to do for W25Q (synchronous SPI)
      res = RES_OK;
      break;

    case GET_SECTOR_COUNT:
      *(DWORD *)buff = flash_total_size / FLASH_BLOCK_SIZE;
      {
        char dbg[64];
        sprintf(dbg, "IOCTL: GET_SECTOR_COUNT=%lu\r\n", *(DWORD *)buff);
        UART_Print(dbg);
      }
      res = RES_OK;
      break;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = FLASH_BLOCK_SIZE;
      {
        char dbg[64];
        sprintf(dbg, "IOCTL: GET_SECTOR_SIZE=%u\r\n", *(WORD *)buff);
        UART_Print(dbg);
      }
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE:
      // Number of sectors per erase block (for erase alignment)
      *(DWORD *)buff = FLASH_SECTOR_SIZE / FLASH_BLOCK_SIZE; // 8
      {
        char dbg[64];
        sprintf(dbg, "IOCTL: GET_BLOCK_SIZE=%lu\r\n", *(DWORD *)buff);
        UART_Print(dbg);
      }
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
      break;
  }
  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

