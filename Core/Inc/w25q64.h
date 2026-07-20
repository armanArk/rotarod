#ifndef W25Q64_H
#define W25Q64_H

#include "main.h"   // Sudah include semua HAL, termasuk SPI

/* ==================== COMMANDS ==================== */
#define W25Q_CMD_WREN       0x06
#define W25Q_CMD_WRDI       0x04
#define W25Q_CMD_RDSR       0x05
#define W25Q_CMD_READ       0x03
#define W25Q_CMD_FAST_READ  0x0B
#define W25Q_CMD_PP         0x02
#define W25Q_CMD_SE         0x20
#define W25Q_CMD_BE         0xD8
#define W25Q_CMD_CE         0xC7
#define W25Q_CMD_READ_ID    0x9F

/* ==================== FUNGSI ==================== */
void     W25Q64_Init(void);
uint32_t W25Q64_ReadID(void);
uint8_t  W25Q64_ReadStatusRegister1(void);
void     W25Q64_WriteEnable(void);
void     W25Q64_WriteDisable(void);
void     W25Q64_WaitBusy(void);
void     W25Q64_SectorErase(uint32_t addr);
void     W25Q64_ReadData(uint32_t addr, uint8_t *buffer, uint32_t length);
void     W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint32_t length);

#endif
