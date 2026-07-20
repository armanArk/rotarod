#include "w25q64.h"
#include <string.h>

/* ==================== PIN CS ==================== */
#define W25Q_CS_PORT   FLASH_CS_GPIO_Port
#define W25Q_CS_PIN    FLASH_CS_Pin

// Deklarasi handle SPI dari main.c
extern SPI_HandleTypeDef hspi1;

/* ==================== PRIVATE FUNGSI ==================== */
static inline void W25Q_CS_Low(void) {
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_RESET);
}
static inline void W25Q_CS_High(void) {
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
}

static uint8_t W25Q_SPI_Transceive(uint8_t tx) {
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* ==================== PUBLIC FUNGSI ==================== */
void W25Q64_Init(void) {
    W25Q_CS_High();          // idle
    HAL_Delay(10);           // stabil
}

uint32_t W25Q64_ReadID(void) {
    uint32_t id = 0;
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_READ_ID);
    uint8_t b1 = W25Q_SPI_Transceive(0xFF);
    uint8_t b2 = W25Q_SPI_Transceive(0xFF);
    uint8_t b3 = W25Q_SPI_Transceive(0xFF);
    W25Q_CS_High();
    id = ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
    return id;
}

uint8_t W25Q64_ReadStatusRegister1(void) {
    uint8_t sr;
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_RDSR);
    sr = W25Q_SPI_Transceive(0xFF);
    W25Q_CS_High();
    return sr;
}

void W25Q64_WriteEnable(void) {
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_WREN);
    W25Q_CS_High();
}

void W25Q64_WriteDisable(void) {
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_WRDI);
    W25Q_CS_High();
}

void W25Q64_WaitBusy(void) {
    while (W25Q64_ReadStatusRegister1() & 0x01) {
        // busy, tunggu
    }
}

void W25Q64_SectorErase(uint32_t addr) {
    W25Q64_WriteEnable();
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_SE);
    W25Q_SPI_Transceive((addr >> 16) & 0xFF);
    W25Q_SPI_Transceive((addr >> 8) & 0xFF);
    W25Q_SPI_Transceive(addr & 0xFF);
    W25Q_CS_High();
    W25Q64_WaitBusy();
}

void W25Q64_ReadData(uint32_t addr, uint8_t *buffer, uint32_t length) {
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_READ);
    W25Q_SPI_Transceive((addr >> 16) & 0xFF);
    W25Q_SPI_Transceive((addr >> 8) & 0xFF);
    W25Q_SPI_Transceive(addr & 0xFF);
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = W25Q_SPI_Transceive(0xFF);
    }
    W25Q_CS_High();
}

void W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint32_t length) {
    if (length > 256) length = 256; // batasi 1 page
    W25Q64_WriteEnable();
    W25Q_CS_Low();
    W25Q_SPI_Transceive(W25Q_CMD_PP);
    W25Q_SPI_Transceive((addr >> 16) & 0xFF);
    W25Q_SPI_Transceive((addr >> 8) & 0xFF);
    W25Q_SPI_Transceive(addr & 0xFF);
    for (uint32_t i = 0; i < length; i++) {
        W25Q_SPI_Transceive(data[i]);
    }
    W25Q_CS_High();
    W25Q64_WaitBusy();
}
