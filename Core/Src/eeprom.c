#include "eeprom.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern void UART_Print(char *msg);

static uint8_t eeprom_addr = 0;
static uint8_t eeprom_ready = 0;

#define EEPROM_PAGE_SIZE 32

void EEPROM_Init(void) {
    eeprom_ready = 0;
    
    // Auto-detect EEPROM address (typically 0x57 for DS3231 modules, but can be 0x50 to 0x57)
    for (uint8_t addr = 0x50; addr <= 0x57; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
            eeprom_addr = addr;
            eeprom_ready = 1;
            char dbg[48];
            sprintf(dbg, "EEPROM detected at 0x%02X\r\n", addr);
            UART_Print(dbg);
            return;
        }
    }
    
    UART_Print("EEPROM not detected!\r\n");
}

uint8_t EEPROM_IsReady(void) {
    return eeprom_ready;
}

void EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size) {
    if (!eeprom_ready) return;
    
    // AT24C32 uses 16-bit memory address
    HAL_I2C_Mem_Read(&hi2c1, eeprom_addr << 1, addr, I2C_MEMADD_SIZE_16BIT, data, size, 100);
}

void EEPROM_Write(uint16_t addr, const uint8_t *data, uint16_t size) {
    if (!eeprom_ready) return;

    while (size > 0) {
        // Calculate space remaining on the current 32-byte page
        uint16_t page_offset = addr % EEPROM_PAGE_SIZE;
        uint16_t space_on_page = EEPROM_PAGE_SIZE - page_offset;
        
        // Write max what can fit in the current page
        uint16_t chunk = (size < space_on_page) ? size : space_on_page;
        
        HAL_I2C_Mem_Write(&hi2c1, eeprom_addr << 1, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, chunk, 100);
        
        // EEPROM requires up to 5ms to complete the internal write cycle
        HAL_Delay(5);
        
        addr += chunk;
        data += chunk;
        size -= chunk;
    }
}
