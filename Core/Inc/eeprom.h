#ifndef EEPROM_H
#define EEPROM_H

#include "main.h"

// Initialize and auto-detect EEPROM address
void EEPROM_Init(void);

// Returns 1 if EEPROM is detected, 0 otherwise
uint8_t EEPROM_IsReady(void);

// Read bytes from EEPROM
void EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size);

// Write bytes to EEPROM (handles page boundaries automatically)
void EEPROM_Write(uint16_t addr, const uint8_t *data, uint16_t size);

#endif
