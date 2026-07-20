#ifndef DS3231_H
#define DS3231_H

#include "main.h"   // Sudah include semua HAL, termasuk I2C

// Alamat I2C DS3231 (7-bit)
#define DS3231_ADDR  0x68

// Register addresses
#define DS3231_REG_SEC    0x00
#define DS3231_REG_MIN    0x01
#define DS3231_REG_HOUR   0x02
#define DS3231_REG_DAY    0x03
#define DS3231_REG_DATE   0x04
#define DS3231_REG_MONTH  0x05
#define DS3231_REG_YEAR   0x06
#define DS3231_REG_TEMP   0x11

// Fungsi
void     DS3231_Init(void);
uint8_t  DS3231_ReadByte(uint8_t reg);
void     DS3231_WriteByte(uint8_t reg, uint8_t data);
void     DS3231_ReadTime(uint8_t *sec, uint8_t *min, uint8_t *hour,
                         uint8_t *day, uint8_t *date, uint8_t *month, uint8_t *year);
void     DS3231_SetTime(uint8_t sec, uint8_t min, uint8_t hour,
                        uint8_t day, uint8_t date, uint8_t month, uint8_t year);
float    DS3231_ReadTemperature(void);
void     DS3231_PrintTime(void); // Cetak ke UART

#endif
