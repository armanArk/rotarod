#include "ds3231.h"
#include <stdio.h>
#include <string.h>

// Deklarasi handle I2C dari main.c
extern I2C_HandleTypeDef hi2c1;
extern void UART_Print(char *msg); // dari main.c

// ====== Helper ======
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// ====== Public ======
void DS3231_Init(void) {
    // Cek koneksi dengan membaca register detik
    uint8_t test = DS3231_ReadByte(DS3231_REG_SEC);
    if (test == 0xFF) {
        UART_Print("DS3231 not detected!\r\n");
    } else {
        UART_Print("DS3231 detected.\r\n");
    }
}

uint8_t DS3231_ReadByte(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

void DS3231_WriteByte(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void DS3231_ReadTime(uint8_t *sec, uint8_t *min, uint8_t *hour,
                     uint8_t *day, uint8_t *date, uint8_t *month, uint8_t *year) {
    uint8_t buffer[7];
    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR << 1, DS3231_REG_SEC, I2C_MEMADD_SIZE_8BIT, buffer, 7, HAL_MAX_DELAY);
    *sec   = bcd_to_dec(buffer[0] & 0x7F); // bit 7 = OSC stop flag
    *min   = bcd_to_dec(buffer[1] & 0x7F);
    *hour  = bcd_to_dec(buffer[2] & 0x3F); // 24-hour mode
    *day   = bcd_to_dec(buffer[3] & 0x07);
    *date  = bcd_to_dec(buffer[4] & 0x3F);
    *month = bcd_to_dec(buffer[5] & 0x1F);
    *year  = bcd_to_dec(buffer[6] & 0xFF);
}

void DS3231_SetTime(uint8_t sec, uint8_t min, uint8_t hour,
                    uint8_t day, uint8_t date, uint8_t month, uint8_t year) {
    uint8_t buffer[7];
    buffer[0] = dec_to_bcd(sec);
    buffer[1] = dec_to_bcd(min);
    buffer[2] = dec_to_bcd(hour);
    buffer[3] = dec_to_bcd(day);
    buffer[4] = dec_to_bcd(date);
    buffer[5] = dec_to_bcd(month);
    buffer[6] = dec_to_bcd(year);
    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR << 1, DS3231_REG_SEC, I2C_MEMADD_SIZE_8BIT, buffer, 7, HAL_MAX_DELAY);
}

float DS3231_ReadTemperature(void) {
    uint8_t msb = DS3231_ReadByte(DS3231_REG_TEMP);
    uint8_t lsb = DS3231_ReadByte(DS3231_REG_TEMP + 1);
    float temp = msb + (float)(lsb >> 6) * 0.25f;
    return temp;
}

void DS3231_PrintTime(void) {
    uint8_t sec, min, hour, day, date, month, year;
    DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);
    char buf[64];
    sprintf(buf, "Time: %02d:%02d:%02d  Date: %02d/%02d/%02d  Day:%d\r\n",
            hour, min, sec, date, month, year, day);
    UART_Print(buf);
}
