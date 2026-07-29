#include "staging.h"
#include "fatfs.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#include "eeprom.h"

/* RAM-backed staging buffer (avoids using tail of flash, which overlaps FAT partitions). */

#define STAGING_MAX_ENTRIES 64
#define STAGING_MAX_LEN     48

#define EEPROM_MAGIC_1 0xEE
#define EEPROM_MAGIC_2 0x11
#define ADDR_MAGIC     0x0000
#define ADDR_COUNT     0x0002
#define ADDR_LENGTHS   0x0010
#define ADDR_PAYLOADS  (ADDR_LENGTHS + (STAGING_MAX_ENTRIES * 4))

static uint8_t s_payloads[STAGING_MAX_ENTRIES][STAGING_MAX_LEN];
static uint32_t s_lengths[STAGING_MAX_ENTRIES];
static uint32_t s_count = 0;

int staging_init(void) {
    s_count = 0;
    
    if (EEPROM_IsReady()) {
        uint8_t magic[2];
        EEPROM_Read(ADDR_MAGIC, magic, 2);
        
        if (magic[0] == EEPROM_MAGIC_1 && magic[1] == EEPROM_MAGIC_2) {
            uint8_t count = 0;
            EEPROM_Read(ADDR_COUNT, &count, 1);
            if (count > 0 && count <= STAGING_MAX_ENTRIES) {
                s_count = count;
                EEPROM_Read(ADDR_LENGTHS, (uint8_t*)s_lengths, count * 4);
                EEPROM_Read(ADDR_PAYLOADS, (uint8_t*)s_payloads, count * STAGING_MAX_LEN);
                
                char dbg[64];
                sprintf(dbg, "Restored %d events from EEPROM\r\n", count);
                UART_Print(dbg);
            }
        }
    }

    return 0;
}

int staging_append(const uint8_t *data, uint32_t len) {
    if (data == NULL || len == 0 || len > STAGING_MAX_LEN) return -1;
    if (s_count >= STAGING_MAX_ENTRIES) return -2;

    memcpy(s_payloads[s_count], data, len);
    s_lengths[s_count] = len;
    
    if (EEPROM_IsReady()) {
        if (s_count == 0) {
            uint8_t magic[2] = {EEPROM_MAGIC_1, EEPROM_MAGIC_2};
            EEPROM_Write(ADDR_MAGIC, magic, 2);
        }
        EEPROM_Write(ADDR_PAYLOADS + (s_count * STAGING_MAX_LEN), data, len);
        EEPROM_Write(ADDR_LENGTHS + (s_count * 4), (uint8_t*)&s_lengths[s_count], 4);
        
        uint8_t count = s_count + 1;
        EEPROM_Write(ADDR_COUNT, &count, 1);
    }

    s_count++;
    return 0;
}

int staging_has_entries(void) {
    return (s_count > 0) ? 1 : 0;
}

int staging_commit(void) {
    if (s_count == 0) return 0;

    FIL f;
    UINT bw;
    FRESULT fr;

    fr = f_open(&f, "ROTAROD.CSV", FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        char dbg[64];
        sprintf(dbg, "Staging replay open err: %d\r\n", fr);
        UART_Print(dbg);
        return -1;
    }

    for (uint32_t i = 0; i < s_count; i++) {
        fr = f_write(&f, s_payloads[i], s_lengths[i], &bw);
        if (fr != FR_OK || bw != s_lengths[i]) {
            char dbg[96];
            sprintf(dbg, "Staging write failed idx=%lu fr=%d bw=%lu expected=%lu\r\n",
                    (unsigned long)i, fr, (unsigned long)bw, (unsigned long)s_lengths[i]);
            UART_Print(dbg);
            f_close(&f);
            return -2;
        }
    }

    f_sync(&f);
    f_close(&f);

    FILINFO finfo;
    if (f_stat("ROTAROD.CSV", &finfo) == FR_OK) {
        char dbg[96];
        sprintf(dbg, "ROTAROD.CSV size after commit: %lu bytes\r\n", (unsigned long)finfo.fsize);
        UART_Print(dbg);
    }

    if (EEPROM_IsReady()) {
        uint8_t count = 0;
        EEPROM_Write(ADDR_COUNT, &count, 1);
    }

    s_count = 0;
    return 0;
}

