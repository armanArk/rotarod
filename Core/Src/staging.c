#include "staging.h"
#include "fatfs.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* RAM-backed staging buffer (avoids using tail of flash, which overlaps FAT partitions). */

#define STAGING_MAX_ENTRIES 64
#define STAGING_MAX_LEN     128

static uint8_t s_payloads[STAGING_MAX_ENTRIES][STAGING_MAX_LEN];
static uint32_t s_lengths[STAGING_MAX_ENTRIES];
static uint32_t s_count = 0;

int staging_init(void) {
    s_count = 0;
    return 0;
}

int staging_append(const uint8_t *data, uint32_t len) {
    if (data == NULL || len == 0 || len > STAGING_MAX_LEN) return -1;
    if (s_count >= STAGING_MAX_ENTRIES) return -2;

    memcpy(s_payloads[s_count], data, len);
    s_lengths[s_count] = len;
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

    for (uint32_t i = 0; i < s_count; i++) {
        fr = f_open(&f, "ROTAROD.CSV", FA_OPEN_APPEND | FA_WRITE);
        if (fr != FR_OK) {
            char dbg[64];
            sprintf(dbg, "Staging replay open err: %d\r\n", fr);
            UART_Print(dbg);
            return -1;
        }

        fr = f_write(&f, s_payloads[i], s_lengths[i], &bw);
        if (fr != FR_OK || bw != s_lengths[i]) {
            char dbg[96];
            sprintf(dbg, "Staging write failed idx=%lu fr=%d bw=%lu expected=%lu\r\n",
                    (unsigned long)i, fr, (unsigned long)bw, (unsigned long)s_lengths[i]);
            UART_Print(dbg);
            f_close(&f);
            return -2;
        }

        f_sync(&f);
        f_close(&f);
    }

    FILINFO finfo;
    if (f_stat("ROTAROD.CSV", &finfo) == FR_OK) {
        char dbg[96];
        sprintf(dbg, "ROTAROD.CSV size after commit: %lu bytes\r\n", (unsigned long)finfo.fsize);
        UART_Print(dbg);
    }

    s_count = 0;
    return 0;
}
