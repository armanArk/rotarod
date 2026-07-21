#include "staging.h"
#include "w25q64.h"
#include "fatfs.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

// Simple sector-per-record staging using last N sectors of the flash.
// Keeps a header in the final sector with magic and next index.

#define STAGING_SECTORS 64
#define SECTOR_SIZE 4096
#define PAGE_SIZE 256
#define STG_MAGIC 0x53544731 // 'STG1'

static uint32_t s_flash_bytes = 0;
static uint32_t s_total_sectors = 0;
static uint32_t s_staging_start_sector = 0; // sector number
static uint32_t s_next_free = 0;

static uint32_t simple_crc(const uint8_t *data, uint32_t len) {
    uint32_t s = 0;
    for (uint32_t i = 0; i < len; i++) s += data[i];
    return s;
}

int staging_init(void) {
    uint32_t id = W25Q64_ReadID();
    uint8_t cap = id & 0xFF;
    s_flash_bytes = (cap == 0x17) ? 8*1024*1024 : (cap == 0x18) ? 16*1024*1024 : (cap==0x19)?32*1024*1024:8*1024*1024;
    s_total_sectors = s_flash_bytes / SECTOR_SIZE;
    if (s_total_sectors <= STAGING_SECTORS) return -1;
    s_staging_start_sector = s_total_sectors - STAGING_SECTORS;

    // Read header sector (last sector)
    uint8_t buf[SECTOR_SIZE];
    uint32_t hdr_addr = (s_total_sectors - 1) * SECTOR_SIZE;
    W25Q64_ReadData(hdr_addr, buf, SECTOR_SIZE);
    uint32_t magic = buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24);
    if (magic != STG_MAGIC) {
        s_next_free = 0;
        return 0;
    }
    s_next_free = buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24);
    if (s_next_free > (STAGING_SECTORS-1)) s_next_free = 0;
    return 0;
}

static int staging_write_sector(uint32_t sector_idx, const uint8_t *data) {
    uint32_t addr = (s_staging_start_sector + sector_idx) * SECTOR_SIZE;
    W25Q64_SectorErase(addr);
    for (uint32_t off = 0; off < SECTOR_SIZE; off += PAGE_SIZE) {
        W25Q64_PageProgram(addr + off, (uint8_t*)(data + off), PAGE_SIZE);
    }
    return 0;
}

int staging_append(const uint8_t *data, uint32_t len) {
    if (len > (SECTOR_SIZE - 16)) return -1; // too large
    if (s_next_free >= (STAGING_SECTORS-1)) return -2; // full

    uint8_t sector[SECTOR_SIZE];
    memset(sector, 0xFF, SECTOR_SIZE);
    // header: magic(4) seq(4) len(4) crc(4) = 16 bytes
    sector[0] = (uint8_t)(STG_MAGIC & 0xFF);
    sector[1] = (uint8_t)((STG_MAGIC>>8)&0xFF);
    sector[2] = (uint8_t)((STG_MAGIC>>16)&0xFF);
    sector[3] = (uint8_t)((STG_MAGIC>>24)&0xFF);
    // seq
    sector[4] = (uint8_t)(s_next_free & 0xFF);
    sector[5] = (uint8_t)((s_next_free>>8)&0xFF);
    sector[6] = (uint8_t)((s_next_free>>16)&0xFF);
    sector[7] = (uint8_t)((s_next_free>>24)&0xFF);
    // len
    sector[8] = (uint8_t)(len & 0xFF);
    sector[9] = (uint8_t)((len>>8)&0xFF);
    sector[10] = (uint8_t)((len>>16)&0xFF);
    sector[11] = (uint8_t)((len>>24)&0xFF);
    uint32_t crc = simple_crc(data, len);
    sector[12] = (uint8_t)(crc & 0xFF);
    sector[13] = (uint8_t)((crc>>8)&0xFF);
    sector[14] = (uint8_t)((crc>>16)&0xFF);
    sector[15] = (uint8_t)((crc>>24)&0xFF);
    memcpy(sector + 16, data, len);

    staging_write_sector(s_next_free, sector);

    // update header (last sector)
    s_next_free++;
    uint8_t hdr[SECTOR_SIZE];
    memset(hdr, 0xFF, SECTOR_SIZE);
    hdr[0] = (uint8_t)(STG_MAGIC & 0xFF);
    hdr[1] = (uint8_t)((STG_MAGIC>>8)&0xFF);
    hdr[2] = (uint8_t)((STG_MAGIC>>16)&0xFF);
    hdr[3] = (uint8_t)((STG_MAGIC>>24)&0xFF);
    hdr[4] = (uint8_t)(s_next_free & 0xFF);
    hdr[5] = (uint8_t)((s_next_free>>8)&0xFF);
    hdr[6] = (uint8_t)((s_next_free>>16)&0xFF);
    hdr[7] = (uint8_t)((s_next_free>>24)&0xFF);
    staging_write_sector(STAGING_SECTORS-1, hdr);

    return 0;
}

int staging_has_entries(void) {
    return (s_next_free > 0);
}

int staging_commit(void) {
    // replay entries into filesystem - caller must ensure FS is mounted
    FIL f;
    UINT bw;
    FRESULT fr;

    for (uint32_t i = 0; i < s_next_free; i++) {
        uint8_t sector[SECTOR_SIZE];
        uint32_t addr = (s_staging_start_sector + i) * SECTOR_SIZE;
        W25Q64_ReadData(addr, sector, SECTOR_SIZE);
        uint32_t magic = sector[0] | (sector[1]<<8) | (sector[2]<<16) | (sector[3]<<24);
        if (magic != STG_MAGIC) continue;
        uint32_t len = sector[8] | (sector[9]<<8) | (sector[10]<<16) | (sector[11]<<24);
        uint32_t crc = sector[12] | (sector[13]<<8) | (sector[14]<<16) | (sector[15]<<24);
        if (len == 0 || len > (SECTOR_SIZE-16)) continue;
        uint32_t calc = simple_crc(sector + 16, len);
        if (calc != crc) continue;

        // Append payload to CSV
        fr = f_open(&f, "ROTAROD.CSV", FA_OPEN_APPEND | FA_WRITE);
        if (fr == FR_OK) {
            f_write(&f, sector + 16, len, &bw);
            f_close(&f);
        } else {
            char dbg[64]; sprintf(dbg, "Staging replay open err: %d\r\n", fr); UART_Print(dbg);
            return -1;
        }
    }

    // Clear header marking entries consumed
    uint8_t hdr[SECTOR_SIZE]; memset(hdr, 0xFF, SECTOR_SIZE);
    staging_write_sector(STAGING_SECTORS-1, hdr);
    s_next_free = 0;
    // erase staging sectors
    for (uint32_t i = 0; i < (STAGING_SECTORS-1); i++) {
        uint32_t addr = (s_staging_start_sector + i) * SECTOR_SIZE;
        W25Q64_SectorErase(addr);
    }
    return 0;
}
