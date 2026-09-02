#include "fs_logger.h"
#include "main.h"
#include "w25q64.h"
#include "staging.h"
#include "cli.h"
#include <string.h>
#include <stdio.h>
#include "ds3231.h"

#define MAX_EVENTS          400
#define MBR_PARTITION2_PTE  462U

extern FATFS USERFatFS;
extern char USERPath[4];


static FIL csv_file;
static UINT bw;
static FRESULT fr;
static uint8_t fs_mounted = 0, fs_formatted = 0;
static uint32_t flash_capacity_bytes = 0;

typedef struct { char timestamp[24]; uint32_t duration_ms; uint16_t rpm; uint8_t lane; } FallEvent_t;
static FallEvent_t event_queue[MAX_EVENTS];
static uint16_t event_count = 0;
static uint8_t csv_header_written = 0;

void FS_SetFlashCapacity(uint32_t capacity_bytes) {
    flash_capacity_bytes = capacity_bytes;
}

uint32_t FS_GetFlashCapacity(void) {
    return flash_capacity_bytes;
}

uint8_t FS_IsMounted(void) {
    return fs_mounted;
}

uint8_t FS_IsFormatted(void) {
    return fs_formatted;
}

void UnmountAllFS(void) {
    f_mount(NULL, USERPath, 0);
    fs_mounted = 0;
    UART_Print("FatFS unmounted\r\n");
}

void MountFS(void) {
    if (fs_mounted) return;
    fr = f_mount(&USERFatFS, USERPath, 1);
    if (fr == FR_OK) {
        fs_mounted = 1;
        UART_Print("FatFS mounted OK\r\n");
        // Ensure volume label is ROTAROD
        FRESULT lab_fr = f_setlabel("ROTAROD");
        if (lab_fr == FR_OK) UART_Print("Volume label set to ROTAROD\r\n");
        else { char msg[48]; sprintf(msg, "Set label err: %d\r\n", lab_fr); UART_Print(msg); }
    } else if (fr == FR_NO_FILESYSTEM) {
        UART_Print("No FS - needs format\r\n");
    } else {
        char msg[32]; sprintf(msg, "Mount err: %d\r\n", fr); UART_Print(msg);
    }
}

static uint8_t VerifyBootSector(void) {
    uint8_t boot[512];
    W25Q64_ReadData(0, boot, 512);
    // Check boot signature 0x55 0xAA at offset 510-511
    if (boot[510] == 0x55 && boot[511] == 0xAA) {
        UART_Print("Boot sector valid (0x55AA)\r\n");
        return 1;
    }
    char dbg[48];
    sprintf(dbg, "Boot sector invalid: %02X %02X\r\n", boot[510], boot[511]);
    UART_Print(dbg);
    return 0;
}

void FormatFS(void) {
    UART_Print("Starting full flash format...\r\n");
    UART_Print("Chip erase (wait ~10-20s)...\r\n");
    W25Q64_ChipErase();
    if (flash_capacity_bytes == 0) {
        UART_Print("Format fail: Capacity 0\r\n");
        return;
    }

    UART_Print("Formatting 1 partition (MBR)...\r\n");
    BYTE work[_MAX_SS];
    FRESULT fmt_fr;
    DWORD part_sizes[2] = {0, 0};
    DWORD total_sectors = flash_capacity_bytes / 512;
    
    part_sizes[0] = total_sectors;
    part_sizes[1] = 0;

    char dbg[96];
    sprintf(dbg, "Partition size: %lu sectors\r\n", part_sizes[0]);
    UART_Print(dbg);

    // Format directly as SFD (Super Floppy Disk) - No MBR.
    // This is much more compatible with Windows for removable USB drives
    // and avoids Windows caching the old 2-partition MBR layout!
    fmt_fr = f_mkfs(USERPath, FM_ANY | FM_SFD, 0, work, sizeof(work));
    sprintf(dbg, "f_mkfs SFD ret=%d\r\n", fmt_fr);
    UART_Print(dbg);

    if (fmt_fr == FR_OK && VerifyBootSector()) {
        UART_Print("Single-partition format OK\r\n");
        fs_formatted = 1;

        for (int i = 0; i < 5; i++) {
            fr = f_mount(&USERFatFS, USERPath, 1);
            if (fr == FR_OK) {
                fs_mounted = 1;
                FRESULT lab_fr = f_setlabel("ROTAROD");
                if (lab_fr == FR_OK) UART_Print("Volume label set to ROTAROD\r\n");
                else {
                    char msg[48]; sprintf(msg, "Set label err: %d\r\n", lab_fr); UART_Print(msg);
                }
                break;
            }
            HAL_Delay(50);
        }

        if (!fs_mounted) {
            char msg[48]; sprintf(msg, "Mount after format failed: %d\r\n", fr); UART_Print(msg);
        }
        return;
    }

    sprintf(dbg, "Format failed, ret=%d\r\n", fmt_fr);
    UART_Print(dbg);
}

void CheckAndFormatIfMismatch(void) {
    uint32_t capacity = FS_GetFlashCapacity();
    if (capacity == 0) {
        UART_Print("Flash capacity 0, skipping check\r\n");
        return;
    }

    uint8_t mbr[512];
    W25Q64_ReadData(0, mbr, 512);

    if (mbr[510] != 0x55 || mbr[511] != 0xAA) {
        UART_Print("No valid MBR signature, formatting...\r\n");
        FormatFS();
        return;
    }

    const uint8_t *pte = &mbr[446];
    uint32_t sector_count = (uint32_t)pte[12] | ((uint32_t)pte[13] << 8) |
                            ((uint32_t)pte[14] << 16) | ((uint32_t)pte[15] << 24);

    uint32_t expected_sectors = capacity / 512;
    // Allow a small margin of error for MBR overhead
    uint32_t diff = (sector_count > expected_sectors) ? (sector_count - expected_sectors) : (expected_sectors - sector_count);

    char dbg[96];
    sprintf(dbg, "Check: p1_sectors=%lu expected=%lu\r\n", sector_count, expected_sectors);
    UART_Print(dbg);

    if (diff > 4000) {
        UART_Print("Capacity mismatch, reformatting for 1 partition...\r\n");
        UnmountAllFS();
        FormatFS();
    } else {
        UART_Print("Partition matches full capacity, skip format\r\n");
    }
}

// WriteCSVHeader removed, checking f_size directly inside Log_FlushToCSV instead


void Log_Init(void) { 
    event_count = 0; 
    csv_header_written = 0; 
}

void Log_AddEvent(uint32_t duration_ms, uint16_t rpm_val, uint8_t lane) {
    if (event_count >= MAX_EVENTS) { UART_Print("Queue full!\r\n"); return; }
    FallEvent_t *ev = &event_queue[event_count++];
    
    uint8_t sec, min, hour, day, date, month, year;
    DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);
    // Simpan tanggal dan waktu secara polos (tanpa trik ="" untuk Excel) 
    // agar kompatibel dengan WPS Office, LibreOffice, atau versi Excel lainnya.
    sprintf(ev->timestamp, "%02d/%02d/%02d,%02d:%02d:%02d", date, month, year, hour, min, sec);
    
    ev->duration_ms = duration_ms;
    ev->rpm = rpm_val;
    ev->lane = lane;
    char msg[128];
    sprintf(msg, "Event: time=%s, dur=%lu, rpm=%u, lane=%u\r\n", ev->timestamp, (unsigned long)duration_ms, rpm_val, lane);
    UART_Print(msg);
}

void Log_FlushToCSV(void) {
    if (event_count == 0) { UART_Print("No events.\r\n"); return; }
    
    if (!fs_mounted) {
        Log_StageEvents();
        return;
    }

    fr = f_open(&csv_file, "ROTAROD.CSV", FA_WRITE | FA_OPEN_APPEND);
    if (fr == FR_OK) {
        if (f_size(&csv_file) == 0) {
            char header[] = "Date (DD/MM/YY),Time (HH:MM:SS),Duration (s),RPM,Lane\r\n";
            f_write(&csv_file, header, strlen(header), &bw);
            csv_header_written = 1;
            UART_Print("CSV header written\r\n");
        }
        for (int i = 0; i < event_count; i++) {
            char line[128];
            int len = sprintf(line, "%s,%lu.%03lu,%u,%u\r\n", 
                              event_queue[i].timestamp, 
                              (unsigned long)(event_queue[i].duration_ms / 1000), 
                              (unsigned long)(event_queue[i].duration_ms % 1000), 
                              event_queue[i].rpm, 
                              event_queue[i].lane);
            f_write(&csv_file, line, len, &bw);
        }
        f_close(&csv_file);
        event_count = 0;
        UART_Print("Flushed to CSV\r\n");
    } else {
        char msg[32]; 
        sprintf(msg, "CSV open err: %d\r\n", fr); 
        UART_Print(msg);
    }
}

void Log_ClearCSV(void) {
    if (!fs_mounted) {
        UART_Print("FS not mounted, cannot clear CSV. Please disconnect USB.\r\n");
        return;
    }
    
    // Open with FA_CREATE_ALWAYS to truncate the file to 0 bytes
    fr = f_open(&csv_file, "ROTAROD.CSV", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        char header[] = "Date (DD/MM/YY),Time (HH:MM:SS),Duration (s),RPM,Lane\r\n";
        f_write(&csv_file, header, strlen(header), &bw);
        f_close(&csv_file);
        
        csv_header_written = 1;
        event_count = 0;
        
        // Also clear EEPROM staging so no old events are restored
        staging_clear();
        
        UART_Print("CSV data cleared successfully.\r\n");
    } else {
        char msg[32]; 
        sprintf(msg, "CSV clear err: %d\r\n", fr); 
        UART_Print(msg);
    }
}

void Log_StageEvents(void) {
    if (event_count == 0) { UART_Print("No events to stage.\r\n"); return; }
    
    if (!csv_header_written) {
        const char header[] = "Date (DD/MM/YY),Time (HH:MM:SS),Duration (s),RPM,Lane\r\n";
        staging_append((const uint8_t*)header, (uint32_t)strlen(header));
        csv_header_written = 1;
        UART_Print("CSV header staged\r\n");
    }

    for (int i = 0; i < event_count; i++) {
        char line[128];
        int len = sprintf(line, "%s,%lu.%03lu,%u,%u\r\n", 
                          event_queue[i].timestamp, 
                          (unsigned long)(event_queue[i].duration_ms / 1000), 
                          (unsigned long)(event_queue[i].duration_ms % 1000), 
                          event_queue[i].rpm, 
                          event_queue[i].lane);
        if (staging_append((const uint8_t*)line, (uint32_t)len) != 0) {
            UART_Print("Staging append failed\r\n");
            return;
        }
    }
    event_count = 0;
    UART_Print("Staged events to flash\r\n");
}

void Log_ReadCSV(void) {
    if (!fs_mounted) { UART_Print("FS not mounted\r\n"); return; }
    fr = f_open(&csv_file, "ROTAROD.CSV", FA_READ);
    if (fr == FR_OK) {
        char buf[256]; UINT br;
        UART_Print("\r\n=== CSV ===\r\n");
        while (f_read(&csv_file, buf, sizeof(buf)-1, &br) == FR_OK && br > 0) {
            buf[br] = '\0'; UART_Print(buf);
        }
        UART_Print("\r\n=== End ===\r\n");
        f_close(&csv_file);
    } else {
        UART_Print("Cannot open CSV\r\n");
    }
}

uint16_t Log_GetEventCount(void) {
    return event_count;
}
