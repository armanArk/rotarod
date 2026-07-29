#include "fs_logger.h"
#include "main.h"
#include "w25q64.h"
#include "staging.h"
#include "cli.h"
#include <string.h>
#include <stdio.h>

#define MAX_EVENTS          400
#define MBR_PARTITION2_PTE  462U

extern FATFS USERFatFS;
extern FATFS USERFatFS2;
extern char USERPath2[4];
extern char USERPath[4];

static FIL csv_file;
static UINT bw;
static FRESULT fr;
static uint8_t fs_mounted = 0, fs_formatted = 0;
static uint32_t flash_capacity_bytes = 0;

typedef struct { uint32_t timestamp; uint16_t duration_ms, rpm; uint8_t lane; } FallEvent_t;
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

uint8_t ReadExportPartition(uint32_t *lba_start, uint32_t *sector_count) {
    uint8_t mbr[512];
    W25Q64_ReadData(0, mbr, 512);
    if (mbr[510] != 0x55 || mbr[511] != 0xAA) return 0;

    const uint8_t *pte = &mbr[MBR_PARTITION2_PTE];
    if (pte[4] == 0) return 0;

    *lba_start = (uint32_t)pte[8] | ((uint32_t)pte[9] << 8) |
                 ((uint32_t)pte[10] << 16) | ((uint32_t)pte[11] << 24);
    *sector_count = (uint32_t)pte[12] | ((uint32_t)pte[13] << 8) |
                    ((uint32_t)pte[14] << 16) | ((uint32_t)pte[15] << 24);
    return (*sector_count > 0) ? 1 : 0;
}

void UnmountAllFS(void) {
    f_mount(NULL, USERPath, 0);
    f_mount(NULL, USERPath2, 0);
    fs_mounted = 0;
    UART_Print("FatFS unmounted\r\n");
}

FRESULT ExportCsvToPartition1(void) {
    FIL src, dst;
    UINT br, bw_local;
    uint8_t buf[512];
    FRESULT res;

    res = f_mount(&USERFatFS2, USERPath2, 1);
    if (res != FR_OK) {
        char msg[48];
        sprintf(msg, "Export mount 1: err: %d\r\n", res);
        UART_Print(msg);
        return res;
    }

    res = f_open(&src, "0:ROTAROD.CSV", FA_READ);
    if (res == FR_NO_FILE) {
        res = f_open(&dst, "1:ROTAROD.CSV", FA_WRITE | FA_CREATE_ALWAYS);
        if (res == FR_OK) {
            const char *header = "timestamp,duration_ms,rpm,lane\r\n";
            f_write(&dst, header, strlen(header), &bw_local);
            f_sync(&dst);
            f_close(&dst);
            UART_Print("Export: empty CSV with header\r\n");
        }
        f_mount(NULL, USERPath2, 0);
        return res;
    }
    if (res != FR_OK) {
        char msg[48];
        sprintf(msg, "Export open 0: err: %d\r\n", res);
        UART_Print(msg);
        f_mount(NULL, USERPath2, 0);
        return res;
    }

    res = f_open(&dst, "1:ROTAROD.CSV", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        f_close(&src);
        f_mount(NULL, USERPath2, 0);
        return res;
    }

    while (f_read(&src, buf, sizeof(buf), &br) == FR_OK && br > 0) {
        if (f_write(&dst, buf, br, &bw_local) != FR_OK || bw_local != br) {
            res = FR_DISK_ERR;
            break;
        }
    }

    f_close(&src);
    f_sync(&dst);
    f_close(&dst);
    f_mount(NULL, USERPath2, 0);

    if (res == FR_OK) UART_Print("Export to partition 1 OK\r\n");
    else UART_Print("Export to partition 1 failed\r\n");
    return res;
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
    UART_Print("Formatting flash with 2 partitions (50%/50%)...\r\n");
    UART_Print("Chip erase (wait ~10-20s)...\r\n");
    W25Q64_ChipErase();
    UART_Print("Chip erase done\r\n");

    // Clear FatFs cache
    f_mount(NULL, USERPath, 0);
    f_mount(NULL, USERPath2, 0);
    fs_mounted = 0;

    BYTE work[_MAX_SS];
    FRESULT fmt_fr;
    DWORD part_sizes[2] = {0, 0};
    DWORD total_sectors = flash_capacity_bytes / 512;
    if (total_sectors > 1) {
        part_sizes[0] = total_sectors / 2;
        part_sizes[1] = total_sectors - part_sizes[0];
    } else {
        part_sizes[0] = 1;
        part_sizes[1] = 0;
    }

    char dbg[96];
    sprintf(dbg, "Partition sizes: %lu / %lu sectors\r\n", part_sizes[0], part_sizes[1]);
    UART_Print(dbg);

    fmt_fr = f_fdisk(0, part_sizes, work);
    sprintf(dbg, "f_fdisk ret=%d\r\n", fmt_fr);
    UART_Print(dbg);

    if (fmt_fr == FR_OK) {
        // Map logical drive 0 to partition 1 and logical drive 1 to partition 2.
        extern PARTITION VolToPart[];
        VolToPart[0].pt = 1;
        VolToPart[1].pt = 2;

        fmt_fr = f_mkfs(USERPath, FM_ANY | FM_FAT32, 0, work, sizeof(work));
        sprintf(dbg, "f_mkfs(0:) ret=%d\r\n", fmt_fr);
        UART_Print(dbg);

        if (fmt_fr == FR_OK) {
            fmt_fr = f_mkfs(USERPath2, FM_ANY | FM_FAT32, 0, work, sizeof(work));
            sprintf(dbg, "f_mkfs(1:) ret=%d\r\n", fmt_fr);
            UART_Print(dbg);
        }
    }

    if (fmt_fr == FR_OK && VerifyBootSector()) {
        UART_Print("Two-partition format OK\r\n");
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

        fr = f_mount(&USERFatFS2, USERPath2, 1);
        if (fr == FR_OK) {
            UART_Print("Second partition verified\r\n");
            f_mount(NULL, USERPath2, 0);
        } else {
            char msg[64]; sprintf(msg, "Second partition mount err: %d\r\n", fr); UART_Print(msg);
        }

        if (!fs_mounted) {
            char msg[48]; sprintf(msg, "Mount after format failed: %d\r\n", fr); UART_Print(msg);
        }
        return;
    }

    char msg[64];
    sprintf(msg, "Two-partition format failed, last: %d\r\n", fmt_fr);
    UART_Print(msg);
}

void CheckAndFormatIfMismatch(void) {
    if (!fs_mounted || flash_capacity_bytes == 0) return;

    DWORD fre_clust;
    FATFS *fs_ptr;
    if (f_getfree(USERPath, &fre_clust, &fs_ptr) != FR_OK) return;

    DWORD fs_total_sectors = (fs_ptr->n_fatent - 2) * fs_ptr->csize;
    DWORD expected_sectors = (flash_capacity_bytes / 2) / 512;

    char dbg[120];
    sprintf(dbg, "FS sectors: %lu / expected: %lu (n_fatent=%lu csize=%lu)\r\n",
        fs_total_sectors, expected_sectors, fs_ptr->n_fatent, fs_ptr->csize);
    UART_Print(dbg);

    // Allow tolerance up to one large erase block (64KB = 128 sectors) to account
    // for alignment differences from mkfs.
    const DWORD allowed_diff_sectors = 128;
    if (fs_total_sectors < (expected_sectors - allowed_diff_sectors) ||
        fs_total_sectors > (expected_sectors + allowed_diff_sectors)) {
        UART_Print("FS size mismatch! Reformatting...\r\n");
        UnmountAllFS();
        FormatFS();
    } else {
        UART_Print("FS size OK\r\n");
    }
}

static void WriteCSVHeader(void) {
    if (!fs_mounted) {
        const char header[] = "timestamp,duration_ms,rpm,lane\r\n";
        staging_append((const uint8_t*)header, (uint32_t)strlen(header));
        csv_header_written = 1;
        UART_Print("CSV header staged\r\n");
        return;
    }
    fr = f_open(&csv_file, "ROTAROD.CSV", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        char header[] = "timestamp,duration_ms,rpm,lane\r\n";
        f_write(&csv_file, header, strlen(header), &bw);
        f_close(&csv_file);
        csv_header_written = 1;
        UART_Print("CSV header written\r\n");
    } else {
        char msg[32]; sprintf(msg, "CSV create err: %d\r\n", fr); UART_Print(msg);
    }
}

static void AppendCSVLine(FallEvent_t *ev) {
    char line[64];
    int len = sprintf(line, "%lu,%u,%u,%u\r\n", ev->timestamp, ev->duration_ms, ev->rpm, ev->lane);
    if (!fs_mounted) {
        staging_append((const uint8_t*)line, (uint32_t)len);
        return;
    }
    fr = f_open(&csv_file, "ROTAROD.CSV", FA_WRITE | FA_OPEN_APPEND);
    if (fr == FR_OK) {
        f_write(&csv_file, line, len, &bw);
        f_close(&csv_file);
    }
}

void Log_Init(void) { 
    event_count = 0; 
    csv_header_written = 0; 
}

void Log_AddEvent(uint16_t duration_ms, uint16_t rpm_val, uint8_t lane) {
    if (event_count >= MAX_EVENTS) { UART_Print("Queue full!\r\n"); return; }
    FallEvent_t *ev = &event_queue[event_count++];
    ev->timestamp = HAL_GetTick() / 1000;
    ev->duration_ms = duration_ms;
    ev->rpm = rpm_val;
    ev->lane = lane;
    char msg[64];
    sprintf(msg, "Event: dur=%u, rpm=%u, lane=%u\r\n", duration_ms, rpm_val, lane);
    UART_Print(msg);
}

void Log_FlushToCSV(void) {
    if (event_count == 0) { UART_Print("No events.\r\n"); return; }
    if (!csv_header_written) WriteCSVHeader();
    for (int i = 0; i < event_count; i++) AppendCSVLine(&event_queue[i]);
    event_count = 0;
    UART_Print("Flushed to CSV\r\n");
}

void Log_StageEvents(void) {
    if (event_count == 0) { UART_Print("No events to stage.\r\n"); return; }
    for (int i = 0; i < event_count; i++) {
        char line[64];
        int len = sprintf(line, "%lu,%u,%u,%u\r\n", event_queue[i].timestamp, event_queue[i].duration_ms, event_queue[i].rpm, event_queue[i].lane);
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
