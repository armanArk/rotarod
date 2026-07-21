/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Rotarod + FatFs on W25Q Flash + USB MSC
  *          - CSV visible in Windows Explorer
  *          - Auto unmount/mount on USB plug/unplug
  *          - Auto-reformat if FS size != flash size
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

#include "TM1637.h"
#include "74hc165.h"
#include "w25q64.h"
#include "ds3231.h"
#include "usbd_core.h"
#include "usb_device.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
/* Control MSC readiness/capacity in USB storage layer */
extern void STORAGE_SetMediaReady(uint8_t ready);
extern void STORAGE_UpdateCapacity(uint32_t bytes);
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "staging.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim4, htim5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define HC165_NUM_CHIPS     3
#define HC165_TOTAL_PINS    (HC165_NUM_CHIPS * 8)
#define NUM_DISPLAYS        7
#define DISPLAY_UPDATE_MS   100
#define DEBUG_UPDATE_MS     1000
#define BUTTON_READ_MS      20
#define PWM_MAX_DUTY        3788
#define MAX_EVENTS          400
#define USB_CHECK_MS        100
#define FALL_DEBOUNCE_MS    500

volatile uint32_t timer_overflow = 0, last_capture = 0, rpm = 0, last_capture_time = 0;
uint32_t last_display_tick = 0, last_debug_tick = 0, adc_raw = 0, pwm_duty = 0, set_value = 0;
uint32_t last_button_read_tick = 0, last_usb_check = 0;

uint8_t status_tombol[HC165_NUM_CHIPS], pins[HC165_TOTAL_PINS];

static GPIO_TypeDef* clk_ports[NUM_DISPLAYS] = {
    DISP_CLK1_GPIO_Port, DISP_CLK2_GPIO_Port, DISP_CLK3_GPIO_Port,
    DISP_CLK4_GPIO_Port, DISP_CLK5_GPIO_Port, DISP_CLK6_GPIO_Port, DISP_CLK7_GPIO_Port
};
static uint16_t clk_pins[NUM_DISPLAYS] = {
    DISP_CLK1_Pin, DISP_CLK2_Pin, DISP_CLK3_Pin, DISP_CLK4_Pin,
    DISP_CLK5_Pin, DISP_CLK6_Pin, DISP_CLK7_Pin
};
static GPIO_TypeDef* dio_ports[NUM_DISPLAYS] = {
    DISP_DIO1_GPIO_Port, DISP_DIO2_GPIO_Port, DISP_DIO3_GPIO_Port,
    DISP_DIO4_GPIO_Port, DISP_DIO5_GPIO_Port, DISP_DIO6_GPIO_Port, DISP_DIO7_GPIO_Port
};
static uint16_t dio_pins[NUM_DISPLAYS] = {
    DISP_DIO1_Pin, DISP_DIO2_Pin, DISP_DIO3_Pin, DISP_DIO4_Pin,
    DISP_DIO5_Pin, DISP_DIO6_Pin, DISP_DIO7_Pin
};

static void Display_ShowNumber(uint8_t idx, uint16_t num, uint8_t dots) {
    if (idx >= NUM_DISPLAYS) return;
    TM1637_DisplayNumber(clk_ports[idx], clk_pins[idx], dio_ports[idx], dio_pins[idx], num, dots);
}
static void Display_SetBrightness(uint8_t idx, uint8_t brightness) {
    if (idx >= NUM_DISPLAYS) return;
    TM1637_SetBrightness(clk_ports[idx], clk_pins[idx], dio_ports[idx], dio_pins[idx], brightness);
}

// FatFs
/* Use FatFs objects provided by MX_FATFS (USERFatFS/USERPath) */
extern FATFS USERFatFS;
FIL csv_file;
UINT bw;
FRESULT fr;
static uint8_t fs_mounted = 0, fs_formatted = 0;
static uint32_t flash_capacity_bytes = 0;

typedef struct { uint32_t timestamp; uint16_t duration_ms, rpm; uint8_t lane; } FallEvent_t;
static FallEvent_t event_queue[MAX_EVENTS];
static uint16_t event_count = 0;
static uint8_t csv_header_written = 0;

// USB
uint8_t usb_connected = 0;
static uint8_t last_usb_state = 0;
// VBUS EXTI event pending flag
volatile uint8_t vbus_event_pending = 0;

// Debounce
static uint8_t prev_fall = 0;
static uint32_t last_fall_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
void UART_Print(char *msg);

/* USER CODE BEGIN 0 */

static void MountFS(void) {
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

static void UnmountFS(void) {
    if (!fs_mounted) return;
    f_mount(NULL, USERPath, 0);
    fs_mounted = 0;
    UART_Print("FatFS unmounted\r\n");
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

static void FormatFS(void) {
    UART_Print("Formatting flash...\r\n");
    UART_Print("Chip erase (wait ~10-20s)...\r\n");
    W25Q64_ChipErase();
    UART_Print("Chip erase done\r\n");

    // Clear FatFs cache
    f_mount(NULL, USERPath, 0);
    fs_mounted = 0;

    BYTE work[_MAX_SS];
    FRESULT fmt_fr;

    // Attempt 1: FM_ANY (auto FAT12/16/32) - best for 8-32MB
    // First try: superfloppy (no MBR) so USB host sees FS at LBA0
    UART_Print("Trying FM_ANY|FM_SFD (superfloppy)...\r\n");
    fmt_fr = f_mkfs(USERPath, FM_ANY | FM_SFD, 0, work, sizeof(work));
    if (fmt_fr != FR_OK) {
        UART_Print("Trying FM_ANY...\r\n");
        fmt_fr = f_mkfs(USERPath, FM_ANY, 0, work, sizeof(work));
    }
    {
      char dbg[64]; sprintf(dbg, "f_mkfs FM_ANY ret=%d\r\n", fmt_fr); UART_Print(dbg);
    }
    if (fmt_fr == FR_OK && VerifyBootSector()) {
        UART_Print("Format FM_ANY OK\r\n");
        fs_formatted = 1;
            // Dump boot sector BPB fields for debugging
            {
                uint8_t boot[512];
                W25Q64_ReadData(0, boot, 512);
                uint16_t bytes_per_sector = boot[11] | (boot[12] << 8);
                uint8_t sectors_per_cluster = boot[13];
                uint16_t reserved_sectors = boot[14] | (boot[15] << 8);
                uint8_t num_fats = boot[16];
                uint16_t root_ent = boot[17] | (boot[18] << 8);
                uint16_t tot16 = boot[19] | (boot[20] << 8);
                uint32_t tot32 = boot[32] | (boot[33] << 8) | (boot[34] << 16) | (boot[35] << 24);
                uint32_t total_sectors_bpb = tot16 ? tot16 : tot32;
                char dbg[128];
                sprintf(dbg, "BPB: bps=%u spc=%u rsv=%u fats=%u root=%u tot=%lu\r\n",
                        bytes_per_sector, sectors_per_cluster, reserved_sectors,
                        num_fats, root_ent, (unsigned long)total_sectors_bpb);
                UART_Print(dbg);

                // Hexdump first 64 bytes
                for (int ln = 0; ln < 64; ln += 16) {
                    char line[96]; char *p = line; int off = 0;
                    off += sprintf(p + off, "%02X: ", ln);
                    for (int j = 0; j < 16; j++) off += sprintf(p + off, "%02X ", boot[ln + j]);
                    off += sprintf(p + off, "\r\n");
                    UART_Print(line);
                }
            }
        // Try mounting a few times to allow driver/media settle
        for (int i = 0; i < 5; i++) {
            fr = f_mount(&USERFatFS, USERPath, 1);
            if (fr == FR_OK) {
                fs_mounted = 1; UART_Print("FatFS mounted OK\r\n");
                    // set volume label
                    FRESULT lab_fr = f_setlabel("ROTAROD");
                    if (lab_fr == FR_OK) UART_Print("Volume label set to ROTAROD\r\n");
                    else { char msg[48]; sprintf(msg, "Set label err: %d\r\n", lab_fr); UART_Print(msg); }
                // Print FS details
                DWORD fre_clust; FATFS *fs_tmp;
                if (f_getfree(USERPath, &fre_clust, &fs_tmp) == FR_OK) {
                    DWORD fs_sectors = (fs_tmp->n_fatent - 2) * fs_tmp->csize;
                    char dbg[256]; sprintf(dbg, "Mounted FS: n_fatent=%lu csize=%lu total_sectors=%lu\r\n",
                                            fs_tmp->n_fatent, fs_tmp->csize, fs_sectors);
                    UART_Print(dbg);
                    sprintf(dbg, "FS layout: volbase=%lu fatbase=%lu dirbase=%lu database=%lu fsize=%lu\r\n",
                            fs_tmp->volbase, fs_tmp->fatbase, fs_tmp->dirbase, fs_tmp->database, fs_tmp->fsize);
                    UART_Print(dbg);

                    // Dump BPB at volume base (in case it's not LBA 0)
                    uint8_t vol_boot[512];
                    uint32_t vol_addr = fs_tmp->volbase * 512UL;
                    W25Q64_ReadData(vol_addr, vol_boot, 512);
                    sprintf(dbg, "BPB at volbase (%lu): bps=%u spc=%u rsv=%u fats=%u root=%u tot16=%u tot32=%lu\r\n",
                            (unsigned long)fs_tmp->volbase,
                            vol_boot[11] | (vol_boot[12] << 8), vol_boot[13], vol_boot[14] | (vol_boot[15]<<8),
                            vol_boot[16], vol_boot[17] | (vol_boot[18]<<8),
                            (unsigned long)(vol_boot[32] | (vol_boot[33]<<8) | (vol_boot[34]<<16) | (vol_boot[35]<<24)));
                    UART_Print(dbg);
                    for (int ln = 0; ln < 64; ln += 16) {
                        char line[96]; int off = 0;
                        off += sprintf(line + off, "%02X: ", ln);
                        for (int j = 0; j < 16; j++) off += sprintf(line + off, "%02X ", vol_boot[ln + j]);
                        off += sprintf(line + off, "\r\n");
                        UART_Print(line);
                    }
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

    // Attempt 2: FM_FAT32 (force FAT32) as superfloppy
    UART_Print("Trying FM_FAT32|FM_SFD...\r\n");
    fmt_fr = f_mkfs(USERPath, FM_FAT32 | FM_SFD, 0, work, sizeof(work));
    if (fmt_fr != FR_OK) {
        UART_Print("Trying FM_FAT32...\r\n");
        fmt_fr = f_mkfs(USERPath, FM_FAT32, 0, work, sizeof(work));
    }
    if (fmt_fr == FR_OK && VerifyBootSector()) {
        UART_Print("Format FAT32 OK\r\n");
        fs_formatted = 1;
        MountFS();
        return;
    }

    // Attempt 3: FM_FAT (FAT12/16) with au=8 (4KB clusters), prefer superfloppy
    UART_Print("Trying FM_FAT|FM_SFD au=8...\r\n");
    fmt_fr = f_mkfs(USERPath, FM_FAT | FM_SFD, 8, work, sizeof(work));
    if (fmt_fr != FR_OK) {
        UART_Print("Trying FM_FAT au=8...\r\n");
        fmt_fr = f_mkfs(USERPath, FM_FAT, 8, work, sizeof(work));
    }
    if (fmt_fr == FR_OK && VerifyBootSector()) {
        UART_Print("Format FAT16 OK\r\n");
        fs_formatted = 1;
        MountFS();
        return;
    }

    char msg[48];
    sprintf(msg, "All format attempts failed, last: %d\r\n", fmt_fr);
    UART_Print(msg);
}

static void CheckAndFormatIfMismatch(void) {
    if (!fs_mounted || flash_capacity_bytes == 0) return;

    DWORD fre_clust;
    FATFS *fs_ptr;
        if (f_getfree(USERPath, &fre_clust, &fs_ptr) != FR_OK) return;

        DWORD fs_total_sectors = (fs_ptr->n_fatent - 2) * fs_ptr->csize;
        DWORD expected_sectors = flash_capacity_bytes / 512;

        char dbg[120];
        sprintf(dbg, "FS sectors: %lu / expected: %lu (n_fatent=%lu csize=%lu)\r\n",
            fs_total_sectors, expected_sectors, fs_ptr->n_fatent, fs_ptr->csize);
        UART_Print(dbg);

    // Allow tolerance up to one large erase block (64KB = 128 sectors) to account
    // for alignment differences from mkfs.
    const DWORD allowed_diff_sectors = 128;
    if (fs_total_sectors < (expected_sectors - allowed_diff_sectors)) {
        UART_Print("FS size mismatch! Reformatting...\r\n");
        UnmountFS();
        FormatFS();
    } else {
        UART_Print("FS size OK\r\n");
    }
}

static void WriteCSVHeader(void) {
    if (!fs_mounted) return;
    if (!fs_mounted) {
        // if FS not mounted, create header in staging area
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

static void Log_Init(void) { event_count = 0; csv_header_written = 0; }

static void Log_AddEvent(uint16_t duration_ms, uint16_t rpm_val, uint8_t lane) {
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

static void Log_FlushToCSV(void) {
    if (event_count == 0) { UART_Print("No events.\r\n"); return; }
    if (!csv_header_written) WriteCSVHeader();
    for (int i = 0; i < event_count; i++) AppendCSVLine(&event_queue[i]);
    event_count = 0;
    UART_Print("Flushed to CSV\r\n");
}

static void Log_StageEvents(void) {
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

static void Log_ReadCSV(void) {
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
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    /* Initialize FatFs (disk detection) before USB so MSC reports correct capacity */
    MX_FATFS_Init();
    MX_USB_DEVICE_Init();

    // 74HC165
    HC165_SetPins(SHIFT_PL_GPIO_Port, SHIFT_PL_Pin, SHIFT_CP_GPIO_Port, SHIFT_CP_Pin,
                  SHIFT_Q7_GPIO_Port, SHIFT_Q7_Pin);

    // CLK idle HIGH
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = DISP_CLK2_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin;
    HAL_GPIO_Init(DISP_CLK2_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, DISP_CLK2_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin, GPIO_PIN_SET);

    // Display init
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_SetBrightness(i, 4);
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_ShowNumber(i, (i+1)*1111, 0);

    // TIM4 IC on PB8
    GPIO_InitStruct.Pin = BLDC_PULSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(BLDC_PULSE_GPIO_Port, &GPIO_InitStruct);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    // PWM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

    // ADC
    HAL_ADC_Start(&hadc1);

    // SPI Flash init
    W25Q64_Init();
    uint8_t sr = W25Q64_ReadStatusRegister1();
    if (sr != 0x00) W25Q64_WriteStatusRegister1(0x00);

    uint32_t flash_id = W25Q64_ReadID();
    uint8_t cap_byte = flash_id & 0xFF;
    flash_capacity_bytes = (cap_byte == 0x17) ? 8*1024*1024 :
                           (cap_byte == 0x18) ? 16*1024*1024 :
                           (cap_byte == 0x19) ? 32*1024*1024 : 8*1024*1024;
    char id_buf[80];
    sprintf(id_buf, "Flash ID: 0x%06lX (%lu MB)\r\n", flash_id, flash_capacity_bytes/(1024*1024));
    UART_Print(id_buf);

    // Mount or format FatFs
    MountFS();
    if (!fs_mounted && !fs_formatted) {
        FormatFS();
    } else if (fs_mounted) {
        CheckAndFormatIfMismatch();
    }

    // RTC
    DS3231_Init();

    // Initialize staging area for queued writes while USB attached
    if (staging_init() == 0) UART_Print("Staging initialized\r\n");

    // Logging
    Log_Init();

    last_display_tick = HAL_GetTick();
    last_debug_tick = HAL_GetTick();
    last_capture_time = HAL_GetTick();
    last_button_read_tick = HAL_GetTick();
    last_usb_check = HAL_GetTick();
    prev_fall = 0;
    last_fall_time = 0;

    UART_Print("System Start\r\n");

    while (1)
    {
        // Shift register
        if (HAL_GetTick() - last_button_read_tick >= BUTTON_READ_MS) {
            HC165_Read(status_tombol, HC165_NUM_CHIPS);
            HC165_Unpack(status_tombol, pins, HC165_NUM_CHIPS);
            last_button_read_tick = HAL_GetTick();
        }

        // Fall simulation (rising edge + debounce)
        if (pins[0] && !prev_fall && (HAL_GetTick() - last_fall_time > FALL_DEBOUNCE_MS)) {
            Log_AddEvent(1000 + (rand() % 2001), rpm, 3);
            last_fall_time = HAL_GetTick();
        }
        prev_fall = pins[0];

        // Flush: D1 IC A
        if (pins[1]) {
            static uint32_t last_flush = 0;
            if (HAL_GetTick() - last_flush > 300) { Log_FlushToCSV(); last_flush = HAL_GetTick(); }
        }

        // Read CSV: D2 IC A
        if (pins[2]) {
            static uint32_t last_read = 0;
            if (HAL_GetTick() - last_read > 300) { Log_ReadCSV(); last_read = HAL_GetTick(); }
        }

        // USB VBUS detection (handled by EXTI or periodic poll)
        if (vbus_event_pending || (HAL_GetTick() - last_usb_check >= USB_CHECK_MS)) {
            uint8_t new_usb = HAL_GPIO_ReadPin(USB_VBUS_SENSE_GPIO_Port, USB_VBUS_SENSE_Pin);
            if (new_usb && !last_usb_state) {
                UART_Print("USB connected - staging events, committing before host probe\r\n");
                // Tell MSC layer to report not-ready while we prepare the media
                STORAGE_SetMediaReady(0);

                // Ensure FS is mounted so we can replay staged entries into the CSV
                if (!fs_mounted) MountFS();

                // Move queued events into staging area
                if (event_count > 0) Log_StageEvents();

                // If there are staged entries, replay them into the mounted FS now
                if (staging_has_entries()) {
                    UART_Print("Committing staged entries to FS before host probe...\r\n");
                    if (staging_commit() == 0) UART_Print("Staging committed\r\n");
                    else UART_Print("Staging commit failed\r\n");
                }

                // Ensure all caches flushed then unmount so host can mount the updated FS
                HAL_Delay(50);
                UnmountFS();

                // Update MSC capacity and mark media ready for the host
                STORAGE_UpdateCapacity(flash_capacity_bytes);
                STORAGE_SetMediaReady(1);
            } else if (!new_usb && last_usb_state) {
                UART_Print("USB disconnected - remounting FS\r\n");
                HAL_Delay(100);
                MountFS();
                if (staging_has_entries()) {
                    UART_Print("Committing staged entries to FS...\r\n");
                    if (staging_commit() == 0) UART_Print("Staging committed\r\n");
                    else UART_Print("Staging commit failed\r\n");
                }
            }
            last_usb_state = new_usb;
            usb_connected = new_usb;
            last_usb_check = HAL_GetTick();
            vbus_event_pending = 0;
        }

        // Display update (100ms)
        if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
            if (HAL_GetTick() - last_capture_time > 500) rpm = 0;
            Display_ShowNumber(2, (uint16_t)rpm, 0);
            last_display_tick = HAL_GetTick();
        }

        // Debug UART (1 sec)
        if (HAL_GetTick() - last_debug_tick >= DEBUG_UPDATE_MS) {
            uint8_t sec, min, hour, day, date, month, year;
            DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);

            char uart_buf[250];
            sprintf(uart_buf, "[%02d:%02d:%02d %02d/%02d/%02d] RPM:%lu ADC:%lu PWM:%lu SET:%lu Q:%d USB:%d FS:%d\r\n",
                    hour, min, sec, date, month, year,
                    (unsigned long)rpm, adc_raw, pwm_duty, set_value,
                    event_count, usb_connected, fs_mounted);
            UART_Print(uart_buf);
            last_debug_tick = HAL_GetTick();
        }

        // ADC & PWM
        static uint32_t last_adc_tick = 0;
        if (HAL_GetTick() - last_adc_tick >= 50) {
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                adc_raw = HAL_ADC_GetValue(&hadc1);
                set_value = (adc_raw * 150) / 4095;
                if (set_value > 150) set_value = 150;
            }
            if (adc_raw < 20) pwm_duty = 0;
            else {
                pwm_duty = (adc_raw * 4200) / 4096;
                if (pwm_duty > 4199) pwm_duty = 4199;
            }
            if (pwm_duty > PWM_MAX_DUTY) pwm_duty = PWM_MAX_DUTY;
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm_duty);
            last_adc_tick = HAL_GetTick();
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 8399;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim4) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
}

static void MX_TIM5_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4199;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim5);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, SHIFT_PL_Pin | SHIFT_CP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO7_Pin |
                            DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin |
                            DISP_CLK2_Pin | DISP_DIO4_Pin | FLASH_CS_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SHIFT_PL_Pin | SHIFT_CP_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO7_Pin |
                          DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin |
                          DISP_CLK2_Pin | DISP_DIO4_Pin | FLASH_CS_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin | SHIFT_Q7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable and set EXTI line Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    GPIO_InitStruct.Pin = DISP_CLK7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DISP_CLK7_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DISP_CLK7_GPIO_Port, DISP_CLK7_Pin, GPIO_PIN_SET);
}

void UART_Print(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        uint32_t current = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t delta = (current > last_capture) ? (current - last_capture)
                        : ((0xFFFF - last_capture) + current + 1);
        delta += timer_overflow * 65536UL;
        timer_overflow = 0;
        rpm = (delta > 0) ? (100000 / delta) : 0;
        last_capture = current;
        last_capture_time = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) timer_overflow++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USB_VBUS_SENSE_Pin) {
        vbus_event_pending = 1;
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
