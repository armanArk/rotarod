#include "cli.h"
#include "fs_logger.h"
#include "motor_control.h"
#include "settings.h"
#include <string.h>
#include <stdio.h>
#include "ds3231.h"
#include "ui.h"
#include <ctype.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern LaneState_t lanes[5];

/* ==================== TX Non-blocking Logger ==================== */
#define UART_BUF_SIZE 4096
static uint8_t uart_buf[UART_BUF_SIZE];
static volatile uint32_t uart_head = 0; // write index
static volatile uint32_t uart_tail = 0; // read index
static volatile uint8_t uart_tx_busy = 0;

static void uart_start_tx_if_needed(void);

void UART_Print(char *msg) {
    uint32_t len = strlen(msg);
    // If message too large, truncate
    if (len == 0) return;
    // Drop message if not enough space to keep non-blocking
    uint32_t free_space = (uart_tail + UART_BUF_SIZE - uart_head - 1) % UART_BUF_SIZE;
    if (len > free_space) return; // drop
    for (uint32_t i = 0; i < len; i++) {
        uart_buf[uart_head] = (uint8_t)msg[i];
        uart_head = (uart_head + 1) % UART_BUF_SIZE;
    }
    // Start TX if idle
    uart_start_tx_if_needed();
}

static void uart_start_tx_if_needed(void) {
    if (uart_tx_busy) return;
    if (uart_tail == uart_head) return; // empty
    uint32_t chunk_len = (uart_head > uart_tail) ? (uart_head - uart_tail) : (UART_BUF_SIZE - uart_tail);
    uart_tx_busy = 1;
    HAL_UART_Transmit_IT(&huart1, &uart_buf[uart_tail], (uint16_t)chunk_len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;
    // advance tail by last transmitted amount
    // HAL stores transferred size in huart->TxXferSize; use that
    uint32_t sent = huart->TxXferSize;
    uart_tail = (uart_tail + sent) % UART_BUF_SIZE;
    uart_tx_busy = 0;
    // start next chunk if available
    uart_start_tx_if_needed();
}


/* ==================== RX Command Parser ==================== */
static char rx_cmd_buf[32];
static uint8_t rx_cmd_idx = 0;
static volatile uint8_t rx_cmd_ready = 0;
static volatile uint32_t rx_last_char_tick = 0;
static char pending_cmd[32];

static float my_atof(const char *s, char **endptr) {
    float res = 0.0f, fraction = 1.0f;
    int sign = 1;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') { res = res * 10.0f + (*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { fraction /= 10.0f; res += (*s - '0') * fraction; s++; }
    }
    if (endptr) *endptr = (char*)s;
    return res * sign;
}

void USART1_Rx_ISR(void) {
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t ch = (uint8_t)(USART1->DR & 0xFF); // Reading DR clears both RXNE and ORE flags
        rx_last_char_tick = HAL_GetTick();
        if (ch == '\r' || ch == '\n') {
            if (rx_cmd_idx > 0) {
                rx_cmd_buf[rx_cmd_idx] = '\0';
                strncpy(pending_cmd, rx_cmd_buf, sizeof(pending_cmd) - 1);
                pending_cmd[sizeof(pending_cmd) - 1] = '\0';
                rx_cmd_ready = 1;
                rx_cmd_idx = 0;
            }
        } else if (ch >= 32 && ch <= 126 && rx_cmd_idx < (sizeof(rx_cmd_buf) - 1)) {
            rx_cmd_buf[rx_cmd_idx++] = (char)ch;
        }
    }
}

void ProcessUartRxCommand(void) {
    // If no newline was sent, auto-complete command after 3000ms of inactivity
    if (!rx_cmd_ready && rx_cmd_idx > 0 && (HAL_GetTick() - rx_last_char_tick >= 3000)) {
        rx_cmd_buf[rx_cmd_idx] = '\0';
        strncpy(pending_cmd, rx_cmd_buf, sizeof(pending_cmd) - 1);
        pending_cmd[sizeof(pending_cmd) - 1] = '\0';
        rx_cmd_ready = 1;
        rx_cmd_idx = 0;
    }

    if (rx_cmd_ready) {
        rx_cmd_ready = 0;
        char cmd[32];
        strncpy(cmd, pending_cmd, sizeof(cmd) - 1);
        cmd[sizeof(cmd) - 1] = '\0';
        for (int i = 0; cmd[i]; i++) cmd[i] = (char)toupper((unsigned char)cmd[i]);

        if (strcmp(cmd, "FORMAT") == 0 || 
            strcmp(cmd, "PARTITION") == 0 ||
            strcmp(cmd, "FDISK") == 0 ||
            strcmp(cmd, "FORMATFS") == 0) {
            UART_Print("\r\n[FTDI Command] Formatting flash into 2 partitions (50%/50%)...\r\n");
            UnmountAllFS();
            FormatFS();
        } else if (strcmp(cmd, "CHECKFS") == 0) {
            UART_Print("\r\n[FTDI Command] Checking FS size...\r\n");
            CheckAndFormatIfMismatch();
        } else if (strncmp(cmd, "PID ", 4) == 0) {
            char *p = pending_cmd + 4;
            float kp = my_atof(p, &p);
            float ki = my_atof(p, &p);
            float kd = my_atof(p, &p);
            
            Motor_SetPID(kp, ki, kd);
            
            int kp_i = (int)kp, kp_f = (int)((kp - kp_i) * 10);
            int ki_i = (int)ki, ki_f = (int)((ki - ki_i) * 10);
            int kd_i = (int)kd, kd_f = (int)((kd - kd_i) * 10);
            if (kp_f < 0) kp_f = -kp_f;
            if (ki_f < 0) ki_f = -ki_f;
            if (kd_f < 0) kd_f = -kd_f;
            
            Motor_SetPID(kp, ki, kd);
            
            MotorSettings s;
            uint32_t hc165_en = 1;
            if (Settings_Load(&s)) hc165_en = s.hc165_enabled;

            char msg[120];
            if (Settings_Save(kp, ki, kd, hc165_en)) {
                sprintf(msg, "\r\n[FTDI Command] PID updated and saved to Flash: Kp=%d.%d, Ki=%d.%d, Kd=%d.%d\r\n", 
                        kp_i, kp_f, ki_i, ki_f, kd_i, kd_f);
            } else {
                sprintf(msg, "\r\n[FTDI Command] PID updated (SAVE FAILED): Kp=%d.%d, Ki=%d.%d, Kd=%d.%d\r\n", 
                        kp_i, kp_f, ki_i, ki_f, kd_i, kd_f);
            }
            UART_Print(msg);
        } else if (strcmp(cmd, "SAVE") == 0) {
            float kp, ki, kd;
            Motor_GetPID(&kp, &ki, &kd);
            
            MotorSettings s;
            uint32_t hc165_en = 1;
            if (Settings_Load(&s)) hc165_en = s.hc165_enabled;

            if (Settings_Save(kp, ki, kd, hc165_en)) {
                int kp_i = (int)kp, kp_f = (int)((kp-(int)kp)*100);
                int ki_i = (int)ki, ki_f = (int)((ki-(int)ki)*100);
                int kd_i = (int)kd, kd_f = (int)((kd-(int)kd)*100);
                if (kp_f<0) kp_f=-kp_f;
                if (ki_f<0) ki_f=-ki_f;
                if (kd_f<0) kd_f=-kd_f;
                char msg[80];
                sprintf(msg, "\r\n[SAVE] Tersimpan ke Flash: Kp=%d.%02d Ki=%d.%02d Kd=%d.%02d\r\n",
                        kp_i, kp_f, ki_i, ki_f, kd_i, kd_f);
                UART_Print(msg);
            } else {
                UART_Print("\r\n[SAVE] GAGAL menyimpan ke Flash!\r\n");
            }
        } else if (strcmp(cmd, "LOAD") == 0) {
            MotorSettings s;
            if (Settings_Load(&s)) {
                Motor_SetPID(s.kp, s.ki, s.kd);
                int kp_i = (int)s.kp, kp_f = (int)((s.kp-(int)s.kp)*100);
                int ki_i = (int)s.ki, ki_f = (int)((s.ki-(int)s.ki)*100);
                int kd_i = (int)s.kd, kd_f = (int)((s.kd-(int)s.kd)*100);
                if (kp_f<0) kp_f=-kp_f;
                if (ki_f<0) ki_f=-ki_f;
                if (kd_f<0) kd_f=-kd_f;
                char msg[80];
                sprintf(msg, "\r\n[LOAD] Berhasil: Kp=%d.%02d Ki=%d.%02d Kd=%d.%02d\r\n",
                        kp_i, kp_f, ki_i, ki_f, kd_i, kd_f);
                UART_Print(msg);
            } else {
                UART_Print("\r\n[LOAD] Tidak ada settings tersimpan di Flash.\r\n");
            }

            char *p = cmd + 4;
            while (*p == ' ') p++;
            if (strcmp(p, "PID") == 0) {
                Motor_SetMode(MOTOR_MODE_PID);
                UART_Print("\r\n[Mode] PID - Potensiometer -> Target RPM -> PID -> PWM\r\n");
            } else if (strcmp(p, "DIRECT") == 0) {
                Motor_SetMode(MOTOR_MODE_DIRECT);
                UART_Print("\r\n[Mode] DIRECT - Potensiometer langsung ke PWM (open loop)\r\n");
            } else if (strcmp(p, "CLI") == 0) {
                Motor_SetMode(MOTOR_MODE_CLI);
                UART_Print("\r\n[Mode] CLI - Gunakan perintah RPM <nilai> untuk atur kecepatan\r\n");
            } else {
                char cur_mode = Motor_GetMode();
                char msg[80];
                sprintf(msg, "\r\nMode saat ini: %s\r\nGunakan: MODE PID | MODE DIRECT | MODE CLI\r\n",
                    cur_mode == MOTOR_MODE_PID ? "PID" : cur_mode == MOTOR_MODE_DIRECT ? "DIRECT" : "CLI");
                UART_Print(msg);
            }
        } else if (strncmp(cmd, "RPM ", 4) == 0) {
            if (Motor_GetMode() != MOTOR_MODE_CLI) {
                UART_Print("\r\n[Error] Perintah RPM hanya aktif di mode CLI. Kirim: MODE CLI\r\n");
            } else {
                char *p = pending_cmd + 4;
                uint32_t target = (uint32_t)my_atof(p, &p);
                Motor_SetCLITarget(target);
                char msg[60];
                sprintf(msg, "\r\n[CLI] Target RPM diset ke: %lu\r\n", target);
                UART_Print(msg);
            }
        } else if (strncmp(cmd, "TIME ", 5) == 0) {
            char *p = pending_cmd + 5;
            int hh, mm, ss, d, m, y;
            if (sscanf(p, "%d %d %d %d %d %d", &hh, &mm, &ss, &d, &m, &y) == 6) {
                // Day of week (1-7) can be set to 1 since we don't actively use it
                DS3231_SetTime((uint8_t)ss, (uint8_t)mm, (uint8_t)hh, 1, (uint8_t)d, (uint8_t)m, (uint8_t)y);
                char msg[80];
                sprintf(msg, "\r\n[RTC] Waktu berhasil diset: %02d:%02d:%02d %02d/%02d/20%02d\r\n", hh, mm, ss, d, m, y);
                UART_Print(msg);
            } else {
                UART_Print("\r\n[Error] Format salah! Gunakan: TIME HH MM SS DD MM YY\r\n");
            }
        } else if (strcmp(cmd, "HC165 ON") == 0 || strcmp(cmd, "HC165 OFF") == 0) {
            uint32_t enable = (strcmp(cmd, "HC165 ON") == 0) ? 1 : 0;
            float kp, ki, kd;
            Motor_GetPID(&kp, &ki, &kd);
            if (Settings_Save(kp, ki, kd, enable)) {
                if (enable) UART_Print("\r\n[HC165] Input tombol & sensor AKTIF (disimpan ke Flash).\r\n");
                else        UART_Print("\r\n[HC165] Input tombol & sensor DISABLED (disimpan ke Flash).\r\n");
            } else {
                UART_Print("\r\n[HC165] GAGAL menyimpan ke Flash!\r\n");
            }
        } else if (strcmp(cmd, "LS") == 0) {
            UART_Print("\r\n[CLI] Membaca isi Flashdisk (Internal MCU)...\r\n");
            if (!FS_IsMounted()) MountFS();
            DIR dir;
            static FILINFO fno;
            FRESULT res = f_opendir(&dir, "/"); // Root directory
            if (res == FR_OK) {
                int file_count = 0;
                for (;;) {
                    res = f_readdir(&dir, &fno);
                    if (res != FR_OK || fno.fname[0] == 0) break; // Break on error or end of dir
                    char msg[128];
                    sprintf(msg, "  -> %s (%lu bytes)\r\n", fno.fname, (unsigned long)fno.fsize);
                    UART_Print(msg);
                    file_count++;
                }
                f_closedir(&dir);
                if (file_count == 0) UART_Print("  (Flashdisk Kosong)\r\n");
            } else {
                char msg[64];
                sprintf(msg, "[Error] Gagal membuka root directory (FR=%d)\r\n", res);
                UART_Print(msg);
            }
        } else if (strcmp(cmd, "TRIGGER") == 0) {
            int random_lane = rand() % 5; // 0 sampai 4
            UI_TriggerFall(random_lane);
            Log_FlushToCSV();
            char msg[100];
            sprintf(msg, "\r\n[CLI] Berhasil: Lane %d, Durasi: %lu ms -> disimpan ke CSV.\r\n", 
                    random_lane + 1, (unsigned long)lanes[random_lane].duration_ms);
            UART_Print(msg);
        } else if (strcmp(cmd, "HELP") == 0) {
            UART_Print("\r\n=== FTDI Commands ===\r\n"
                       "  MODE PID              : Potensiometer -> Target RPM -> PID\r\n"
                       "  MODE DIRECT           : Potensiometer langsung ke PWM (open loop)\r\n"
                       "  MODE CLI              : CLI kontrol penuh via perintah RPM\r\n"
                       "  RPM <nilai>           : Set target RPM (hanya di MODE CLI)\r\n"
                       "  PID <Kp> <Ki> <Kd>   : Tune parameter PID\r\n"
                       "  TIME <HH> <MM> <SS> <DD> <MM> <YY> : Set RTC DS3231 time\r\n"
                       "  HC165 ON / OFF        : Aktifkan/Matikan input sensor (EEPROM)\r\n"
                       "  TRIGGER               : Tembakkan dummy jatuh (Lane acak, Waktu acak)\r\n"
                       "  LS                    : Lihat daftar file di dalam flashdisk\r\n"
                       "  FORMAT / FDISK        : Format flash 2 partisi (50%/50%)\r\n"
                       "  CHECKFS               : Verifikasi ukuran partisi\r\n"
                       "  HELP                  : Tampilkan daftar perintah\r\n");
        } else {
            char err[64];
            sprintf(err, "\r\nUnknown FTDI command '%s'. Type HELP.\r\n", cmd);
            UART_Print(err);
        }
    }
}
