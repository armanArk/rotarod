#include "settings.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// ============================================================
// Konfigurasi Flash Sector untuk Settings
// Board Anda: STM32F401CCU6 (Black Pill 256KB)
// Flash Sector terakhir adalah Sector 5 (128KB) @ 0x08020000
// ============================================================
#define FLASH_SETTINGS_SECTOR   FLASH_SECTOR_5
#define FLASH_SETTINGS_ADDRESS  0x08020000UL
#define FLASH_VOLTAGE_RANGE     FLASH_VOLTAGE_RANGE_3  // 2.7V - 3.6V

// Hitung XOR checksum dari semua field kecuali checksum itu sendiri
static uint32_t calc_checksum(const MotorSettings *s) {
    const uint32_t *p = (const uint32_t *)s;
    uint32_t chk = 0;
    for (size_t i = 0; i < (sizeof(MotorSettings) / sizeof(uint32_t)) - 1; i++) {
        chk ^= p[i];
    }
    return chk;
}

bool Settings_Load(MotorSettings *out) {
    if (out == NULL) return false;

    const MotorSettings *flash = (const MotorSettings *)FLASH_SETTINGS_ADDRESS;

    // Cek magic number
    if (flash->magic != SETTINGS_MAGIC) return false;

    // Cek checksum integritas data
    uint32_t expected = calc_checksum(flash);
    if (flash->checksum != expected) return false;

    // Validasi nilai PID (sanity check agar tidak load nilai gila)
    if (flash->kp < 0.0f || flash->kp > 100.0f) return false;
    if (flash->ki < 0.0f || flash->ki > 100.0f) return false;
    if (flash->kd < 0.0f || flash->kd > 100.0f) return false;

    memcpy(out, flash, sizeof(MotorSettings));
    return true;
}

bool Settings_Save(float kp, float ki, float kd, uint32_t hc165_enabled) {
    MotorSettings s;
    memset(&s, 0, sizeof(s));
    s.magic    = SETTINGS_MAGIC;
    s.kp       = kp;
    s.ki       = ki;
    s.kd       = kd;
    s.hc165_enabled = hc165_enabled;
    s.checksum = calc_checksum(&s);

    HAL_FLASH_Unlock();

    // Hapus sektor terlebih dahulu
    FLASH_EraseInitTypeDef erase;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_SETTINGS_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE;

    uint32_t sector_error = 0xFFFFFFFF;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);

    if (status == HAL_OK) {
        // Tulis word per word (32-bit)
        const uint32_t *src  = (const uint32_t *)&s;
        uint32_t        addr = FLASH_SETTINGS_ADDRESS;
        for (size_t i = 0; i < sizeof(MotorSettings) / sizeof(uint32_t); i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
            if (status != HAL_OK) break;
            addr += sizeof(uint32_t);
        }
    }

    HAL_FLASH_Lock();
    return (status == HAL_OK);
}
