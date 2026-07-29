#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

// Magic number untuk validasi data di Flash
#define SETTINGS_MAGIC 0xA5B60001UL

// Struct settings yang disimpan ke Flash
// Ukuran harus kelipatan 4 byte
typedef struct __attribute__((packed)) {
    uint32_t magic;     // Identifikasi data valid
    float    kp;        // PID Proportional gain
    float    ki;        // PID Integral gain
    float    kd;        // PID Derivative gain
    uint32_t hc165_enabled; // 1 = aktif, 0 = disable (abaikan input HC165 karena noise)
    uint32_t checksum;  // XOR checksum untuk validasi integritas data
} MotorSettings;

// Load settings dari Flash ke struct output
// Return: true jika data valid, false jika Flash kosong/korup
bool Settings_Load(MotorSettings *out);

// Simpan Kp, Ki, Kd, dan status HC165 ke Flash
// Return: true jika berhasil
bool Settings_Save(float kp, float ki, float kd, uint32_t hc165_enabled);

#endif // SETTINGS_H
