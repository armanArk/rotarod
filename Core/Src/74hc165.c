#include "74hc165.h"

#define HC165_DELAY_LOOP 500

/* Batas aman untuk buffer internal (bukan pembatas num_chips secara umum,
 * hanya menjaga agar operasi di dalam library tidak menulis di luar batas
 * jika suatu saat num_chips diisi lebih besar dari yang didukung). */
#define HC165_MAX_CHIPS_INTERNAL 16

// --- Wiring disimpan di sini, diisi lewat HC165_SetPins() ---
static GPIO_TypeDef *s_PL_Port, *s_CP_Port, *s_Q7_Port;
static uint16_t       s_PL_Pin,  s_CP_Pin,  s_Q7_Pin;

void HC165_SetPins(GPIO_TypeDef *PL_Port, uint16_t PL_Pin,
                    GPIO_TypeDef *CP_Port, uint16_t CP_Pin,
                    GPIO_TypeDef *Q7_Port, uint16_t Q7_Pin) {
    s_PL_Port = PL_Port; s_PL_Pin = PL_Pin;
    s_CP_Port = CP_Port; s_CP_Pin = CP_Pin;
    s_Q7_Port = Q7_Port; s_Q7_Pin = Q7_Pin;
}

// --- Helper Functions (menjaga kode utama tetap bersih) ---
static void HC165_Delay(void) {
    for (volatile int i = 0; i < HC165_DELAY_LOOP; i++);
}

static void HC165_Latch(void) {
    HAL_GPIO_WritePin(s_PL_Port, s_PL_Pin, GPIO_PIN_RESET);
    HC165_Delay();
    HAL_GPIO_WritePin(s_PL_Port, s_PL_Pin, GPIO_PIN_SET);
    HC165_Delay();
}

static void HC165_PulseClock(void) {
    HAL_GPIO_WritePin(s_CP_Port, s_CP_Pin, GPIO_PIN_SET);
    HC165_Delay();
    HAL_GPIO_WritePin(s_CP_Port, s_CP_Pin, GPIO_PIN_RESET);
    HC165_Delay();
}

// --- Fungsi Utama (num_chips ditentukan pemanggil, bukan macro di library) ---
void HC165_Read(uint8_t *data_out, uint8_t num_chips) {
    if (num_chips > HC165_MAX_CHIPS_INTERNAL) {
        num_chips = HC165_MAX_CHIPS_INTERNAL; // batas aman buffer internal
    }

    HC165_Latch(); // Mengunci data dari input paralel di SEMUA IC sekaligus

    for (int chip = 0; chip < num_chips; chip++) {
        uint8_t byte_in = 0;
        for (int bit = 0; bit < 8; bit++) {
            byte_in <<= 1;
            if (HAL_GPIO_ReadPin(s_Q7_Port, s_Q7_Pin) == GPIO_PIN_SET) {
                byte_in |= 0x01;
            }
            HC165_PulseClock();
        }
        data_out[chip] = byte_in;
    }
}

// ---------------------------------------------------------
// HC165_Unpack
// Mengubah byte mentah per-IC menjadi array boolean per-pin,
// supaya pemanggil tidak perlu bitwise AND/OR manual.
// ---------------------------------------------------------
void HC165_Unpack(const uint8_t *data_in, uint8_t *pins_out, uint8_t num_chips) {
    for (int chip = 0; chip < num_chips; chip++) {
        for (int bit = 0; bit < 8; bit++) {
            pins_out[chip * 8 + bit] = (data_in[chip] >> bit) & 0x01;
        }
    }
}
