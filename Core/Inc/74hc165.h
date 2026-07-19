#ifndef INC_74HC165_H_
#define INC_74HC165_H_

#include "main.h"

/* Set wiring (panggil SEKALI di awal main(), sebelum HC165_Read dipakai). */
void HC165_SetPins(GPIO_TypeDef *PL_Port, uint16_t PL_Pin,
                    GPIO_TypeDef *CP_Port, uint16_t CP_Pin,
                    GPIO_TypeDef *Q7_Port, uint16_t Q7_Pin);

/* Membaca semua IC dalam chain sekaligus.
 * num_chips  : jumlah IC 74HC165 yang di-cascade (didefinisikan di main.c, bukan di sini)
 * data_out   : array uint8_t[num_chips] yang sudah dialokasikan pemanggil
 *              (data mentah per-IC, masih dalam bentuk byte) */
void HC165_Read(uint8_t *data_out, uint8_t num_chips);

/* Mengubah data mentah (byte per IC) menjadi array boolean per-pin,
 * supaya pembacaan tidak perlu bitwise AND/OR sama sekali.
 *
 * data_in   : hasil dari HC165_Read(), array uint8_t[num_chips]
 * pins_out  : array uint8_t[num_chips * 8], tiap elemen berisi 0 atau 1
 * num_chips : jumlah IC (harus sama dengan yang dipakai saat HC165_Read)
 *
 * Penomoran pins_out:
 *   pins_out[0..7]   = D0..D7 pada IC pertama (data_in[0])
 *   pins_out[8..15]  = D0..D7 pada IC kedua  (data_in[1])
 *   dst.
 */
void HC165_Unpack(const uint8_t *data_in, uint8_t *pins_out, uint8_t num_chips);

#endif /* INC_74HC165_H_ */
