#include "TM1637.h"

// Mapping untuk angka 0-9
static const uint8_t segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f
};
/*
 *
 *
  // ==========================================
	  // AWAL LOOP: Tampilkan angka bersih (Semua Titik MATI)
	  // ==========================================
	  TM1637_DisplayNumber(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 0);
	  // Jalankan fungsi set ke 0 agar internal state tracker library ikut ter-reset
	  TM1637_SetColon(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 0);
	  HAL_Delay(1000);

	  // 1. Menyalakan Titik Dua (Colon) -> Menjadi 12:34
	  TM1637_SetColon(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 1);
	  HAL_Delay(1000);

	  // 2. Mematikan kembali Titik Dua (Colon) -> Kembali jadi 1234
	  TM1637_SetColon(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 0);
	  HAL_Delay(1000);

	  // 3. Menyalakan Titik Desimal di digit ke-1 (Index 0) -> Menjadi 1.234
	  TM1637_SetDecimalDot(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 0, 1);
	  HAL_Delay(1000);

	  // 4. Menyalakan juga Titik Desimal di digit ke-3 (Index 2) -> Menjadi 1.23.4
	  // (Sekarang indeks 0 dan indeks 2 sama-sama NYALA)
	  TM1637_SetDecimalDot(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 2, 1);
	  HAL_Delay(1000);

	  // 5. Mematikan Titik Desimal di index 0 saja -> Menjadi 123.4
	  // (Indeks 2 TETAP NYALA karena menggunakan state tracker terbaru)
	  TM1637_SetDecimalDot(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, nilai_display, 0, 0);
	  HAL_Delay(1000);
 *
 *
 *
 */

	  /* USER CODE END WHILE */
// Variabel internal untuk mencatat status gabungan semua titik (Colon & Decimal Dots)
static uint8_t global_display_dots = 0x00;

// Delay untuk memenuhi timing protokol TM1637
static void TM1637_Delay(void) {
    for (volatile int i = 0; i < 500; i++);
}

static void TM1637_Start(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin) {
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIO_Port, DIO_Pin, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(DIO_Port, DIO_Pin, GPIO_PIN_RESET);
    TM1637_Delay();
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_RESET);
    TM1637_Delay();
}

static void TM1637_Stop(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin) {
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIO_Port, DIO_Pin, GPIO_PIN_RESET);
    TM1637_Delay();
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(DIO_Port, DIO_Pin, GPIO_PIN_SET);
    TM1637_Delay();
}

static void TM1637_WriteByte(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin, uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIO_Port, DIO_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data >>= 1;
        TM1637_Delay();
        HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_SET);
        TM1637_Delay();
    }
    // Siklus ACK
    HAL_GPIO_WritePin(DIO_Port, DIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_RESET);
    TM1637_Delay();
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(CLK_Port, CLK_Pin, GPIO_PIN_RESET);
    TM1637_Delay();
}

void TM1637_SetBrightness(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin, uint8_t brightness) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set semua pin DIO ke mode Input (Idle) agar tidak konflik[cite: 8]
    GPIO_InitStruct.Pin = ALL_DIO_PINS;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set pin DIO target ke mode Output Open-Drain untuk komunikasi[cite: 8]
    GPIO_InitStruct.Pin = DIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIO_Port, &GPIO_InitStruct);

    TM1637_Start(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);
    TM1637_WriteByte(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, (brightness == 0) ? 0x80 : (0x88 | (brightness - 1)));
    TM1637_Stop(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);
}

void TM1637_DisplayNumber(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin, uint16_t num, uint8_t dots) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Inisialisasi pin DIO target ke mode Open-Drain[cite: 8]
    GPIO_InitStruct.Pin = DIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIO_Port, &GPIO_InitStruct);

    uint8_t digit[4] = {(num/1000)%10, (num/100)%10, (num/10)%10, num%10};

    TM1637_Start(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);
    TM1637_WriteByte(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, 0x40);
    TM1637_Stop(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);

    TM1637_Start(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);
    TM1637_WriteByte(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, 0xC0);
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t data = segmentMap[digit[i]];
        if (dots & (1 << i)) data |= 0x80; // Menyalakan titik desimal / colon[cite: 8]
        TM1637_WriteByte(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, data);
    }
    TM1637_Stop(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin);
}

/**
  * @brief  Menyalakan atau mematikan titik dua (Colon ':') di tengah display.
  * @param  current_num: Angka dasar yang sedang aktif ditampilkan.
  * @param  status: 1 untuk AKTIF, 0 untuk MATI.
  */
void TM1637_SetColon(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin, uint16_t current_num, uint8_t status) {
    if (status == 1) {
        global_display_dots |= 0x02;  // Set bit ke-1 untuk jalur fisik Colon
    } else {
        global_display_dots &= ~0x02; // Clear bit ke-1
    }
    TM1637_DisplayNumber(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, current_num, global_display_dots);
}

/**
  * @brief  Menyalakan atau mematikan titik desimal (Decimal Dot '.') pada digit tertentu secara spesifik.
  * @param  current_num: Angka dasar yang sedang aktif ditampilkan.
  * @param  digit_index: Posisi digit yang ditarget (0 = paling kiri, 3 = paling kanan).
  * @param  status: 1 untuk AKTIF, 0 untuk MATI.
  */
void TM1637_SetDecimalDot(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin, GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin, uint16_t current_num, uint8_t digit_index, uint8_t status) {
    if (digit_index > 3) return;

    if (status == 1) {
        global_display_dots |= (1 << digit_index);  // Gabungkan bit desimal dot baru tanpa menghapus status lama
    } else {
        global_display_dots &= ~(1 << digit_index); // Buang bit desimal dot target
    }
    TM1637_DisplayNumber(CLK_Port, CLK_Pin, DIO_Port, DIO_Pin, current_num, global_display_dots);
}
