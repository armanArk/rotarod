#include "ui.h"
#include "TM1637.h"
#include "74hc165.h"
#include "motor_control.h"
#include "fs_logger.h"
#include <stdlib.h>

#define HC165_NUM_CHIPS     3
#define HC165_TOTAL_PINS    (HC165_NUM_CHIPS * 8)
#define NUM_DISPLAYS        7
#define DISPLAY_UPDATE_MS   100
#define BUTTON_READ_MS      20
#define FALL_DEBOUNCE_MS    500

static uint8_t status_tombol[HC165_NUM_CHIPS], pins[HC165_TOTAL_PINS];

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

void UI_Init(void) {
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_SetBrightness(i, 4);
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_ShowNumber(i, (i+1)*1111, 0);
}

void UI_Process(void) {
    static uint32_t last_button_read_tick = 0;
    static uint32_t last_display_tick = 0;
    static uint8_t prev_fall = 0;
    static uint32_t last_fall_time = 0;

    // Shift register
    if (HAL_GetTick() - last_button_read_tick >= BUTTON_READ_MS) {
        HC165_Read(status_tombol, HC165_NUM_CHIPS);
        HC165_Unpack(status_tombol, pins, HC165_NUM_CHIPS);
        last_button_read_tick = HAL_GetTick();
    }

    // Fall simulation (rising edge + debounce)
    if (pins[0] && !prev_fall && (HAL_GetTick() - last_fall_time > FALL_DEBOUNCE_MS)) {
        Log_AddEvent(1000 + (rand() % 2001), Motor_GetRPM(), 3);
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

    // Display update (100ms) - Target RPM
    if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
        Display_ShowNumber(2, (uint16_t)Motor_GetSetValue(), 0); // Display 2 = Target RPM
        last_display_tick = HAL_GetTick();
    }

    // Display 1 (merah) = Actual RPM, diperbarui setiap 2 detik dengan rata-rata
    static float rpm_sum = 0;
    static uint32_t rpm_sample_count = 0;
    static uint32_t last_rpm_sample_tick = 0;
    static uint32_t last_avg_display_tick = 0;

    // Kumpulkan sample setiap 100ms
    if (HAL_GetTick() - last_rpm_sample_tick >= DISPLAY_UPDATE_MS) {
        rpm_sum += Motor_GetPreciseOutputRPM();
        rpm_sample_count++;
        last_rpm_sample_tick = HAL_GetTick();
    }

    // Setiap 2 detik, hitung rata-rata lalu tampilkan
    if (HAL_GetTick() - last_avg_display_tick >= 2000) {
        // Tampilkan RPM bulat, karena hardware display hanya mendukung titik dua (colon)
        uint16_t avg_rpm = (rpm_sample_count > 0) ? (uint16_t)(rpm_sum / (float)rpm_sample_count + 0.5f) : 0;
        
        Display_ShowNumber(1, avg_rpm, 0); 
        
        rpm_sum = 0;
        rpm_sample_count = 0;
        last_avg_display_tick = HAL_GetTick();
    }
}
