#include "ui.h"
#include "TM1637.h"
#include "74hc165.h"
#include "motor_control.h"
#include "fs_logger.h"
#include <stdlib.h>

#define HC165_NUM_CHIPS     4
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

extern LaneState_t lanes[5];
LaneState_t lanes[5];

void UI_Init(void) {
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_SetBrightness(i, 4);
    for (uint8_t i = 0; i < NUM_DISPLAYS; i++) Display_ShowNumber(i, 0, 0);
    
    for (int i = 0; i < 5; i++) {
        lanes[i].status = LANE_IDLE;
        lanes[i].duration_ms = 0;
        lanes[i].prev_btn_start = 0;
        lanes[i].prev_btn_stop = 0;
        lanes[i].prev_btn_reset = 0;
        lanes[i].prev_ext_btn = 0;
        lanes[i].prev_magnet = 0;
        lanes[i].last_magnet_tick = 0;
    }
}

void UI_Process(void) {
    static uint32_t last_button_read_tick = 0;
    static uint32_t last_display_tick = 0;

    // Shift register
    if (HAL_GetTick() - last_button_read_tick >= BUTTON_READ_MS) {
        HC165_Read(status_tombol, HC165_NUM_CHIPS);
        HC165_Unpack(status_tombol, pins, HC165_NUM_CHIPS);
        last_button_read_tick = HAL_GetTick();
        
        // Cek apakah input HC165 dimatikan dari EEPROM (untuk mencegah noise)
        #include "settings.h"
        MotorSettings s;
        uint32_t hc165_en = 1; // Default nyala
        if (Settings_Load(&s)) hc165_en = s.hc165_enabled;

        // Pin mapping berdasarkan skematik hardware fisik
        const uint8_t map_start[5]  = {0, 3, 6, 9, 12};
        const uint8_t map_stop[5]   = {1, 4, 7, 10, 13};
        const uint8_t map_reset[5]  = {2, 5, 8, 11, 14};
        const uint8_t map_magnet[5] = {15, 16, 17, 18, 19};
        const uint8_t map_detect[5] = {20, 21, 22, 23, 24};
        const uint8_t map_ext[5]    = {25, 26, 27, 28, 29};

        if (hc165_en) {
            for (int i = 0; i < 5; i++) {
            uint8_t btn_start = pins[map_start[i]];
            uint8_t btn_stop  = pins[map_stop[i]];
            uint8_t btn_reset = pins[map_reset[i]];
            uint8_t magnet    = pins[map_magnet[i]];
            uint8_t cable_det = pins[map_detect[i]]; // LOW (0) = connected
            uint8_t ext_btn   = pins[map_ext[i]];

            uint8_t start_edge = (btn_start && !lanes[i].prev_btn_start);
            uint8_t stop_edge  = (btn_stop && !lanes[i].prev_btn_stop);
            uint8_t ext_edge   = (ext_btn && !lanes[i].prev_ext_btn);
            uint8_t reset_edge = (btn_reset && !lanes[i].prev_btn_reset);
            uint8_t magnet_edge= (magnet && !lanes[i].prev_magnet);

            lanes[i].prev_btn_start = btn_start;
            lanes[i].prev_btn_stop  = btn_stop;
            lanes[i].prev_btn_reset = btn_reset;
            lanes[i].prev_ext_btn   = ext_btn;
            lanes[i].prev_magnet    = magnet;

            uint8_t trigger_start = 0;
            uint8_t trigger_stop  = 0;

            if (cable_det == 0) {
                // Cable connected: Onboard buttons ignored. Ext button toggles.
                if (ext_edge) {
                    if (lanes[i].status == LANE_IDLE || lanes[i].status == LANE_STOPPED) {
                        trigger_start = 1;
                    } else if (lanes[i].status == LANE_RUNNING) {
                        trigger_stop = 1;
                    }
                }
            } else {
                // Cable disconnected: Onboard buttons active.
                if (start_edge) trigger_start = 1;
                if (stop_edge)  trigger_stop = 1;
            }

            // Execute Start
            if (trigger_start && lanes[i].status != LANE_RUNNING) {
                lanes[i].status = LANE_RUNNING;
                lanes[i].start_tick = HAL_GetTick();
                lanes[i].duration_ms = 0;
            }

            // Execute Stop (Button/Ext or Magnet)
            uint8_t fall_detected = (magnet_edge && (HAL_GetTick() - lanes[i].last_magnet_tick > FALL_DEBOUNCE_MS));
            
            if ((trigger_stop || fall_detected) && lanes[i].status == LANE_RUNNING) {
                lanes[i].status = LANE_STOPPED;
                lanes[i].duration_ms = HAL_GetTick() - lanes[i].start_tick;
                
                if (fall_detected) {
                    Log_AddEvent(lanes[i].duration_ms, Motor_GetSetValue(), i + 1);
                    lanes[i].last_magnet_tick = HAL_GetTick();
                }
            }

            // Execute Reset (only if not running)
            if (reset_edge && lanes[i].status != LANE_RUNNING) {
                lanes[i].status = LANE_IDLE;
                lanes[i].duration_ms = 0;
            }
        } // End of for loop
    } // End of if (hc165_en)
    } // End of HAL_GetTick()
    
    // Displays Update (100ms)
    if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
        
        // 1. Lane Timers (Displays 0 to 4) format MM:SS
        for (int i = 0; i < 5; i++) {
            uint32_t current_ms = lanes[i].duration_ms;
            if (lanes[i].status == LANE_RUNNING) {
                current_ms = HAL_GetTick() - lanes[i].start_tick;
            }
            
            uint32_t total_sec = current_ms / 1000;
            uint16_t mm = total_sec / 60;
            uint16_t ss = total_sec % 60;
            if (mm > 99) mm = 99; // cap display to 99:59
            
            uint16_t disp_val = (mm * 100) + ss;
            // Show colon dots (0x02 enables the colon on standard TM1637)
            Display_ShowNumber(i, disp_val, 2); 
        }
        
        // 2. Set RPM (Display 6)
        Display_ShowNumber(6, (uint16_t)Motor_GetSetValue(), 0);
        
        last_display_tick = HAL_GetTick();
    }
    
    // Actual RPM Display (Display 5), perbarui setiap 2 detik dengan rata-rata
    static float rpm_sum = 0;
    static uint32_t rpm_sample_count = 0;
    static uint32_t last_rpm_sample_tick = 0;
    static uint32_t last_avg_display_tick = 0;

    if (HAL_GetTick() - last_rpm_sample_tick >= DISPLAY_UPDATE_MS) {
        rpm_sum += Motor_GetPreciseOutputRPM();
        rpm_sample_count++;
        last_rpm_sample_tick = HAL_GetTick();
    }

    if (HAL_GetTick() - last_avg_display_tick >= 2000) {
        uint16_t avg_rpm = (rpm_sample_count > 0) ? (uint16_t)(rpm_sum / (float)rpm_sample_count + 0.5f) : 0;
        Display_ShowNumber(5, avg_rpm, 0); 
        
        rpm_sum = 0;
        rpm_sample_count = 0;
        last_avg_display_tick = HAL_GetTick();
    }
}

bool UI_TriggerFall(uint8_t lane_index) {
    if (lane_index >= 5) return false;
    
    if (lanes[lane_index].status == LANE_RUNNING) {
        lanes[lane_index].status = LANE_STOPPED;
        lanes[lane_index].duration_ms = HAL_GetTick() - lanes[lane_index].start_tick;
    } else {
        // Jika sedang tidak berjalan, buat durasi dummy secara acak agar cepat
        lanes[lane_index].duration_ms = 1000 + (rand() % 5000); 
    }
    
    Log_AddEvent(lanes[lane_index].duration_ms, Motor_GetSetValue(), lane_index + 1);
    lanes[lane_index].last_magnet_tick = HAL_GetTick();
    return true;
}

bool UI_StartLane(uint8_t lane_index) {
    if (lane_index >= 5) return false;
    if (lanes[lane_index].status != LANE_RUNNING) {
        lanes[lane_index].status = LANE_RUNNING;
        lanes[lane_index].start_tick = HAL_GetTick();
        lanes[lane_index].duration_ms = 0;
        return true;
    }
    return false;
}

bool UI_StopLane(uint8_t lane_index) {
    if (lane_index >= 5) return false;
    if (lanes[lane_index].status == LANE_RUNNING) {
        lanes[lane_index].status = LANE_STOPPED;
        lanes[lane_index].duration_ms = HAL_GetTick() - lanes[lane_index].start_tick;
        return true;
    }
    return false;
}

// Returns the last-read raw byte from each of the 4 HC165 ICs.
// ICA = IC0 (pins 0-7), ICB = IC1 (8-15), ICC = IC2 (16-23), ICD = IC3 (24-31)
void UI_GetShiftRegStatus(uint8_t dst[4]) {
    dst[0] = status_tombol[0];
    dst[1] = status_tombol[1];
    dst[2] = status_tombol[2];
    dst[3] = status_tombol[3];
}
