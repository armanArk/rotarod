#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "main.h"

typedef enum {
    LANE_IDLE = 0,
    LANE_RUNNING,
    LANE_STOPPED
} LaneStatus_t;

typedef struct {
    LaneStatus_t status;
    uint32_t start_tick;
    uint32_t duration_ms;

    uint8_t prev_btn_start;
    uint8_t prev_btn_stop;
    uint8_t prev_btn_reset;
    uint8_t prev_ext_btn;
    uint8_t prev_magnet;
    uint32_t last_magnet_tick;
} LaneState_t;

// Initialize displays
void UI_Init(void);

// Process button inputs and update displays periodically
void UI_Process(void);

// Manually trigger a fall event for a specific lane (0 to 4)
#include <stdbool.h>
bool UI_TriggerFall(uint8_t lane_index);
bool UI_StartLane(uint8_t lane_index);
bool UI_StopLane(uint8_t lane_index);

#endif // UI_H
