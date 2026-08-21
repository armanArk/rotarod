#ifndef FS_LOGGER_H
#define FS_LOGGER_H

#include <stdint.h>
#include "fatfs.h"

// Initialize logging variables
void Log_Init(void);

// Set flash capacity (must be called before mounting/formatting)
void FS_SetFlashCapacity(uint32_t capacity_bytes);
uint32_t FS_GetFlashCapacity(void);

// Filesystem management
void MountFS(void);
void UnmountAllFS(void);
void FormatFS(void);
void CheckAndFormatIfMismatch(void);

uint8_t FS_IsMounted(void);
uint8_t FS_IsFormatted(void);

typedef struct {
    char timestamp[32]; // Format: DD/MM/YY HH:MM:SS
    uint32_t duration_ms;
    uint16_t rpm;
    uint8_t lane;
} RotarodEvent;

// Event Logging
void Log_AddEvent(uint32_t duration_ms, uint16_t rpm_val, uint8_t lane);
void Log_FlushToCSV(void);
void Log_ClearCSV(void);
void Log_StageEvents(void);
void Log_ReadCSV(void);

// Get the current number of events in queue (useful for debug print)
uint16_t Log_GetEventCount(void);

#endif // FS_LOGGER_H
