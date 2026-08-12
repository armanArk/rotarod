#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "main.h"

typedef enum {
    DEBUG_MODE_AUTO  = 0,  // Auto: PID when motor active, FLASH otherwise
    DEBUG_MODE_PID   = 1,  // Always show PID/motor debug
    DEBUG_MODE_FLASH = 2,  // Always show USB/flash debug
    DEBUG_MODE_SHIFTR= 3,  // Always show Shift Register states
} CliDebugMode;

// Non-blocking UART print
void UART_Print(char *msg);

// Process incoming commands over FTDI UART
void ProcessUartRxCommand(void);

// UART RX ISR Handler (must be called from USART1_IRQHandler)
void USART1_Rx_ISR(void);

// Get current debug display mode
CliDebugMode CLI_GetDebugMode(void);

#endif // CLI_H
