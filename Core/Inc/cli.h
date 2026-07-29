#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "main.h"

// Non-blocking UART print
void UART_Print(char *msg);

// Process incoming commands over FTDI UART
void ProcessUartRxCommand(void);

// UART RX ISR Handler (must be called from USART1_IRQHandler)
void USART1_Rx_ISR(void);

#endif // CLI_H
