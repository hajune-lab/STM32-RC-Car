#ifndef RC_DEBUG_UART_H
#define RC_DEBUG_UART_H

#include <stdbool.h>
#include <stdint.h>
#define DEBUG_UART_BUFFER_SIZE 320U
void DebugUart_Init(void);
void DebugUart_Poll(uint32_t now);
bool DebugUart_IsBusy(void);
char *DebugUart_Buffer(void);
void DebugUart_Commit(unsigned length);

#endif /* RC_DEBUG_UART_H */
