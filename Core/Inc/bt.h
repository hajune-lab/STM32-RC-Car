#ifndef RC_BT_H
#define RC_BT_H

#include <stdint.h>
/* USART1 transport and F/B/L/R/S decoding; manual speed is in rc_config.h. */
void BT_Poll(uint32_t now);
void BT_HandleCommand(char command, uint32_t now);
void BT_HandleFault(uint32_t now);
uint32_t BT_ErrorCount(void);

#endif /* RC_BT_H */
