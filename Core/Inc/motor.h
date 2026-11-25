#ifndef RC_MOTOR_H
#define RC_MOTOR_H

#include <stdint.h>
void Motor_Init(void);
void Motor_Set(int left, int right);
void Motor_Stop(uint32_t now);
void Motor_Task(uint32_t now);
int Motor_GetActual(unsigned side);
void Motor_CancelHold(void);
void Motor_HoldUntil(uint32_t until);
void Motor_TestPhysical(unsigned channel, int duty);

#endif /* RC_MOTOR_H */
