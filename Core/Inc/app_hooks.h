#ifndef RC_APP_HOOKS_H
#define RC_APP_HOOKS_H

/* Internal integration points; not tuning settings or shared global storage.
 * Sensor driver reports side samples; app supplies measurement priorities.
 * UART drivers report commands; only app.c changes the drive state. */
#include "ultrasonic.h"
bool App_SensorFrontPriority(void);
bool App_SideHistoryIsFresh(uint32_t good_at);
void App_ObserveSideSample(SensorId id, float cm, bool fresh, uint32_t now);
void App_DebugCommand(char c, uint32_t now);
bool App_IsManual(void);
void App_ManualDrive(int left, int right, uint32_t now);
void App_StopWithReason(uint32_t now, const char *why);

#endif /* RC_APP_HOOKS_H */
