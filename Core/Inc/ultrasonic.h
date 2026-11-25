#ifndef RC_ULTRASONIC_H
#define RC_ULTRASONIC_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { FRONT, LEFT, RIGHT, SENSOR_COUNT } SensorId;
typedef enum { EMPTY, GOOD, NO_RISE, NO_FALL, STUCK_HIGH, TOO_CLOSE, BAD_PULSE, NO_TARGET } RangeCode;
typedef enum { RX_IDLE, RX_TRIGGER, RX_RISE, RX_FALL } EchoPhase;
/* direction=-1: LEFT, +1: RIGHT. Steering>0 also means LEFT. */
typedef struct {
    GPIO_TypeDef *trig_port; uint16_t trig_pin;
    GPIO_TypeDef *echo_port; uint16_t echo_pin;
} SensorPins;
typedef struct {
    RangeCode code;
    float raw, filtered, last_good;
    float rise_candidate, closing_cmps;
    float approach_wall;
    uint32_t approach_at, measured_gap_at;
    uint8_t gap_votes;
    bool have_approach, measured_gap;
    uint32_t started, finished, good_at, sequence, opening_at, no_target_at;
    uint16_t fails, good_count, close_count, short_count;
    uint8_t rising_count;
    bool have_good, have_opening, have_no_target;
    uint16_t no_target_count;
} Range;

/* The controller also owns a few counters in each Range, as in the original.
 * Keep this mutable access within app.c; other users should only inspect values. */
Range *Ultrasonic_GetRanges(void);
void Ultrasonic_Init(void);
void Ultrasonic_Task(uint32_t now);
bool Ultrasonic_IsTriggerActive(void);
bool Ultrasonic_IsIdle(void);
bool Ultrasonic_LatestGood(SensorId id, uint32_t now);
bool Ultrasonic_RecentGood(SensorId id, uint32_t now, uint32_t age);
bool Ultrasonic_NoTarget(SensorId id, uint32_t now);
bool Ultrasonic_RecentObservation(SensorId id, uint32_t now);

#endif /* RC_ULTRASONIC_H */
