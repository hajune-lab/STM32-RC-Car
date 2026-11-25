#ifndef RC_RC_CONFIG_H
#define RC_RC_CONFIG_H

/* 주행/속도/센서 튜닝 값: 원본 REBUILD_16_BT와 동일합니다. */
#define APP_VERSION             "REBUILD_16_BT"
/* Physical channels remain ENA=PB8, ENB=PB9. Only logical side assignment. */
#ifndef MOTOR_SWAP_SIDES
#define MOTOR_SWAP_SIDES        0
#endif
#define ENA_FORWARD_SIGN        1 /* ENA positive: PB13 high */
#define ENB_FORWARD_SIGN        1 /* ENB positive: PB15 high */
#define PWM_PERIOD              4999U
#define DRIVE_MIN_PWM           65
#define DRIVE_SLOW_PWM          70
#define DRIVE_FAST_PWM          100
#define DRIVE_CRUISE_PWM        90
#define FAST_FRONT_CM           65.0f
#define FAST_FRONT_AGE_MS       120U
#define FAST_SIDE_AGE_MS        220U
#define FAST_SIDE_MARGIN_CM    12.0f
#define FAST_CENTER_ERROR      0.35f
#define CRUISE_SIDE_MARGIN_CM  14.0f /* keep ordinary cruise scaling unchanged */
#define FAST_CLOSING_CMPS       5.0f
#define FAST_CONFIRM_SAMPLES   2U
#define CRUISE_FULL_FRONT_CM   55.0f /* reach cruise sooner; independent of wall/gap detection */
#define TURN_PWM                72 /* counterrotation */
#define CORNER_OUTER_PWM        78
#define ARC_INNER_PWM           65
#define ARC_OUTER_PWM           82
#define ARC_FRONT_CM            20.0f
#define ARC_INNER_SPACE_CM      30.0f
#define ARC_OUTER_SPACE_CM      15.0f
#define ARC_MAX_RUN_MS          180U /* applied rolling turn, then tighten radius */
#define TURN_FRONT_AGE_MS       120U
#define BACK_PWM                72
#define MANUAL_DRIVE_PWM        100
#define MANUAL_TURN_PWM         85
#define TEST_PWM                72
#define LEFT_TRIM               0
#define RIGHT_TRIM              0
#define MOTOR_RAMP_STEP         5
#define MOTOR_DECEL_STEP        6
#define MOTOR_PERIOD_MS         10U
#define REVERSE_DEAD_MS         40U

/* Geometry distances; a 3 cm margin releases a corner before long-range acceleration. */
/* 정상 직진은 약한 PD 보정만 사용합니다. 회전 상태는 정면 장애물로만 시작.
 * 센서 무응답만으로 즉석 좌/우 회전을 만들지 않습니다.
 * 일반 코너는 한쪽 바퀴를 멈춰 회전하고, 좁을 때만 반대 바퀴를 후진.
 * 회전 중에도 순차 측정합니다. 정상 측정을 위한 주기적인 정지는 없습니다.
 * 새 정면값이 열림을 보이면 회전 출력을 먼저 풀고 전진 보정으로 복귀.
 * 정상 측정 주기가 끝나도 반사 신호가 없으면 NO_TARGET: 기본 서행 직진.
 * NO_RISE/상시 HIGH 같은 실제 신호 이상만 재측정 상태에서 대기.
 */
#define FRONT_EMERGENCY_CM      10.0f /* try stationary turn before straight reverse */
#define FRONT_BACK_CM           6.0f
#define ROTATE_SIDE_CM          9.0f
#define CLOSE_ROTATE_MS         220U /* accumulated applied rotation while front stays close */
#define FRONT_TURN_CM           24.0f
#define FRONT_CLEAR_CM          28.0f /* acceleration reference */
#define CORNER_RELEASE_CM       (FRONT_TURN_CM + 3.0f)
#define FRONT_SLOW_CM           75.0f
#define TIGHT_TURN_CM           14.0f
#define SIDE_CLOSE_CM           7.0f
#define SIDE_AVOID_START_CM     16.0f
#define SIDE_AVOID_FULL_CM      7.0f
#define SIDE_AVOID_MAX_PWM      10.0f
#define SIDE_AVOID_DIFF_CM      2.0f
#define SIDE_FULL_SLOW_CM       11.0f
#define SPEED_BALANCE_GAIN      1.0f
#define SIDE_LOOKAHEAD_S        0.12f
#define SIDE_PREDICT_MAX_CM     4.0f
#define SIDE_CLOSING_MAX_CMPS   40.0f
#define SIDE_TIGHT_TURN_CM      11.0f /* 바깥쪽 벽 근접 시 전진 성분을 없앰 */
#define SIDE_WALL_LIMIT_CM      80.0f
#define CORNER_CONFIRM_SAMPLES  2U
#ifndef CENTER_KP
#define CENTER_KP               9.0f
#endif
#ifndef CENTER_KD
#define CENTER_KD               0.8f
#endif
#ifndef CENTER_MAX_PWM
#define CENTER_MAX_PWM          4.0f
#endif
#define CENTER_DEADBAND         0.05f
#define WALL_JUMP_RATIO         1.8f
#define OPENING_HINT_MS         1000U
#define GAP_GROWTH_RATIO        1.6f
#define GAP_GROWTH_CM           6.0f
#define GAP_CONFIRM_SAMPLES     2U

#define CONTROL_MS              10U
#define RANGE_FRESH_MS          300U
#define FRONT_HOLD_MS           350U
#define ALL_STALE_MS            500U
#define ACQUIRE_SETTLE_MS       40U
#define TURN_STALL_MS           1400U
#define TURN_PROGRESS_CM        3.0f
#define AFTER_TURN_MS           300U
#define CORNER_CLEAR_RUN_MS     450U /* applied forward output, not wall-clock delay */
#define CORNER_CLEAR_SAMPLES    3U
#define BACK_RUN_MS             100U
#define BUTTON_DEBOUNCE_MS      30U
#define TEST_RUN_MS             500U
#define LOG_MS                  500U

#define TRIG_US                 12U
#define PING_SPACING_MS         40U
#define SAME_SENSOR_MS          65U
#define QUIET_MS                12U
#define ECHO_TIMEOUT_MS         30U
#define RANGE_MIN_CM            2.0f
#define RANGE_MAX_CM            400.0f
#define ECHO_GLITCH_US          60U
#define FRONT_CAL_A             1.0f
#define FRONT_CAL_B             0.0f
#define LEFT_CAL_A              1.0f
#define LEFT_CAL_B              0.0f
#define RIGHT_CAL_A             1.0f
#define RIGHT_CAL_B             0.0f


#endif /* RC_RC_CONFIG_H */
