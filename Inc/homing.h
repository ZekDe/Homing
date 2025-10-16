#ifndef HOMING_H
#define HOMING_H

#include "stdint.h"
#include "ton.h"
#include "edge_detection.h"

#define HOMING_DEBOUNCE_TIME          50
#define HOMING_TIMEOUT               30000
#define HOMING_SETTLE_TIME           100
#define HOMING_RETRY_COUNT           3

#define ACTUATOR_STROKE_MM  100

#define HOMING_DEBUG


typedef enum
{
    ACTUATOR_DIR_STOP = 0,
    ACTUATOR_DIR_EXTEND,
    ACTUATOR_DIR_RETRACT
} actuator_direction_t;


typedef enum
{
    HOMING_STATE_IDLE = 0,
    HOMING_STATE_INIT,
    HOMING_STATE_MOVE_TO_RETRACT_LIMIT, // retract switch going to known position
    HOMING_STATE_SETTLE_AT_RETRACT, // wait there to be sure
    HOMING_STATE_MEASURE_EXTEND,
    HOMING_STATE_SETTLE_AT_EXTEND, // wait there to be sure
	HOMING_STATE_MEASURE_RETRACT, // *
    HOMING_STATE_SETTLE_AT_RETRACT_2, //*		
    HOMING_STATE_MOVE_TO_CENTER,
    HOMING_STATE_COMPLETE,
    HOMING_STATE_ERROR
} homing_state_t;


typedef enum
{
    HOMING_ERROR_NONE = 0,
    HOMING_ERROR_TIMEOUT,
    HOMING_ERROR_SWITCH_STUCK,
    HOMING_ERROR_NO_SWITCH_DETECTED,
    HOMING_ERROR_INVALID_TRAVEL
} homing_error_t;

typedef struct
{
    homing_state_t state;
    homing_state_t prevstate;
    homing_error_t error;

    // TON timers
    ton_t ton_retract_switch;
    ton_t ton_extend_switch;
    ton_t ton_timeout;
    ton_t ton_settle;
    ton_t ton_center_move;

    // Edge detection
    edge_detection_t ed_retract_switch;
    edge_detection_t ed_extend_switch;

    // Switch states
    uint8_t retract_switch_raw;
    uint8_t retract_switch_debounced;
    uint8_t retract_switch_pulse;

    uint8_t extend_switch_raw;
    uint8_t extend_switch_debounced;
    uint8_t extend_switch_pulse;

    // Timing measurements
    uint32_t retract_limit_reached_time;
	uint32_t extend_limit_reached_time;
    uint32_t extend_travel_time_ms;
	uint32_t retract_travel_time_ms;

    // Configuration
    uint32_t debounce_time_ms;
    uint32_t timeout_ms;
    uint32_t settle_time_ms;
    uint8_t retry_count;
    uint8_t current_retry;

    // Status flags
    uint8_t is_homing_active;
    uint8_t is_homed;
    uint8_t progress_percent;
} homing_t;


typedef struct
{
    void (*setActuatorDirection)(actuator_direction_t dir);
    uint8_t (*readRetractSwitch)(void);
    uint8_t (*readExtendSwitch)(void);
    uint32_t (*getSysTick)(void);
} homing_funcs_t;

void homingInit(homing_t *homing, const homing_funcs_t *funcs);
uint8_t homingStart(homing_t *homing);
void homingProcess(homing_t *homing);
void homingAbort(homing_t *homing);
uint8_t homingIsComplete(const homing_t *homing);
uint8_t homingIsActive(const homing_t *homing);
homing_error_t homingGetError(const homing_t *homing);
uint8_t homingGetProgress(const homing_t *homing);
homing_state_t homingGetState(const homing_t *homing);

#endif
