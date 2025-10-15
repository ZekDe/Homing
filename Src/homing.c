#include "homing.h"
#include "string.h"
#include "retarget.h"

static const homing_funcs_t *funcs_ptr = NULL;


static void updateSwitchInputs(homing_t *homing, uint32_t current_time);
static void setActuatorDirection(actuator_direction_t dir);
static void transitionToError(homing_t *homing, homing_error_t error);
static void stopActuator(void);
static void resetAllTimers(homing_t *homing);



void homingInit(homing_t *homing, const homing_funcs_t *funcs)
{
    // Store HAL pointer
	funcs_ptr = funcs;

    // Initialize structure
    memset(homing, 0, sizeof(homing_t));

    // Set configuration parameters
    homing->debounce_time_ms = HOMING_DEBOUNCE_TIME;
    homing->timeout_ms = HOMING_TIMEOUT;
    homing->settle_time_ms = HOMING_SETTLE_TIME;
    homing->retry_count = HOMING_RETRY_COUNT;

    homing->state = HOMING_STATE_IDLE;
    homing->is_homing_active = 0;
    homing->is_homed = 0;
}


uint8_t homingStart(homing_t *homing)
{
    if (homing->is_homing_active)
    {
        return 0; // Already running
    }

    // Reset state
    homing->state = HOMING_STATE_INIT;
    homing->error = HOMING_ERROR_NONE;
    homing->is_homing_active = 1;
    homing->is_homed = 0;
    homing->progress_percent = 0;
    homing->current_retry = 0;
    homing->extend_travel_time_ms = 0;

    // Reset all timers and edge detectors
    resetAllTimers(homing);

    return 1;
}




void homingProcess(homing_t *homing)
{
    if (!homing->is_homing_active)
    {
        return;
    }

    uint32_t current_time = funcs_ptr->getSysTick();

    updateSwitchInputs(homing, current_time);

    uint8_t state_changed = (homing->state != homing->prevstate);
    homing->prevstate = homing->state;

    // Check timeout for movement states
    uint8_t timeout_occurred = 0;
    if (homing->state >= HOMING_STATE_MOVE_TO_RETRACT_LIMIT &&
        homing->state <= HOMING_STATE_MOVE_TO_CENTER)
    {
        timeout_occurred = TON(&homing->ton_timeout, 1, current_time,
                              homing->timeout_ms);

        if (timeout_occurred)
        {
            transitionToError(homing, HOMING_ERROR_TIMEOUT);
            return;
        }
    }



    switch (homing->state)
    {
        case HOMING_STATE_INIT:
        {
            homing->progress_percent = 5;
            homing->state = HOMING_STATE_MOVE_TO_RETRACT_LIMIT;

            homing->ton_timeout.aux = 0;

            setActuatorDirection(ACTUATOR_DIR_RETRACT);
            if(state_changed)
            	print("HOMING_STATE_INIT\r\n");
            break;
        }

        case HOMING_STATE_MOVE_TO_RETRACT_LIMIT:
        {
            homing->progress_percent = 15;
            if(state_changed)
            	print("GOTO RETRACT SW\r\n");
            if (homing->retract_switch_pulse)
            {
                stopActuator();
                homing->ton_settle.aux = 0;
                homing->state = HOMING_STATE_SETTLE_AT_RETRACT;
            }
            break;
        }

        case HOMING_STATE_SETTLE_AT_RETRACT:
        {
            homing->progress_percent = 20;

            //Settle süresini bekle
            if (TON(&homing->ton_settle, 1, current_time, homing->settle_time_ms))
            {
                // Settle tamamlandı, ölçüm state'ine geç
                homing->retract_limit_reached_time = current_time;
                homing->ton_timeout.aux = 0;
                homing->state = HOMING_STATE_MEASURE_EXTEND;
                setActuatorDirection(ACTUATOR_DIR_EXTEND);
                print("HOMING_STATE_SETTLE_AT_RETRACT\r\n");
            }
            break;
        }

        case HOMING_STATE_MEASURE_EXTEND:
        {
            homing->progress_percent = 40;
            if(state_changed)
            print("GOTO EXTEND SW\r\n");
            if (homing->extend_switch_pulse)
            {
                //Toplam yol süresini hesapla
                homing->extend_travel_time_ms = current_time - homing->retract_limit_reached_time;

                //Yol süresini doğrula
                if (homing->extend_travel_time_ms < 100)
                {
                    transitionToError(homing, HOMING_ERROR_INVALID_TRAVEL);
                    break;
                }

                stopActuator();
                homing->ton_settle.aux = 0;
                homing->state = HOMING_STATE_SETTLE_AT_EXTEND;
            }
            break;
        }

        case HOMING_STATE_SETTLE_AT_EXTEND:
        {
            homing->progress_percent = 50;

            // Settle süresini bekle
            if (TON(&homing->ton_settle, 1, current_time, homing->settle_time_ms))
            {
                // Settle tamamlandı, merkeze gitmeye başla
                homing->ton_timeout.aux = 0;
                homing->ton_center_move.aux = 0;
                homing->state = HOMING_STATE_MOVE_TO_CENTER;
                setActuatorDirection(ACTUATOR_DIR_RETRACT);
                print("HOMING_STATE_SETTLE_AT_EXTEND\r\n");
            }
            break;
        }

        case HOMING_STATE_MOVE_TO_CENTER:
        {
            uint32_t half_travel = homing->extend_travel_time_ms / 2;

            // Progress hesaplama
            if (half_travel > 0 && homing->ton_center_move.aux)
            {
                uint32_t elapsed = current_time - homing->ton_center_move.since;
                uint32_t progress_range = (elapsed * 45) / half_travel;
                homing->progress_percent = 50 + (uint8_t)progress_range;
                if (homing->progress_percent > 95)
                {
                    homing->progress_percent = 95;
                }
            }

            // Yarı yolu bekle
            if (TON(&homing->ton_center_move, 1, current_time, half_travel))
            {
                stopActuator();
                homing->state = HOMING_STATE_COMPLETE;
                homing->is_homed = 1;
                homing->progress_percent = 100;
                print("NOW CENTER POS\r\n");
            }
            break;
        }

        case HOMING_STATE_COMPLETE:
        {
            homing->is_homing_active = 0;
            if(state_changed)
            print("COMPLETED\r\n");
            break;
        }

        case HOMING_STATE_ERROR:
        {
            stopActuator();
            print("ERROR\r\n");
            // Retry logic
            if (homing->current_retry < homing->retry_count)
            {
                homing->current_retry++;
                homing->state = HOMING_STATE_INIT;
                homing->error = HOMING_ERROR_NONE;
                resetAllTimers(homing);
            }
            else
            {
                homing->is_homing_active = 0;
            }
            break;
        }

        default:
        {
            homing->state = HOMING_STATE_IDLE;
            homing->is_homing_active = 0;
            break;
        }
    }

}



void homingAbort(homing_t *homing)
{
    stopActuator();
    homing->state = HOMING_STATE_IDLE;
    homing->is_homing_active = 0;
    homing->is_homed = 0;
    homing->progress_percent = 0;
    resetAllTimers(homing);
}

uint8_t homingIsComplete(const homing_t *homing)
{
    return homing->is_homed;
}

uint8_t homingIsActive(const homing_t *homing)
{
    return homing->is_homing_active;
}

homing_error_t homingGetError(const homing_t *homing)
{
    return homing->error;
}

uint8_t homingGetProgress(const homing_t *homing)
{
    return homing->progress_percent;
}

homing_state_t homingGetState(const homing_t *homing)
{
    return homing->state;
}



static void updateSwitchInputs(homing_t *homing, uint32_t current_time)
{
    // Read raw switch states from hardware
    homing->retract_switch_raw = funcs_ptr->readRetractSwitch();
    homing->extend_switch_raw = funcs_ptr->readExtendSwitch();

    // Debounce retract switch with TON
    homing->retract_switch_debounced = TON(&homing->ton_retract_switch,
                                           homing->retract_switch_raw,
                                           current_time,
                                           homing->debounce_time_ms);

    // Debounce extend switch with TON
    homing->extend_switch_debounced = TON(&homing->ton_extend_switch,
                                          homing->extend_switch_raw,
                                          current_time,
                                          homing->debounce_time_ms);

    // Detect rising edges (0->1 transition detection)
    homing->retract_switch_pulse = edgeDetection(&homing->ed_retract_switch,
                                                        homing->retract_switch_debounced);

    homing->extend_switch_pulse = edgeDetection(&homing->ed_extend_switch,
                                                       homing->extend_switch_debounced);
}

static void setActuatorDirection(actuator_direction_t dir)
{
    if (funcs_ptr && funcs_ptr->setActuatorDirection)
    {
        funcs_ptr->setActuatorDirection(dir);
    }
}


static void stopActuator(void)
{
    setActuatorDirection(ACTUATOR_DIR_STOP);
}

static void transitionToError(homing_t *homing, homing_error_t error)
{
    stopActuator();
    homing->error = error;
    homing->state = HOMING_STATE_ERROR;
}

static void resetAllTimers(homing_t *homing)
{
    // Reset all TON timer auxiliary variables
    homing->ton_retract_switch.aux = 0;
    homing->ton_extend_switch.aux = 0;
    homing->ton_timeout.aux = 0;
    homing->ton_settle.aux = 0;
    homing->ton_center_move.aux = 0;

    // Reset edge detection auxiliary variables
    homing->ed_retract_switch.aux = 0;
    homing->ed_extend_switch.aux = 0;
}
