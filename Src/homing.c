#include "homing.h"
#include "string.h"
#include "stdio.h"

static const homing_funcs_t *funcs_ptr = NULL;


static void updateSwitchInputs(homing_t *homing, uint32_t current_time);
static void setActuatorDirection(actuator_direction_t dir);
static void transitionToError(homing_t *homing, homing_error_t error);
static void stopActuator(void);
static void resetAllTimers(homing_t *homing);


void homingInit(homing_t *homing, const homing_funcs_t *funcs)
{
	funcs_ptr = funcs;

    memset(homing, 0, sizeof(homing_t));

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
        return 0;
    }

    homing->state = HOMING_STATE_INIT;
    homing->error = HOMING_ERROR_NONE;
    homing->is_homing_active = 1;
    homing->is_homed = 0;
    homing->progress_percent = 0;
    homing->current_retry = 0;
    homing->extend_travel_time_ms = 0;
	homing->retract_travel_time_ms = 0;

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
			#ifdef HOMING_DEBUG
            if(state_changed)
            	printf("HOMING_STATE_INIT\r\n");
			#endif
            break;
        }

        case HOMING_STATE_MOVE_TO_RETRACT_LIMIT:
        {
            homing->progress_percent = 15;
            if(state_changed)
				#ifdef HOMING_DEBUG
            	printf("GOTO RETRACT SW\r\n");
			#endif
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

            if (TON(&homing->ton_settle, 1, current_time, homing->settle_time_ms))
            {
                homing->retract_limit_reached_time = current_time;
                homing->ton_timeout.aux = 0;
                homing->state = HOMING_STATE_MEASURE_EXTEND;
                setActuatorDirection(ACTUATOR_DIR_EXTEND);
				#ifdef HOMING_DEBUG
                printf("HOMING_STATE_SETTLE_AT_RETRACT\r\n");
				#endif
            }
            break;
        }

        case HOMING_STATE_MEASURE_EXTEND:
        {
            homing->progress_percent = 40;
			#ifdef HOMING_DEBUG
            if(state_changed)
				printf("GOTO EXTEND SW\r\n");
			#endif
            if (homing->extend_switch_pulse)
            {
                homing->extend_travel_time_ms = current_time - homing->retract_limit_reached_time;

                if (homing->extend_travel_time_ms < 100)
                {
                    transitionToError(homing, HOMING_ERROR_INVALID_TRAVEL);
                    break;
                }
				#ifdef HOMING_DEBUG
				printf("extend_travel_time_ms = %lu\r\n", homing->extend_travel_time_ms);
				#endif
                stopActuator();
                homing->ton_settle.aux = 0;
                homing->state = HOMING_STATE_SETTLE_AT_EXTEND;
            }
            break;
        }

        case HOMING_STATE_SETTLE_AT_EXTEND:
        {
            homing->progress_percent = 50;

            if (TON(&homing->ton_settle, 1, current_time, homing->settle_time_ms))
            {
				homing->extend_limit_reached_time = current_time;
                homing->ton_timeout.aux = 0;
                homing->ton_center_move.aux = 0;
                homing->state = HOMING_STATE_MEASURE_RETRACT;
                setActuatorDirection(ACTUATOR_DIR_RETRACT);
				#ifdef HOMING_DEBUG
                printf("HOMING_STATE_SETTLE_AT_EXTEND\r\n");
				#endif
            }
            break;
        }

		// ****
		case HOMING_STATE_MEASURE_RETRACT:
        {                
            homing->progress_percent = 60;
            #ifdef HOMING_DEBUG
			if (state_changed)
                printf("GOTO RETRACT SW\r\n");
			#endif
            if (homing->retract_switch_pulse)
            {

                homing->retract_travel_time_ms = current_time - homing->extend_limit_reached_time;
                #ifdef HOMING_DEBUG
                printf("retract_travel_time_ms = %lu\r\n", homing->retract_travel_time_ms);
				#endif
                if (homing->retract_travel_time_ms < 100)
                {
                    transitionToError(homing, HOMING_ERROR_INVALID_TRAVEL);
                    break;
                }

                stopActuator();
                homing->ton_settle.aux = 0;
                homing->state = HOMING_STATE_SETTLE_AT_RETRACT_2;
            }
            break;
        }
		
		case HOMING_STATE_SETTLE_AT_RETRACT_2:
        {      
            homing->progress_percent = 70;

            // Settle süresini bekle
            if (TON(&homing->ton_settle, 1, current_time, homing->settle_time_ms))
            {
                homing->ton_timeout.aux = 0;
                homing->ton_center_move.aux = 0;
                homing->state = HOMING_STATE_MOVE_TO_CENTER;
                setActuatorDirection(ACTUATOR_DIR_EXTEND);
				#ifdef HOMING_DEBUG
				printf("HOMING_STATE_SETTLE_AT_RETRACT_2\r\n");
				#endif
            }
            break;
        }
		
		case HOMING_STATE_MOVE_TO_CENTER:
        {
            //Extend yönünde ölçülen sürenin yarısı kadar git
            uint32_t half_extend_time = homing->extend_travel_time_ms / 2;

            //Progress hesaplama
            if (half_extend_time > 0 && homing->ton_center_move.aux)
            {
                uint32_t elapsed = current_time - homing->ton_center_move.since;
                uint32_t progress_range = (elapsed * 25) / half_extend_time;
                homing->progress_percent = 70 + (uint8_t)progress_range;
				
                if (homing->progress_percent > 95)
                {
                    homing->progress_percent = 95;
                }
            }

            //Extend yönünde yarı yolu bekle
            if (TON(&homing->ton_center_move, 1, current_time, half_extend_time))
            {
                stopActuator();
                homing->state = HOMING_STATE_COMPLETE;
                homing->is_homed = 1;
                homing->progress_percent = 100;
				#ifdef HOMING_DEBUG
                printf("NOW AT CENTER POSITION!\r\n");
                printf("Extend time: %lu ms, Retract time: %lu ms\r\n", 
                      homing->extend_travel_time_ms, 
                      homing->retract_travel_time_ms);
				printf("spped ratio(E/R): %f\r\n",homing->extend_travel_time_ms / (float)homing->retract_travel_time_ms);
				
				float extend_velocity = ACTUATOR_STROKE_MM  / (float)homing->extend_travel_time_ms;
				float retract_velocity = ACTUATOR_STROKE_MM / (float)homing->retract_travel_time_ms;

				printf("Extend speed: %f m/s\r\n", extend_velocity);
				printf("Retract speed: %f m/s\r\n", retract_velocity);
				#endif
				
            }
            break;
        }

        case HOMING_STATE_COMPLETE:
        {
            homing->is_homing_active = 0;
			#ifdef HOMING_DEBUG
            if(state_changed)
				printf("COMPLETED\r\n");
			#endif
            break;
        }

        case HOMING_STATE_ERROR:
        {
            stopActuator();
			#ifdef HOMING_DEBUG
            printf("ERROR\r\n");
			#endif
            //Retry
            if (homing->current_retry < homing->retry_count)
            {
                homing->current_retry++;
				#ifdef HOMING_DEBUG
				printf("Retry %d/%d\r\n", homing->current_retry, homing->retry_count);
				#endif
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
