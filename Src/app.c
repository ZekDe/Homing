#include "app.h"
#include "main.h"
#include "homing.h"

static void setActuatorDirection(actuator_direction_t dir)
{
    switch (dir)
    {
        case ACTUATOR_DIR_EXTEND:
            HAL_GPIO_WritePin(ACTUATOR_EXTEND_PORT, ACTUATOR_EXTEND_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ACTUATOR_RETRACT_PORT, ACTUATOR_RETRACT_PIN, GPIO_PIN_RESET);
            break;

        case ACTUATOR_DIR_RETRACT:
            HAL_GPIO_WritePin(ACTUATOR_EXTEND_PORT, ACTUATOR_EXTEND_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_RETRACT_PORT, ACTUATOR_RETRACT_PIN, GPIO_PIN_SET);
            break;

        case ACTUATOR_DIR_STOP:
        default:
            HAL_GPIO_WritePin(ACTUATOR_EXTEND_PORT, ACTUATOR_EXTEND_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_RETRACT_PORT, ACTUATOR_RETRACT_PIN, GPIO_PIN_RESET);
            break;
    }
}

uint8_t retractswitch, extendswitch;
static uint8_t readRetractSwitch(void)
{
	return retractswitch;
    //return HAL_GPIO_ReadPin(SWITCH_RETRACT_PORT, SWITCH_RETRACT_PIN);
}

static uint8_t readExtendSwitch(void)
{
	return extendswitch;
    //return HAL_GPIO_ReadPin(SWITCH_EXTEND_PORT, SWITCH_EXTEND_PIN);
}

static uint32_t getSysTick(void)
{
    return HAL_GetTick();
}


static homing_t homing_obj;

static const homing_funcs_t homing_funcs =
{
    .setActuatorDirection = setActuatorDirection,
    .readRetractSwitch = readRetractSwitch,
    .readExtendSwitch = readExtendSwitch,
    .getSysTick = getSysTick
};

static ton_t ton_btn_startstop;
static edge_detection_t ed_btn_startstop;

static ton_t ton_blink;

static uint8_t blink;

void runOne(void)
{
	homingInit(&homing_obj, &homing_funcs);
}

void run(void)
{
	uint32_t now = HAL_GetTick();
	uint8_t start_pulse = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
	start_pulse = TON(&ton_btn_startstop, start_pulse, now, 50);
	start_pulse = edgeDetection(&ed_btn_startstop, start_pulse);

	if(start_pulse && !homingIsActive(&homing_obj))
		{homingStart(&homing_obj);}

	homingProcess(&homing_obj);

	if (homingIsActive(&homing_obj))
	{
		 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, blink);
	}
	else if (homingIsComplete(&homing_obj))
	{
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}
	else if (homingGetError(&homing_obj) != HOMING_ERROR_NONE)
	{
	 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, blink);
	 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}

	// BLINKS
	if (TON(&ton_blink, 1, now, 100))
	{
		ton_blink.aux = 0;
		blink = !blink;
	}
	//blink_pulse = edgeDetection(&ed_blink, blink_pulse);
}

















