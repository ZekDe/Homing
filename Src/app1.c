#include "app.h"
#include "SW2023.h"
#include "pinConfig.h"
#include "ton.h"
#include "edge_detection.h"
#include "systemtick.h"
#include "steady_clock.h"
#include "MXADC.h"
#include "MXIR.h"
#include "ir_receiver.h"
#include "MXParams.h"
#include "app_relay.h"
#include "buzzer_nonblocking.h"
#include "weekly_schedule.h"
#include "lcdlib_ex.h"
#include "lcdzone.h"
#include "alarm_manager.h"
#include "device_context.h"
#include "macro.h"
#include "utils.h"
#include "buzzer_patterns.h"
#include "flash_manager.h"
#include "device_flash_operations.h"
#include "string_utils.h"
#include "device_functions.h"
#include "ring_buffer_getline.h"
#include "device_temperature_control.h"


#define BTN_COUNT 	4
#define BUTTONS 	BTN01, BTN02, BTN03, BTN04

static void deviceTestRun(void);
static void setRtcByTimeLib(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second);

static void handleOnOp(void);
static void handleOffOp(void);
static void handleMainScreenOp(uint8_t blink_state);
static void handleTempSetPointOp(uint8_t blink_state, uint8_t blink_btn);
static void handleMenuOp(uint8_t blink_state, uint8_t blink_btn);

static void menuWeeklyScheduleCleanUp(void);
static void menuOpCleanUp(void);
static void tempSetPointOpCleanUp(void);
static void menuRtcCleanUp(void);
static void mainScreenOpCleanUp(void);
static void menuNavigationCleanUp(void);
static void menuWifiSettingsCleanUp(void);

static int printDayOfWeekByDateTime(int y, int mon, int d, int h, int min);
static void printTimeAndDayOfWeekByIdx(int h, int min, int wd);
static void printTimeAndDayOfWeek(void);
static void printMenuNumber(uint8_t menu_number);
static void printMenuSelectionRtc(void);
static void menuRtcHelperPrintCase(uint8_t case_, int y, int mon, int d, int wd, int h, int min);

static void menuNavigation(uint8_t *menu_idx, uint8_t *time_idx, uint8_t *schedule_idx, uint8_t blink_state);
static void menuRtcHelperClearScreen(uint8_t case_, int wd);
static void menuRtc(uint8_t blink_state, uint8_t blink_btn, uint8_t *time_idx, uint8_t *menu_idx);
static void menuWeeklySchedule(uint8_t blink_state, uint8_t blink_btn, uint8_t *schedule_idx, uint8_t *menu_idx);
static void menuWeeklySchedulePrintSelectedDays(uint8_t blink_state, uint8_t selected_day);

static void handleMainScreenModeSymbol(uint8_t blink_state);

void setArrows(uint8_t val);
void updateNtcErr(float fval);
void sendRfTelemetryData(void);

static void handleRfCommands(char* data);
static void handleIrCommands(void);
static void handleBtnTouchBacklight(void);
static void handleChildLock(void);
static void handleNoTouch(void);
static void handleFallOver(void);
static void handleErrBlink(uint8_t blink_state);

void cleanUps(void);


static edge_detection_t ed_onoff,ed_onoff_long, ed_plus, ed_minus, ed_menu, ed_child_lock_long;
static edge_detection_t ed_menu_falling;
static edge_detection_t ed_plus_long, ed_minus_long, ed_menu_long, ed_btn_device_on;


static edge_detection_t ed_mainscreen_op_cleanup;
static edge_detection_t ed_temp_setpoint_op_cleanup;
static edge_detection_t ed_menu_op_cleanup;

static edge_detection_t ed_menu_weekly_sche_cleanup;
static edge_detection_t ed_menu_rtc_cleanup;
static edge_detection_t ed_menu_navigation_cleanup;
static edge_detection_t ed_menu_temp_ctrl_sel_cleanup;
static edge_detection_t ed_menu_wifi_settings_cleanup;

static ton_t ton_onoff,ton_plus,ton_minus,ton_menu, ton_btn_device_on, ton_child_lock, ton_child_lock_pressed;
static ton_t ton_onoff_pressed, ton_plus_pressed, ton_minus_pressed, ton_menu_pressed, ton_touched;

static uint8_t btn_onoff, btn_onoff_long, btn_onoff_pulse, btn_onoff_long_pulse;
static uint8_t btn_plus, btn_plus_long, btn_plus_pulse, btn_plus_long_pulse;
static uint8_t btn_minus, btn_minus_long, btn_minus_pulse, btn_minus_long_pulse;
static uint8_t btn_menu, btn_menu_long, btn_menu_pulse, btn_menu_long_pulse;
static uint8_t btn_device_on, btn_device_on_pulse, anybutton_backlight;
static uint8_t btn_child_lock, btn_child_lock_long, btn_child_lock_long_pulse;
static uint8_t btn_menu_pulse_falling;

static uint8_t ir_onoff_pressed, ir_onoff_pulse, ir_plus_pulse, ir_minus_pulse, ir_plus_pressed, ir_minus_pressed, ir_repeat;

volatile uint8_t fail_pin;


static uint8_t day_symbol[7] = {SYMBOL_SUN, SYMBOL_MON, SYMBOL_TUE, SYMBOL_WED, SYMBOL_THU, SYMBOL_FRI, SYMBOL_SAT};

state_t state = IDLE;
state_t prev_state = IDLE;

static inline void setState(state_t new_state)
{
    __disable_irq();
    prev_state = state;
    state = new_state;
    __enable_irq();
}





















#include "homing.h"
static void setActuatorDirection(actuator_direction_t dir)
{
    switch (dir)
    {
        case ACTUATOR_DIR_EXTEND:
			lcdSetSymbol(SYMBOL_ARROW1, 1);
            lcdSetSymbol(SYMBOL_ARROW2, 0);
            break;

        case ACTUATOR_DIR_RETRACT:
            lcdSetSymbol(SYMBOL_ARROW1, 0);
            lcdSetSymbol(SYMBOL_ARROW2, 1);
            break;

        case ACTUATOR_DIR_STOP:
        default:
            lcdSetSymbol(SYMBOL_ARROW1, 0);
            lcdSetSymbol(SYMBOL_ARROW2, 0);
            break;
    }
}



static uint8_t readRetractSwitch(void)
{
	return !BTN03;
    //return HAL_GPIO_ReadPin(SWITCH_RETRACT_PORT, SWITCH_RETRACT_PIN);
}

static uint8_t readExtendSwitch(void)
{
	return !BTN04;
    //return HAL_GPIO_ReadPin(SWITCH_EXTEND_PORT, SWITCH_EXTEND_PIN);
}

static uint32_t getSysTick(void)
{
    return systick;
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
	deviceModuleStart();
	buzzerInit();

	homingInit(&homing_obj, &homing_funcs);
}


void run(void)
{
	static ton_t ton_blink, ton_blink2, ton_blink3, ton_btn, ton_btn2;
	static uint8_t blink_state, blink_state_pulse,blink_state2, blink_state3_pulse; 
	static uint8_t blink_btn_pulse, blink_adc_pulse, reserve[2];
	
	static edge_detection_t ed_blink, ed_blink_btn, ed_blink3;
	
	enum {BTN_PRESSED_TIMEOUT = 75, ONOFF_LONG_TIMEOUT = 1000, BTN_LONG_TIMEOUT = 1000, 
	MENU_LONG_TIMEOUT = 1000, BTN_DEVICE_ON_TIMEOUT = 350, BTN_CHILD_LOCK_TIMEOUT = 2000,};//ms
	
	btn_onoff = !BTN01 && BTN02 && BTN03 && BTN04;
	btn_onoff = TON(&ton_onoff_pressed, btn_onoff, systick, BTN_PRESSED_TIMEOUT);
	btn_onoff_long = TON(&ton_onoff, btn_onoff, systick, ONOFF_LONG_TIMEOUT);
	btn_onoff_pulse = edgeDetection(&ed_onoff, btn_onoff);
	btn_onoff_long_pulse = edgeDetection(&ed_onoff_long, btn_onoff_long);
	btn_device_on = TON(&ton_btn_device_on, btn_onoff, systick, BTN_DEVICE_ON_TIMEOUT);
	btn_device_on_pulse = edgeDetection(&ed_btn_device_on, btn_device_on);
	
	btn_plus  = BTN01 && BTN02 && !BTN03 && BTN04;
	btn_plus = TON(&ton_plus_pressed, btn_plus, systick, BTN_PRESSED_TIMEOUT);
	btn_plus_long = TON(&ton_plus, btn_plus, systick, BTN_LONG_TIMEOUT);
	btn_plus_pulse = edgeDetection(&ed_plus, btn_plus);
	btn_plus_long_pulse = edgeDetection(&ed_plus_long, btn_plus_long);
	
	btn_minus  = BTN01 && !BTN02 && BTN03 && BTN04;
	btn_minus = TON(&ton_minus_pressed, btn_minus, systick, BTN_PRESSED_TIMEOUT);
	btn_minus_long = TON(&ton_minus, btn_minus, systick, BTN_LONG_TIMEOUT);
	btn_minus_pulse = edgeDetection(&ed_minus, btn_minus);
	btn_minus_long_pulse = edgeDetection(&ed_minus_long, btn_minus_long);
	
	btn_menu  = BTN01 && BTN02 && BTN03 && !BTN04;
	btn_menu = TON(&ton_menu_pressed, btn_menu, systick, BTN_PRESSED_TIMEOUT);
	btn_menu_long = TON(&ton_menu, btn_menu, systick, MENU_LONG_TIMEOUT);
	btn_menu_pulse = edgeDetection(&ed_menu, btn_menu);
	btn_menu_pulse_falling = edgeDetection(&ed_menu_falling, !btn_menu);
	btn_menu_long_pulse = edgeDetection(&ed_menu_long, btn_menu_long);
	
	btn_child_lock = !BTN01 && BTN02 && BTN03 && !BTN04;
	btn_child_lock = TON(&ton_child_lock_pressed, btn_child_lock, systick, BTN_PRESSED_TIMEOUT);
	btn_child_lock_long = TON(&ton_child_lock, btn_child_lock, systick, BTN_CHILD_LOCK_TIMEOUT);
	btn_child_lock_long_pulse = edgeDetection(&ed_child_lock_long, btn_child_lock_long);
	
	
	if(btn_onoff_pulse && !homingIsActive(&homing_obj))
		{homingStart(&homing_obj);}
		
		homingProcess(&homing_obj);

	if (homingIsActive(&homing_obj))
	{
		 setBackLightWRB(blink_state,0,0);
	}
	else if (homingIsComplete(&homing_obj))
	{
		setBackLightWRB(1,0,0);
	}
	else if (homingGetError(&homing_obj) != HOMING_ERROR_NONE)
	{
		setBackLightWRB(0,blink_state,0);
	}
	else
	{
		setBackLightWRB(0,0,0);
	}



	
	// Blinkler ve Blink Pulse'lar(Pulse olanlar sonraki sart saglanana dek, bir kez 1 olur sonraki döngüde 0. 
	
	if (TON(&ton_btn, 1, systick, 50))
	{
		TON(&ton_btn, 0, 0, 0);
		blink_btn_pulse = !blink_btn_pulse; 
	}
	blink_btn_pulse = edgeDetection(&ed_blink_btn, blink_btn_pulse);

	
	if (TON(&ton_blink3, 1, systick, MAINSCREEN_REFRESH_TIMEOUT - 50))
	{
		TON(&ton_blink3, 0, 0, 0);
		blink_state3_pulse = !blink_state3_pulse; 
	}
	blink_state3_pulse = edgeDetection(&ed_blink3, blink_state3_pulse);
	
	
	if (TON(&ton_blink, 1, systick, 100))
	{
		TON(&ton_blink, 0, 0, 0);
		blink_state = !blink_state; 
	}
	blink_state_pulse =  edgeDetection(&ed_blink, blink_state);
	
	if (TON(&ton_blink2, 1, systick, blink_state2 ? 750 : 250))
	{
		TON(&ton_blink2, 0, 0, 0);
		blink_state2 = !blink_state2;
	}
	
	
}


static void handleIrCommands(void)
{
	static edge_detection_t ed_ir_onoff;
	static edge_detection_t ed_ir_plus;
	static edge_detection_t ed_ir_minus;
	static ton_t ton_repeat, ton_nodata;
	
	static uint32_t last_data_time;
	static uint8_t last_command;
	static uint8_t last_address;
	static uint8_t is_active; 
	static uint8_t is_repeat;
	
	ir_data_t ir_data;
	
    uint32_t now = systick;
    
    if (irNecDataAvailable())
    {
        ir_data_t ir_data = irNecGetData();
        
        if (ir_data.is_valid)
        {
			is_active = 1;
            last_data_time = now;
			last_address = ir_data.address;
			last_command = ir_data.command;
			is_repeat = ir_data.is_repeat;
			ir_repeat = TON(&ton_repeat, ir_data.is_repeat, systick, 300);
			TON(&ton_nodata, 0, 0, 0);
        }
        
        irNecClearData();
    }
    
    // Timeout kontrolü
    if (TON(&ton_nodata, 1, systick, 200))
    {
        is_active = 0;
		ir_repeat = 0;
		is_repeat = 0;
		TON(&ton_repeat, 0, 0, 0);
    }
    
    
    ir_onoff_pressed = 0;
    ir_plus_pressed = 0;
    ir_minus_pressed = 0;

    
    if (is_active)
    {
        if ((last_address == DEVICE_ONOFF_IR_ADDR) && (last_command == DEVICE_ONOFF_IR_CMD) && !is_repeat)
        {
            // ONOFF sadece ilk basista
            ir_onoff_pressed = 1;
        }
        else if ((last_address == DEVICE_PLUS_IR_ADDR) && (last_command == DEVICE_PLUS_IR_CMD))
        {
            // PLUS hem ilk basista hem repeat'te
            ir_plus_pressed = 1;
        }
        else if ((last_address == DEVICE_MINUS_IR_ADDR) && (last_command == DEVICE_MINUS_IR_CMD))
        {
            // MINUS hem ilk basista hem repeat'te
            ir_minus_pressed = 1;
        }
    }
    
    // Edge detection
    ir_onoff_pulse = edgeDetection(&ed_ir_onoff, ir_onoff_pressed);
    ir_plus_pulse = edgeDetection(&ed_ir_plus, ir_plus_pressed);
    ir_minus_pulse = edgeDetection(&ed_ir_minus, ir_minus_pressed);

}


static void handleFallOver(void)
{
	if(!PF15)
	{
		SET_ERR(ERR_FALLOVER);
		playErrorSound();
	}
	else
	{
		RESET_ERR(ERR_FALLOVER);
	}	
}

void cleanUps(void)
{
	mainScreenOpCleanUp();
	menuOpCleanUp();
	
	menuWeeklyScheduleCleanUp();
	menuRtcCleanUp();
	menuNavigationCleanUp();
	menuWifiSettingsCleanUp();
}

static void handleNoTouch(void)
{
	// Ilgili alanda ise ve kisi butona tiklamadiysa MAIN_SCREEN_OP dönüs
	if((state == TEMP_SETPOINT_OP) || (state == MENU_OP))
	{
		if(btn_plus_pulse || btn_plus_long_pulse|| btn_plus_long || ir_plus_pulse || ir_minus_pulse
			|| btn_minus_pulse || btn_minus_long_pulse || btn_minus_long ||ir_plus_pressed || ir_minus_pressed
			|| btn_menu_pulse || btn_menu_long_pulse || btn_menu_long)
		{
			TON(&ton_touched, 0, 0, 0);
		}
		
		else if(TON(&ton_touched, 1, systick, NO_TOUCH_TIMEOUT))
		{
			cleanUps();
			tempSetPointOpCleanUp();
			setState(MAIN_SCREEN_OP);
		}
			
	}
}

static void handleChildLock(void)
{
	if(!dev_data.child_lock)
		return;
	if(!BTN01 || !BTN02 || !BTN03 || !BTN04)
		playWarningSound();
}

static void handleBtnTouchBacklight(void)
{
	if(!dev_data.device_on)
		return;
	
    enum { BACKLIGHT_TIMEOUT = 5000, BACKLIGHT_MIN = 10 };
    static ton_t ton_backlight;
	static uint16_t backlight_saved = 0;
    static uint16_t auto_brightness_active = 0;  // ? Flag
	
    
    // Herhangi bir tusa basildi mi?
    uint8_t btn_cond = btn_onoff_pulse || btn_plus_pulse || btn_minus_pulse || btn_menu_pulse;
    
	uint8_t total_cond = btn_cond && !auto_brightness_active;
	
    if (total_cond)
	{
		TON(&ton_backlight, 0, 0, 0);
		anybutton_backlight = 1;
		auto_brightness_active = 1;
		backlight_saved = dev_data.backlight_intensity;
		
		if(backlight_saved < 10)  dev_data.backlight_intensity = 50;
        setBackLightWRB(dev_data.w, dev_data.r, dev_data.b);
    }
    else if (TON(&ton_backlight, 1, systick, BACKLIGHT_TIMEOUT)) 
	{
		if (auto_brightness_active)
        {
            dev_data.backlight_intensity = backlight_saved;
            setBackLightWRB(dev_data.w, dev_data.r, dev_data.b);
            auto_brightness_active = 0;  // Deaktif et
            backlight_saved = 0;
			anybutton_backlight = 0;
        }
    }
}

void updateNtcErr(float current_temp)
{	
	enum{ERROR_LIMIT = 3000, RECOVERY_LIMIT = 3000};
	
	static ton_t ton_err, ton_recovery;
	uint16_t adc_raw = adc_ReadSingleCh(0);
	//uint8_t is_error  = current_temp > 40.0f || current_temp < 1.0f;
	uint8_t is_error = adc_raw > 3850 || adc_raw < 500;
	
	if(TON(&ton_err, is_error , systick, ERROR_LIMIT))
	{
		SET_ERR(ERR_NTC);
		playWarningSound();
	}
	else if(TON(&ton_recovery, !is_error , systick, RECOVERY_LIMIT) && GET_ERR(ERR_NTC))
	{
		RESET_ERR(ERR_NTC);
	}
}
	
static void handleRfCommands(char* data)
{
    bool retval = 0;
	char cmd[30];
    char params[30];
//    trim(cmd);
//    toUpper(cmd);
	
	if (sscanf(data, "%24[^:]:%24s", cmd, params) != 2)
		return;

    if (strcmp(cmd, "DEVICE_ONOFF") == 0)
    {
		int val = -1;
		sscanf(params, "%d", &val) ;
		if(val == 0 && dev_data.device_on) {setState(DEVICE_OFF_OP);}	
		else if(val == 1 && !dev_data.device_on) {setState(DEVICE_ON_OP);}	
    }
	else if (strcmp(cmd, "DATETIME") == 0)
	{
		int32_t year, month, day, hour, min, sec;
        if (sscanf(params, "%04d,%02d,%02d,%02d,%02d,%02d", &year, &month, &day, &hour, &min, &sec) == 6)
        {
            setRtcByTimeLib(year, month, day, hour, min, sec);
			//playBuzzerOK();
        }
	}
	else if (strcmp(cmd, "WRB") == 0)
	{
		int w,r,b;
		if (sscanf(params, "%d,%d,%d", &w, &r, &b) == 3)
		{
			dev_data.w = w;
			dev_data.r = r;
			dev_data.b = b;
		}
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "MQTT") == 0)
	{
		int val = -1;
		sscanf(params, "%d", &val) ;
		dev_data.mqtt_status = val;
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "WIFI") == 0)
	{
		int val = -1;
		sscanf(params, "%d", &val) ;
		dev_data.wifi_status = val;
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "WIFI_EN") == 0)
	{
		int val = -1;
		sscanf(params, "%d", &val) ;
		dev_data.is_wifi_exist = val;
		flashManagerMarkDirty(is_wifi_exist.id);
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "TEMP") == 0)
	{
		float fval = -1;
		sscanf(params, "%f", &fval) ;
		dev_data.temp_setpoint = fval;
		flashManagerMarkDirty(temp_setpoint.id);
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "MODE") == 0)
	{
		int val = 0;
		sscanf(params, "%d", &val) ;
		dev_data.mode = val;
		flashManagerMarkDirty(mode.id);
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "WEEKSCHE") == 0)
	{
		int val = 0;
		sscanf(params, "%d", &val) ;
		g_weekly_schedule.is_active = val;
		flashManagerMarkDirty(weekly_schedule.id);
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "BACKLIGHT_INTENSITY") == 0)
	{
		int val = 0;
		sscanf(params, "%d", &val) ;
		dev_data.backlight_intensity = val;
		flashManagerMarkDirty(backlight_intensity.id);
		//playBuzzerOK();
	}
	else if (strcmp(cmd, "CLEAR_ERRORS") == 0)
	{
		dev_data.err = 0;
		//playBuzzerOK();
	}
}


void setArrows(uint8_t val)
{
	lcdSetSymbol(SYMBOL_ARROW1, val == 1);
	lcdSetSymbol(SYMBOL_ARROW2, val == 2);
	lcdSetSymbol(SYMBOL_ARROW3, val == 3);
	lcdSetSymbol(SYMBOL_ARROW4, val == 4);
	lcdSetSymbol(SYMBOL_ARROW5, val == 5);
}

/**
*	dayOfweek: RTC_SUNDAY ...
*/
static void setRtcByTimeLib(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second)
{

	S_RTC_TIME_DATA_T sDateTimeInit, sDateTimeAlarm;
	
	struct tm t = {0};
	t.tm_year = year - 1900;
	t.tm_mon = month - 1;
	t.tm_mday = day;
	t.tm_hour = hour;
	t.tm_min = minute;
	t.tm_sec = second;
	t.tm_isdst = -1;
	
	mktime(&t);
	

	/* Configure RTC initial date and time structure. */
	sDateTimeInit.u32Year       = year;
	sDateTimeInit.u32Month      = month;
	sDateTimeInit.u32Day        = day;
	sDateTimeInit.u32DayOfWeek  = t.tm_wday;
	sDateTimeInit.u32Hour       = hour;
	sDateTimeInit.u32Minute     = minute;
	sDateTimeInit.u32Second     = second;
	sDateTimeInit.u32TimeScale  = RTC_CLOCK_24;
	sDateTimeInit.u32AmPm       = 0;

	RTC_SetDateAndTime(&sDateTimeInit);
}

static void handleOnOp(void)
{
	dev_data.device_on = 1;
	char *version[2] = {"u1", "00"};
	LCD_SetAllPixels(1);
	
	cleanUps();
	edgeDetection(&ed_temp_setpoint_op_cleanup, 0); 
	//*************************************
	SYS_UnlockReg();
	pinConfig_deinit_pa();
	pinConfig_deinit_bpwm0();
	pinConfig_init_pa_tradeshow();
	pinConfig_init_bpwm0_tradeshow();
	BPWM0_Init_treadeshow();
	GPIO_Init_tradeshow();
	SYS_LockReg();
	playStartupSound();
	
	for(uint8_t i = 0; i < 100; ++i)
	{
		setBackLightPWM(WHITE, i);
		delayms(10);
	}
	setBackLightOffPWM();
	
	SYS_UnlockReg();
	pinConfig_deinit_pa_tradeshow();
	pinConfig_deinit_bpwm0_tradeshow();
	pinConfig_init_pa();
	pinConfig_init_bpwm0();
	BPWM0_Init();
	GPIO_Init();
	SYS_LockReg();
	//*************************************
	LCD_SetAllPixels(0);
	
	lcdPutString(ZONE_DIGIT5_DIGIT, "ON ");
	lcdPutString(ZONE_DIGIT1_DIGIT, version[0]);
	lcdPutString(ZONE_DIGIT3_DIGIT, version[1]);
	delayms(1000);
	
	sendRf("DEV_ON:1\n");
	playGoodBad();
	setState(MAIN_SCREEN_OP);
}

static void handleOffOp(void)
{
	dev_data.device_on = 0;
	LCD_SetAllPixels(0);
	lcdPutString(ZONE_DIGIT5_DIGIT, "OFF");
	//*************************************
	SYS_UnlockReg();
	pinConfig_deinit_pa();
	pinConfig_deinit_bpwm0();
	pinConfig_init_pa_tradeshow();
	pinConfig_init_bpwm0_tradeshow();
	BPWM0_Init_treadeshow();
	GPIO_Init_tradeshow();
	SYS_LockReg();
	
	playShutdownSound();
	setBackLightOffPWM();
	
	for(uint8_t i = 100; i > 0; --i)
	{
		setBackLightPWM(WHITE, i);
		delayms(10);
	}
	
	SYS_UnlockReg();
	pinConfig_deinit_pa_tradeshow();
	pinConfig_deinit_bpwm0_tradeshow();
	pinConfig_init_pa();
	pinConfig_init_bpwm0();
	BPWM0_Init();
	GPIO_Init();
	SYS_LockReg();
	//*************************************
	LCD_SetAllPixels(0);
	dev_data.wifi_status = 0;
	resetRelays();
	setState(IDLE);
}

static void printTimeAndDayOfWeek(void)
{
	char buf[3];   // max 2 basamak + null
	time_t now = time(NULL);
	struct tm *lt = localtime(&now);
	sprintf(buf, "%02d", lt->tm_hour); 
	lcdPutString(ZONE_DIGIT1_DIGIT, buf);
	sprintf(buf, "%02d", lt->tm_min); 
	lcdPutString(ZONE_DIGIT3_DIGIT, buf);
	lcdSetSymbol(day_symbol[lt->tm_wday], 1);
	
	for(uint8_t i = 0; i < 7; i++)
		if(day_symbol[lt->tm_wday] != day_symbol[i])
			lcdSetSymbol(day_symbol[i], 0);
}

static void printTimeAndDayOfWeekByIdx(int h, int min, int wd)
{
	char buf[3];   // max 2 basamak + null
	sprintf(buf, "%02d", h); 
	lcdPutString(ZONE_DIGIT1_DIGIT, buf);
	sprintf(buf, "%02d", min); 
	lcdPutString(ZONE_DIGIT3_DIGIT, buf);
	lcdSetSymbol(day_symbol[wd], 1);
	
	for(uint8_t i = 0; i < 7; i++)
		if(i != wd) lcdSetSymbol(day_symbol[i], 0);	
}

/**
* @return: dayofweek 0 - 6, 0: Sunday
*/
static int printDayOfWeekByDateTime(int y, int mon, int d, int h, int min)
{
//	printf("Girdi: y=%d, mon=%d, d=%d, h=%d, min=%d\n", y, mon, d, h, min);
	struct tm t = {0};
    t.tm_year = y - 1900;
    t.tm_mon = mon - 1;
    t.tm_mday = d;
    t.tm_hour = h; 
	t.tm_min = min;
    t.tm_isdst = -1;
	
	mktime(&t);
	
	lcdSetSymbol(day_symbol[t.tm_wday], 1);
	
	for(uint8_t i = 0; i < 7; i++)
		if(t.tm_wday != i) lcdSetSymbol(day_symbol[i], 0);
		
		return t.tm_wday;
		
//	printf("%s\n", pdays[t.tm_wday]);
//	printf("tm struct: tm_year=%d, tm_mon=%d, tm_mday=%d\n", t.tm_year, t.tm_mon, t.tm_mday);
}

static void menuRtcCleanUp(void)
{
	edgeDetection(&ed_menu_rtc_cleanup, 0);
}

static void menuRtcHelperPrintCase(uint8_t case_, int y, int mon, int d, int wd, int h, int min)
{
	char buf[3];   // max 2 basamak + null
	
	switch(case_)
		{
			case MENU_RTC_YEAR:
				// Yil bilgisini sabit birak
				sprintf(buf, "%02d", y % 100); 
				lcdPutString(ZONE_DIGIT5_DIGIT, buf);
				lcdPutChar(ZONE_DIGIT7_DIGIT, 'y');
			break;
			
			case MENU_RTC_MONTH:
				// Ay bilgisini sabit birak
				sprintf(buf, "%02d", mon); 
				lcdPutString(ZONE_DIGIT5_DIGIT, buf);
				lcdPutChar(ZONE_DIGIT7_DIGIT, 'n');
			break;
			
			case MENU_RTC_DAY:
				// Gün bilgisini sabit birak
				sprintf(buf, "%02d", d); 
				lcdPutString(ZONE_DIGIT5_DIGIT, buf);
				lcdPutChar(ZONE_DIGIT7_DIGIT, 'd');
				
				// Haftanin günü sembolünü de sabit göster
				lcdSetSymbol(day_symbol[wd], 1);
			break;
			
			case MENU_RTC_HOUR:
			case MENU_RTC_MINUTE:
				lcdSetSymbol(SYMBOL_COL, 1);
				// Saat bilgisini sabit birak
				sprintf(buf, "%02d", h); 
				lcdPutString(ZONE_DIGIT1_DIGIT, buf);
		
				// Dakika bilgisini sabit birak
				sprintf(buf, "%02d", min); 
				lcdPutString(ZONE_DIGIT3_DIGIT, buf);
			break;
		}
}

static void menuRtcHelperClearScreen(uint8_t case_, int wd)
{
	switch(case_)
	{
		case MENU_RTC_YEAR:
			lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
			lcdPutChar(ZONE_DIGIT7_DIGIT, ' ');
		break;
		case MENU_RTC_MONTH:
			lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
			lcdPutChar(ZONE_DIGIT7_DIGIT, ' ');
		break;
		
		case MENU_RTC_DAY:
			lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
			lcdPutChar(ZONE_DIGIT7_DIGIT, ' ');
		
			for(uint8_t i = 0; i < 7; i++) 
				if(day_symbol[i] != day_symbol[wd])
				lcdSetSymbol(day_symbol[i], 0);
		break;
		
		case MENU_RTC_HOUR:
			lcdPutString(ZONE_DIGIT1_DIGIT, "  ");
		break;
		
		case MENU_RTC_MINUTE:
			lcdPutString(ZONE_DIGIT3_DIGIT, "  ");
		break;
		
		
	}
}

static void menuRtc(uint8_t blink_state, uint8_t blink_btn, uint8_t *time_idx, uint8_t *menu_idx)
{
	char buf[3];   // max 2 basamak + null
	static int y,mon,d, wd,h,min;
	uint8_t max_day;
	time_t now = time(NULL);
	struct tm *lt = localtime(&now);

	if(edgeDetection(&ed_menu_rtc_cleanup, 1)) 
	{
		LCD_SetAllPixels(0);
		y = lt->tm_year + 1900;
		mon = lt->tm_mon + 1;
		d = lt->tm_mday;
		h = lt->tm_hour;
		min = lt->tm_min;
		printDayOfWeekByDateTime(y, mon, d, h, min);
	}
	
	if (btn_menu_pulse) 
	{
		//menuRtcHelperPrintCase'in bu noktada çagrilmasi gerek bir önceki case durumunu yazdirabilmek için
		menuRtcHelperPrintCase(*time_idx, y, mon, d, wd, h, min);
		*time_idx = (*time_idx + 1) % (MENU_RTC_NONE);
	}
	
	switch(*time_idx)
	{
		case MENU_RTC_YEAR:

			if(btn_plus_pulse || (btn_plus_long && blink_btn)) {y = (y == 2099) ? 2025 : (y + 1);}
			else if (btn_minus_pulse || (btn_minus_long && blink_btn)) {y = (y == 2025) ? 2099 : (y - 1);}
			
		break;
			
		case MENU_RTC_MONTH:
						
			if(btn_plus_pulse || (btn_plus_long && blink_btn)) 			{mon = (mon == 12) ? 1 : (mon + 1);}
			else if (btn_minus_pulse || (btn_minus_long && blink_btn))  {mon = (mon == 1) ? 12 : (mon - 1);}
	
		break;
		
		case MENU_RTC_DAY:	
			max_day = getDaysInMonth(y, mon);
		
			if(btn_plus_pulse || (btn_plus_long && blink_btn)) 			{d = (d == max_day) ? 1 : (d + 1);}
			else if (btn_minus_pulse || (btn_minus_long && blink_btn))  {d = (d == 1) ? max_day : (d - 1);}	
			
		break;

		case MENU_RTC_HOUR:
				
			if(btn_plus_pulse || (btn_plus_long && blink_btn)) 			{h = (h + 1) % 24;} 
			else if (btn_minus_pulse || (btn_minus_long && blink_btn))  {h = (h == 0) ? 23 : (h - 1);}
				
		break;
			
		case MENU_RTC_MINUTE:
			
			if(btn_plus_pulse || (btn_plus_long && blink_btn)) 			{min = (min+ 1) % 60;}
			else if (btn_minus_pulse || (btn_minus_long && blink_btn))  {min = (min == 0) ? 59 : (min - 1);}
				
		break;

	}
	
	if(btn_menu_pulse || btn_plus_pulse || btn_minus_pulse)
	{
		wd = printDayOfWeekByDateTime(y, mon, d, h, min);
	}

	
	// Ekran kontrol
	if(blink_state)
		menuRtcHelperPrintCase(*time_idx, y, mon, d, wd, h, min);
	else
		menuRtcHelperClearScreen(*time_idx, wd);	
	
	
	if(btn_menu_long_pulse)
	{
		setRtcByTimeLib(y, mon, d, h, min, 0); 
		
		for(uint8_t i = 0; i < 7; ++i)
				lcdSetSymbol(day_symbol[i], 0);
		
		playBuzzerBeep(500);
		*menu_idx = 0;
		edgeDetection(&ed_menu_rtc_cleanup, 0);
	}
	
	if(btn_onoff_pulse)
	{
		edgeDetection(&ed_menu_rtc_cleanup, 0);
		*menu_idx = 0;
	}		
	
}


static void menuWeeklyScheduleCleanUp(void)
{
	edgeDetection(&ed_menu_weekly_sche_cleanup, 0);
}

static void menuWeeklySchedulePrintSelectedDays(uint8_t blink_state, uint8_t selected_day)
{
	if (selected_day <= 6) 
	{
		lcdSetSymbol(day_symbol[selected_day], blink_state);
		for(uint8_t i = 0; i < 7; i++)
			if(i != selected_day) {lcdSetSymbol(day_symbol[i], 0);}
			
			lcdSetSymbol(SYMBOL_LINE1, 0);
			lcdSetSymbol(SYMBOL_LINE2, 0);
	} 
	else if (selected_day == 7) 
	{
		// Hafta içi (WD) // Pazartesi-Cuma
		for(uint8_t i = 1; i <= 5; i++) 
			lcdSetSymbol(day_symbol[i], blink_state);
			
		lcdSetSymbol(SYMBOL_LINE1, blink_state);
		lcdSetSymbol(SYMBOL_LINE2, 0);
		// Pazar ve Cumartesi söndür
		lcdSetSymbol(day_symbol[0], 0);
        lcdSetSymbol(day_symbol[6], 0);
			
	}
	else if (selected_day == 8)
	{
		// Hafta sonu (WE)
		lcdSetSymbol(day_symbol[0], blink_state);  // Pazar
		lcdSetSymbol(day_symbol[6], blink_state);  // Cumartesi
		lcdSetSymbol(SYMBOL_LINE2, blink_state);
		lcdSetSymbol(SYMBOL_LINE1, 0);
		
		for(uint8_t i = 1; i <= 5; i++)  // Hafta içi söndür
			lcdSetSymbol(day_symbol[i], 0);
	}
	else if (selected_day == 9)
	{
		// Tüm günler (7 gün hepsi)
		for(uint8_t i = 0; i < 7; i++) 
			lcdSetSymbol(day_symbol[i], blink_state);
		
		lcdSetSymbol(SYMBOL_LINE1, blink_state);
		lcdSetSymbol(SYMBOL_LINE2, blink_state);
	}
	else if (selected_day == 10)
	{
		// Haftalik program devre disi - tüm semboller söndür
		for(uint8_t i = 0; i < 7; i++) 
			lcdSetSymbol(day_symbol[i], 0);
		
		lcdSetSymbol(SYMBOL_LINE1, 0);
		lcdSetSymbol(SYMBOL_LINE2, 0);
		
		// Ekranda "OFF" veya "--" göster
		lcdPutString(ZONE_DIGIT5_DIGIT, "---");
	}
	
}

/**
* @brief schedule_idx 0: gün seçimi, 1: saat editleme
*/
void menuWeeklySchedule(uint8_t blink_state, uint8_t blink_btn, uint8_t *schedule_idx, uint8_t *menu_idx)
{
	static uint8_t edit_hour = 0;          // Hangi saat editliyoruz (0-23)     
	uint8_t *selected_day = &dev_data.weekly_sche_days;
		
	char buf[3];   // max 2 basamak + null
	
	if(edgeDetection(&ed_menu_weekly_sche_cleanup, 1)) 
	{
		LCD_SetAllPixels(0);
		edit_hour = 0;
	}
	
	switch(*schedule_idx)
	{
		case MENU_WEEKLY_SCH_DAY:
			
			sprintf(buf, "%02d", edit_hour);
			lcdPutString(ZONE_DIGIT1_DIGIT, buf);
			lcdPutString(ZONE_DIGIT3_DIGIT, "00");
			
            // Plus/Minus ile gün seçimi (0-8: SU-SAT, Weekdays, Weekend, entire days, weekly schedule canceled)
            if (btn_plus_pulse) {*selected_day = (*selected_day + 1) % 11;}
            if (btn_minus_pulse) {*selected_day = (*selected_day == 0) ? 10 : (*selected_day - 1);}
			if(*selected_day > 10)  *selected_day = 10;
			 // Menu buton ile saat ayarlamaya geç
            if (btn_menu_pulse)
			{
                *schedule_idx = 1;
                edit_hour = 0;
            }
			
            menuWeeklySchedulePrintSelectedDays(blink_state, *selected_day);

			if(*selected_day == 10)
			{
				g_weekly_schedule.is_active  = 0;
			}
			else
			{
				g_weekly_schedule.is_active  = 1;
				lcdPutString(ZONE_DIGIT5_DIGIT, "   ");
			}

		break;
		
		case MENU_WEEKLY_SCH_HOUR:
			
			// Plus/Minus ile saat seçimi
			if(btn_plus_pulse || (btn_plus_long && blink_btn))  		edit_hour = (edit_hour + 1) % 24;
			else if (btn_minus_pulse || (btn_minus_long && blink_btn)) 	edit_hour = (edit_hour == 0) ? 23 : (edit_hour - 1);
			/**************** Menu pulse ile mevcut saati set et BEGIN ****************/

		
			if (btn_menu_pulse && *selected_day <= 6) 
			{
				// Tek gün için set et
				uint8_t hour_index = weeklyScheduleDayHourToIndex(*selected_day, edit_hour);
				uint16_t current_value = weeklyScheduleGetHour(hour_index);
				weeklyScheduleSetHour(hour_index, current_value ? 0 : 1);  // Toggle
			} 
			else if (btn_menu_pulse && *selected_day == 7) 
			{
				// Hafta içi tümü için set et
				for(uint8_t day = 1; day <= 5; day++) 
				{
					uint8_t hour_index = weeklyScheduleDayHourToIndex(day, edit_hour);
					uint16_t current_value = weeklyScheduleGetHour(hour_index);
					weeklyScheduleSetHour(hour_index, current_value ? 0 : 1);
				}
			} 
			else if (btn_menu_pulse && *selected_day == 8)
			{
				// Hafta sonu için set et
				uint8_t weekend_days[] = {0, 6};  // Pazar, Cumartesi
				for(uint8_t i = 0; i < 2; i++) 
				{
					uint8_t hour_index = weeklyScheduleDayHourToIndex(weekend_days[i], edit_hour);
					uint16_t current_value = weeklyScheduleGetHour(hour_index);
					weeklyScheduleSetHour(hour_index, current_value ? 0 : 1);
				}
			}
			
			else if (btn_menu_pulse && *selected_day == 9)
			{
				// Tüm günler için set et
				for(uint8_t day = 0; day < 7; ++day) 
				{
					uint8_t hour_index = weeklyScheduleDayHourToIndex(day, edit_hour);
					uint16_t current_value = weeklyScheduleGetHour(hour_index);
					weeklyScheduleSetHour(hour_index, current_value ? 0 : 1);
				}
			}
			
			/**************** Menu pulse ile mevcut saati set et END ****************/
			
			
			menuWeeklySchedulePrintSelectedDays(1, *selected_day);
			
			// Mevcut saatin degerini göster
            uint16_t display_value = 0;
            if (*selected_day <= 6)
			{
                uint8_t hour_index = weeklyScheduleDayHourToIndex(*selected_day, edit_hour);
                display_value = weeklyScheduleGetHour(hour_index);
            } 
			else if(*selected_day <= 9)
			{
                // Grup seçiminde ilk günün degerini göster, gerisi ayni
                uint8_t check_day = (*selected_day == 7) ? 1 : 0; // 7=Pazartesi, 8,9=Pazar
                uint8_t hour_index = weeklyScheduleDayHourToIndex(check_day, edit_hour);
                display_value = weeklyScheduleGetHour(hour_index);
            }
			
			if(*selected_day != 10)
			{
				char str[4];
				sprintf(buf, "%s", display_value? "ON " : "OFF");
				lcdPutString(ZONE_DIGIT5_DIGIT, buf);
			}
			
			// Saati blink ile göster
            if (blink_state) 
			{
                sprintf(buf, "%02d", edit_hour);
                lcdPutString(ZONE_DIGIT1_DIGIT, buf);
				lcdPutString(ZONE_DIGIT3_DIGIT, "00");
				lcdSetSymbol(SYMBOL_COL, 1);
			}
            else
                lcdPutString(ZONE_DIGIT1_DIGIT, "  ");
			
			// Menu long pulse ile gün seçimine geri dön
            if (btn_menu_long_pulse)
			{
				*schedule_idx = 0;
			}
                
			
		break;
	}
	
	lcdSetSymbol(SYMBOL_WEEK_SCHEDULE, blink_state ? 1 : 0);
	
	//çikista yapilacaklar
	if(btn_onoff_pulse)
	{
		edgeDetection(&ed_menu_weekly_sche_cleanup, 0);
		playBuzzerBeep(500);
		flashManagerMarkDirty(weekly_sche_days.id);
		flashManagerMarkDirty(weekly_schedule.id);
		*menu_idx = 0;
	}
}


static void menuWifiSettingsCleanUp(void)
{
	edgeDetection(&ed_menu_wifi_settings_cleanup, 0);
}

static void menuWifiSettings(uint8_t blink_state, uint8_t *menu_idx)
{
	static uint16_t state = 1;
	static uint16_t wifi_exist_done = 0;
	
	if(edgeDetection(&ed_menu_wifi_settings_cleanup, 1))
	{
		LCD_SetAllPixels(0);
		state = 0;
		wifi_exist_done = dev_data.is_wifi_exist;
	}
	
	if(btn_plus_pulse) {state = (state + 1) % WIFI_END;}
	else if(btn_minus_pulse) {state = (state + WIFI_END - 1) % WIFI_END;}
	
	switch(state)
	{		
		case WIFI_ONOFF:
				
			lcdPutString(ZONE_DIGIT5_DIGIT, wifi_exist_done ? "ON " : "OFF");
		
			if(btn_menu_pulse)
			{
				wifi_exist_done = !wifi_exist_done;
			}
			if(btn_menu_long_pulse) 
			{
				dev_data.is_wifi_exist = wifi_exist_done;
				flashManagerMarkDirty(is_wifi_exist.id);
				sendRf("WIFI:1\n");
			}
		break;
		
		case WIFI_RESET:
			lcdPutString(ZONE_DIGIT5_DIGIT,"RST");
			if(btn_menu_long_pulse)
			{
				sendRf("RESET_WIFI\n");
			}
		break;
			
		default:
		break;
	}
	
	lcdSetSymbol(SYMBOL_WIFI, dev_data.is_wifi_exist ? 1 : blink_state);
	
	if(btn_menu_pulse)
	{
		playBuzzerBeep(200);
	}
	if(btn_menu_long_pulse)
	{
		playBuzzerBeep(500);
	}

	if(btn_onoff_pulse)
	{
		edgeDetection(&ed_menu_wifi_settings_cleanup, 0);
		*menu_idx = 0;
	}
}

static void printMenuSelectionRtc(void)
{
	printTimeAndDayOfWeek();
	lcdSetSymbol(SYMBOL_WEEK_SCHEDULE, 0);
}

static void printMenuNumber(uint8_t menu_number)
{
	char buf[3];
	sprintf(buf, "%02d", menu_number);
	lcdPutString(ZONE_DIGIT5_DIGIT, buf);
}

void menuNavigationCleanUp(void)
{
	edgeDetection(&ed_menu_navigation_cleanup, 0);
}

static void menuNavigation(uint8_t *menu_idx, uint8_t *time_idx, uint8_t *schedule_idx, uint8_t blink_state)
{
	static uint32_t menu_idx_unselected = 1;
	
	if(edgeDetection(&ed_menu_navigation_cleanup, 1))
	{
		LCD_SetAllPixels(0);
		//*menu_idx = 0xFF;
		*time_idx = 0xFF;
		*schedule_idx = 0xFF;
		lcdSetSymbol(SYMBOL_MENU,1);
	}
		 /********* Menu navigasyon BEGIN ********/
	
	const uint32_t MENU_COUNT = MENU_NONE - 1;
	// menü navigasyon degiskeninin +,- ile degistirilmesi, 1 - MENU_COUNT arasi
	if(btn_plus_pulse) {menu_idx_unselected = menu_idx_unselected % MENU_COUNT+1;}
	else if(btn_minus_pulse) {menu_idx_unselected = (menu_idx_unselected - 2 + MENU_COUNT) % MENU_COUNT + 1;}
	
	if(btn_plus_pulse || btn_minus_pulse)
		LCD_SetAllPixels(0);
	
	printMenuNumber(menu_idx_unselected);
	
	
	// Menüye girmeden önce sadece navigasyon esnasinda ekranda gösterilecekler
	switch(menu_idx_unselected)
	{
		case MENU_SET_RTC:
			printMenuSelectionRtc();
		break;
		
		case MENU_WEEKLY_SCHEDULE:
			printTimeAndDayOfWeek();
			lcdSetSymbol(SYMBOL_WEEK_SCHEDULE, blink_state);
		
		break;
		
		case MENU_WIFI_SETTINGS:
			lcdSetSymbol(SYMBOL_WIFI, blink_state);
		break;
		
	}
	
	if(btn_menu_pulse) 
	{
		edgeDetection(&ed_menu_navigation_cleanup, 0);
		*menu_idx = menu_idx_unselected;
		*time_idx = 0;
		*schedule_idx = 0;
	}
	
	if(btn_onoff_pulse)
	{
		edgeDetection(&ed_menu_navigation_cleanup, 0);
		cleanUps();
	}
				
}



static void menuOpCleanUp(void)
{
	edgeDetection(&ed_menu_op_cleanup, 0); 
}

static void handleMenuOp(uint8_t blink_state, uint8_t blink_btn)
{	
	static uint8_t menu_idx = 0;  // P1, P2, P3... // hangi menude oldugumuz
	static uint8_t time_idx = 0xFF;
	static uint8_t schedule_idx = 0xFF;
	static uint8_t reserve; //alignment

	/********* Giriste bir kez çalisacak alan BEGIN ********/
	
	if(edgeDetection(&ed_menu_op_cleanup, 1)) 
	{
		LCD_SetAllPixels(0); 
		lcdSetSymbol(SYMBOL_MENU, 1);
		lcdSetSymbol(SYMBOL_COL, 1); 
		menu_idx = 0;
		time_idx = 0xFF;
		schedule_idx = 0xFF;
	}
	/********* Giriste bir kez çalisacak alan END ********/


	/******** Seçilen menü için parametre ayarlama BEGIN ***********/
	switch(menu_idx) 
	{
		case MENU_NAVIGATION:
		
			menuNavigation(&menu_idx, &time_idx, &schedule_idx, blink_state);
		
			if(btn_onoff_pulse) 
			{
				// Çikmadan önce çalisacak alan , menü seçiminde isem.
				// Parametre ayari esnasinda iki kez onoff'a basilirsa dogal olarak çikis 
				edgeDetection(&ed_menu_op_cleanup, 0); 
				lcdSetSymbol(SYMBOL_MENU, 0); 
				lcdSetSymbol(SYMBOL_WEEK_SCHEDULE, 0);
				setState(MAIN_SCREEN_OP);
			}
		
		break;
		
		case MENU_SET_RTC:
			menuRtc(blink_state, blink_btn, &time_idx, &menu_idx);
		break;
			
		case MENU_WEEKLY_SCHEDULE:
			menuWeeklySchedule(blink_state, blink_btn, &schedule_idx, &menu_idx);
		break;
		
		case MENU_WIFI_SETTINGS:
			menuWifiSettings(blink_state, &menu_idx);
		break;
		
		default:
		break;
		
	}
	/******** Seçilen menü için parametre ayarlama END ***********/

	

}

static void handleMainScreenModeSymbol(uint8_t blink_state)
{
	uint8_t cond = (dev_data.mode == MODE_WINTER) && (dev_data.is_heating_on && blink_state);
	lcdSetSymbol(SYMBOL_HOT, cond);
	
//	cond = (dev_data.mode == MODE_SUMMER);
//	lcdSetSymbol(SYMBOL_COLD, cond);
	
}
static void mainScreenOpCleanUp(void)
{
	edgeDetection(&ed_mainscreen_op_cleanup, 0);
}

static void handleErrBlink(uint8_t blink_state)
{
	//uint8_t temp = dev_data.backlight_intensity;
	//dev_data.backlight_intensity = 100;
	setBackLight(RED, blink_state ? 1: 0);
	setBackLight(WHITE, 0);
	setBackLight(BLUE, 0);
	//dev_data.backlight_intensity = temp;
}

void sendRfTelemetryData(void)
{
	char buf[3];   // max 2 basamak + null
	time_t now = time(NULL);
	struct tm *lt = localtime(&now);
	// %5.2f -> toplam 5 karakter ve virgülden sonra 2 karakter. Yani xx.xx(5 karakter)
	sendRf("ALL:%5.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", dev_data.current_temp, dev_data.device_on, dev_data.is_heating_on, 
					dev_data.w, dev_data.r, dev_data.b, 
					g_weekly_schedule.is_active, dev_data.child_lock, dev_data.mode, lt->tm_hour, lt->tm_min, 
					lt->tm_wday, GET_ERR(ERR_FALLOVER), GET_ERR(ERR_NTC));
}

static void handleMainScreenOp(uint8_t blink_state)
{
	static ton_t ton_screen_refresh;
	static ton_t ton_mqtt_err;
	
	// ilk giriste yapilacaklar
	if(edgeDetection(&ed_mainscreen_op_cleanup, 1)) 
	{
		LCD_SetAllPixels(0);
		lcdSetSymbol(SYMBOL_ROOM, 1);
		lcdSetSymbol(SYMBOL_DOT, 1);
		lcdSetSymbol(SYMBOL_CELCIUS, 1);
		ton_screen_refresh.aux = 1; // ilk giriste TON dogrudan çalismali ki senkronize olsun 2 sn beklemesin
		
		//todo: asagidakiler sadece FUAR için
		lcdSetSymbol(SYMBOL_BLUETOOTH, 1);
		lcdSetSymbol(SYMBOL_WINDOW, 1);
		lcdSetSymbol(SYMBOL_RF, 1);
		
		dev_data.current_temp = adc_getNTCvalue(0,0);
	}
	
	// MAINSCREEN_UPDATE_TIMEOUT süresi doldukça yenilenir
	if(TON(&ton_screen_refresh, 1,systick, MAINSCREEN_REFRESH_TIMEOUT))
	{
		TON(&ton_screen_refresh, 0 ,0, 0);
		
		char buf[3];   // max 2 basamak + null
		
		sprintf(buf, "%d", (uint32_t)dev_data.current_temp); 
		lcdPutString(ZONE_DIGIT5_DIGIT, buf);
		
		uint8_t frac = (dev_data.current_temp - (uint32_t)dev_data.current_temp) * 10;
		sprintf(buf, "%d", frac); 
		lcdPutChar(ZONE_DIGIT7_DIGIT, buf[0]);

		printTimeAndDayOfWeek();			

		lcdSetSymbol(SYMBOL_WEEK_SCHEDULE, g_weekly_schedule.is_active);
		
		if(!dev_data.err && !anybutton_backlight)
		{
			setBackLightWRB(dev_data.w, dev_data.r, dev_data.b);
		}
		
		setArrows(dev_data.mode);
		
		lcdSetSymbol(SYMBOL_LOCK, dev_data.child_lock);

	}
	
	handleMainScreenModeSymbol(blink_state);
	lcdSetSymbol(SYMBOL_COL, blink_state);
	
	
	uint8_t wifi_try_con = dev_data.is_wifi_exist && (dev_data.wifi_status == 1 && blink_state);
	uint8_t wifi_status = dev_data.is_wifi_exist && (dev_data.wifi_status == 2);
	lcdSetSymbol(SYMBOL_WIFI, wifi_try_con || wifi_status);
	
	//uint8_t mqtt_status = (dev_data.mqtt_status == 0) || (dev_data.mqtt_status == 1 && blink_state);


	lcdSetSymbol(SYMBOL_EXCLAM, dev_data.err && blink_state);
	
	if(GET_ERR(ERR_FALLOVER) && !anybutton_backlight)
	{
		handleErrBlink(blink_state);
		lcdPutString(ZONE_DIGIT5_DIGIT, ERR_FALLOVER_STR);
	}
	else if(GET_ERR(ERR_NTC) && !anybutton_backlight)
	{
		handleErrBlink(blink_state);
		lcdPutString(ZONE_DIGIT5_DIGIT, ERR_NTC_STR);
	}
	else if(TON(&ton_mqtt_err, (dev_data.mqtt_status == 0) && dev_data.is_wifi_exist && !anybutton_backlight,systick, 1000))
	{
		handleErrBlink(blink_state);
		lcdPutString(ZONE_DIGIT5_DIGIT, ERR_MQTT_STR);
	}
	else if(TON(&ton_mqtt_err, (dev_data.mqtt_status == 1) && dev_data.is_wifi_exist && !anybutton_backlight,systick, 1000))
	{
		handleErrBlink(blink_state);
		lcdPutString(ZONE_DIGIT5_DIGIT, blink_state ? ERR_MQTT_STR : "   ");
	}
	
	
	// Buton tiklamalari
	if(btn_menu_pulse_falling && !dev_data.child_lock)
	{
		++dev_data.mode;
		if(dev_data.mode > 5) dev_data.mode = 1;
		setArrows(dev_data.mode);
		flashManagerMarkDirty(mode.id);
	}
	
	if(btn_child_lock_long_pulse)
	{
		dev_data.child_lock = !dev_data.child_lock;
	}
	
	if(btn_onoff_pulse && !dev_data.child_lock)
	{
//		flashManagerLoadAll();
//			
//		flashManagerPrintUsageReport();
//		printf("***************Saatler ONOFF BEGIN*********************\n");
//		for(int i = 0; i < HOURS_IN_WEEK; ++i)
//		{
//			if(i % 24 == 0)
//				printf("\n");
//			printf("%d ", g_weekly_schedule.schedule_hours[i]);		
//		}
//		printf("\n***************Saatler ONOFF END*********************\n");
//		printf("Current Mode: %d\n", g_weekly_schedule.current_mode);
//		printf("Is Active: %d\n", g_weekly_schedule.is_active);
//		printf("setpoint = %f\n", dev_data.temp_setpoint);
//		printf("Dirty durumu: %d\n", flashManagerIsDirty(temp_setpoint.id));
//		printf("Yazma sayisi: %d\n", flashManagerGetWriteCount(PARAM_CAT_DYNAMIC));
	}
}

static float setpoint;
static void tempSetPointOpCleanUp(void)
{
	edgeDetection(&ed_temp_setpoint_op_cleanup, 0); 
	//playBuzzerBeep(500);
	flashManagerMarkDirty(temp_setpoint.id);
	dev_data.temp_setpoint = setpoint;
}

static void handleTempSetPointOp(uint8_t blink_state, uint8_t blink_btn)
{
	const float setpoint_step = 0.1f;
	const float setpoint_step_ir = 0.5;
	const float max = 35.0f;
	const float min = 15.0f;
	char buf[4];   // max 3 basamak + null
	
	// ilk giriste yapilacaklar
	if(edgeDetection(&ed_temp_setpoint_op_cleanup, 1))
	{
		lcdSetSymbol(SYMBOL_ROOM, 1);
		setpoint = dev_data.temp_setpoint;
	}
					/*** ARTTIR - AZALT ***/
	/***************************************************************/

	
	if(btn_plus_pulse  || (btn_plus_long && blink_btn)) 			setpoint += setpoint_step;
	else if (btn_minus_pulse || (btn_minus_long && blink_btn))		setpoint -= setpoint_step;
	else if(ir_plus_pulse || (ir_repeat && ir_plus_pressed && blink_btn)) 		setpoint += setpoint_step_ir;
	else if(ir_minus_pulse || (ir_repeat && ir_minus_pressed && blink_btn)) 	setpoint -= setpoint_step_ir;
	
	
	if(setpoint > max) setpoint = max;
	else if(setpoint < min) setpoint = min;
	
	/***************************************************************/
	
	if(btn_plus_pulse || btn_minus_pulse || ir_plus_pulse || ir_minus_pulse) playBuzzerBeep(100);
	
					/*** BLINK YAP ***/
	/***************************************************************/
	if (blink_state)
	{
		uint32_t uval = setpoint * 10;
		sprintf(buf, "%3d", uval );
		lcdPutString(ZONE_DIGIT5_DIGIT, buf);
	}
	else
	{
		lcdPutString(ZONE_DIGIT5_DIGIT, "   ");
	}
	lcdSetSymbol(SYMBOL_DOT, blink_state);
	/***************************************************************/

	if(btn_onoff_pulse || ir_onoff_pulse)
	{
		setState(MAIN_SCREEN_OP);
		edgeDetection(&ed_temp_setpoint_op_cleanup, 0);
	}
		
}

// 1 ve 4, TIMEOVER sn içinde 500 ms basili tutulduysa test moduna geçecek
static void deviceTestRun(void)
{
	enum {TIMEOVER = 3000, TIMEABORT = 100000};
	
	ton_t ton_btn = {0};
	ton_t ton_timeover= {0};
	ton_t ton_abort= {0};
	edge_detection_t ed;
	uint8_t one_time[2] = {0};
	
	int32_t state = -1;
	uint8_t btn_pulse = 0;
	uint32_t btn_pressed_time = 500;
	
	lcdPutString(ZONE_DIGIT5_DIGIT, "--");
	
	while(1)
	{
		uint32_t btn[] = { BUTTONS};
		uint8_t btn_test = !btn[0] && !btn[3];
		// buton0 ve 4'e basildi mi? Senaryoya giris
		btn_pulse = edgeDetection(&ed, TON(&ton_btn, btn_test, systick, btn_pressed_time));
		
		if(btn_pulse && (TEST_IDLE == state)) // Test baslangic kosulu
		{
			playBuzzerBeep(200);
			state = 0;
			btn_pressed_time = 100;
		}
		
		switch(state)
		{
			case ALL_PIXELS_ON: 
				
				LCD_SetAllPixels(1);
				one_time[0] = 1;
				btn_pulse && (state = 1, 0);
			
			break;

			case CHECK_BUTTONS:
				
				for (int i = 1; i < BTN_COUNT+1; i++) 
				{
					if(!btn[i-1])
					{
						if(one_time[0])
						{
							LCD_SetAllPixels(0);
							one_time[0] = 0;
						}
						char buf[3];   // max 1 basamak + null
						sprintf(buf, "b%d", i);
						lcdPutString(ZONE_DIGIT5_DIGIT, buf);
						//LCDLIB_PrintNumber(ZONE_DIGIT5_DIGIT, i);
					}
						
				}
				btn_pulse && (state = 2, 0);
				
			break;
				
			case CHECK_ADC:
					
				lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
				
				float fval = adc_getNTCvalue(0,0);
				char buf[3];   // max 2 basamak + null
				
				sprintf(buf, "%d", (uint32_t)fval); 
				lcdPutString(ZONE_DIGIT1_DIGIT, buf);
				
				fval = adc_getNTCvalue(1,0);
				sprintf(buf, "%d", (uint32_t)fval);
				lcdPutString(ZONE_DIGIT3_DIGIT, buf);

				btn_pulse && (state = 3, 0);
			
			break;
				
			case CHECK_RELAYS:
						
				lcdPutString(ZONE_DIGIT5_DIGIT, "RL");
				setRelay(RELAY_1, 1);
				setRelay(RELAY_2, 1);
				lcdPutString(ZONE_DIGIT1_DIGIT, "R1");
				lcdPutString(ZONE_DIGIT3_DIGIT, "R2");

				btn_pulse && (state = 4, 0);
			
				break;
				
			case CHECK_FAIL_PIN:
				
				if(!one_time[0])
				{
					one_time[0] = 1;
					lcdPutChar(ZONE_DIGIT1_DIGIT, ' ');
					lcdPutChar(ZONE_DIGIT2_DIGIT, ' ');
					lcdPutChar(ZONE_DIGIT3_DIGIT, ' ');
					lcdPutChar(ZONE_DIGIT4_DIGIT, ' ');
				}

				
				setRelay(RELAY_1, 0);
				setRelay(RELAY_2, 0);
				
				lcdPutString(ZONE_DIGIT5_DIGIT, "FL");
				if(fail_pin)
				{
					fail_pin = 0;
					playBuzzerOK();
					lcdPutString(ZONE_DIGIT1_DIGIT, "yE");
					lcdPutChar(ZONE_DIGIT3_DIGIT, 'S');
				}
				
				btn_pulse && (state = 5, 0);
			
			break;
					
			case CHECK_IR:
					
				if(!one_time[1])
				{
					one_time[1] = 1;
					lcdPutString(ZONE_DIGIT1_DIGIT, "  ");
					lcdPutChar(ZONE_DIGIT3_DIGIT, ' ');
				}
				lcdPutString(ZONE_DIGIT5_DIGIT, "IR");
				
				if(getIrData())
				{
					clearIrData();
					playBuzzerOK();
					lcdPutString(ZONE_DIGIT1_DIGIT, "yE");
					lcdPutChar(ZONE_DIGIT3_DIGIT, 'S');
				}
				
				btn_pulse && (state = 6, 0);
			
			break;
					
			case TEST_DONE:
				
				playBuzzerBeep(1000);
				LCD_SetAllPixels(0);
				btn_pulse && (state = 7, 0);
				return;
				
			break;
		}
		
		
		if(TON(&ton_timeover, 1, systick, TIMEOVER) && (-1 == state))
		{
			lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
			return;
		}
		
		if(TON(&ton_abort, 1, systick, TIMEABORT))
		{
			lcdPutString(ZONE_DIGIT5_DIGIT, "  ");
			return;
		}
		
	}
	
}


