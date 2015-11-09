/* linefollower.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define LOWER_LIMIT 5	// black
#define UPPER_LIMIT 80	// white

DeclareCounter(SysTimerCnt);

DeclareResource(lightRes);
DeclareResource(runningRes);

DeclareTask(Task1);
DeclareTask(task_touch);
DeclareTask(task_light);

DeclareEvent(TouchSensorOnEvent);
DeclareEvent(TouchSensorOffEvent);
DeclareEvent(ColorSensorEvent);

// array with [r,g,b]
S16 color_data[3];
U8 running = 0;

void ecrobot_device_initialize(void) {
	ecrobot_init_color_sensor(NXT_PORT_S3);
}

void ecrobot_device_terminate(void) {
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);	// left motor
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);	// right motor
	
	ecrobot_term_color_sensor(NXT_PORT_S3);
}

void user_1ms_isr_type2(void) {
	StatusType ercd;
	
	ercd = SignalCounter(SysTimerCnt);
	if (ercd != E_OK) {
		ShutdownOS(ercd);
	}
}

TASK(Task1) {
	while(1) {
		if(running){
			// react on TouchSensorOnEvent
			// WaitEvent(TouchSensorOnEvent);
			// ClearEvent(TouchSensorOnEvent);
			// nxt_motor_set_speed(NXT_PORT_B, 50, 1);
			// nxt_motor_set_speed(NXT_PORT_C, 50, 1);
			
			WaitEvent(ColorSensorEvent);
			ClearEvent(ColorSensorEvent);
			
			// GetResource(lightRes);
			S16 red = color_data[0];
			// ReleaseResource(lightRes);
			
			if(red < LOWER_LIMIT) {
				// drive right, it's in black area
				display_goto_xy(0, 3);
				display_string("right");
				display_update();
				nxt_motor_set_speed(NXT_PORT_B, 90, 1);		// left motor
				nxt_motor_set_speed(NXT_PORT_C, -50, 1);		// right motor
			} else if (red > UPPER_LIMIT) {
				// drive left, it's in white area
				display_goto_xy(0, 3);
				display_string("left");
				display_update();
				nxt_motor_set_speed(NXT_PORT_B, -50, 1);		// left motor
				nxt_motor_set_speed(NXT_PORT_C, 90, 1);		// right motor
			} else {
				// straight forward
				display_goto_xy(0, 3);
				display_string("straight");
				display_update();
				nxt_motor_set_speed(NXT_PORT_B, 25, 1);		// left motor
				nxt_motor_set_speed(NXT_PORT_C, 25, 1);		// right motor
			}
		
		
		
		// react on TouchSensorOffEvent
		// WaitEvent(TouchSensorOffEvent);
		// ClearEvent(TouchSensorOffEvent);
		// nxt_motor_set_speed(NXT_PORT_B, 0, 1);
		// nxt_motor_set_speed(NXT_PORT_C, 0, 1);
		} else {
			nxt_motor_set_speed(NXT_PORT_B, 0, 1);
			nxt_motor_set_speed(NXT_PORT_C, 0, 1);			
		}
	}
}

TASK(task_touch){
	// static U8 running;
	U8 touch = ecrobot_get_touch_sensor(NXT_PORT_S4);
	
	static U8 oldState;
	
	if((touch && !oldState) && !running) {
		SetEvent(Task1, TouchSensorOnEvent);
		GetResource(runningRes);
		running = 1;
		ReleaseResource(runningRes);
	} else if ((touch && !oldState) && running) {
		SetEvent(Task1, TouchSensorOffEvent);
		GetResource(runningRes);
		running = 0;
		ReleaseResource(runningRes);
	}
	
	oldState = touch;
	TerminateTask();
}

TASK(task_light) {
	// systick_wait_ms(500);
	display_clear(0);
	
	GetResource(lightRes);
	ecrobot_get_color_sensor(NXT_PORT_S3, color_data);
	ReleaseResource(lightRes);
	SetEvent(Task1, ColorSensorEvent);
	
	display_goto_xy(0, 0);
	display_string("red: ");
	display_int(color_data[0], 0);
	
	display_goto_xy(0, 1);
	display_string("green: ");
	display_int(color_data[1], 0);
	
	display_goto_xy(0, 2);
	display_string("blue: ");
	display_int(color_data[2], 0);
	
	display_update();
	TerminateTask();
}
