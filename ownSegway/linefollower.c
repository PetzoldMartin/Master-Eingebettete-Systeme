/* linefollower.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define LOWER_LIMIT 20 // black
#define UPPER_LIMIT 80 // white
#define BASE_SPEED 30 
#define INTEGRAL_LENGTH 5
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
    nxt_motor_set_speed(NXT_PORT_B, 0, 1); // left motor
    nxt_motor_set_speed(NXT_PORT_C, 0, 1); // right motor

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

    
    int I = 0;
    int integral[INTEGRAL_LENGTH];
     int wcount = 0;
                while (wcount < INTEGRAL_LENGTH) {
                    integral[wcount] = 0;
                    wcount++;
                }
    int count = 0;
    int errorOld=0;
    int tempUL=UPPER_LIMIT;
    int tempLL=LOWER_LIMIT;
    
    while (1) {
        if (running) {

            WaitEvent(ColorSensorEvent);
            ClearEvent(ColorSensorEvent);

            S16 blue = color_data[2];
            if(blue<tempLL){
                tempLL=blue;
            };
            if(blue>tempUL){
                tempUL=blue;
            };
            
            S16 xdelta = tempUL-tempLL;
            S16 ydelta = 2 * BASE_SPEED;
            float KP = ((float) ydelta / (float) xdelta);
            float KI = KP/2;
            float KD = KP/2;

            S16 error = (blue - (xdelta / 2));
            //Anti Overshooting
            if (error == 0) {
                I = 0.0F;
                int wcount = 0;
                while (wcount < INTEGRAL_LENGTH) {
                    integral[wcount] = 0;
                    wcount++;
                }
                //END Anti Overshooting  
            } else {
                if (count > INTEGRAL_LENGTH) {
                    count = 0;
                }
                integral[count] = error;
                count++;
            };
            //Integral Function
            I=0;
            int wcount = 0;
            while (wcount < INTEGRAL_LENGTH) {
                I += integral[wcount];
                wcount++;
            }
            I = I / INTEGRAL_LENGTH;
            
            //Differential Function
            int D=error-errorOld;

            S16 turn = (KP * error)+(KI * I)+(KD* D);

            display_goto_xy(0, 1);
            display_string("I: ");
            display_int(I, 0);

            display_goto_xy(0, 2);
            display_string("D: ");
            display_int(D, 0);
                    
             display_goto_xy(0, 3);
            display_string("turn: ");
            display_int(turn, 0);       
            
            display_goto_xy(0, 4);
            display_string("error: ");
            display_int(error, 0);
            
            display_goto_xy(0, 5);
            display_string("lower limit: ");
            display_int(tempLL, 0);
            
            display_goto_xy(0, 6);
            display_string("upper limit: ");
            display_int(tempUL, 0);


            nxt_motor_set_speed(NXT_PORT_B, BASE_SPEED + turn, 1); // left motor
            nxt_motor_set_speed(NXT_PORT_C, BASE_SPEED - turn, 1);
            errorOld=error;
            


        } else {
            nxt_motor_set_speed(NXT_PORT_B, 0, 1);
            nxt_motor_set_speed(NXT_PORT_C, 0, 1);
        }
    }
}

TASK(task_touch) {
    // static U8 running;
    U8 touch = ecrobot_get_touch_sensor(NXT_PORT_S4);

    static U8 oldState;

    if ((touch && !oldState) && !running) {
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

    //display_goto_xy(0, 0);
    //display_string("red: ");
    //display_int(color_data[0], 0);

    //display_goto_xy(0, 1);
    //display_string("green: ");
    //display_int(color_data[1], 0);

    display_goto_xy(0, 0);
    display_string("blue: ");
    display_int(color_data[2], 0);

    display_update();
    TerminateTask();
}
