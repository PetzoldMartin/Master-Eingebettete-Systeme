/* linefollower.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/*
    Balance bei 585 +-3
    nach hinten kippen -> 600
    nach vorn kippen -> 500
*/

#define LOWER_LIMIT 500 // black
#define UPPER_LIMIT 650 // white
#define BASE_SPEED 30 
#define INTEGRAL_LENGTH 5
DeclareCounter(SysTimerCnt);

DeclareResource(gyroRes);

DeclareTask(Task1);
DeclareTask(task_gyro);

DeclareEvent(GyroSensorEvent);

// sensordata of gyro
U16 gyro_data;

void ecrobot_device_initialize(void) {
    // nothing to do
}

void ecrobot_device_terminate(void) {
    nxt_motor_set_speed(NXT_PORT_B, 0, 1); // left motor
    nxt_motor_set_speed(NXT_PORT_C, 0, 1); // right motor
}

void user_1ms_isr_type2(void) {
    StatusType ercd;

    ercd = SignalCounter(SysTimerCnt);
    if (ercd != E_OK) {
        ShutdownOS(ercd);
    }
}

TASK(Task1) {
    // int I = 0;
    // int integral[INTEGRAL_LENGTH];
    // int wcount = 0;
    // while (wcount < INTEGRAL_LENGTH) {
    //     integral[wcount] = 0;
    //     wcount++;
    // }
    int count = 0;
    // int errorOld=0;
    int tempLL=LOWER_LIMIT;
    
    while (1) {
        WaitEvent(GyroSensorEvent);
        ClearEvent(GyroSensorEvent);

        // U16 tmp_gyro_data = gyro_data;
        // if(tmp_gyro_data < LOWER_LIMIT){
        //     tempLL=tmp_gyro_data;
        // };
        // if(tmp_gyro_data>/* FIXME deleted */){
        //     /* FIXME deleted */=tmp_gyro_data;
        // };
        
        U16 xdelta = gyro_data;
        S16 ydelta = 2 * BASE_SPEED;
        float KP = ((float) ydelta / (float) xdelta);
        // float KI = KP/2;
        // float KD = KP/2;

        S16 error = (gyro_data - (xdelta / 2));
        // //Anti Overshooting
        // if (error == 0) {
        //     I = 0.0F;
        //     int wcount = 0;
        //     while (wcount < INTEGRAL_LENGTH) {
        //         integral[wcount] = 0;
        //         wcount++;
        //     }
        //     //END Anti Overshooting  
        // } else {
        //     if (count > INTEGRAL_LENGTH) {
        //         count = 0;
        //     }
        //     integral[count] = error;
        //     count++;
        // };
        // //Integral Function
        // I=0;
        // int wcount = 0;
        // while (wcount < INTEGRAL_LENGTH) {
        //     I += integral[wcount];
        //     wcount++;
        // }
        // I = I / INTEGRAL_LENGTH;
        
        // //Differential Function
        // int D=error-errorOld;

        S16 turn = (KP * error); // +(KI * I)+(KD* D);

        display_goto_xy(0, 1);
        display_string("turn: ");
        display_int(turn, 0);       
        
        display_goto_xy(0, 2);
        display_string("error: ");
        display_int(error, 0);
        
        nxt_motor_set_count(NXT_PORT_B, turn);
        nxt_motor_set_count(NXT_PORT_C, turn);

        // errorOld=error;
    }
}

TASK(task_gyro) {
    gyro_data = ecrobot_get_gyro_sensor(NXT_PORT_S4);
    TerminateTask();
}
