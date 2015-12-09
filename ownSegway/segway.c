/* linefollower.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/*
    Balance bei 585 +-3
    nach hinten kippen -> 600
    nach vorn kippen -> 500
*/

#define LOWER_LIMIT 500
#define UPPER_LIMIT 650
#define BASE_SPEED 30 
#define INTEGRAL_LENGTH 5
DeclareCounter(SysTimerCnt);
DeclareResource(gyroRes);
DeclareTask(Task1);
DeclareTask(task_gyro);
DeclareEvent(GyroSensorEvent);
// sensordata of gyro
U16 gyro_data;

typedef struct
{
    double dState;
    double iState;
    double iMax, iMin;
    double iGain,
           pGain,
           dGain;
} SPid;

void print_display(int y, char* text, int value) {
    display_goto_xy(0, y);
    display_string(text);
    display_string(": ");
    display_int(value, 0);
    display_update();
}

double update_pid(SPid * pid, double error, double position) {
    double pTerm, dTerm, iTerm;

    pTerm = pid->pGain * error; // calculate proportional term

    // calculate integral state with appropriate limiting
    pid->iState += error;
    if (pid->iState > pid->iMax) pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin) pid->iState = pid->iMin;

    iTerm = pid->iGain * pid->iState;    // calculate integral term

    dTerm = pid->dGain * (pid->dState - position);
    pid->dState = position;

    return pTerm + dTerm + iTerm;
}

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
    SPid pid;
    pid.iMin = LOWER_LIMIT;
    pid.iMax = UPPER_LIMIT;
    pid.pGain = 0.636;
    pid.iGain = 0.2688;
    pid.dGain = 0.000504;
    pid.iState = 583;
    pid.dState = 583;
    
    while (1) {
        WaitEvent(GyroSensorEvent);
        ClearEvent(GyroSensorEvent);

        int plantCommand = 593;
        int position = gyro_data;

        S16 error = update_pid(&pid, plantCommand - position, position);
        // S16 turn = (pid.pGain * error) - 57;
        S16 turn = pid.pGain * (error - 180);

        print_display(0, "error", error);
        print_display(1, "turn", turn);
        print_display(2, "pid err", plantCommand - position);
        print_display(5, "iState", pid.iState);
        print_display(6, "dState", pid.dState);
        
        nxt_motor_set_speed(NXT_PORT_B, turn, 1);
        nxt_motor_set_speed(NXT_PORT_C, turn, 1);
    }
}

TASK(task_gyro) {
    gyro_data = ecrobot_get_gyro_sensor(NXT_PORT_S4);
    print_display(7, "Gyro", gyro_data);
    SetEvent(Task1, GyroSensorEvent);
    TerminateTask();
}
