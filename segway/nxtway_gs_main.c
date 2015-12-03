/**
 ******************************************************************************
 ** FILE NAME : nxtway_gs_main.c
 **
 ** ABSTRUCT  : NXTway-GS (with a HiTechnic Gyro Sensor)
 **             Two wheeled self-balancing R/C robot controlled via Bluetooth.
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "balancer.h" /* NXTway-GS C API header file */ <-This header file has to be included 
#include "nxt_config.h"

/*============================================================================
 * MACRO DEFINITIONS
 *===========================================================================*/
typedef enum {
  INIT_MODE,   /* system initialize mode */
  CAL_MODE,    /* gyro sensor offset calibration mode */
  CONTROL_MODE /* balance and RC control mode */
} MODE;

#define MIN_DISTANCE (25)    /* minimum distance in cm for obstacle avoidance */
#define BT_RCV_BUF_SIZE (32) /* it must be 32bytes with NXT GamePad */

/*============================================================================
 * DATA DEFINITIONS
 *===========================================================================*/
MODE nxtway_gs_mode = INIT_MODE; /* NXTway-GS mode */
volatile U8 obstacle_flag;       /* 1(obstacle detected)/0(no obstacle) */

/*============================================================================
 * FUNCTIONS
 *===========================================================================*/
/*============================================================================
 * Embedded Coder Robot hook functions
 *===========================================================================*/
//*****************************************************************************
// FUNCTION    : ecrobot_device_initialize
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : ECRobot device init hook function
//*****************************************************************************
void ecrobot_device_initialize(void)
{
  ecrobot_init_sonar_sensor(PORT_SONAR);
  ecrobot_init_bt_slave(BT_PASS_KEY);
}

//*****************************************************************************
// FUNCTION    : ecrobot_device_terminate
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : ECRobot device term hook function
//*****************************************************************************
void ecrobot_device_terminate(void)
{
  nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
  nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
  ecrobot_term_sonar_sensor(PORT_SONAR);
  ecrobot_term_bt_connection();
}

/*============================================================================
 * TOPPERS ATK specific Function/Tasks
 *===========================================================================*/
DeclareCounter(SysTimerCnt);

//*****************************************************************************
// FUNCTION    : user_1ms_isr_type2
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : 1msec periodical OSEK type 2 ISR
//*****************************************************************************
void user_1ms_isr_type2(void)
{
  /* Increment System Timer Count to activate periodical Tasks */
  (void)SignalCounter(SysTimerCnt);
}

//*****************************************************************************
// TASK        : OSEK_Task_ts1
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : 4msec periodical Task and controls NXTway-GS 
//               INIT_MODE
//               ?
//               CAL_MODE
//               ?
//               CONTROL_MODE 
//*****************************************************************************
TASK(OSEK_Task_ts1)
{
  static U32 gyro_offset;    /* gyro sensor offset value */
  static U32 avg_cnt;        /* average counter to calc gyro offset */
  static U32 cal_start_time; /* calibration start time */
  static U8 bt_receive_buf[BT_RCV_BUF_SIZE]; /* Bluetooth receive buffer(32bytes) */
  SINT i;
  S8 cmd_forward,cmd_turn;
  S8 pwm_l,pwm_r;

  switch(nxtway_gs_mode){
    case(INIT_MODE):
      gyro_offset = 0;
      avg_cnt = 0;
      for (i = 0; i < BT_RCV_BUF_SIZE; i++){
        bt_receive_buf[i] = 0;
      }
      balance_init(); /* NXTway-GS C API initialize function */ <-Call balance init function 
      nxt_motor_set_count(PORT_MOTOR_L, 0); /* reset left motor count */
      nxt_motor_set_count(PORT_MOTOR_R, 0); /* reset right motor count */
      cal_start_time = ecrobot_get_systick_ms();
      nxtway_gs_mode = CAL_MODE;
      break;

    case(CAL_MODE):
      gyro_offset += (U32)ecrobot_get_gyro_sensor(PORT_GYRO);
      avg_cnt++;
      /* 1000msec later, start balancing */
      if ((ecrobot_get_systick_ms() - cal_start_time) >= 1000U){
        gyro_offset /= avg_cnt;
        ecrobot_sound_tone(440U, 500U, 30); /* beep a tone */
        nxtway_gs_mode = CONTROL_MODE;
      }
      break;

    case(CONTROL_MODE):
      /* 
       * R/C command from NXT GamePad 
       * buf[0]: -100(forward max.) to 100(backward max.)
       * buf[1]: -100(turn left max.) to 100(turn right max.)
       */
      (void)ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);
      cmd_forward = -(S8)bt_receive_buf[0]; /* reverse the direction */
      cmd_turn = (S8)bt_receive_buf[1];
      if (obstacle_flag == 1){
        /* make NXTway-GS move backward to avoid obstacle */
        cmd_forward = -100;
        cmd_turn = 0;
      }

      /* NXTway-GS C API balance control function (has to be invoked in 4msec period) */
      balance_control( <-Call balance control function
        (F32)cmd_forward,
        (F32)cmd_turn,
        (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
        (F32)gyro_offset,
        (F32)nxt_motor_get_count(PORT_MOTOR_L),
        (F32)nxt_motor_get_count(PORT_MOTOR_R),
        (F32)ecrobot_get_battery_voltage(),
        &pwm_l,
        &pwm_r);
      nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
      nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

      ecrobot_bt_data_logger(cmd_forward, cmd_turn); /* Bluetooth data logger */
      break;

    default:
      /* unexpected mode */
      nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
      nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
      break;
  }

  TerminateTask(); /* terminates this task and executed by System Timer Count */
}

//*****************************************************************************
// TASK        : OSEK_Task_ts2
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : 40msec periodical Task and detects obstacle in front of 
//               NXTway-GS by using a sonar sensor
//*****************************************************************************
TASK(OSEK_Task_ts2)
{
  obstacle_flag = 0; /* no obstacles */
  if ((nxtway_gs_mode == CONTROL_MODE) && 
    (ecrobot_get_sonar_sensor(PORT_SONAR) <= MIN_DISTANCE)){
     obstacle_flag = 1; /* obstacle detected */
  }

  TerminateTask(); /* terminates this task and executed by System Timer Count */
}

//*****************************************************************************
// TASK        : OSEK_Task_Background
// ARGUMENT    : none
// RETURN      : none
// DESCRIPTION : Background(never terminated) Task
//*****************************************************************************
TASK(OSEK_Task_Background)
{
  while(1){
    ecrobot_status_monitor("NXTway-GS"); /* LCD display */
    systick_wait_ms(500); /* 500msec wait */
  }
}

/******************************** END OF FILE ********************************/