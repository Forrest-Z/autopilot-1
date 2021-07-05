/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_panel_comm.h
-	Author	   : fushuai
-	Date	   : 2019/05/24 15:33
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef __DOCKING_PANEL_COMM_H_
#define __DOCKING_PANEL_COMM_H_
#include "usv_include.h"

#define	MSG_VALID                  1  /*手操盒有效消息*/
#define	MSG_INVALID                0  /*手操盒无效消息*/

#define	JOYSTICK_AUTHORITY         0  /*手操盒逻辑权限*/
#define ARM_AUTHORITY              1  /*本地逻辑权限*/
//Mode select
typedef enum
{
	MANUAL_MODE = 0,
	SEMI_INTELLIGENT_MODE = 1,
	INTELLIGENT_MODE = 2
}SbusVarMode;
//panel SBUS channel maping table
typedef enum
{
	SB_AUTHOR_CH = 0,
	SA_MODE_CH = 1,
	SC_PORT_ENG_STS_CH = 2,
	SF_STBD_ENG_STS_CH = 3,
	T3_ENG_THR_GEAR_CH = 4,
	T2_STERR_CH = 5,
	LD_PWM_CH = 6,
	RD_PWM_CH = 7,
	SD_SWITCH_CH = 8,
	SE_SWITCH_CH = 9,
	T1_CH = 10,
	T4_CH,
	T5_CH,
	T6_CH,
	T7_CH,
	T8_CH
}SbusCfgCh_t;

//Manual operation message value definition
#define BUTTON_RESET                     0
#define BUTTON_SET                       1

#define PORT                             0
#define STBD                             1
#define ENGINE_START                     1
#define ENGINE_RESET                     0
#define ENGINE_STOP                      2

#define ENGINE_MAX_THR_OPEN            255
#define ENGINE_MIN_THR_OPEN               0

#define GEAR_MAX_OPEN                   255
#define GEAR_MID_OPEN                    0
#define GEAR_MIN_OPEN                  -255

#define STEER_MAX_OPEN                  255
#define STEER_MID_OPEN                   0
#define STEER_MIN_OPEN                 -255

#define PWM_MAX_OUT                    100
#define PWM_ZERO_OUT                   0
#define PWM_MIN_OUT                   -100

#define SWITCH_ON                       1
#define SWITCH_OFF                      0
#define SWITCH_STOP                     0
#define SWITCH_UP                       1
#define SWITCH_DOWN                     2


//Semi-intelligent mode message value definition
#define USV_SPEED_EPS                  10                             /*!< Speed magnitude representation accuracy = 0.1. */
#define USV_SPEED_VALUE_MAX            (5*USV_SPEED_EPS)              /*!< Max speed. (kn) */
#define USV_SPEED_VALUE_MIN            (-(5*USV_SPEED_EPS))            

#define USV_YAW_RATE_EPS               10                              /*!< Yaw rate velocity representation accurary = 1/2^(USV_YAW_RATE_EPS). */
#define USV_YAW_RATE_MAX               (360*USV_YAW_RATE_EPS)         /*!< Max yaw angular velocity. (deg/s)*/
#define USV_YAW_RATE_MIN               (-(360* USV_YAW_RATE_EPS))

typedef struct
{
	uint8_t msg_status;                   /*!< Data effective */
	uint8_t oper_author;                  /*!< Operation authority. */
	uint8_t oper_mode;                    /*!< Operation mode.      */

	/* Manual operation mode messages. */
	uint16_t eng_sts[2];                 /*!< Engine fire on/off.  */
	uint16_t eng_throttle;               /*!< Engine throttle opening. */
	int16_t  gear_open;                  /*!< Gear opening. */
	int16_t  steer_open;                 /*!< Steer opening. */
	int16_t  PWMDuty[2];                /*!< Regulate PWM duty cycle to servo some motors. */
	uint16_t on_off_switch;               /*!< Switch control on/off state. */
	uint16_t up_down_switch;              /*!< Switch control Up/Stop/Down state. */

	/* Semi-intelligent mode messages. */
	int16_t speed_lat;                  /*!< Lateral velocity.*/
	int16_t speed_lng;                  /*!< Longitudinal velocity. */
	int16_t yaw_ang_vel;                 /*!< Yaw angle velocity.*/
}SbusCfgVarList;
extern SbusCfgVarList sbus_var_list;


//外部手操盒控制消息初始化
void panelMsgInit(void);
//外部手操盒控制消息更新
void panelMsgUpdate(void);

#endif// __DOCKING_PANEL_COMM_H_