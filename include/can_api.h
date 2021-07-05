/*
* can_api.h --CAN初始化
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#ifndef	CAN_API_H
#define     CAN_API_H
/*
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <signal.h>

#ifndef WINNT
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>

#endif

#include "USV_const.h"
*/
#include "../include/usv_include.h"


#define	CAN							"can0"								//CAN
#define 	CAN0_bitrate   				250000							//定义CAN波特率
#define	SP_ST1_PGN					65305								//智能面板状态PGN1
#define	SP_ST2_PGN					65306								//智能面板状态PGN2
#define	SP_ST3_PGN					65307								//智能面板状态PGN3
#define	SP_ST4_PGN					65308								//智能面板状态PGN4
#define	SP_ST5_PGN					65309								//智能面板状态PGN5
#define	SP_ST6_PGN					65310								//智能面板状态PGN6
#define	SP_ST7_PGN					65311								//智能面板状态PGN7
#define	UAV_PGN					65320								//UAV状态数据PGN
#define	SR_Speed_PGN				65330								//智能舵速度状态数据PGN
#define	SR_Rudder_PGN				65331								//智能舵舵角状态数据PGN
#define	POW_CON_PGN				65340								//电源管理状态数据PGN
#define	SP_Model_PGN				65353								//智能面板模式控制PGN
#define	SP_CON1_PGN				65350								//智能面板控制PGN1
#define	SP_CON2_PGN				65351								//智能面板控制PGN2
#define	SP_CON3_PGN				65352								//智能面板控制PGN3
#define	ST_PL_PGN					65360								//稳定平台状态数据PGN
#define	BAT_ST_PGN					65370								//能源管理状态数据PGN
#define	BAT_L_PGN					65371								//能源管理左电池状态数据PGN
#define	BAT_R_PGN					65372								//能源管理右电池状态数据PGN



//智能舵状态结构体
typedef struct  
{
       uint16	Motor_RPM_ACT_L;						//左发动机实际转速
       uint16	Motor_RPM_ACT_R;						//右发动机实际转速
       uint8	Motor_Gear_ACT_L;						//左发动机实际档位
       uint8	Motor_Gear_ACT_R;						//右发动机实际档位
       uint16	Rudder_ACT_L;							//左舵实际舵角
       uint16	Rudder_ACT_R;							//右舵实际舵角
}Motor_ACT;
//无人机平台状态结构体
typedef struct  
{
       uint8	Motor_RPM_MAX_L;						//左发动机最高转速
       uint8 	Motor_RPM_MIN_L;						//左发动机最低转速
       uint8	Motor_RPM_MAX_R;						//右发动机最高转速
       uint8	Motor_RPM_MIN_R;						//右发动机最低转速
       uint8	Motor_Gear_MAX_L;						//左发动机最高档位
       uint8	Motor_Gear_MIN_L;						//左发动机最低档位
       uint8	Motor_Gear_MAX_R;						//右发动机最高档位
       uint8	Motor_Gear_MIN_R;						//右发动机最低档位
}UAV_ACT;
//舵角期望结构体
typedef struct  
{
       uint8	Rudder_Limit_L;							//左舵控制舵角
       uint8	Rudder_Limit_R;							//右舵控制舵角
}Rudder_EXP_Con;

//智能航行信息结构体
typedef struct  
{
       uint16	USV_Speed_EXP;							//期望航速
       uint16	USV_Heading_EXP;							//期望航向,航行方向
}Sailing_EXP_Con;

typedef struct  
{
	uint8 CAN_add;
	uint8  CAN_Priority;
	uint16 CAN_PGN;
	uint8	CAN_Data[8];
}CAN_Struct;


extern CAN_Struct 	CAN_Msg;
extern uint8 FAULT_CODE_Last[2][10];
extern uint8 Sealight_Count,Sealight_Sign;

#ifdef WINNT
uint16 read_can(HANDLE hCom,struct can_frame *buff,uint16 len);
int8 write_can(HANDLE hCom,struct can_frame *lpOutBuffer,uint16 write_len);
#else
uint16 read_can(int hCom,struct can_frame *buff,uint16 len);
int8 write_can(int hCom,struct can_frame *lpOutBuffer,uint16 write_len);
#endif

void CAN_Analyzing(CAN_Struct *frame);
void Check_Communication_State(void);
void CAN_Init(void);
void Send_CAN_Control(void);
void Get_CAN_SR_Con();
void Get_CAN_SR_Smart_Con();
void Get_CAN_SR_Engine_Config();
void Get_CAN_SR_Gear_Config();
void Get_CAN_SR_Gear2_Config();
void Get_CAN_SR_Rudder_Config();
void Get_CAN_SR_Rudder2_Config();
void Get_CAN_SR_Rudder3_Config();
void Get_CAN_UAV_Con();
void Get_CAN_POW_Con();
void Get_CAN_ST_PL_Con();
void Get_CAN_Energy_Con();
void Send_CAN_State(void);
void Send_CAN_65305();
void Send_CAN_65306();
void Send_CAN_65307();
void Send_CAN_65308();
void Send_CAN_65309();
void Send_CAN_65310();
void Send_CAN_65311();
void Send_CAN_Cfg();
void Write_Config();

//void Get_Radar_DCPA_TCPA(uint32 DCPA,sint16 TCPA);//获取雷达障碍目标的碰撞参数

#endif /* CAN_API_H */
