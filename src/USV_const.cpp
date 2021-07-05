#include "stdafx.h"
#include "../include/usv_include.h"

#ifdef WINNT
HANDLE UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//定义串口标志	
HANDLE CAN_0;//定义CAN
#else
int UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//定义串口标志	
int CAN_0;//定义CAN
#endif
Radar_OBS Radar_OBS_Msg;				//雷达障碍物的信息
Path_Coordinate Path_Coordinate_Msg;	//避障规划点坐标

IP_ADDRESS_struct  ip_address;
SR_Config SR_Config_Msg;
UAV_CAN	UAV_CAN_State;//无人机平台状态
SR_CAN	SR_CAN_State;//智能舵状态
POW_CAN POW_CAN_State;//电源管理系统状态
SP_Model_CAN	SP_CAN_Model_Con;//面板控制模式指令
SP_Sail_CAN		SP_Sail_CAN_Con;//面板控制发动机指令
SP_Equipment_CAN 	SP_Equipment_CAN_Con;//面板控制稳定平台指令
ST_PL_CAN ST_PL_CAN_State;//稳定平台状态
BAT_CON_CAN BAT_CON_CAN_State;//能源管理系统状态
Motor_CAN Motor_CAN_State;//发动机状态
RECEIVE_STATU_MACHINE_T Receive_Status[2];//J1939接收状态机
FAULT_CODE_T	Fault_Code[2];//发动机故障码

AIS_Msg AIS_Msg_St;//图传电台发送AIS信息

Motor_Detail_State	Motor_Detail_St;//图传电台发送发动机详细信息
Rudder_Detail_State Rudder_Detail_St;//图传电台发送智能舵详细信息
Stable_Platform_Detail_State Stable_Platform_St;//图传电台发送稳定平台详细信息
UAV_Detail_State UAV_Detail_St;//图传电台发送无人机平台详细信息
MCU_State MCU_St;//图传电台发送控制器监测系统详细信息
Hull_State Hull_St;//图传电台发送船体监测详细信息
Panel_Control_Msg Panel_Control_St;//图传电台发送船载智能控制面板详细信息
Energy_Control_Msg Energy_Control_St;//图传电台发送能源管理系统详细信息
Power_Control_Msg  Power_Control_St;//图传电台发送电源管理系统详细信息
Check_Collision		USV_Check_Collision[AIS_Collision_Num];//碰撞计算
Control_Message	USV_Control;//无人船控制指令
Sailing_Message		USV_Sailing;				//无人船航行任务
Dradio_Config_Message Dradio_Config;			//无人车配置信息，从配置文件获得
State_Message 		USV_State;					//无人船状态
LAN0_RM_Message	USV_RM_MSG;//MPC发送RAM网口报文
Waypoint				Point_Return[2];			//一键返航点 0--起航时的位置  1--当前位置
DSP_State DSP_State_Msg;//DSP状态
	
Spd_PID	vPID;						//发动机转速调整PID参数，及船速
Spd_PID	rPID;						//舵角调整PID参数
Spd_PID	aPID;                       //注释掉 没用
Spd_S	vSpd,rSrd;							//定义速度航向S面控制结构体

TASK_TIME_STRUCT	task_time;
Track_Control		track_control;		//航迹控制数据
E_STOP_ALL	e_stop_inf;					//急停信息
	
uint8	Dradio_Com1_Sign;//数字电台COM1连接状态
uint8	Dradio_Com2_Sign;//数字电台COM2连接状态
uint8	SpareDradio_Com1_Sign;//冗余数字电台COM1连接状态
uint8	SpareDradio_Com2_Sign;//冗余数字电台COM2连接状态
uint8	Bradio_Con_Sign;//宽带电台连接状态
uint8	BDS_Con_Sign;//北斗电台连接状态
uint8	SP_CON_Sign;//船体智能面板获取控制权限标志 1--船体智能面板控制  0--窗体放弃控制
uint8	BD_Engine_Run_L;	//接收点火指令L
uint8	BD_Engine_Run_R;	//北斗点火指令R


int LAN0_Fd,LAN1_Fd;//定义网口

uint8	Radio_Sign,Radio_Sign_old,Connect_Sign;
uint8	Sailing_Sign;
uint8	Sailing_Cnt_Old;			//当前航行的航线序号
uint8	Avoid_Point_Arrive=0;			//避障点到达标志
double Speed_EXP,Heading_EXP;//智能算法计算的期望航向航速，Speed_EXP--速度期望值，Heading_EXP--方向期望值
double	Collision_Speed;//避障航速
double	Collision_Heading;//避障航向
double	Radar_Collision_Heading;	//雷达避障航向
sint16 GLB_TCPA;
int32	GLB_Safe_DCPA,GLB_DCPA;
uint8 Collision_Num;
uint8	Rudder_Con_Angle;//控制舵角，实际舵角不能大于控制舵角
uint8	Rudder_Count;	//打某一方向舵的持续时间
uint8	Collision_Sign;	//碰撞标志
int Watch_fd;
double lau_old,lnu_old;//计算里程的经纬度起点
uint32  Mileage;//里程
uint8 E_Stop,E_Stop_INS,E_Stop_DSP,E_Stop_Dradio,E_Stop_SmartRudder,Time_Sign;
int Rudder_Zero_Angle;//舵角零点,补偿舵角偏移
int Accelerator_L,Accelerator_R,Rudder_L,Rudder_R,Gear_L,Gear_R;			//Accelerator--油门 Rudder--舵角 Gear--档位
double Accelerator_Coefficient_L,Accelerator_Coefficient_R,Rudder_Coefficient_L,Rudder_Coefficient_R,Gear_Coefficient_L_F,Gear_Coefficient_L_B,Gear_Coefficient_R_F,Gear_Coefficient_R_B;
uint16 Heartbeat_Num;

float Dst_monitor=0;
int32 NoStopPoint_Sign=0;				//停船标志
int32 Obstacle_Sign = 0;				//避障标志	PSO 避障标志

int32 radar_obstacle_sign =0;			//动态障碍物碰撞标志
int32 radar_avoid_type=0;				//0 无动作 1 相遇  2 追赶 3 左交叉  4 右交叉

int sign,Get_Control_Sign,count_Estop,SP_CAN_Count;
uint8	USV_State_Current[State_Current_Num];//无人船当前状态，电台上送
char Version[10][100];
int ARMVersion_sign,DSPVersion_sign,MPCVersion_sign,UAVVersion_sign,SRVersion_sign,POWVersion_sign,SPVersion_sign,STVersion_sign,BATVersion_sign;

//调试用sockid
int16	sockid_test = 0;
uint8	sockid_test_buf[20];


//船体版本
int ship_version=0;

//船号
uint16 usv_num = 0;