/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_param_tuning.h
-	Author	   : fushuai
-	Date	   : 2019/06/02 9:43
-   Version    : 1.0
-	Description:进坞控制参数设定
-	Others:
*********************************************************************/
#ifndef __DOCKING_TUNING_H_
#define __DOCKING_TUNING_H_
#include "usv_include.h"

#ifndef WINNT 
const char USV_DOCKING_SETTING_FILE_NAME[] = "../cfg/usv_dock_control.inf";						//进坞控制配置文件
#else
const char USV_DOCKING_SETTING_FILE_NAME[] = "../../cfg/usv_dock_control.inf";						//进坞控制配置文件
#endif


//typedef struct
//{
//	int cmd_r;
//	//float pidctrSign;		/*控制符号*/
//	float pid_P;			/*比例增益*/
//	float pid_I;			/*积分增益*/
//	float pid_D;			/*微分增益*/
//	float pid_EI;			/*积分误差区间*/
//	float pid_KB;			/*饱和溢出反馈增益*/
//	float pid_ZB;			/*零点偏置*/
//}DockingParamsPID;
#pragma pack(1)
typedef struct  
{
	uint8 u8_reset;		 //复位
	uint8 u8_return_set; //返航点
	uint8 u8_startp_set;//出发点 用于调试是人为设置 最后发布用计算获取
	uint8 u8_dockin;	 //进坞
	uint8 u8_dockout;	 //出坞
	uint8 u8_ready_in;	 //可进坞
	uint8 u8_already_in; //已进坞
	uint8 u8_ready_out;	 //可出坞
	uint8 u8_already_out;//已出坞
}SumlinkCmd;
#pragma pack()
#pragma pack(1)
typedef struct
{

	uint8	u8_st_oil1;				//油门值
	uint8	u8_st_oil2;				//油门值
	int16	i16_st_moter1Rudder;	// 舵角			33
	int16	i16_st_moter2Rudder;	// 36


	uint16	u16_st_moter1Rpm;		// 发动机转速 31
	uint16	u16_st_moter2Rpm;		// 34

	uint16	u16_st_speed;		// 速度			
	uint16	u16_st_heading;		// 艏向				
	int16	i16_st_rot;			// 转向率	

	uint16 i16_cur_cx;
	uint16 i16_cur_cy;
	uint8  u8_cur_lock;

	int16 u16_ex_cx;


	uint8	u8_st_lonSt;		// 经度标志		
	uint8	u8_st_lonDeg;		// 经度 度		
	uint8	u8_st_lonMin;		// 经度 分		
	uint8	u8_st_lonSec;		// 经度 秒		
	uint8	u8_st_lonSecDec;	// 经度 秒小数	

	uint8	u8_st_latSt;		// 纬度标志		
	uint8	u8_st_latDeg;		// 纬度 度		
	uint8	u8_st_latMin;		// 纬度 分		
	uint8	u8_st_latSec;		// 纬度 秒		
	uint8	u8_st_latSecDec;	// 纬度 秒小数

	//uint8 u8_return_set;
	//uint8 u8_dockin;
	//uint8 u8_dockout;
	//uint8 u8_ready_in;
	//uint8 u8_ready_out;
	//uint8 u8_already_in;

}SumLinkDocking;
#pragma pack()

typedef struct 
{
	uint8  u8_lidar_lock_on;	//锁定状态
	float  f32_lidar_x;			//收到雷达数据x		米
	float  f32_lidar_y;			//收到雷达数据y		米
	double f64_dis;			    //目标距离
	double f64_target_angle;	//船坞方向			度
	double f64_real_angle;		//船当前航向		度
	double f64_track_angle;		//跟踪调整角度		度
	double f64_track_err;		//跟踪横向偏差		米
}SumlinkDockingLidar;

extern char docking_contrl_cfg[30];
extern uint32 SUMLINK_PORT;
extern uint32 SUMLINK_RECVPORT;
extern SumLinkDocking sumlink_dockin_state;
extern SumlinkDockingLidar sumlink_lidardockin_state;
extern SumlinkCmd sumlink_cmd;
void *tuningParsmsSumlinkTempThread(void *aa);
#endif