//uart_deal_ins.h
#ifndef __UART_DEAL_INS__H_
#define __UART_DEAL_INS__H_

#include "usv_include.h"
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include "../src/proto/obstacle/header.pb.h"
#include "../src/proto/obstacle/error_code.pb.h"
#include "../src/proto/obstacle/geometry.pb.h"
#include "../src/proto/obstacle/gnss_un237.pb.h"

#ifndef WINNT
    typedef int UART_TYPE;
#else
    typedef HANDLE UART_TYPE;
#endif

#define INS_COMM_TIMEMAX 3



//惯导状态结构体
typedef struct  
{
	uint16	USV_Speed;						//总航速  接收后 *100 为单位（0.01节）
	uint16	USV_Heading;					//航向
	uint16	USV_Position_Heading;			//艏向角
	uint16	USV_Move_Heading;				//航迹角
	int16	USV_Roll;						//横滚(横摇)
	int16	USV_Pitch;						//俯仰(纵摇)
	uint16	USV_Bow;						//艏摇N
	uint16	USV_Transverse;					//横荡N
	uint16	USV_Surge;						//纵荡N
	uint16	USV_Heave;						//升沉(垂荡)   
	uint16	USV_Yaw;						//偏流角N
	uint16	USV_ROT;						//转向率			
	uint32	USV_Height;						//海拔
	uint8	Latitude_Sign_St;				//无人船位置经度标志
	uint8	Longitude_Sign_St;				//无人船位置维度标志	1--北半球 ，2--南半球
	uint8	USV_Latitude_Degree;			//USV位置经度值度
	uint8	USV_Latitude_Minute;			//USV位置经度值分
	uint8	USV_Latitude_Second;			//USV位置经度值秒
	uint8	USV_Latitude_Decimal_2;			//USV位置经度值秒小数位1/2位
	uint8	USV_Latitude_Decimal_4;			//USV位置经度值秒小数位3/4位
	uint8	USV_Longitude_Degree;			//USV位置维度值度
	uint8	USV_Longitude_Minute;			//USV位置维度值分
	uint8	USV_Longitude_Second;			//USV位置维度值秒
	uint8	USV_Longitude_Decimal_2;		//USV位置维度值秒小数位1/2位	
	uint8	USV_Longitude_Decimal_4;		//USV位置维度值秒小数位3/4位
	uint8	USV_Year;						//年
	uint8	USV_Month;						//月
	uint8	USV_Date;						//日
	uint8	USV_Hour;						//时
	uint8	USV_Minute;						//分
	uint8	USV_Second;						//秒
	uint8	USV_Second_2;					//秒的小数点后两位
	uint8	Satellite_Num_1;				//天线1卫星个数
	uint8	Satellite_Num_2;				//天线2卫星个数
	uint8	Sys_State_1;					//系统状态1
	uint8	Sys_State_2;					//系统状态2 N
	uint8	Power_Light;					//电源灯
	uint8	Serial_Light;					//串口灯
	uint8	Main_Antenna_Light;				//主天线指示灯
	uint8	Minor_Antenna_Light;			//副天线指示灯N
	uint8	Differerntial_Signal_Light;		//差分信号指示灯
	uint8	Differential_Position_Light;	//差分定位指示灯
	uint8	Smart_Navigation_Heatbeat;		//心跳
	double	USV_Lat;						//经度，小数表示
	double	USV_Lng;						//维度，小数表示
}Smart_Navigation_Msg;


typedef struct{
	uint8	u8_sateliteNum1;			//定位卫星个数
	uint8	u8_sateliteNum2;			//天线2卫星个数
	uint8	u8_sysState1;				//GPS定向 / 陀螺定向
	uint8	u8_sysState1_old;			//GPS定向 / 陀螺定向历史值
	uint8	u8_sysState2;				//备用
	uint8	b1_diffSignalValid_old;		//差分信号有效历史值
	uint8	b1_diffSignalValid;			//差分信号有效
	uint8	b1_diffPostionValid;		//差分定位有效
	uint8	b1_dateValid;				//日期有效
	uint8	b1_timeValid;				//时间有效
	char	c_rmcValid;					//GPRMC位置有效
	char	c_rmcValid_old;				//GPRMC位置有效old
}INS_STATE;


typedef struct{
	double	speed;					// kn
	double	heading;				// 艏向
	double  motionDirection;		// 地面航向
	double	longitude;				// 经度
	double	latitude;				// 纬度
	float	rotRate;				// 转向率

	float	pseudorRangeError;		// 伪距标准差均方根
	uint8	locationState;			// GPGGA定位状态 0 无定位 1 单点定位 2 差分定位 4 RTK固定解 5 RTK浮点解
	uint8	u8_year		;	//1
	uint8	u8_month	;	//1
	uint8	u8_date		;	//1

	uint8	u8_hour		;	//1
	uint8	u8_minute	;	//1
	uint8	u8_second	;	//1
	uint8	u8_second_2	;	//秒小数

	uint16	u16_speed	;	//
	uint16	u16_heading	;	//
	uint16  u16_volecityDir;
	
	int16	i16_rot		;	//
	int16	i16_pitch	;	//
	int16	i16_roll	;	//
	int16	i16_heaving	;	//

	uint8	u8_longiSt		;	//
	uint8	u8_longiDeg		;	//
	uint8	u8_longiMin		;	//
	uint8	u8_longiSec		;	//
	uint8	u8_longiSecDec	;	//
	uint8	u8_longiSecDec2	;	//

	uint8	u8_latiSt		;	//
	uint8	u8_latiDeg		;	//
	uint8	u8_latiMin		;	//
	uint8	u8_latiSec		;	//
	uint8	u8_latiSecDec	;	//秒小数 1 2 位
	uint8	u8_latiSecDec2	;	//秒小数 3 4 位

	INS_STATE insState		;
}INS_DETELL;	//计算用位置及速度参数


typedef struct{
	void*	zmq_context;
	void*	zmq_publisher;
	char	zmq_cfg[30];
}ZMQ_SOCKET;

typedef struct{
	uint8	insValid;
	double  latitude;
	double	longitude;
	double	heading	;
}INS_PUB_MSG;

typedef struct{
	uint8	b_primary;
	uint8	b_secondary;
	uint8	b_diff;
	uint8	b_dGps;
	uint8	b_rtcErr;
}UN237_HARDWARE_STATE;

extern Smart_Navigation_Msg Smart_Navigation_St;	//惯导系统详细信息
extern INS_DETELL	ins_msg;						//惯导数据double型
extern COMM_SIGN	ins_sign;						//惯导通讯状态
extern UN237_HARDWARE_STATE ins_hardState;
extern char	ins_nanoAddr[30];

extern char ins_udp_addr[30]; //惯导数据转发输出udp地址
extern uint32 ins_udp_port;   //惯导数据转发输出port端口

void *uart_deal_ins(void *aa);
void writeSysTime(void);

void ins_CommCal(void *);	//统计通讯超时
extern void insCommCalInit(void);

//zmq_publisher
extern ZMQ_SOCKET ins_zmq_pub;
void ins_zmq_init(void);
void ins_zmq_publish(void);
void ins_zmq_close(void);


void *nano_deal_ins(void *aa);

//----------un237 GPIO 读取-----------------//
//GPIO初始化
int initUn237Gpio(void);
void getHardwareState(void);


#endif /*__UART_DEAL_INS__H_*/