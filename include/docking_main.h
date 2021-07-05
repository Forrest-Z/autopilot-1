/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : dock_communication.h
-	Author	   : fushuai
-	Date	   : 2019/05/13 16:50
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef  __DOCK_COMMUNICATION_H_
#define  __DOCK_COMMUNICATION_H_

#define CMAERA_TRACKON	1				//摄像头跟踪开始
#define CMAERA_TRACKOFF 2				//摄像头跟踪关闭
#define CAN_USV_ADD 0x80
#define CAN_DOCK_ADD 0x8a
#define CAN_DOCK_PS 0xe9
#define  CAN_DOCK_DISCONNECT_MAX 100 //50ms一个周期
#include "usv_include.h"

#ifndef WINNT
#include <sys/time.h>
#else
#include<winsock.h>
#endif
#define		RETURN_POINT_MAX_NUMBER			3			//最大的返航点数
#ifndef WINNT 
const char USV_DOCKING_FILE_NAME[] = "../cfg/usv_flash_docking.inf";		//usv运行状态
#else
const char USV_DOCKING_FILE_NAME[] = "../../cfg/usv_flash_docking.inf";		//usv运行状态
#endif


#include <climits>
#include <stdint.h>

enum STATS
{
	STAT_RESET = 0,
	STATS_ON,
	STATS_OFF,
};
//外部调用控制结构体
//对应can网中的ps = 13
#pragma pack(1)
typedef struct
{
	uint8 cmd_state;
	float heading;
	float vx;
	float wr;

}DockingControlCmd;
#pragma pack()


//外部调用状态结构体
#pragma pack(1)
typedef struct
{
	uint8 x_open;					/*可进坞*/
	uint8 x_in;						/*船在坞*/
	uint8 x_outgoing;				/*可出坞*/
	uint8 x_in_entrance;			/*到达船坞口*/
}DOCK_STATE;
#pragma pack()

//仿真按钮接口
#pragma pack(1)
typedef struct
{
	uint8 x_reset;					/*复位*/
	uint8 x_open;					/*可进坞*/
	uint8 x_in;						/*船在坞*/
	uint8 x_outgoing;				/*可出坞*/
	uint8 x_outsuccess;				/*已出坞*/
}DOCK_SUMLINK;
#pragma pack()

typedef struct 
{
	uint8 power_on;					/*上电成功*/
	uint8 set_retpos_on;			/*设置返航点成功状态 0:缺省 1:成功 2:定位无效 3:返航点写入失败*/
	uint8 cmd_feedback;				/*进出坞命令收到反馈*/
	uint8 entry_docking;			/*进坞中*/
	uint8 out_docking;				/*出坞中*/
	uint8 succcessed_entry;			/*成功进坞*/
	uint8 succcessed_out;			/*成功出坞*/
	uint8 log_b1_dock_state;		/*缓存泊岸状态*/
	uint8 log_b1_return_state;		/*缓存返航状态*/
}USV_STATE;
#pragma pack(1)

typedef struct {
	
	uint8   dock_entrydock_readyon;		/*船坞是否具备进坞条件*/
	uint8   dock_entrydock_success;		/*船坞是否成功进坞*/
	uint8   dock_outdock_readyon;		/*船坞是否具备出坞*/
	uint8   dock_entrydock_entrance;	/*船坞是否到达进坞口*/
}DOCK_COMM_COIL_READ;
#pragma pack()

typedef struct 
{
	uint8 usv_entrydock_req;			/*船进船坞请求*/
	uint8 usv_prower_off_successed;		/*船熄火成功*/ 
	uint8 usv_prower_on_successed;		/*船上电启动初始化成功*/
	uint8 usv_outdock_successed;		/*船出船坞成功*/

}DOCK_COMM_COIL_SET;

typedef enum
{
	EVENT_POWERON_USV = 0,		  /*船上电初始化完成*/
	EVENT_OUTDOCK_READY,		  /*船出坞准备*/
	EVENT_OUTING,				  /*船出坞中*/
	EVENT_ENTRY_DOCK_READY,		  /*船进坞准备*/
	EVENT_ENTRYING,				  /*船进坞中*/
	EVENT_ENTRYED_USV,		  /*船熄火*/
	EVENT_OUTED_USV,		  /*船已出坞待命中*/
}USVDockEventState;

//返航点
typedef struct  
{
	double	heading;	/*航向*/
	double	longitude;	/*经度*/
	double	latitude;	/*纬度*/
}RETURN_POINT;

//船坞目标位置
typedef struct
{
	uint16 cols;		//图像的宽
	uint16 rows;		//图像的高
	uint16 target_w;	//目标的宽
	uint16 target_h;	//目标的高
	uint16 t_x;		//目标的像素位置x
	uint16 t_y;		//目标的像素位置y
	uint8 lock_on;
}TargetPos;

typedef struct
{
	float delta_x;
	float delta_y;
	uint8 lock_on;
}TargetPosLidar;

//usv内部感知交互变量
extern char docking_tracker_cfg[30];
extern TargetPos t_pos;
extern char lidar_tracker_cfg[30];
extern TargetPosLidar target_pos_lidar;

//船控制交互结构体
extern DOCK_SUMLINK dock_sumlink;
extern DOCK_STATE dock_sign;
extern USV_STATE usv_sign;
extern uint8 dockin_fun_enable;
extern RETURN_POINT return_point[RETURN_POINT_MAX_NUMBER];
extern int8 return_num;
extern float docking_speed;

//dock can 
extern COMM_SIGN DOCK_comm_sign;

extern DockingControlCmd docking_control_cmd;//船坞控制命令
//船——船坞功能函数

extern void *dockCommunicationUSVRun(void *);//船 - 坞交互操作
extern void *cameraTrackPosFlush(void *);//摄像头跟踪数据
extern void *lidarTrackerPosFlush(void *);//激光雷达跟踪数据

//can
extern void initDocksendTask(void);
extern void DOCK_Init();
extern void DOCK_recv(uint8 ps_id, uint8* data);
extern void DOCK_reInit();
extern void DOCK_sendMsg();
extern void DOCK_CommCal(void);
void runDocksendTask(void *);
//los 参数
extern uint16 n_ship;//船长倍数
extern float l_ship;//船长
extern float temp_dockin_distance;//临时近坞点计算距离
extern float image_servo_start_distance;//视觉介入距离
extern float image_servo_end_distance;//视觉介入距离
extern float image_servo_pix_multi;//像素距离比系数
extern uint8 motorEnable_sign;//发动机使能标致位
extern int8 dockingIn();
#endif//DOCK_COMMUNICATION_H_