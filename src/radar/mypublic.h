#pragma once 

#include <vector>
#include <list>


typedef  unsigned char    uint8;
typedef  signed char      sint8;
typedef  unsigned short   uint16;
typedef  signed short     sint16;
typedef  unsigned int     uint32;
typedef  signed int       sint32;
typedef  float            float32;
typedef  unsigned char *  PChar;
typedef  unsigned short   ISTRING;
typedef	 signed short	  int16;

typedef  struct
{
	uint8         byte0;	//四字节BCD的最高8位
	uint8         byte1;
	uint8         byte2;
	uint8         byte3;	//四字节BCD的最低8位
} tSetBCD;


uint16 calcCRC16(uint16 pseed, const uint8 *buf, uint16 len);

uint32 CRC16(uint8 *arr_buff, uint8 len);

uint8 CRC_XOR(uint8 *arr_buff, uint8 len);

//////////////////////////////////////////////////////////////////////////
enum DefSysID
{
	SysID_CentreBase = 1,   //中心地面站
	SysID_Boat = 16,             //无人船（光伏船规约版本）
	//SysID_Ctrl,             //遥控器
	//SysID_PortableBase,     //便携基站
};

enum BaseComponentID        //地面站组件定义
{
	ID_BaseCtrlSys = 0,
	ID_DebugProgram,        //调试工具(预留)
	ID_SettingProgram,      //配置工具(预留)
};

enum BoatComponentID        //无人船组件定义
{
	ID_BoatEnergy = 0,      //船体能源块
	ID_BoatSwitchCtrl,      //船载开关量控制
	ID_BoatCtrl,            //船体控制块
	ID_BoatPose,            //船体位姿块
	ID_BoatPower,           //船体动力块
	ID_BoatAlarm,           //船体告警信息
	ID_BoatTask,            //航行任务模块
	ID_BoatSensor = 17,     //船载附加传感器汇总
	ID_BoatSensor_IPC_Up,   //IPC上送传感器报文
	ID_Lidar_IPC_Up,        //IPC激光雷达上送报文
	ID_WQAC,                //IPC水质分析采样系统
	ID_SevenPara = 32,      //参数组件，不入库
	ID_BoatSprayOffsetPara, //喷淋偏移参数组件，不入库
	ID_Alarm = 48,          //告警组件，遥信
};

enum UniqueMsgID
{
	UnMsgID_BoatEnergy_ArmUp = 200,          //船体能源块，arm上送消息
	UnMsgID_BoatSwitch_BaseDown = 2,         //船载开关量控制，地面站下发消息
	UnMsgID_BoatCtrl_ArmUp,                  //船体控制模块，arm上送消息
	UnMsgID_BoatCtrl_BaseDown,               //船体控制模块，地面站下发消息
	UnMsgID_BoatPose_ArmUp,                  //船体位姿模块，arm上送消息
	UnMsgID_BoatPower_ArmUp,                 //船体动力模块，arm上送消息
	UnMsgID_BoatTask_BaseDown = 50,          //航行任务模块，地面站下发消息
	UnMsgID_BoatTask_ArmUp,                  //航行任务模块，arm上送消息
	UnMsgID_BoatSensor_ArmUp = 12,           //船载附加传感器模块，arm上送消息
	UnMsgID_BoatSensor_IPC_Arm = 31,         //船载附加传感器模块，IPC到arm发送消息
	UnMsgID_SampleQuanity_IPC_Base = 400,    //实时采样量，IPC到地面站消息
	UnMsgID_SampleTask_Base_IPC,             //采样任务，地面站下发IPC消息
	UnMsgID_SampleTask_IPC_Base,             //采样任务，IPC回复地面站消息
	UnMsgID_AnalysisRslt_IPC_Base,           //水质分析结果，IPC上送地面站消息
	UnMsgID_AnalysisRslt_Base_IPC,           //水质分析结果，地面站回复IPC消息
	UnMsgID_BoatLidar_IPC_Arm_Target1 = 500, //船载激光雷达模块，IPC到ARM,障碍物信息，圆+极坐标
	UnMsgID_BoatLidar_IPC_Arm_Target2,       //船载激光雷达模块，IPC到ARM,障碍物信息，圆+经纬度
	UnMsgID_BoatLidar_IPC_Arm_Navigation,    //船载激光雷达模块，IPC到ARM，辅助导航信息
	UnMsgID_BoatLidar_IPC_Arm_Pos,           //船载激光雷达模块，IPC到ARM，反算的位置信息
	UnMsgID_BoatLidar_Arm_IPC_Pos,           //船载激光雷达模块，ARM到IPC，惯导的位置信息
	UnMsgID_BoatLidar_IPC_Arm_Cross,         //船载激光雷达模块，IPC到ARM，过桥洞信息
	UnMsgID_BoatLidar_Arm_IPC_Cross,         //船载激光雷达模块，ARM到IPC，过桥洞信息
	UnMsgID_VirtualObstacle_Base_ARM = 520,  //虚拟障碍物下发信息，地面站到ARM
	UnMsgID_VirtualObstacle_ARM_Base,        //虚拟障碍物接收反馈，ARM到地面站
	UnMsgID_ElecFenceDown_Base_ARM = 530,    //电子围栏下发，地面站到ARM
	UnMsgID_ElecFenceDown_ARM_Base,          //电子围栏下发反馈，ARM到地面站
	UnMsgID_ElecFenceUp_Base_ARM,            //电子围栏召唤，地面站到ARM
	UnMsgID_ElecFenceUp_Recv_ARM_Base,       //电子围栏召唤反馈，ARM到地面站
};

enum SampleTaskStatus
{
	Status_None = 0,                         //无
	Status_BaseDown,                         //下发
	Status_BoatRecv,                         //接受
	Status_BoatExec,                         //执行中
	Stauts_BoatFinish,                       //完成
	Status_BoatErr,                          //错误
};

enum SampleTaskCmdID
{
	CMD_None = 0,
	CMD_AutoAnalyse = 6,
	CMD_AutoClearPipeline = 8,
	CMD_AutoSingleSample,
	CMD_AutoEmptyWater,
	CMD_AutoContinueAnalyse,
	CMD_Cancel = 94,
	CMD_Reset = 96,
};

