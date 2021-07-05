#ifndef __WARN_PROCESS__H_
#define __WARN_PROCESS__H_
#include "usv_include.h"

#ifndef WINNT
const char ARM_WARN_CFG_FILE_NAME[] = "../cfg/arm_warn_setting.inf";		//配置文件
const char IHC_WARN_CFG_FILE_NAME[] = "../cfg/ihc_warn_setting.inf";		//配置文件
#else
const char ARM_WARN_CFG_FILE_NAME[] = "../../cfg/arm_warn_setting.inf";		//配置文件
const char IHC_WARN_CFG_FILE_NAME[] = "../../cfg/ihc_warn_setting.inf";		//配置文件
#endif

const uint16 ARM_WARN_SQ_BASE = 0;
const uint16 IHC_WARN_SQ_BASE = 800;

#define WARN_MSG_LEN_MAX 128
typedef enum {
	WARN_SRC_ARM = 0,
	WARN_SRC_IHC,
	WARN_SRC_MPC,
	WARN_SRC_IDU
} WARN_SOURCE;		

typedef enum {
	ARM_WARN_MAIN = 0,
	ARM_WARN_DRADIO_TIMEOUT		,		//数字电台通讯超时
	ARM_WARN_BRADIO_TIMEOUT		,		//宽带电台通讯超时
	ARM_WARN_INS_RCV_TIMEOUT	,		//惯导报文接收超时
	ARM_WARN_INS_LOC_INVALID	,		//惯导定位失败
	ARM_WARN_INS_DIF_INVALID	,		//惯导差分失败
	ARM_WARN_INS_GYRO_HEADING	,		//惯导陀螺定向
	ARM_WARN_IHC_RCV_TIMEOUT	,		//IHC通讯超时
	ARM_WARN_PREDICTION_TIMEOUT	,		//感知设备通讯超时
	ARM_WARN_PREDICTION_INVALID	,		//感知设备故障
	ARM_WARN_SAMPLE_TASK_BLOCK	,		//采样系统采样任务阻塞
	ARM_WARN_SAMPLE_TIMEOUT		,		//采样系统通讯中断
	ARM_WARN_SAMPLE_REP_ERROR	,		//采样系统任务回复错误
	ARM_WARN_POWER_LOW_RETURN	,		//低电量自动返航启动
	ARM_WARN_COUNT						//
}ARM_WARN_DESCRIPTION_ENUM;

typedef enum{
	WARN_OFF = 0,
	WARN_ON  = 1
}ARM_WARN_STATE;



typedef struct __WARN_TIME__
{
	uint16	year;
	uint16	month;
	uint16	day;
	uint16	hour;
	uint16	minute;
	uint16	second;
	uint16	m_second;
}WARN_TIME;


typedef struct __WARN_MSG__
{
	uint8	warn_source;		//告警信息来源
	uint16	warn_squence;		//告警信息编号
	int8	warn_stat;			//告警信息状态
	WARN_TIME warn_time;			//告警发生时间
}WARN_MSG;

//读取系统时间
void ReadWarnTime(WARN_TIME *systime);

//初始化告警模块
int iniWarnModel(void);

//初始化告警队列
void initWarnMsgQueue(void);	
//告警信息入队
void WarnMsgQueuePut(int warnSource, int warnSquence, int8 warn_stat);
//读取配置文件
int read_warn_setting(void);
//告警信息出队测试
void WarnMsgQueuGetPub(void);

//初始化nanoMsg
int initNanoPubSender(char *nanoAddr);
//nanomsg端口发送告警消息
void warnNanoPub(char *warnMsg);

//UDP 报文接口
int8 isWarnMsgUdpEmpty(void);
int8 getWarnMsgFromUdpQueue(WARN_MSG *pWarn_msg);
int8 delWarnMsgFromUdpQueue(void);

#endif /*__WARN_PROCESS__H_*/


