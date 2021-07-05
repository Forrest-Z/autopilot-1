#ifndef __SAILWAYPOINT__H_
#define __SAILWAYPOINT__H_

#include "usv_include.h"

#define WAYPOINT_MAX 256
#define ARRIVAL_SPEED 1.0

typedef struct{
	double f64_latitude			;	
	double f64_longitude		;	
	uint16 u16_stopTime			;	// seconds
	double f64_expSpeed			;	// kn
	uint64 u64_sailPointID		;	// waypoint id
	uint16 u16_sampleVolume		;	// ml
	uint8  b1_type				;	// 0: do not sampling;
                                    // 1: sampling without taking water;
                                    // 2: sampling with taking water;
                                    // 3: nosampling with taking water;
	uint8  b1_samplingComplete	;	//
	uint8  b1_samplingCommand	;	//
	uint8  b1_sailArrival		;	//
}WAYPOINT;


typedef struct{
	//uint8		u8_usvNum				;	//无人船编号
	uint8		u8_pointSum				;	//航点总数
	uint8		u8_msgSrc				;	//命令来源 0:宽带电台
	uint8		u8_sailNum				;	//任务编号
	WAYPOINT	wayPoint[WAYPOINT_MAX]	;	//航点信息
}SAIL_MESG;

typedef struct{
	uint8		u8_St_sailMsgRev	;	//航行任务执行状态 0：无任务 1：有任务 2：任务完成
	uint8		u8_PointNum			;	//正在执行航点编号
	SAIL_MESG	sailMsg;
}SAIL_TASK;

extern SAIL_TASK sailTask;
extern void getSailMsg(uint8* recBuf);
extern void getSailMsgWaterQuality(uint8* recBuf);


#endif /*__SAILWAYPOINT__H_*/