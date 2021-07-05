/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// 
// (C) Copyright 2017,Beijing
// All Rights Reserved
//
//
// FileName:        read_usv_state.cpp
// Programmer(s):  syp ,2017-6-22
// Description: 读取文件内容
//                  [s1] s2=value ";"后为注释
*******************************************************************************************/

#ifndef _READ_USV_STATE_H_
#define _READ_USV_STATE_H_


#ifndef WINNT 
const char USV_STATE_FILE_NAME[]="../cfg/usv_flash_state.inf";		//usv运行状态
#else
const char USV_STATE_FILE_NAME[]="../../cfg/usv_flash_state.inf";		//usv运行状态
#endif

#define WRITE_METER_CLOCK_TIME 10


//船的里程表信息
typedef struct {
	uint32	run_time_second;			//运行时间 	单位:	秒
	uint32	run_total_dist;				//总里程		单位:	0.1海里
	uint32	run_total_dist_this_time;	//本次里程		单位:	0.1海里
	uint32	run_time_second_this_time;	//本次运行时间 单位:	秒
	uint32	run_dist_10min;				//十分钟里程	单位:	节*100 *s
	uint32	run_average_speed;			//平均航速	单位:	0.1kn/h
} USV_Meter_Clock;


//全局变量
extern USV_Meter_Clock USV_Meter_Clock_Data;



//函数声明
int8	read_usv_state(void);
int8	Update_Meter_Clock(void);
int8    write_usv_state(void);
int8	updateMeterClock(void);		//小型化装置里程表
#endif
