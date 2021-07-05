/*==========================================================*
 * 模块说明: control_BD.cpp                                 *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员: shaoyuping                                     *
 * 创建时间: 2018/4/26 11:39:54                             *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_BD.h"
/******************************  Local Variable  *****************************/
BD_NAVI_ST BDNaviSt;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void BD_operation();
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void BD_operation( void )
{
	//关闭情况怠速
	if(bd_sailTask.u8_cmd_sailOnOff == 0)
		idlingSpeedCtrl();	
	else if(bd_sailTask.u8_cmd_sailOnOff == 1)
		BDNavigation();

}

void BDcalSpeedHeading(void)
{
	BDNaviSt.double_dst = Get_distance(	ins_msg.latitude,							\
										ins_msg.longitude,							\
										bd_sailTask.sailMsg.wayPoint.f64_latitude,	\
										bd_sailTask.sailMsg.wayPoint.f64_longitude	\
										);
	BDNaviSt.double_speed_exp   = bd_sailTask.sailMsg.wayPoint.f64_expSpeed;
	BDNaviSt.double_heading_exp = Get_heading(ins_msg.latitude,							\
											  ins_msg.longitude,							\
											  bd_sailTask.sailMsg.wayPoint.f64_latitude,	\
											  bd_sailTask.sailMsg.wayPoint.f64_longitude	\
											  );
}

void BDJudgeDstArrive(void)
{
	if (BDNaviSt.double_dst < 70.0)	//小于70m 认为到达
	{
		//todo到达逻辑
		bd_sailTask.u8_St_BDSailState = 2;//执行完毕
		bd_sailTask.u8_St_sailMsgRev  = 0;//任务清除
	}
	return;
}

void BDRudderOpenDeg(void)
{
	double openDeg;
	double rudder;
	openDeg = simu_speed_PID(BDNaviSt.double_speed_exp,ins_msg.speed);
	rudder  = simu_heading_PID(BDNaviSt.double_heading_exp,ins_msg.heading);
	//油门开度
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
	//舵角
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudder;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudder;
	//档位
	jetGearFoward();	//前进
	//
	bd_sailTask.u8_St_BDSailState = 1;	//执行中
}


void BDNavigation( void )
{
	BDcalSpeedHeading();		//计算期望航向,期望航速，距离
	BDRudderOpenDeg();
	BDJudgeDstArrive();
}
