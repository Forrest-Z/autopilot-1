/*==========================================================*
 * 模块说明: ctrl_semiAuto_speedConstant.cpp                *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员: shaoyuping                                     *
 * 创建时间: 2018年2月27日 20点59分                         *
 * Copyright(c) sf-auto.ltd                                 *
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
 *==========================================================*
 *=========================================================*/
 
 /**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/ctrl_semiAuto_speedConstant.h"

/******************************  Local Variable  *****************************/
SPEED_CONSTANT_ST speed_const_st;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void GetSpeedConstant(void)
{
	if(semi_modeTrig.b1_trig_speedConstant == 1)
	{
		pPidAutoSpeed->setOutputValue(jet_system.jetL.u8_Cmd_MotorOpenDeg);	//初始化油门开度
		//v_PID.pid_out = jet_system.jetL.u8_Cmd_MotorOpenDeg;	
	}
	speed_const_st.double_speed_exp = command_signal.semi_mode_cmd.speed_const_value;
}

void calcSpeedConstant(void)
{
	//double openDeg;
	//openDeg = simu_speed_PID(speed_const_st.double_speed_exp,ins_msg.speed);


	////油门开度控制
	//jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	//jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;

	////前进档位
	//jetGearFoward();	//前进
}

void SemiAutoSpeedConstant(void)
{
	GetSpeedConstant();
	if(command_signal.func_mode_cmd.b1_speedConstant == COMMAND_ON)
	{
		nCalOpenDeg(speed_const_st.double_speed_exp);
	}
}