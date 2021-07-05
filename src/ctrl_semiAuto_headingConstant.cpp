/*==========================================================*
 * 模块说明: ctrl_semiAuto_headingConstant.cpp              *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员:                                                *
 * 创建时间: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/ctrl_semiAuto_headingConstant.h"
/******************************  Local Variable  *****************************/
HEADING_CONSTANT_ST heading_const_st;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void GetHeadingConstant(void)
{
  //  if(semi_modeTrig.b1_trig_headingConstant == 1)   
		//heading_const_st.double_heading_exp = ins_msg.heading;
	heading_const_st.double_heading_exp = command_signal.semi_mode_cmd.heading_const_value;
}


void calcHeadingConstant(void)
{
	float motor_rudder;
	//motor_rudder = simu_heading_PID(heading_const_st.double_heading_exp,ins_msg.heading);
	
	//天极号专用，左右舵角PID参数分离 simu_heading_TJ_PID
	//motor_rudder = simu_heading_TJ_PID(heading_const_st.double_heading_exp,ins_msg.heading);
	
	if(ins_msg.speed>4.5)
		motor_rudder = simu_heading_TJ_PID(heading_const_st.double_heading_exp,ins_msg.heading);
	else
		motor_rudder = simu_heading_TJ_PID_LowSpeed(heading_const_st.double_heading_exp,ins_msg.heading);

	jet_system.jetL.i16_Cmd_MotorRudderDeg = motor_rudder;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = motor_rudder;
}


void SemiAutoHeadingConstant(void)
{
	GetHeadingConstant();
	if(command_signal.func_mode_cmd.b1_headingConstant == COMMAND_ON)
	{
		/*calcHeadingConstant();*/
		nCalRudder(heading_const_st.double_heading_exp);
	}
}
