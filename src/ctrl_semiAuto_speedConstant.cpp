/*==========================================================*
 * ģ��˵��: ctrl_semiAuto_speedConstant.cpp                *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
 * ����ʱ��: 2018��2��27�� 20��59��                         *
 * Copyright(c) sf-auto.ltd                                 *
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
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
		pPidAutoSpeed->setOutputValue(jet_system.jetL.u8_Cmd_MotorOpenDeg);	//��ʼ�����ſ���
		//v_PID.pid_out = jet_system.jetL.u8_Cmd_MotorOpenDeg;	
	}
	speed_const_st.double_speed_exp = command_signal.semi_mode_cmd.speed_const_value;
}

void calcSpeedConstant(void)
{
	//double openDeg;
	//openDeg = simu_speed_PID(speed_const_st.double_speed_exp,ins_msg.speed);


	////���ſ��ȿ���
	//jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	//jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;

	////ǰ����λ
	//jetGearFoward();	//ǰ��
}

void SemiAutoSpeedConstant(void)
{
	GetSpeedConstant();
	if(command_signal.func_mode_cmd.b1_speedConstant == COMMAND_ON)
	{
		nCalOpenDeg(speed_const_st.double_speed_exp);
	}
}