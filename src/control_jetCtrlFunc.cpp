/*==========================================================*
 * ģ��˵��: control_jetCtrlFunc.cpp                        *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա:                                                *
 * ����ʱ��: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_jetCtrlFunc.h"
/******************************  Local Variable  *****************************/
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
void idlingSpeedCtrl(void)
{
	

	jet_system.jetL.u8_Cmd_MotorOpenDeg    = 0;
	jet_system.jetL.i16_Cmd_MotorGearDeg   = 0;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;

	jet_system.jetR.u8_Cmd_MotorOpenDeg    = 0;
	jet_system.jetR.i16_Cmd_MotorGearDeg   = 0;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;
}



void idlingSpeedRoundCtrl( void )
{
	
	if (1 != IHC_rev_msg.mid_st.b1_St_MotorOn)
	{
		return ;
	}
	jet_system.jetL.u8_Cmd_MotorOpenDeg  = 0;
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = RUDDER_UP;

	jet_system.jetR.u8_Cmd_MotorOpenDeg  = 0;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = RUDDER_UP;
}



void jetGearFoward(void)
{
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
}

void jetGearBackward(void)
{
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_DOWN;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_DOWN;
}

void idlingRollLeft( void )
{
	
	if (1 != IHC_rev_msg.mid_st.b1_St_MotorOn)
	{
		return ;
	}
	jet_system.jetL.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].accelerator_L;
	jet_system.jetL.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].gear_L;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_L;

	jet_system.jetR.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].accelerator_R;
	jet_system.jetR.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].gear_R;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_R;
}

void idlingRollRight( void )
{

	if (1 != IHC_rev_msg.mid_st.b1_St_MotorOn)
	{
		return ;
	}
	jet_system.jetL.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].accelerator_L;
	jet_system.jetL.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].gear_L;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].rudder_L;

	jet_system.jetR.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].accelerator_R;
	jet_system.jetR.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].gear_R;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].rudder_R;
}

void idlingForward( void )
{


	jet_system.jetL.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].accelerator_L;
	jet_system.jetL.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].gear_L;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].rudder_L;

	jet_system.jetR.u8_Cmd_MotorOpenDeg    = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].accelerator_R;
	jet_system.jetR.i16_Cmd_MotorGearDeg   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].gear_R;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].rudder_R;

}
