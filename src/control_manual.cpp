/*==========================================================*
 * ģ��˵��: control_manual.cpp                             *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
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
#include "../include/control_manual.h"
/******************************  Local Variable  *****************************/
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void manual_combineOperation(void);	//���ŵ�λ����
void manual_separteOperation(void);	//���ŵ�λ�������
void manual_yamahaOperation(void);	//�켫�޷��ǿ���
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
#define GEAR_SP_MAXIMUN	  15
#define GEAR_SP_MINIMUN	 -15
#define GEAR_SP_COFF	  17
#define OPEN_DEG_COFF	  3.4	
//#define OPEN_DEG_COFF	  1.4	

void manual_operation( void )
{
	docking_control_cmd.cmd_state = 0;

	if(ship_version == TIANJI_VERSION)
		manual_yamahaOperation();
	else if(ship_version == WATER_QUALITY)
		manual_separteOperation();
}



void manual_combineOperation(void)
{
	
	jet_system.jetL.i16_Cmd_MotorGearDeg = command_signal.joystick_cmd.i16_Y1 *JOYSTICK_GEAR_COFF;
	jet_system.jetL.i16_Cmd_MotorGearDeg = i16UpDownLimit(GEAR_UP,GEAR_DOWN,jet_system.jetL.i16_Cmd_MotorGearDeg);
	jet_system.jetR.i16_Cmd_MotorGearDeg = jet_system.jetL.i16_Cmd_MotorGearDeg;


	jet_system.jetL.i16_Cmd_MotorRudderDeg = command_signal.joystick_cmd.i16_X2 * JOYSTICK_RUDDER_COFF;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16UpDownLimit(RUDDER_UP,RUDDER_DOWN,jet_system.jetL.i16_Cmd_MotorRudderDeg);
	jet_system.jetR.i16_Cmd_MotorRudderDeg = jet_system.jetL.i16_Cmd_MotorRudderDeg;


	jet_system.jetL.u8_Cmd_MotorOpenDeg = (uint8)(abs(command_signal.joystick_cmd.i16_Y1) * JOYSTICK_MOTOR_OPENDEG_COFF);
	jet_system.jetL.u8_Cmd_MotorOpenDeg = u8UpLimit(OPEN_DEG_MAX,jet_system.jetL.u8_Cmd_MotorOpenDeg);
	jet_system.jetR.u8_Cmd_MotorOpenDeg = jet_system.jetL.u8_Cmd_MotorOpenDeg;
}

void manual_separteOperation( void )
{
	int16	i16Y = command_signal.joystick_cmd.i16_Y1;
	int16	i16X = command_signal.joystick_cmd.i16_X2;


	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16X * JOYSTICK_RUDDER_COFF;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16UpDownLimit(RUDDER_UP,RUDDER_DOWN,jet_system.jetL.i16_Cmd_MotorRudderDeg);
	jet_system.jetR.i16_Cmd_MotorRudderDeg = jet_system.jetL.i16_Cmd_MotorRudderDeg;

	if(i16Y<GEAR_SP_MAXIMUN && i16Y>GEAR_SP_MINIMUN)	
	{
		jet_system.jetL.i16_Cmd_MotorGearDeg = i16Y * GEAR_SP_COFF ;	
		jet_system.jetR.i16_Cmd_MotorGearDeg = jet_system.jetL.i16_Cmd_MotorGearDeg;
		jet_system.jetL.u8_Cmd_MotorOpenDeg  = 0;		
		jet_system.jetR.u8_Cmd_MotorOpenDeg  = 0;	
	}
	else	
	{
		if(i16Y > 0)
		{
			jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
			jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
		}
		else
		{
			jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_DOWN;
			jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_DOWN;
		}
		jet_system.jetL.u8_Cmd_MotorOpenDeg = (uint8)i16UpDownLimit(OPEN_DEG_MAX,0,((abs(i16Y) - GEAR_SP_MAXIMUN)*OPEN_DEG_COFF));
		jet_system.jetR.u8_Cmd_MotorOpenDeg = jet_system.jetL.u8_Cmd_MotorOpenDeg;
	}
}

void manual_yamahaOperation( void )
{
	int16	i16Y = command_signal.joystick_cmd.i16_Y1;
	int16	i16X = command_signal.joystick_cmd.i16_X2;


	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16X * JOYSTICK_RUDDER_COFF;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16UpDownLimit(RUDDER_UP,RUDDER_DOWN,jet_system.jetL.i16_Cmd_MotorRudderDeg);
	jet_system.jetR.i16_Cmd_MotorRudderDeg = jet_system.jetL.i16_Cmd_MotorRudderDeg;


	if(i16Y<GEAR_SP_MAXIMUN && i16Y>GEAR_SP_MINIMUN)	
	{
		jet_system.jetL.i16_Cmd_MotorGearDeg = i16Y * GEAR_SP_COFF ;	
		jet_system.jetR.i16_Cmd_MotorGearDeg = jet_system.jetL.i16_Cmd_MotorGearDeg;
		jet_system.jetL.u8_Cmd_MotorOpenDeg  = 0;		
		jet_system.jetR.u8_Cmd_MotorOpenDeg  = 0;	
	}
	else	
	{
		if(i16Y > 0)
		{
			jet_system.jetL.u8_Cmd_MotorOpenDeg = (uint8)i16UpDownLimit(OPEN_DEG_MAX,0,((abs(i16Y) - GEAR_SP_MAXIMUN)*OPEN_DEG_COFF));
			jet_system.jetR.u8_Cmd_MotorOpenDeg = jet_system.jetL.u8_Cmd_MotorOpenDeg;
		}
		else
		{
			jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;
			jet_system.jetR.u8_Cmd_MotorOpenDeg = jet_system.jetL.u8_Cmd_MotorOpenDeg;
		}

	}
}
