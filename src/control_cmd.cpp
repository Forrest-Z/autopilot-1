/*==========================================================*
 * ģ��˵��: control_cmd.cpp                                *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
 * ����ʱ��: 2018��2��8��14:54:03                           *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/

/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_cmd.h"
/******************************  Local Variable  *****************************/
COMMAND_SIGNAL command_signal;
uint8		   log_u8_authroity;
/******************************  Extern Variable  ****************************/

/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void initControlCmd(void)
{
	memset((uint8 *)&command_signal,0,sizeof(COMMAND_SIGNAL));
	log_u8_authroity = 255;
	addTask(1,runControlCmd,(void *)0);
}

void setNoneAuthority(void)
{
	command_signal.sail_mode_cmd.u8_authority = NO_AUTHORITY;
}

void runControlCmd(void *)
{
#if 0
	br_usv_cmd.u8_cmd_getAuthority = 1;
	if ((IOP_rev_msg.b1_Cmd_getAuthority == 1 && IOP_comm_sign.comm_sign == COMM_CONNECT_OK) || br_usv_cmd.u8_cmd_getAuthority == 2)
	{

		command_signal.b1_authority = 1;	//Ȩ���ڴ���
		getCmdFromIOP();

	}else{
		command_signal.b1_authority = 0;	//������Ȩ��



		if (dradio_sign.comm_sign == COMM_CONNECT_OK)
		{
			getCmdFromDR();
			removeBDTask();
		}
		else if (bradio_sign.comm_sign == COMM_CONNECT_OK)
		{
			getCmdFromBR();
			removeBDTask();
		}
		else if (brCc_sign.comm_sign == COMM_CONNECT_OK && br_usv_cmd.u8_cmd_getAuthority == 1)
		{
			getCmdFromCC();
			removeBDTask();
		}
		else if(bd_sailTask.u8_St_sailMsgRev == 1 )
		{
			getCmdFromBD();
		}
		else
		{
			setNoneAuthority();
		}	

		if (brCc_sign.comm_sign == COMM_CONNECT_OK && br_usv_cmd.u8_cmd_getAuthority == 1){

			getCmdFromCC();
			removeBDTask();
		}
		else{
			setNoneAuthority();
		}
	}
#endif
	if (brCc_sign.comm_sign == COMM_CONNECT_OK && br_usv_cmd.u8_cmd_getAuthority == 1){
		getCmdFromCC();
	}
	else{
		setNoneAuthority();
	}

	if(log_u8_authroity != command_signal.sail_mode_cmd.u8_authority)
	{
		switch(command_signal.sail_mode_cmd.u8_authority)
		{
		case BRCC_AUTHORITY:	

			break;
		case IOP_AUTHORITY:		
		
			break;
		case DIOP_AUTHORITY:	
		
			break;
		case BIOP_AUTHORITY:	
	
			break;
		case BD_AUTHORITY:		
		
			break;
		case NO_AUTHORITY:		
		
			break;
		default:
			break;
		}
	}
	log_u8_authroity = command_signal.sail_mode_cmd.u8_authority;

}

void getCmdFromIOP(void)
{
	command_signal.start_button_cmd.b2_motorOnOff     = IOP_rev_msg.b2_Cmd_motor;

	command_signal.sail_mode_cmd.b1_emergencyMode = IOP_rev_msg.b1_Cmd_emergencyMode;
	command_signal.sail_mode_cmd.b1_emergencyStop = IOP_rev_msg.b1_Cmd_emergencyStop;
	command_signal.sail_mode_cmd.u8_authority	  = IOP_AUTHORITY	;	
	command_signal.sail_mode_cmd.b2_sailMode	  = IOP_rev_msg.b2_Cmd_sailMode;
	command_signal.sail_mode_cmd.b2_sailTask	  = IOP_rev_msg.b2_Cmd_sailTask;

	command_signal.func_mode_cmd.b1_speedConstant   = IOP_rev_msg.b1_Cmd_speedConstant;
	command_signal.func_mode_cmd.b1_headingConstant = IOP_rev_msg.b1_Cmd_headingConstant;
	command_signal.func_mode_cmd.b1_setReturn		= IOP_rev_msg.b1_Cmd_setReturnPoint;
	command_signal.func_mode_cmd.b1_autoReturn		= IOP_rev_msg.b1_Cmd_autoReturn;
	command_signal.func_mode_cmd.b1_dock_cmd		= IOP_rev_msg.b1_Cmd_berthMode;

	command_signal.joystick_cmd.i16_X1	=	IOP_rev_msg.i16_Cmd_joy1X;
	command_signal.joystick_cmd.i16_Y1	=	IOP_rev_msg.i16_Cmd_joy1Y;
	command_signal.joystick_cmd.i16_Z1	=	IOP_rev_msg.i16_Cmd_joy1Z;
	command_signal.joystick_cmd.i16_X2	=	IOP_rev_msg.i16_Cmd_joy2X;
	command_signal.joystick_cmd.i16_Y2	=	IOP_rev_msg.i16_Cmd_joy2Y;
	command_signal.joystick_cmd.i16_Z2	=	IOP_rev_msg.i16_Cmd_joy2Z;


	command_signal.equpment_cmd.b1_elecWindlass1OnOff	 =   IOP_rev_msg.b1_Cmd_elecWindlass1OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass1UpDown	 =   IOP_rev_msg.b1_Cmd_elecWindlass1UpDown	;
	command_signal.equpment_cmd.b1_elecWindlass2OnOff	 =   IOP_rev_msg.b1_Cmd_elecWindlass2OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass2UpDown	 =   IOP_rev_msg.b1_Cmd_elecWindlass2UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip1OnOff		 =   IOP_rev_msg.b1_Cmd_sprayStrip1OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip1UpDown	 =   IOP_rev_msg.b1_Cmd_sprayStrip1UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip2OnOff		 =   IOP_rev_msg.b1_Cmd_sprayStrip2OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip2UpDown	 =   IOP_rev_msg.b1_Cmd_sprayStrip2UpDown	;

	command_signal.equpment_cmd.b1_SystemRestart = TEST_DEST_0;

	command_signal.equpment_cmd.b1_periPowK1  =  IOP_rev_msg.b1_Cmd_periPowerS1;
	command_signal.equpment_cmd.b1_periPowK2  =  IOP_rev_msg.b1_Cmd_periPowerS2;
	command_signal.equpment_cmd.b1_periPowK3  =  IOP_rev_msg.b1_Cmd_periPowerS3;
	command_signal.equpment_cmd.b1_periPowK4  =  IOP_rev_msg.b1_Cmd_periPowerS4;
	command_signal.equpment_cmd.b1_periPowK5  =  IOP_rev_msg.b1_Cmd_periPowerS5;
	command_signal.equpment_cmd.b1_periPowK6  =  IOP_rev_msg.b1_Cmd_periPowerS6;
	command_signal.equpment_cmd.b1_periPowK7  =  IOP_rev_msg.b1_Cmd_periPowerS7;
	command_signal.equpment_cmd.b1_periPowK8  =  IOP_rev_msg.b1_Cmd_periPowerS8;

	command_signal.equpment_cmd.b1_periPowK9   =  IOP_rev_msg.b1_Cmd_periPowerS9 ;
	command_signal.equpment_cmd.b1_periPowK10  =  IOP_rev_msg.b1_Cmd_periPowerS10;
	command_signal.equpment_cmd.b1_periPowK11  =  IOP_rev_msg.b1_Cmd_periPowerS11;
	command_signal.equpment_cmd.b1_periPowK12  =  IOP_rev_msg.b1_Cmd_periPowerS12;
	command_signal.equpment_cmd.b1_periPowK13  =  IOP_rev_msg.b1_Cmd_periPowerS13;
	command_signal.equpment_cmd.b1_periPowK14  =  IOP_rev_msg.b1_Cmd_periPowerS14;
	command_signal.equpment_cmd.b1_periPowK15  =  IOP_rev_msg.b1_Cmd_periPowerS15;
	command_signal.equpment_cmd.b1_periPowK16  =  IOP_rev_msg.b1_Cmd_periPowerS16;

	command_signal.equpment_cmd.b1_periPowK17  =  IOP_rev_msg.b1_Cmd_periPowerS17;
	command_signal.equpment_cmd.b1_periPowK18  =  IOP_rev_msg.b1_Cmd_periPowerS18;
	command_signal.equpment_cmd.b1_periPowK19  =  IOP_rev_msg.b1_Cmd_periPowerS19;
	command_signal.equpment_cmd.b1_periPowK20  =  IOP_rev_msg.b1_Cmd_periPowerS20;
}

void getCmdFromDR(void)
{
	//todo
	command_signal.start_button_cmd.b2_motorOnOff     = DrIOP_rev_msg.b2_Cmd_motor;

	command_signal.sail_mode_cmd.b1_emergencyMode = DrIOP_rev_msg.b1_Cmd_emergencyMode;
	command_signal.sail_mode_cmd.b1_emergencyStop = DrIOP_rev_msg.b1_Cmd_emergencyStop;
	command_signal.sail_mode_cmd.u8_authority	  = DIOP_AUTHORITY	;	
	command_signal.sail_mode_cmd.b2_sailMode	  = DrIOP_rev_msg.b2_Cmd_sailMode;
	command_signal.sail_mode_cmd.b2_sailTask	  = DrIOP_rev_msg.b2_Cmd_sailTask;

	command_signal.func_mode_cmd.b1_speedConstant   = DrIOP_rev_msg.b1_Cmd_speedConstant;
	command_signal.func_mode_cmd.b1_headingConstant = DrIOP_rev_msg.b1_Cmd_headingConstant;
	command_signal.func_mode_cmd.b1_setReturn		= 0;
	command_signal.func_mode_cmd.b1_autoReturn		= 0;
	command_signal.func_mode_cmd.b1_dock_cmd		= DrIOP_rev_msg.b1_Cmd_berthMode;

	command_signal.joystick_cmd.i16_X1	=	DrIOP_rev_msg.i16_Cmd_joy1X;
	command_signal.joystick_cmd.i16_Y1	=	DrIOP_rev_msg.i16_Cmd_joy1Y;
	command_signal.joystick_cmd.i16_Z1	=	DrIOP_rev_msg.i16_Cmd_joy1Z;
	command_signal.joystick_cmd.i16_X2	=	DrIOP_rev_msg.i16_Cmd_joy2X;
	command_signal.joystick_cmd.i16_Y2	=	DrIOP_rev_msg.i16_Cmd_joy2Y;
	command_signal.joystick_cmd.i16_Z2	=	DrIOP_rev_msg.i16_Cmd_joy2Z;


	command_signal.equpment_cmd.b1_elecWindlass1OnOff	 =   DrIOP_rev_msg.b1_Cmd_elecWindlass1OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass1UpDown	 =   DrIOP_rev_msg.b1_Cmd_elecWindlass1UpDown	;
	command_signal.equpment_cmd.b1_elecWindlass2OnOff	 =   DrIOP_rev_msg.b1_Cmd_elecWindlass2OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass2UpDown	 =   DrIOP_rev_msg.b1_Cmd_elecWindlass2UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip1OnOff		 =   DrIOP_rev_msg.b1_Cmd_sprayStrip1OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip1UpDown	 =   DrIOP_rev_msg.b1_Cmd_sprayStrip1UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip2OnOff		 =   DrIOP_rev_msg.b1_Cmd_sprayStrip2OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip2UpDown	 =   DrIOP_rev_msg.b1_Cmd_sprayStrip2UpDown	;

	command_signal.equpment_cmd.b1_SystemRestart = DrIOP_rev_msg.b1_Cmd_SystemRestart;

	command_signal.equpment_cmd.b1_periPowK1  =  DrIOP_rev_msg.b1_Cmd_periPowerS1;
	command_signal.equpment_cmd.b1_periPowK2  =  DrIOP_rev_msg.b1_Cmd_periPowerS2;
	command_signal.equpment_cmd.b1_periPowK3  =  DrIOP_rev_msg.b1_Cmd_periPowerS3;
	command_signal.equpment_cmd.b1_periPowK4  =  DrIOP_rev_msg.b1_Cmd_periPowerS4;
	command_signal.equpment_cmd.b1_periPowK5  =  DrIOP_rev_msg.b1_Cmd_periPowerS5;
	command_signal.equpment_cmd.b1_periPowK6  =  DrIOP_rev_msg.b1_Cmd_periPowerS6;
	command_signal.equpment_cmd.b1_periPowK7  =  DrIOP_rev_msg.b1_Cmd_periPowerS7;
	command_signal.equpment_cmd.b1_periPowK8  =  DrIOP_rev_msg.b1_Cmd_periPowerS8;

	command_signal.equpment_cmd.b1_periPowK9   =  DrIOP_rev_msg.b1_Cmd_periPowerS9 ;
	command_signal.equpment_cmd.b1_periPowK10  =  DrIOP_rev_msg.b1_Cmd_periPowerS10;
	command_signal.equpment_cmd.b1_periPowK11  =  DrIOP_rev_msg.b1_Cmd_periPowerS11;
	command_signal.equpment_cmd.b1_periPowK12  =  DrIOP_rev_msg.b1_Cmd_periPowerS12;
	command_signal.equpment_cmd.b1_periPowK13  =  DrIOP_rev_msg.b1_Cmd_periPowerS13;
	command_signal.equpment_cmd.b1_periPowK14  =  DrIOP_rev_msg.b1_Cmd_periPowerS14;
	command_signal.equpment_cmd.b1_periPowK15  =  DrIOP_rev_msg.b1_Cmd_periPowerS15;
	command_signal.equpment_cmd.b1_periPowK16  =  DrIOP_rev_msg.b1_Cmd_periPowerS16;

	command_signal.equpment_cmd.b1_periPowK17  =  DrIOP_rev_msg.b1_Cmd_periPowerS17;
	command_signal.equpment_cmd.b1_periPowK18  =  DrIOP_rev_msg.b1_Cmd_periPowerS18;
	command_signal.equpment_cmd.b1_periPowK19  =  DrIOP_rev_msg.b1_Cmd_periPowerS19;
	command_signal.equpment_cmd.b1_periPowK20  =  DrIOP_rev_msg.b1_Cmd_periPowerS20;
}

void getCmdFromBR( void )
{
	command_signal.start_button_cmd.b2_motorOnOff     = BrIOP_rev_msg.b2_Cmd_motor;

	command_signal.sail_mode_cmd.b1_emergencyMode = BrIOP_rev_msg.b1_Cmd_emergencyMode;
	command_signal.sail_mode_cmd.b1_emergencyStop = BrIOP_rev_msg.b1_Cmd_emergencyStop;
	command_signal.sail_mode_cmd.u8_authority	  = BIOP_AUTHORITY	;	
	command_signal.sail_mode_cmd.b2_sailMode	  = BrIOP_rev_msg.b2_Cmd_sailMode;
	command_signal.sail_mode_cmd.b2_sailTask	  = BrIOP_rev_msg.b2_Cmd_sailTask;

	command_signal.func_mode_cmd.b1_speedConstant   = BrIOP_rev_msg.b1_Cmd_speedConstant;
	command_signal.func_mode_cmd.b1_headingConstant = BrIOP_rev_msg.b1_Cmd_headingConstant;
	command_signal.func_mode_cmd.b1_setReturn		= 0;
	command_signal.func_mode_cmd.b1_autoReturn		= 0;
	command_signal.func_mode_cmd.b1_dock_cmd		= BrIOP_rev_msg.b1_Cmd_berthMode;

	command_signal.joystick_cmd.i16_X1	=	BrIOP_rev_msg.i16_Cmd_joy1X;
	command_signal.joystick_cmd.i16_Y1	=	BrIOP_rev_msg.i16_Cmd_joy1Y;
	command_signal.joystick_cmd.i16_Z1	=	BrIOP_rev_msg.i16_Cmd_joy1Z;
	command_signal.joystick_cmd.i16_X2	=	BrIOP_rev_msg.i16_Cmd_joy2X;
	command_signal.joystick_cmd.i16_Y2	=	BrIOP_rev_msg.i16_Cmd_joy2Y;
	command_signal.joystick_cmd.i16_Z2	=	BrIOP_rev_msg.i16_Cmd_joy2Z;


	command_signal.equpment_cmd.b1_elecWindlass1OnOff	 =   BrIOP_rev_msg.b1_Cmd_elecWindlass1OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass1UpDown	 =   BrIOP_rev_msg.b1_Cmd_elecWindlass1UpDown	;
	command_signal.equpment_cmd.b1_elecWindlass2OnOff	 =   BrIOP_rev_msg.b1_Cmd_elecWindlass2OnOff	;
	command_signal.equpment_cmd.b1_elecWindlass2UpDown	 =   BrIOP_rev_msg.b1_Cmd_elecWindlass2UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip1OnOff		 =   BrIOP_rev_msg.b1_Cmd_sprayStrip1OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip1UpDown	 =   BrIOP_rev_msg.b1_Cmd_sprayStrip1UpDown	;
	command_signal.equpment_cmd.b1_sprayStrip2OnOff		 =   BrIOP_rev_msg.b1_Cmd_sprayStrip2OnOff	;	
	command_signal.equpment_cmd.b1_sprayStrip2UpDown	 =   BrIOP_rev_msg.b1_Cmd_sprayStrip2UpDown	;

	command_signal.equpment_cmd.b1_SystemRestart = BrIOP_rev_msg.b1_Cmd_SystemRestart;

	command_signal.equpment_cmd.b1_periPowK1  =  BrIOP_rev_msg.b1_Cmd_periPowerS1;
	command_signal.equpment_cmd.b1_periPowK2  =  BrIOP_rev_msg.b1_Cmd_periPowerS2;
	command_signal.equpment_cmd.b1_periPowK3  =  BrIOP_rev_msg.b1_Cmd_periPowerS3;
	command_signal.equpment_cmd.b1_periPowK4  =  BrIOP_rev_msg.b1_Cmd_periPowerS4;
	command_signal.equpment_cmd.b1_periPowK5  =  BrIOP_rev_msg.b1_Cmd_periPowerS5;
	command_signal.equpment_cmd.b1_periPowK6  =  BrIOP_rev_msg.b1_Cmd_periPowerS6;
	command_signal.equpment_cmd.b1_periPowK7  =  BrIOP_rev_msg.b1_Cmd_periPowerS7;
	command_signal.equpment_cmd.b1_periPowK8  =  BrIOP_rev_msg.b1_Cmd_periPowerS8;

	command_signal.equpment_cmd.b1_periPowK9   =  BrIOP_rev_msg.b1_Cmd_periPowerS9 ;
	command_signal.equpment_cmd.b1_periPowK10  =  BrIOP_rev_msg.b1_Cmd_periPowerS10;
	command_signal.equpment_cmd.b1_periPowK11  =  BrIOP_rev_msg.b1_Cmd_periPowerS11;
	command_signal.equpment_cmd.b1_periPowK12  =  BrIOP_rev_msg.b1_Cmd_periPowerS12;
	command_signal.equpment_cmd.b1_periPowK13  =  BrIOP_rev_msg.b1_Cmd_periPowerS13;
	command_signal.equpment_cmd.b1_periPowK14  =  BrIOP_rev_msg.b1_Cmd_periPowerS14;
	command_signal.equpment_cmd.b1_periPowK15  =  BrIOP_rev_msg.b1_Cmd_periPowerS15;
	command_signal.equpment_cmd.b1_periPowK16  =  BrIOP_rev_msg.b1_Cmd_periPowerS16;

	command_signal.equpment_cmd.b1_periPowK17  =  BrIOP_rev_msg.b1_Cmd_periPowerS17;
	command_signal.equpment_cmd.b1_periPowK18  =  BrIOP_rev_msg.b1_Cmd_periPowerS18;
	command_signal.equpment_cmd.b1_periPowK19  =  BrIOP_rev_msg.b1_Cmd_periPowerS19;
	command_signal.equpment_cmd.b1_periPowK20  =  BrIOP_rev_msg.b1_Cmd_periPowerS20;
}


void getCmdFromCC(void)
{
	
	command_signal.start_button_cmd.b2_motorOnOff = br_usv_cmd.u8_cmd_moterOnOff;

	command_signal.sail_mode_cmd.b1_emergencyMode = 0;		//��Ӧ��ģʽ
	command_signal.sail_mode_cmd.b1_emergencyStop = br_usv_cmd.u8_cmd_emergencyStop;
	command_signal.sail_mode_cmd.u8_authority = BRCC_AUTHORITY;
	command_signal.sail_mode_cmd.b2_sailMode = br_usv_cmd.u8_cmd_sailMode;
	command_signal.sail_mode_cmd.b2_sailTask = br_usv_cmd.u8_cmd_sailSwitch;

	command_signal.func_mode_cmd.b1_speedConstant = br_usv_cmd.u8_cmd_speedConstant;
	command_signal.func_mode_cmd.b1_headingConstant = br_usv_cmd.u8_cmd_headingConstant;
	command_signal.func_mode_cmd.b1_setReturn = br_usv_cmd.u8_cmd_reserve5;
	command_signal.func_mode_cmd.b1_autoReturn = 0;
	command_signal.func_mode_cmd.b1_dock_cmd = br_usv_cmd.u8_cmd_return_dock;
	command_signal.semi_mode_cmd.speed_const_value = br_usv_cmd.u16_cmd_setSpeed / 10.0;
	command_signal.semi_mode_cmd.heading_const_value = br_usv_cmd.u16_cmd_setHeading / 10.0;

	command_signal.joystick_cmd.i16_X1 = br_usv_cmd.i8_cmd_joy1X;
	command_signal.joystick_cmd.i16_Y1 = br_usv_cmd.i8_cmd_joy1Y;
	command_signal.joystick_cmd.i16_Z1 = br_usv_cmd.i8_cmd_joy1X;
	command_signal.joystick_cmd.i16_X2 = br_usv_cmd.i8_Cmd_joy2X;
	command_signal.joystick_cmd.i16_Y2 = br_usv_cmd.i8_Cmd_joy2Y;
	command_signal.joystick_cmd.i16_Z2 = br_usv_cmd.i8_Cmd_joy2Z;


	command_signal.equpment_cmd.b1_elecWindlass1OnOff	= 		0;
	command_signal.equpment_cmd.b1_elecWindlass1UpDown	= 		0;
	command_signal.equpment_cmd.b1_elecWindlass2OnOff	= 		0;
	command_signal.equpment_cmd.b1_elecWindlass2UpDown	= 		0;
	command_signal.equpment_cmd.b1_sprayStrip1OnOff		= 		0;
	command_signal.equpment_cmd.b1_sprayStrip1UpDown	= 		0;
	command_signal.equpment_cmd.b1_sprayStrip2OnOff		= 		0;
	command_signal.equpment_cmd.b1_sprayStrip2UpDown	= 		0;

	command_signal.equpment_cmd.b1_SystemRestart = 0;

	command_signal.equpment_cmd.b1_periPowK1 = 	  0;
	command_signal.equpment_cmd.b1_periPowK2 = 	  0;
	command_signal.equpment_cmd.b1_periPowK3 = 	  0;
	command_signal.equpment_cmd.b1_periPowK4 = 	  0;
	command_signal.equpment_cmd.b1_periPowK5 = 	  0;
	command_signal.equpment_cmd.b1_periPowK6 = 	  0;
	command_signal.equpment_cmd.b1_periPowK7 = 	  0;
	command_signal.equpment_cmd.b1_periPowK8 = 	  0;

	command_signal.equpment_cmd.b1_periPowK9 = 	  0;
	command_signal.equpment_cmd.b1_periPowK10 =	  0;
	command_signal.equpment_cmd.b1_periPowK11 =	  0;
	command_signal.equpment_cmd.b1_periPowK12 =	  0;
	command_signal.equpment_cmd.b1_periPowK13 =	  0;
	command_signal.equpment_cmd.b1_periPowK14 =	  0;
	command_signal.equpment_cmd.b1_periPowK15 =	  0;
	command_signal.equpment_cmd.b1_periPowK16 =	  0;

	command_signal.equpment_cmd.b1_periPowK17 =  0;
	command_signal.equpment_cmd.b1_periPowK18 =  0;
	command_signal.equpment_cmd.b1_periPowK19 =  0;
	command_signal.equpment_cmd.b1_periPowK20 =  0;
}



void getCmdFromBD( void )
{
	command_signal.sail_mode_cmd.u8_authority	   =  BD_AUTHORITY	;
}






