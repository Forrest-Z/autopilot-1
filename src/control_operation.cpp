
/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_operation.h"
#include <util/easylogging++.h>
/******************************  Local Variable  *****************************/
JET_SYSTEM	jet_system;
ESTOP_POSITION eStop_position;
CTRL_CONFIG jet_config;
uint8	    log_b1_emergencyStop;	
uint8		log_b2_motorFireOn;		
uint8		log_b2_sailMode;		

/******************************  Extern Variable  ****************************/
char disconnectCfg[20];	
/******************************  Local Function   ****************************/

/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void initControlOperation(void)
{
	memset((uint8*)&jet_system,0,sizeof(JET_SYSTEM));
	//addTask(1,runControlOperation,(void *)0);
	eStop_position.double_lat=ins_msg.latitude;
	eStop_position.double_lng=ins_msg.longitude;
	log_b1_emergencyStop = 0;
	log_b2_motorFireOn = 0;
	log_b2_sailMode = 0;
}

void eStopControl(void)
{
	dspControlOFF();

	jet_system.jetL.b2_Cmd_MotorOnOff = 2;		
	jet_system.jetL.i16_Cmd_MotorGearDeg = 0;	
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;	
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;	

	jet_system.jetR.b2_Cmd_MotorOnOff = 2;		
	jet_system.jetR.i16_Cmd_MotorGearDeg = 0;	
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;	
	jet_system.jetR.u8_Cmd_MotorOpenDeg = 0;	

	command_signal.equpment_cmd.b1_elecWindlass1OnOff	= 0	;	   
	command_signal.equpment_cmd.b1_elecWindlass1UpDown	= 0	;	   
	command_signal.equpment_cmd.b1_elecWindlass2OnOff	= 0	;	   
	command_signal.equpment_cmd.b1_elecWindlass2UpDown	= 0	;	   
	command_signal.equpment_cmd.b1_sprayStrip1OnOff		= 0	;	   
	command_signal.equpment_cmd.b1_sprayStrip1UpDown	= 0	;	   
	command_signal.equpment_cmd.b1_sprayStrip2OnOff		= 0	;	   
	command_signal.equpment_cmd.b1_sprayStrip2UpDown	= 0	;	   
}

extern double start_lat;
extern double start_lng ;
void * runControlOperation(void*)
{

static int count_shark=0;
for(;;){

	log_b1_emergencyStop = jet_system.b1_cmd_emergencyStop;
	
	jet_system.b1_cmd_emergencyStop = calEmergency_Stop(); 

	// if(BD_AUTHORITY == command_signal.sail_mode_cmd.u8_authority)
	// {
	// 	BD_operation();
	// 	return;
	// }

	log_b2_motorFireOn = jet_system.jetL.b2_Cmd_MotorOnOff;
	if (usv_sign.succcessed_entry == STATS_ON){  //用于进坞后自主熄火

		jet_system.jetL.u8_Cmd_MotorOpenDeg = 255*0.2;
		jet_system.jetR.u8_Cmd_MotorOpenDeg = 255*0.2;

		jet_system.jetL.i16_Cmd_MotorGearDeg = 0;
		jet_system.jetR.i16_Cmd_MotorGearDeg = 0;

	}
	else if (usv_sign.out_docking == STATS_ON && usv_sign.succcessed_out == 0){ // 用于出坞时自主点火
		jet_system.jetL.b2_Cmd_MotorOnOff = 1; 
		jet_system.jetR.b2_Cmd_MotorOnOff = 1; 
	}
	else{ //非进出坞情况下点火熄火
		jet_system.jetL.b2_Cmd_MotorOnOff = command_signal.start_button_cmd.b2_motorOnOff;
		jet_system.jetR.b2_Cmd_MotorOnOff = command_signal.start_button_cmd.b2_motorOnOff;
	}
	
	if(log_b2_motorFireOn != jet_system.jetL.b2_Cmd_MotorOnOff)
	{
		switch(jet_system.jetL.b2_Cmd_MotorOnOff)
		{
		case 1:
				
			break;
		case 2: 
				
			break;
		default:
			break;
		}
	}

  if (usv_sign.out_docking == 1)//自主出坞状态
	{
		if (1 == dockOutUSVControl()){//出坞是否成功
			usv_sign.log_b1_dock_state = 2;//已出坞
		}
		else{
			usv_sign.log_b1_dock_state = 0;
		}
	}
	else if (command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_MANUAL || IHC_rev_msg.mid_st.b1_St_Authority == 1)		//�ֶ�ģʽ
	{
		if (jet_system.b1_cmd_emergencyStop == COMM_DISCONNECT_EMERGENCY)
		{
			if (strcmp(disconnectCfg, "stop") == 0)		
			{
				eStopControl();
			}
			else
			{
				if (pAutoReturnInst->IsAutoReturnNeeded()){		
					pAutoReturnInst->ControlAutoReturnMain();
				}
				else{
					//standby(eStop_position.double_lat, eStop_position.double_lng);
				idlingSpeedCtrl();
				}

			}
		}
		else
		{
			pAutoReturnInst->reset_AutoReturn();
			manual_operation();
		}
			
	}
	else if(command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_SEMIAUTO)	
	{
		if (jet_system.b1_cmd_emergencyStop == COMM_DISCONNECT_EMERGENCY)
		{
			if (strcmp(disconnectCfg, "stop") == 0)		
			{
				eStopControl();
			}
			else
			{
				if (pAutoReturnInst->IsAutoReturnNeeded()){		
					pAutoReturnInst->ControlAutoReturnMain();
				}
				else{
					standby(eStop_position.double_lat, eStop_position.double_lng);
				}
			}
		}
		else
		{
			pAutoReturnInst->reset_AutoReturn();
			manual_operation();		
			semiAuto_operation();
		}
	}
	else if(command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_AUTO)		
	{

		if (pAutoReturnInst->IsAutoReturnNeeded()){		
			pAutoReturnInst->ControlAutoReturnMain();
			command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
		}
		else
		{
			if (jet_system.b1_cmd_emergencyStop == COMM_DISCONNECT_EMERGENCY){
				if (strcmp(disconnectCfg, "stop") == 0)		
				{
					eStopControl();
				}
				else if (strcmp(disconnectCfg, "stay") == 0)	
				{
					standby(eStop_position.double_lat, eStop_position.double_lng);
				}
				else if (strcmp(disconnectCfg, "continue") == 0) 
				{
					auto_operation();
				}
				else
				{
					eStopControl();
				}
			}
			else{
				pAutoReturnInst->reset_AutoReturn();
				auto_operation();
			}
		}
		 
		
	}
	else if(command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_STANBY)	
	{
	
		stanby_operation();
	}

	if(log_b2_sailMode != command_signal.sail_mode_cmd.b2_sailMode)
	{
		switch(command_signal.sail_mode_cmd.b2_sailMode)
		{
		case SAIL_MODE_MANUAL:		
									
			break;
		case SAIL_MODE_SEMIAUTO:	
									
			break;
		case SAIL_MODE_AUTO:		
									
			break;
		case SAIL_MODE_STANBY:		
								
			break;
		default:
			break;
		}
	}

	log_b2_sailMode = command_signal.sail_mode_cmd.b2_sailMode;

	jetContrlModify();

	#ifdef OUTBOARD_SINGLE_ENGINE
		outBoardEngineModify();
	#endif

	check_mode();

	sleep_1(50);
	}
	LOG(ERROR) << "Exit auto control thread!";
	return ((void *)0);
}


uint8 calEmergency_Stop(void)
{
	uint8 iretEStop;
	uint8 eStopLogic = 0;	  
	uint8 eStopCmd	 = 0;	  
	uint8 eStopComm	 = 0;	  
	uint8 eStopDeviceErr = 0; 

	eStopCmd = command_signal.sail_mode_cmd.b1_emergencyStop;

	eStopLogic = (IHC_comm_sign.comm_sign == COMM_CONNECT_FAIL);

	if (command_signal.sail_mode_cmd.u8_authority == NO_AUTHORITY){
		eStopComm = 1;
	}	
	else{
		eStopComm = 0;
	}	

	if(eStopCmd || eStopLogic || eStopDeviceErr){
		iretEStop = ENGINE_STOP_EMERGENCY;
	}
	else if(eStopComm){
		iretEStop = COMM_DISCONNECT_EMERGENCY;
	}
	else{
		iretEStop = 0;
	}
	if(log_b1_emergencyStop == 0 && iretEStop >= 1)
	{
		if(eStopCmd)	   
		{
	
		}
		if(eStopLogic)	   
		{

		}
		if(eStopDeviceErr) 
		{

			if(IHC_rev_msg.b1_Wn_mainBoardPower	) 
			{

			}
			if(IHC_rev_msg.b1_Wn_mainBoardTempe	) 
			{

			}
			if(IHC_rev_msg.b1_Wn_SPI			) 
			{

			}
			if(IHC_rev_msg.b1_Wn_TL16C554_1Comm	) 
			{

			}
			if(IHC_rev_msg.b1_Wn_TL16C554_2Comm	)
			{

			}
			if(IHC_rev_msg.b1_Wn_TL16C554_3Comm	)
			{

			}
			if(IHC_rev_msg.b1_Wn_TL16C554_4Comm	) 
			{

			}
		}
		if(eStopComm)	   
		{
			eStop_position.double_lat = ins_msg.latitude;
			eStop_position.double_lng = ins_msg.longitude;
		}
	}
	if(log_b1_emergencyStop >= 1 && iretEStop == 0)
	{

	}
	//test start
	//printf("command_signal.sail_mode_cmd.i8_authority = %d\n",command_signal.sail_mode_cmd.i8_authority);
	//printf("iretEStop = %d\t eStopCmd= %d\t eStopLogic = %d\t eStopDeviceErr = %d\t eStopComm = %d\n",iretEStop,eStopCmd,eStopLogic,eStopDeviceErr,eStopComm);
	//test end
	return iretEStop;
}

void jetContrlModify( void )
{
	rudderModify();
	gearModify();
	MotorModify();
}

void gearModify( void )
{
	
}

void MotorModify( void )
{
	
}

void rudderModify( void )
{
	jet_system.jetL.i16_Cmd_MotorRudderDeg += jet_config.rudderConfig.i16_rudder1NaturlAngle; 
	jet_system.jetR.i16_Cmd_MotorRudderDeg += jet_config.rudderConfig.i16_rudder2NaturlAngle; 
	jet_system.jetL.i16_Cmd_MotorRudderDeg = i16UpDownLimit(RUDDER_UP,RUDDER_DOWN,jet_system.jetL.i16_Cmd_MotorRudderDeg);
	jet_system.jetR.i16_Cmd_MotorRudderDeg = i16UpDownLimit(RUDDER_UP,RUDDER_DOWN,jet_system.jetR.i16_Cmd_MotorRudderDeg);
}

void outBoardEngineModify( void )
{
	static int16 i16_MotorGearDeg = 0;
	if((i16_MotorGearDeg == GEAR_UP && jet_system.jetL.i16_Cmd_MotorGearDeg == GEAR_UP)||(i16_MotorGearDeg == GEAR_DOWN && jet_system.jetL.i16_Cmd_MotorGearDeg == GEAR_DOWN))
	{
		jet_system.jetL.u8_Cmd_MotorOpenDeg = jet_system.jetL.u8_Cmd_MotorOpenDeg;
	}
	else
	{
		jet_system.jetL.u8_Cmd_MotorOpenDeg  = 0;
	}
	i16_MotorGearDeg  = jet_system.jetL.i16_Cmd_MotorGearDeg;
}






