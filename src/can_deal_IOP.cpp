/*==========================================================*
 * 模块说明: can_deal_IOP.cpp                               *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员:                                                *
 * 创建时间:                                                *
 * Copyright(c) sf-auto.ltd                                 *
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/can_deal_IOP.h"
#include "../include/usv_include.h"
/******************************  Local Variable  *****************************/
IOP_REV_MSG		IOP_rev_msg		;
IOP_REV_CONFIG	IOP_rev_config	;
IOP_SEND_MSG	IOP_send_msg	;
COMM_SIGN	IOP_comm_sign = {0,0};
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void IOP_rvMsg_Init(void)
{
	memset(&IOP_rev_msg,0,sizeof(IOP_REV_MSG));
}

void IOP_config_Init(void)
{
	memset(&IOP_rev_config,0,sizeof(IOP_REV_CONFIG));
}

void IOP_sdMsg_Init(void)
{
	memset(&IOP_send_msg,0,sizeof(IOP_SEND_MSG));
	IOP_send_msg.b1_Wn_IOPCommOutage = &IOP_comm_sign.comm_sign;
	IOP_send_msg.b1_Wn_IOPWarn = &IOP_rev_msg.b1_Wn_warn;
	IOP_send_msg.b1_Wn_IHCCommOutage = &IHC_comm_sign.comm_sign;
	IOP_send_msg.b1_Wn_IHCWarn = &IHC_rev_msg.mid_st.b1_Wn_CommonWarn;
	IOP_send_msg.b1_Wn_IDUCommOutage = &IDU_comm_sign.comm_sign;
	IOP_send_msg.b1_Wn_IDUWarn = &IDU_rev_msg.b1_Wn_warn;
	IOP_send_msg.b1_Wn_MCUCommOutage = (uint8*)&TEST_DEST_0;
	IOP_send_msg.b1_Wn_MCUWarn = (uint8*)&TEST_DEST_0;

	IOP_send_msg.b1_Wn_radio1 = &dradio_sign.comm_sign;
	IOP_send_msg.b1_Wn_radio2 = &bradio_sign.comm_sign;
	IOP_send_msg.b1_Wn_BD = (uint8*)&TEST_DEST_0;

	IOP_send_msg.u8_St_year = &state_signal.time.u8_year;///&ins_msg.u8_year;
	IOP_send_msg.u8_St_month = &state_signal.time.u8_month;//&ins_msg.u8_month;
	IOP_send_msg.u8_St_date = &state_signal.time.u8_date;//&ins_msg.u8_day;

	IOP_send_msg.u8_St_hour = &state_signal.time.u8_hour;//&ins_msg.u8_hour;
	IOP_send_msg.u8_St_minute = &state_signal.time.u8_minute;//&ins_msg.u8_minute;
	IOP_send_msg.u8_St_second = &state_signal.time.u8_second;//&ins_msg.u8_second;

	IOP_send_msg.u16_St_speed = &ins_msg.u16_speed;
	IOP_send_msg.u16_St_heading = &ins_msg.u16_heading;


	IOP_send_msg.i16_St_rot = &ins_msg.i16_rot;
	IOP_send_msg.i16_St_pitch = &ins_msg.i16_pitch;
	IOP_send_msg.i16_St_roll = &ins_msg.i16_roll;
	IOP_send_msg.i16_St_heaving = &ins_msg.i16_heaving;

	IOP_send_msg.u8_St_longiSt = &ins_msg.u8_longiSt;
	IOP_send_msg.u8_St_longiDeg = &ins_msg.u8_longiDeg;
	IOP_send_msg.u8_St_longiMin = &ins_msg.u8_longiMin;
	IOP_send_msg.u8_St_longiSec = &ins_msg.u8_longiSec;
	IOP_send_msg.u8_St_longiSecDec = &ins_msg.u8_longiSecDec;

	IOP_send_msg.u8_St_latiSt = &ins_msg.u8_latiSt;
	IOP_send_msg.u8_St_latiDeg = &ins_msg.u8_latiDeg;
	IOP_send_msg.u8_St_latiMin = &ins_msg.u8_latiMin;
	IOP_send_msg.u8_St_latiSec = &ins_msg.u8_latiSec;
	IOP_send_msg.u8_St_latiSecDec = &ins_msg.u8_latiSecDec;

	IOP_send_msg.b1_St_emergencyMode = &command_signal.sail_mode_cmd.b1_emergencyMode;
	IOP_send_msg.b1_St_emergencyStop = &jet_system.b1_cmd_emergencyStop;
	IOP_send_msg.b1_St_localAuthority = &command_signal.b1_authority;
	IOP_send_msg.b2_St_sailMode = &command_signal.sail_mode_cmd.b2_sailMode;
	IOP_send_msg.b2_St_sailTask = &command_signal.sail_feedBack.b2_sailTask;

	IOP_send_msg.b1_St_speedConstant = &command_signal.func_mode_cmd.b1_speedConstant;
	IOP_send_msg.b1_St_headingConstant = &command_signal.func_mode_cmd.b1_headingConstant;
	IOP_send_msg.b1_St_berthMode = &command_signal.func_mode_cmd.b1_dock_cmd;
	IOP_send_msg.b1_St_autoReturn = &command_signal.func_mode_cmd.b1_autoReturn;

	IOP_send_msg.b1_St_motorSt = (uint8*)&IHC_rev_msg.mid_st.b1_St_MotorOn;
	IOP_send_msg.b1_St_motor1Wn = (uint8*)&IHC_rev_msg.b1_Wn_PORTMotorWarn;
	IOP_send_msg.b1_St_motor2Wn = (uint8*)&IHC_rev_msg.b1_Wn_STBDMotorWarn;


	IOP_send_msg.u16_St_motor1Rpm = &IHC_rev_msg.u16_St_Motor1Rpm;	//需要转换
	IOP_send_msg.i16_St_motor1Gear = &IHC_rev_msg.i16_St_Motor1Gear;
	IOP_send_msg.i16_St_motor1Rudder = &IHC_rev_msg.i16_St_Motor1Rudder;

	IOP_send_msg.u16_St_motor2Rpm = &IHC_rev_msg.u16_St_Motor2Rpm;	//需要转换
	IOP_send_msg.i16_St_motor2Gear = &IHC_rev_msg.i16_St_Motor2Gear;
	IOP_send_msg.i16_St_motor2Rudder = &IHC_rev_msg.i16_St_Motor2Rudder;

	IOP_send_msg.u16_St_motor3Rpm = (uint16*)&TEST_DEST_0;	//需要转换
	IOP_send_msg.i16_St_motor3Gear = (int16*)&TEST_DEST_0;
	IOP_send_msg.i16_St_motor3Rudder = (int16*)&TEST_DEST_0;

	IOP_send_msg.u16_St_motor4Rpm = (uint16*)&TEST_DEST_0;	//需要转换
	IOP_send_msg.i16_St_motor4Gear = (int16*)&TEST_DEST_0;
	IOP_send_msg.i16_St_motor4Rudder = (int16*)&TEST_DEST_0;

	IOP_send_msg.b1_St_remoteKey = &IDU_rev_msg.b1_St_remoteKey;
	IOP_send_msg.b1_St_PORTMotorCharge = &IDU_rev_msg.b1_St_PORTMotorCharge;
	IOP_send_msg.b1_St_STBDMotorCharge = &IDU_rev_msg.b1_St_STBDMotorCharge;
	IOP_send_msg.b1_St_shorePower = &IDU_rev_msg.b1_St_shorePower;
	IOP_send_msg.b1_St_PORTShoreCharge = &IDU_rev_msg.b1_St_PORTShoreCharge;
	IOP_send_msg.b1_St_STBDShoreCharge = &IDU_rev_msg.b1_St_STBDShoreCharge;
	IOP_send_msg.b1_St_ShoreChargeEnd = &IDU_rev_msg.b1_St_ShoreChargeEnd;
	IOP_send_msg.b1_St_systemPowerOn = &IDU_rev_msg.b1_St_SystemPowerOn;

	IOP_send_msg.b3_St_supplySource = &IDU_rev_msg.b3_St_supplySource;
	IOP_send_msg.b1_Wn_batLow = &IDU_rev_msg.b1_Wn_batLow;
	IOP_send_msg.b1_Wn_oilLow = &IDU_rev_msg.b1_Wn_oilLow;
	IOP_send_msg.b1_Wn_volOver = &IDU_rev_msg.midMsg.b1_Wn_volOver;
	IOP_send_msg.b1_Wn_volBelow = &IDU_rev_msg.midMsg.b1_Wn_volBelow;
	IOP_send_msg.b1_Wn_curOver = &IDU_rev_msg.midMsg.b1_Wn_curOver;

	IOP_send_msg.u8_St_PORTBatLvl = &IDU_rev_msg.u8_St_PORTBatLvl;
	IOP_send_msg.u8_St_STBDBatLvl = &IDU_rev_msg.u8_St_STBDBatLvl;
	IOP_send_msg.u8_St_PORTOilLvl = &IDU_rev_msg.u8_St_PortOilLvl;
	IOP_send_msg.u8_St_STBDOilLvl = &IDU_rev_msg.u8_St_STBDOilLvl;


	IOP_send_msg.b1_St_periPowK1 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[0]);
	IOP_send_msg.b1_St_periPowK2 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[1]);
	IOP_send_msg.b1_St_periPowK3 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[2]);
	IOP_send_msg.b1_St_periPowK4 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[3]);
	IOP_send_msg.b1_St_periPowK5 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[4]);
	IOP_send_msg.b1_St_periPowK6 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[5]);
	IOP_send_msg.b1_St_periPowK7 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[6]);
	IOP_send_msg.b1_St_periPowK8 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[7]);

	IOP_send_msg.b1_St_periPowk9 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[8]);
	IOP_send_msg.b1_St_periPowk10 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[9]);
	IOP_send_msg.b1_St_periPowk11 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[10]);
	IOP_send_msg.b1_St_periPowk12 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[11]);
	IOP_send_msg.b1_St_periPowk13 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[12]);
	IOP_send_msg.b1_St_periPowk14 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[13]);
	IOP_send_msg.b1_St_periPowk15 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[14]);
	IOP_send_msg.b1_St_periPowk16 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[15]);

	IOP_send_msg.b1_St_periPowk17 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[16]);
	IOP_send_msg.b1_St_periPowk18 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[17]);
	IOP_send_msg.b1_St_periPowk19 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[18]);
	IOP_send_msg.b1_St_periPowk20 = DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[19]);

	IOP_send_msg.b1_St_elecWindlass1St = &IHC_rev_msg.b1_St_elecWindlass1OnOff;
	IOP_send_msg.b1_St_elecWindlass2St = &IHC_rev_msg.b1_St_elecWindlass2OnOff;
	IOP_send_msg.b1_St_sprayStrip1St = &IHC_rev_msg.b1_St_sprayStrip1OnOff;
	IOP_send_msg.b1_St_sprayStrip2St = &IHC_rev_msg.b1_St_sprayStrip2OnOff;


}

void IOP_Init(void)
{
	IOP_rvMsg_Init();
	IOP_config_Init();
	IOP_sdMsg_Init();
	comm_time_return(&(IOP_comm_sign.timer),&(IOP_comm_sign.comm_sign));
}

//统计通讯超时
void IOP_CommCal(void)
{
	comm_time_cal(CAN_IOP_DISCONNECT_MAX,&(IOP_comm_sign.timer),&(IOP_comm_sign.comm_sign));
}

//IOP接收
void IOP_recv(uint8 psID,uint8* data)
{
	if(0 == psID)
	{
		IOP_rev_msg.b1_Wn_warn				=      (data[0]&0x01)        ;
		IOP_rev_msg.b1_Wn_mainBoardPower	=      (data[0]&0x02) >> 1   ;
		IOP_rev_msg.b1_Wn_mainBoardTempe	=      (data[0]&0x04) >> 2   ;
		IOP_rev_msg.b1_Wn_int				=      (data[0]&0x08) >> 3   ;
		IOP_rev_msg.b1_Wn_CANComm			=      (data[0]&0x10) >> 4   ;
		IOP_rev_msg.b1_Wn_SCIA				=      (data[0]&0x20) >> 5   ;
		IOP_rev_msg.b1_Wn_SCIB				=      (data[0]&0x40) >> 6   ;

		IOP_rev_msg.b1_Wn_externSP1         =      (data[1]&0x01)        ;
		IOP_rev_msg.b1_Wn_externSP2         =      (data[1]&0x02) >> 1   ;
		IOP_rev_msg.b1_Wn_externSP3         =      (data[1]&0x04) >> 2   ;
		IOP_rev_msg.b1_Wn_externSP4         =      (data[1]&0x08) >> 3   ;
		IOP_rev_msg.b1_Wn_externSP5         =      (data[1]&0x10) >> 4   ;
		IOP_rev_msg.b1_Wn_externSP6         =      (data[1]&0x20) >> 5   ;
		IOP_rev_msg.b1_Wn_externSP7         =      (data[1]&0x40) >> 6   ;
		IOP_rev_msg.b1_Wn_externSP8         =      (data[1]&0xF0) >> 7   ;

		IOP_rev_msg.b2_Cmd_motor			=	   (data[2]&0x03)		 ;
		IOP_rev_msg.b1_Cmd_systemReset		=	   (data[2]&0x80)>>7	 ;

		IOP_rev_msg.b1_Cmd_emergencyMode         =      (data[3]&0x01)        ;
		IOP_rev_msg.b1_Cmd_emergencyStop         =      (data[3]&0x02) >> 1   ;
		IOP_rev_msg.b1_Cmd_getAuthority	         =      (data[3]&0x04) >> 2   ;
		IOP_rev_msg.b2_Cmd_sailMode		         =      (data[3]&0x30) >> 4   ;
		IOP_rev_msg.b2_Cmd_sailTask		         =      (data[3]&0xc0) >> 6   ;

		IOP_rev_msg.b1_Cmd_speedConstant         =      (data[4]&0x01)        ;
		IOP_rev_msg.b1_Cmd_headingConstant       =      (data[4]&0x02) >> 1   ;
		IOP_rev_msg.b1_Cmd_berthMode 	         =      (data[4]&0x04) >> 2   ;
		IOP_rev_msg.b1_Cmd_setReturnPoint        =      (data[4]&0x08) >> 3   ;
		IOP_rev_msg.b1_Cmd_autoReturn            =      (data[4]&0x10) >> 4   ;

	}
	else if(1 == psID)
	{
		IOP_rev_msg.i16_Cmd_joy1X   = u8toi16(&(data[0]));
		IOP_rev_msg.i16_Cmd_joy1Y   = u8toi16(&(data[2]));
		IOP_rev_msg.i16_Cmd_joy1Z   = u8toi16(&(data[4]));
	}
	else if(2 == psID)
	{
		IOP_rev_msg.i16_Cmd_joy2X   = u8toi16(&(data[0]));
		IOP_rev_msg.i16_Cmd_joy2Y   = u8toi16(&(data[2]));
		IOP_rev_msg.i16_Cmd_joy2Z   = u8toi16(&(data[4]));
	}
	else if(3 == psID)
	{
		IOP_rev_msg.b1_Cmd_periPowerS1 =  (data[0]&0x01)      ;
		IOP_rev_msg.b1_Cmd_periPowerS2 =  (data[0]&0x02) >> 1 ;
		IOP_rev_msg.b1_Cmd_periPowerS3 =  (data[0]&0x04) >> 2 ;
		IOP_rev_msg.b1_Cmd_periPowerS4 =  (data[0]&0x08) >> 3 ;
		IOP_rev_msg.b1_Cmd_periPowerS5 =  (data[0]&0x10) >> 4 ;
		IOP_rev_msg.b1_Cmd_periPowerS6 =  (data[0]&0x20) >> 5 ;
		IOP_rev_msg.b1_Cmd_periPowerS7 =  (data[0]&0x40) >> 6 ;
		IOP_rev_msg.b1_Cmd_periPowerS8 =  (data[0]&0xF0) >> 7 ;

		IOP_rev_msg.b1_Cmd_periPowerS9  =  (data[1]&0x01)      ;
		IOP_rev_msg.b1_Cmd_periPowerS10 =  (data[1]&0x02) >> 1 ;
		IOP_rev_msg.b1_Cmd_periPowerS11 =  (data[1]&0x04) >> 2 ;
		IOP_rev_msg.b1_Cmd_periPowerS12 =  (data[1]&0x08) >> 3 ;
		IOP_rev_msg.b1_Cmd_periPowerS13 =  (data[1]&0x10) >> 4 ;
		IOP_rev_msg.b1_Cmd_periPowerS14 =  (data[1]&0x20) >> 5 ;
		IOP_rev_msg.b1_Cmd_periPowerS15 =  (data[1]&0x40) >> 6 ;
		IOP_rev_msg.b1_Cmd_periPowerS16 =  (data[1]&0xF0) >> 7 ;

		IOP_rev_msg.b1_Cmd_periPowerS17 =  (data[2]&0x01)      ; 
		IOP_rev_msg.b1_Cmd_periPowerS18 =  (data[2]&0x02) >> 1 ; 
		IOP_rev_msg.b1_Cmd_periPowerS19 =  (data[2]&0x04) >> 2 ; 
		IOP_rev_msg.b1_Cmd_periPowerS20 =  (data[2]&0x08) >> 3 ; 

		IOP_rev_msg.b1_Cmd_elecWindlass1OnOff	=  (data[3]&0x01)      ;
		IOP_rev_msg.b1_Cmd_elecWindlass1UpDown	=  (data[3]&0x02) >> 1 ;
		IOP_rev_msg.b1_Cmd_elecWindlass2OnOff	=  (data[3]&0x04) >> 2 ;
		IOP_rev_msg.b1_Cmd_elecWindlass2UpDown	=  (data[3]&0x08) >> 3 ;
		IOP_rev_msg.b1_Cmd_sprayStrip1OnOff		=  (data[3]&0x10) >> 4 ;
		IOP_rev_msg.b1_Cmd_sprayStrip1UpDown	=  (data[3]&0x20) >> 5 ;
		IOP_rev_msg.b1_Cmd_sprayStrip2OnOff		=  (data[3]&0x40) >> 6 ;
		IOP_rev_msg.b1_Cmd_sprayStrip2UpDown	=  (data[3]&0xF0) >> 7 ;
	}

	else if(128 == psID)
	{
		IOP_rev_config.u16_version	=	u8tou16(&(data[0]));
		IOP_rev_config.u16_svn		=	u8tou16(&(data[2]));
		IOP_rev_config.u16_crc		=	u8tou16(&(data[4]));
	}
	else if(129 == psID)
	{
		IOP_rev_config.u8_confState = (data[0]&0x01)		;
		IOP_rev_config.u8_initState = (data[1]&0x02)>>1		;
	}
	else
	{
		return;
	}
	comm_time_return(&(IOP_comm_sign.timer),&(IOP_comm_sign.comm_sign));
}


void IOP_sendMsg(void)
{
	uint8 pData[8];
	#if 0
	//PS31
	memset(pData,0,8);
	pData[0] = (*IOP_send_msg.b1_Wn_IOPCommOutage	&0x01  )	;
	pData[0]|= (*IOP_send_msg.b1_Wn_IOPWarn			&0x01  )<<1	;
	pData[0]|= (*IOP_send_msg.b1_Wn_IHCCommOutage	&0x01  )<<2	;
	pData[0]|= (*IOP_send_msg.b1_Wn_IHCWarn			&0x01  )<<3	;
	pData[0]|= (*IOP_send_msg.b1_Wn_IDUCommOutage	&0x01  )<<4	;
	pData[0]|= (*IOP_send_msg.b1_Wn_IDUWarn			&0x01  )<<5	;
	pData[0]|= (*IOP_send_msg.b1_Wn_MCUCommOutage	&0x01  )<<6	;
	pData[0]|= (*IOP_send_msg.b1_Wn_MCUWarn			&0x01  )<<7	;

	pData[1] = (*IOP_send_msg.b1_Wn_radio1 & 0x01)		;
	pData[1]|= (*IOP_send_msg.b1_Wn_radio2 & 0x01) <<1	;
	pData[1]|= (*IOP_send_msg.b1_Wn_BD	  & 0x01) <<2	;
	sendCanMsg(can_0,255,31,128,pData);

	//PS32
	memset(pData,0,8);
	pData[0] = *IOP_send_msg.u8_St_year	   ;
	pData[1] = *IOP_send_msg.u8_St_month	   ;
	pData[2] = *IOP_send_msg.u8_St_date	   ;
	pData[3] = *IOP_send_msg.u8_St_hour	   ;
	pData[4] = *IOP_send_msg.u8_St_minute	   ;
	pData[5] = *IOP_send_msg.u8_St_second	   ;
	sendCanMsg(can_0,255,32,128,pData);
#endif

	//PS33
	memset(pData,0,8);
	//*IOP_send_msg.u16_St_speed = 100;
	//*IOP_send_msg.u16_St_heading = 200;
	memcpy(&(pData[0]),(uint8*)(IOP_send_msg.u16_St_speed),2);
	memcpy(&(pData[2]),(uint8*)(IOP_send_msg.u16_St_heading),2);

  #if 0
	//add @foo 为了给IHC做旋转测试
	int16 vx, vy;
	vx = (cos(Radian(ins_msg.motionDirection - ins_msg.heading))*ins_msg.speed)*10;
	vy = (sin(Radian(ins_msg.motionDirection - ins_msg.heading))*ins_msg.speed)*10;
	//printf("vx == %d, vy == %d \n",vx , vy);
	//vx = -10;
	//vy = 10;
	memcpy(&(pData[4]), (uint8*)(&vx), 2);
	memcpy(&(pData[6]), (uint8*)(&vy), 2);
	#endif
	sendCanMsg(can_0,255,33,128,pData);

	//PS34
	memset(pData,0,8);
	memcpy(&(pData[0]),(uint8*)(IOP_send_msg.i16_St_rot),2);
	memcpy(&(pData[2]),(uint8*)(IOP_send_msg.i16_St_pitch),2);
	memcpy(&(pData[4]),(uint8*)(IOP_send_msg.i16_St_roll),2);
	memcpy(&(pData[6]),(uint8*)(IOP_send_msg.i16_St_heaving),2);
	sendCanMsg(can_0,255,34,128,pData);

	//PS35
	memset(pData,0,8);
	pData[0] = *IOP_send_msg.u8_St_longiSt		;
	pData[1] = *IOP_send_msg.u8_St_longiDeg		;
	pData[2] = *IOP_send_msg.u8_St_longiMin		;
	pData[3] = *IOP_send_msg.u8_St_longiSec		;
	pData[4] = *IOP_send_msg.u8_St_longiSecDec	;
	sendCanMsg(can_0,255,35,128,pData);

	//PS36
	memset(pData,0,8);
	pData[0] = *IOP_send_msg.u8_St_latiSt		;
	pData[1] = *IOP_send_msg.u8_St_latiDeg		;
	pData[2] = *IOP_send_msg.u8_St_latiMin		;
	pData[3] = *IOP_send_msg.u8_St_latiSec		;
	pData[4] = *IOP_send_msg.u8_St_latiSecDec	;
	sendCanMsg(can_0,255,36,128,pData);

#if 0
	//PS37
	memset(pData,0,8);
	pData[0]  = (*IOP_send_msg.b1_St_emergencyMode  & 0x01 )		;
	pData[0] |= (*IOP_send_msg.b1_St_emergencyStop  & 0x01 )<<1	;
	pData[0] |= (*IOP_send_msg.b1_St_localAuthority & 0x01 )<<2	;
	pData[0] |= (*IOP_send_msg.b2_St_sailMode       & 0x03 )<<4	;
	pData[0] |= (*IOP_send_msg.b2_St_sailTask       & 0x03 )<<6	;

	pData[1]  =  (*IOP_send_msg.b1_St_speedConstant     &0x01)       ;
	pData[1] |=  (*IOP_send_msg.b1_St_headingConstant   &0x01) << 1  ;
	pData[1] |=  (*IOP_send_msg.b1_St_berthMode         &0x01) << 2  ;
	//pData[1] |=  (*IOP_send_msg.b1_St_setReturn         &0x01) << 3  ;
	pData[1] |=  (*IOP_send_msg.b1_St_autoReturn        &0x01) << 4  ;
	

	pData[2]  =  (*IOP_send_msg.b1_St_motorSt   &0x01)			;
	pData[2] |=  (*IOP_send_msg.b1_St_motor1Wn  &0x01) << 1		;
	pData[2] |=  (*IOP_send_msg.b1_St_motor2Wn  &0x01) << 2		;
	sendCanMsg(can_0,255,37,128,pData);

	//PS38
	memset(pData,0,8);
	memcpy(&(pData[0]),(uint8*)(IOP_send_msg.u16_St_motor1Rpm),2);
	memcpy(&(pData[2]),(uint8*)(IOP_send_msg.i16_St_motor1Gear),2);
	memcpy(&(pData[4]),(uint8*)(IOP_send_msg.i16_St_motor1Rudder),2);
	sendCanMsg(can_0,255,38,128,pData);
	
	//PS39
	memset(pData,0,8);
	memcpy(&(pData[0]),(uint8*)(IOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[2]),(uint8*)(IOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[4]),(uint8*)(IOP_send_msg.i16_St_motor2Rudder),2);
	sendCanMsg(can_0,255,39,128,pData);

	//PS40
	//memset(pData,0,8);
	//memcpy(&(pData[0]),(uint8*)&(IOP_send_msg.u16_St_motor3Rpm),2);
	//memcpy(&(pData[2]),(uint8*)&(IOP_send_msg.i16_St_motor3Gear),2);
	//memcpy(&(pData[4]),(uint8*)&(IOP_send_msg.i16_St_motor3Rudder),2);
	//sendCanMsg(can_0,255,40,128,pData);

	////PS41
	//memset(pData,0,8);
	//memcpy(&(pData[0]),(uint8*)&(IOP_send_msg.u16_St_motor4Rpm),2);
	//memcpy(&(pData[2]),(uint8*)&(IOP_send_msg.i16_St_motor4Gear),2);
	//memcpy(&(pData[4]),(uint8*)&(IOP_send_msg.i16_St_motor4Rudder),2);
	//sendCanMsg(can_0,255,41,128,pData);

	//PS42
	memset(pData,0,8);
	pData[0]  =  (*IOP_send_msg.b1_St_remoteKey			   &0x01)       ; 
	pData[0] |=  (*IOP_send_msg.b1_St_PORTMotorCharge	   &0x01) << 1  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_STBDMotorCharge	   &0x01) << 2  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_shorePower		       &0x01) << 3  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_PORTShoreCharge	   &0x01) << 4  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_STBDShoreCharge	   &0x01) << 5  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_ShoreChargeEnd	       &0x01) << 6  ; 
	pData[0] |=  (*IOP_send_msg.b1_St_systemPowerOn		   &0x01) << 7	;

	pData[1]  =  (*IOP_send_msg.b3_St_supplySource      &0x07)       ;  
	pData[1] |=  (*IOP_send_msg.b1_Wn_batLow            &0x01) << 3  ;  
	pData[1] |=  (*IOP_send_msg.b1_Wn_oilLow            &0x01) << 4  ;  
	pData[1] |=  (*IOP_send_msg.b1_Wn_volOver           &0x01) << 5  ;  
	pData[1] |=  (*IOP_send_msg.b1_Wn_volBelow          &0x01) << 6  ;  
	pData[1] |=  (*IOP_send_msg.b1_Wn_curOver           &0x01) << 7  ; 

	pData[2] = *IOP_send_msg.u8_St_PORTBatLvl;
	pData[3] = *IOP_send_msg.u8_St_STBDBatLvl;
	pData[4] = *IOP_send_msg.u8_St_PORTOilLvl;
	pData[5] = *IOP_send_msg.u8_St_STBDOilLvl;
	sendCanMsg(can_0,255,42,128,pData);

	//PS43
	memset(pData,0,8);
	pData[0]  =  (*IOP_send_msg.b1_St_periPowK1    &0x01)       ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK2    &0x01) << 1  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK3    &0x01) << 2  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK4    &0x01) << 3  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK5    &0x01) << 4  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK6    &0x01) << 5  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK7    &0x01) << 6  ;  
	pData[0] |=  (*IOP_send_msg.b1_St_periPowK8    &0x01) << 7  ;  

	pData[1]  =  (*IOP_send_msg.b1_St_periPowk9	   &0x01)       ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk10    &0x01) << 1  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk11    &0x01) << 2  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk12    &0x01) << 3  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk13    &0x01) << 4  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk14    &0x01) << 5  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk15    &0x01) << 6  ;     
	pData[1] |=  (*IOP_send_msg.b1_St_periPowk16    &0x01) << 7  ;     

	pData[2]  =  (*IOP_send_msg.b1_St_periPowk17    &0x01)       ; 
	pData[2] |=  (*IOP_send_msg.b1_St_periPowk18    &0x01) << 1  ; 
	pData[2] |=  (*IOP_send_msg.b1_St_periPowk19    &0x01) << 2  ; 
	pData[2] |=  (*IOP_send_msg.b1_St_periPowk20    &0x01) << 3  ; 

	pData[3]  =  (*IOP_send_msg.b1_St_elecWindlass1St	    &0x01)       ; 
	pData[3] |=  (*IOP_send_msg.b1_St_elecWindlass2St	    &0x01) << 1  ; 
	pData[3] |=  (*IOP_send_msg.b1_St_sprayStrip1St		    &0x01) << 2  ; 
	pData[3] |=  (*IOP_send_msg.b1_St_sprayStrip2St		    &0x01) << 3  ; 
	sendCanMsg(can_0,255,43,128,pData);

   //PS44




	//printf("IOP_send_msg.i16_St_rot == %d\n", *IOP_send_msg.i16_St_rot);
	//printf("IOP_send_msg.u16_St_speed == %d\n", *IOP_send_msg.u16_St_speed);
	//printf("IOP_send_msg.u16_St_heading == %d\n", *IOP_send_msg.u16_St_heading);

	#endif
}

extern void initIOPsendTask( void )
{
	IOP_sdMsg_Init();
	addTask(2,runIOPSendMsg,(void*)0);	//加入50ms任务
}

void runIOPSendMsg(void *)
{
	IOP_sendMsg();
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IOP_SN].send_ok_number++;	//接收正确计数
}