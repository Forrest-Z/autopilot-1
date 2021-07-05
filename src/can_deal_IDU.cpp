/*==========================================================*
 * 模块说明: can_deal_IDU.cpp                                *
 * 文件版本: v1.00 (说明本文件的版本信息)						*
 * 开发人员: shaoyuping                                      *
 * 创建时间: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * 程序修改记录(最新的放在最前面):								*
 *  <修改日期>, <修改人员>: <修改功能概述>						*
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/can_deal_IDU.h"
#include "../include/usv_include.h"
/******************************  Local Variable  *****************************/
IDU_REV_MSG		IDU_rev_msg		;
IDU_SEND_MSG	IDU_send_msg	;
IDU_REV_CONFIG	IDU_rev_config	;
	
uint8		U8_DEST_VALUE_0 = 0;
uint8		U8_DEST_VALUE_1 = 1;
COMM_SIGN	IDU_comm_sign ={0,0};
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void IDU_rvMsg_Init( void )
{
	memset(&IDU_rev_msg,0,sizeof(IDU_REV_MSG));
}

void IDU_sdMsg_Init(void)
{
	memset(&IDU_send_msg,0,sizeof(IDU_SEND_MSG));
	IDU_send_msg.b1_St_MotorOnOff     = &IHC_rev_msg.mid_st.b1_St_MotorOn;

	IDU_send_msg.b1_Cmd_SystemRestart = &command_signal.equpment_cmd.b1_SystemRestart;
	
	IDU_send_msg.b1_Cmd_periPowK1	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[0])	  ;
	IDU_send_msg.b1_Cmd_periPowK2	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[1])   ;
	IDU_send_msg.b1_Cmd_periPowK3	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[2])   ;
	IDU_send_msg.b1_Cmd_periPowK4	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[3])   ;
	IDU_send_msg.b1_Cmd_periPowK5	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[4])   ;
	IDU_send_msg.b1_Cmd_periPowK6	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[5])   ;
	IDU_send_msg.b1_Cmd_periPowK7	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[6])   ;
	IDU_send_msg.b1_Cmd_periPowK8	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[7])   ;

	IDU_send_msg.b1_Cmd_periPowK9 	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[8])	  ;
	IDU_send_msg.b1_Cmd_periPowK10	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[9])   ;
	IDU_send_msg.b1_Cmd_periPowK11	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[10])  ;
	IDU_send_msg.b1_Cmd_periPowK12	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[11])  ;
	IDU_send_msg.b1_Cmd_periPowK13	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[12])  ;
	IDU_send_msg.b1_Cmd_periPowK14	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[13])  ;
	IDU_send_msg.b1_Cmd_periPowK15	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[14])  ;
	IDU_send_msg.b1_Cmd_periPowK16	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[15])  ;

	IDU_send_msg.b1_Cmd_periPowK17	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[16])  ;
	IDU_send_msg.b1_Cmd_periPowK18	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[17])  ;
	IDU_send_msg.b1_Cmd_periPowK19	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[18])  ;
	IDU_send_msg.b1_Cmd_periPowK20	=   IDU_Switch_connect(peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[19])  ;

	IDU_send_msg.b1_Cmd_SailLightPowK1	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK2	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK3	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK4	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK5	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK6	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK7	= (uint8*)&TEST_DEST_0;
	IDU_send_msg.b1_Cmd_SailLightPowK8	= (uint8*)&TEST_DEST_0;

}

void IDU_config_Init(void)
{
	memset(&IDU_rev_config,0,sizeof(IDU_REV_CONFIG));
}


void IDU_Init(void)
{
	IDU_rvMsg_Init();
	IDU_config_Init();
	comm_time_return(&(IDU_comm_sign.timer),&(IDU_comm_sign.comm_sign));
}

//统计通讯超时
void IDU_CommCal(void)
{
	comm_time_cal(CAN_IDU_DISCONNECT_MAX,&(IDU_comm_sign.timer),&(IDU_comm_sign.comm_sign));
}

void IDU_recv( uint8 psID,uint8* data )
{
	if(0 == psID)
	{
		IDU_rev_msg.b1_St_remoteKey         =   (data[0]&0x01)        ;
		IDU_rev_msg.b1_St_PORTMotorCharge   =   (data[0]&0x02) >> 1   ;
		IDU_rev_msg.b1_St_STBDMotorCharge   =   (data[0]&0x04) >> 2   ;
		IDU_rev_msg.b1_St_shorePower        =   (data[0]&0x08) >> 3   ;
		IDU_rev_msg.b1_St_PORTShoreCharge   =   (data[0]&0x10) >> 4   ;
		IDU_rev_msg.b1_St_STBDShoreCharge   =   (data[0]&0x20) >> 5   ;
		IDU_rev_msg.b1_St_ShoreChargeEnd    =   (data[0]&0x40) >> 6   ;
		IDU_rev_msg.b1_St_SystemPowerOn		=   (data[0]&0x80) >> 7	  ;

		IDU_rev_msg.b3_St_supplySource		=    (data[1]&0x07)         ;
		IDU_rev_msg.b1_Wn_batLow			=    (data[1]&0x08) >> 3    ;
		IDU_rev_msg.b1_Wn_oilLow			=    (data[1]&0x10) >> 4    ;
		IDU_rev_msg.b1_Wn_autoOff			=    (data[1]&0x20) >> 5    ;
		IDU_rev_msg.b1_Wn_oilLevelGauge		=    (data[1]&0x40) >> 6    ;
		IDU_rev_msg.b1_Wn_ShoreVolOver		=    (data[1]&0x80) >> 7    ;

		IDU_rev_msg.b1_Wn_ShoreVolBelow	    =     (data[2]&0x01)        ; 
		IDU_rev_msg.b1_Wn_ShoreCurOver	    =     (data[2]&0x02) >> 1   ; 
		IDU_rev_msg.b1_Wn_PORTVolOver	    =     (data[2]&0x04) >> 2   ; 
		IDU_rev_msg.b1_Wn_PORTVolBelow	    =     (data[2]&0x08) >> 3   ; 
		IDU_rev_msg.b1_Wn_PORTCurOver	    =     (data[2]&0x10) >> 4   ; 
		IDU_rev_msg.b1_Wn_STBDVolOver	    =     (data[2]&0x20) >> 5   ; 
		IDU_rev_msg.b1_Wn_STBDVolBelow	    =     (data[2]&0x40) >> 6   ; 
		IDU_rev_msg.b1_Wn_STBDCurOver	    =     (data[2]&0x80) >> 7   ;

		IDU_rev_msg.b1_Wn_warn				=     (data[3]&0x01)        ;
		IDU_rev_msg.b1_Wn_mainBoardPower	=     (data[3]&0x02) >> 1   ;
		IDU_rev_msg.b1_Wn_mainBoardTempe	=     (data[3]&0x04) >> 2   ;
		IDU_rev_msg.b1_Wn_int				=     (data[3]&0x08) >> 3   ;
		IDU_rev_msg.b1_Wn_CANComm			=     (data[3]&0x10) >> 4   ;
		IDU_rev_msg.b1_Wn_SCIA				=     (data[3]&0x20) >> 5   ;
		IDU_rev_msg.b1_Wn_SCIB				=     (data[3]&0x40) >> 6   ;
		
#ifdef TIANJI_SIGLE_ENGINE			//天极号不适用IDU,电量上送适用IHC接收数据
		IDU_rev_msg.u8_St_PORTBatLvl = IHC_rev_msg.u8_St_PORTBatLvl;
		IDU_rev_msg.u8_St_STBDBatLvl = IHC_rev_msg.u8_St_STBDBatLvl;
#else
		IDU_rev_msg.u8_St_PORTBatLvl		=     data[4]               ;
		IDU_rev_msg.u8_St_STBDBatLvl		=     data[5]               ;
#endif
		IDU_rev_msg.u16_St_remainBatTime	=	  u8tou16(&data[6])		;

		IDU_rev_msg.midMsg.b1_Wn_volOver	=	IDU_rev_msg.b1_Wn_ShoreVolOver |IDU_rev_msg.b1_Wn_PORTVolOver |IDU_rev_msg.b1_Wn_STBDVolOver	;
		IDU_rev_msg.midMsg.b1_Wn_volBelow	=   /*IDU_rev_msg.b1_Wn_ShoreVolBelow|*/IDU_rev_msg.b1_Wn_PORTVolBelow|IDU_rev_msg.b1_Wn_STBDVolBelow	;
		IDU_rev_msg.midMsg.b1_Wn_curOver	=	IDU_rev_msg.b1_Wn_ShoreCurOver |IDU_rev_msg.b1_Wn_PORTCurOver |IDU_rev_msg.b1_Wn_STBDCurOver	;

		
	}
	else if(1 == psID)
	{
		IDU_rev_msg.u16_St_PORTInstanVol	=    u8tou16(&data[0])		;
		IDU_rev_msg.u16_St_PORTInstanCur	=    u8tou16(&data[2])      ;
		IDU_rev_msg.u16_St_PORTInstanPow	=    u8tou16(&data[4])      ;
	}
	else if(2 == psID)
	{
		IDU_rev_msg.u16_St_STBDInstanVol    =    u8tou16(&data[0])		;
		IDU_rev_msg.u16_St_STBDInstanCur    =    u8tou16(&data[2])		;
		IDU_rev_msg.u16_St_STBDInstanPow    =    u8tou16(&data[4])		;
	}
	else if(3 == psID)
	{
		IDU_rev_msg.u16_St_ShoreInstanVol   =    u8tou16(&data[0])		;
		IDU_rev_msg.u16_St_ShoreInstanCur   =    u8tou16(&data[2])		;
		IDU_rev_msg.u16_St_ShoreInstanPow   =    u8tou16(&data[4])		;
	}
	else if(4 == psID)
	{
		IDU_rev_msg.u8_St_PortOilLvl		=	 data[0];
		IDU_rev_msg.f32_St_PORTIntergFlow	=	 u8tofloat(&(data[4]));
	}
	else if(5 == psID)
	{
		IDU_rev_msg.f32_St_PORTInstanFlow   =     u8tofloat(&(data[0]));
		IDU_rev_msg.f32_St_PORTIntergTime   =     u8tofloat(&(data[4]));
	}
	else if(6 == psID)
	{
		IDU_rev_msg.u8_St_STBDOilLvl		=	 data[0];
		IDU_rev_msg.f32_St_STBDIntergFlow	=	 u8tofloat(&(data[4]));
	}
	else if(7 == psID)
	{
		IDU_rev_msg.f32_St_STBDInstanFlow   =     u8tofloat(&(data[0]));
		IDU_rev_msg.f32_St_STBDIntergTime   =     u8tofloat(&(data[4]));
	}
	else if(8 == psID)
	{
		IDU_rev_msg.b1_St_periPowK1    =     (data[0]&0x01)        ; 
		IDU_rev_msg.b1_St_periPowK2    =     (data[0]&0x02) >> 1   ; 
		IDU_rev_msg.b1_St_periPowK3    =     (data[0]&0x04) >> 2   ; 
		IDU_rev_msg.b1_St_periPowK4    =     (data[0]&0x08) >> 3   ; 
		IDU_rev_msg.b1_St_periPowK5    =     (data[0]&0x10) >> 4   ; 
		IDU_rev_msg.b1_St_periPowK6    =     (data[0]&0x20) >> 5   ; 
		IDU_rev_msg.b1_St_periPowK7    =     (data[0]&0x40) >> 6   ; 
		IDU_rev_msg.b1_St_periPowK8    =     (data[0]&0x80) >> 7   ; 

		IDU_rev_msg.b1_St_periPowk9	    =     (data[1]&0x01)        ;   
		IDU_rev_msg.b1_St_periPowk10    =     (data[1]&0x02) >> 1   ;   
		IDU_rev_msg.b1_St_periPowk11    =     (data[1]&0x04) >> 2   ;   
		IDU_rev_msg.b1_St_periPowk12    =     (data[1]&0x08) >> 3   ;   
		IDU_rev_msg.b1_St_periPowk13    =     (data[1]&0x10) >> 4   ;   
		IDU_rev_msg.b1_St_periPowk14    =     (data[1]&0x20) >> 5   ;   
		IDU_rev_msg.b1_St_periPowk15    =     (data[1]&0x40) >> 6   ;   
		IDU_rev_msg.b1_St_periPowk16    =     (data[1]&0x80) >> 7   ;   

		IDU_rev_msg.b1_St_periPowk17    =     (data[2]&0x01)        ; 
		IDU_rev_msg.b1_St_periPowk18    =     (data[2]&0x02) >> 1   ; 
		IDU_rev_msg.b1_St_periPowk19    =     (data[2]&0x04) >> 2   ; 
		IDU_rev_msg.b1_St_periPowk20    =     (data[2]&0x08) >> 3   ; 

		IDU_rev_msg.b1_St_sailLightK1    =     (data[3]&0x01)        ;
		IDU_rev_msg.b1_St_sailLightK2    =     (data[3]&0x02) >> 1   ;
		IDU_rev_msg.b1_St_sailLightK3    =     (data[3]&0x04) >> 2   ;
		IDU_rev_msg.b1_St_sailLightK4    =     (data[3]&0x08) >> 3   ;
		IDU_rev_msg.b1_St_sailLightK5    =     (data[3]&0x10) >> 4   ;
		IDU_rev_msg.b1_St_sailLightK6    =     (data[3]&0x20) >> 5   ;
		IDU_rev_msg.b1_St_sailLightK7    =     (data[3]&0x40) >> 6   ;
		IDU_rev_msg.b1_St_sailLightK8    =     (data[3]&0x80) >> 7   ;

		IDU_rev_msg.b1_St_plugConK1    =     (data[4]&0x01)        ; 
		IDU_rev_msg.b1_St_plugConK2    =     (data[4]&0x02) >> 1   ; 
		IDU_rev_msg.b1_St_plugConK3    =     (data[4]&0x04) >> 2   ; 
		IDU_rev_msg.b1_St_plugConK4    =     (data[4]&0x08) >> 3   ; 
		IDU_rev_msg.b1_St_plugConK5    =     (data[4]&0x10) >> 4   ; 
		IDU_rev_msg.b1_St_plugConK6    =     (data[4]&0x20) >> 5   ; 
		IDU_rev_msg.b1_St_plugConK7    =     (data[4]&0x40) >> 6   ; 
		IDU_rev_msg.b1_St_plugConK8    =     (data[4]&0x80) >> 7   ; 

		IDU_rev_msg.b1_St_plugConK9     =     (data[5]&0x01)        ;  
		IDU_rev_msg.b1_St_plugConK10    =     (data[5]&0x02) >> 1   ;  
		IDU_rev_msg.b1_St_plugConK11    =     (data[5]&0x04) >> 2   ;  
		IDU_rev_msg.b1_St_plugConK12    =     (data[5]&0x08) >> 3   ;  
		IDU_rev_msg.b1_St_plugConK13    =     (data[5]&0x10) >> 4   ;  
		IDU_rev_msg.b1_St_plugConK14    =     (data[5]&0x20) >> 5   ;  
		IDU_rev_msg.b1_St_plugConK15    =     (data[5]&0x40) >> 6   ;  
		IDU_rev_msg.b1_St_plugConK16    =     (data[5]&0x80) >> 7   ;  

		IDU_rev_msg.b1_St_plugConK17    =     (data[6]&0x01)        ;  
		IDU_rev_msg.b1_St_plugConK18    =     (data[6]&0x02) >> 1   ;  
		IDU_rev_msg.b1_St_plugConK19    =     (data[6]&0x04) >> 2   ;  
		IDU_rev_msg.b1_St_plugConK20    =     (data[6]&0x08) >> 3   ;  
	}
	else if(128 == psID)
	{
		IDU_rev_config.u16_version	=	u8tou16(&(data[0]));
		IDU_rev_config.u16_svn		=	u8tou16(&(data[2]));
		IDU_rev_config.u16_crc		=	u8tou16(&(data[4]));
	}
	else if(129 == psID)
	{
		IDU_rev_config.u8_confState = (data[0]&0x01)		;
		IDU_rev_config.u8_initState = (data[1]&0x02)>>1		;
	}
	else
	{
		return;
	}
	comm_time_return(&(IDU_comm_sign.timer),&(IDU_comm_sign.comm_sign));	//通讯计数复归
}

void IDU_sendMsg(void)
{
	uint8 pData[8];
	memset(pData,0,8);
	pData[0]  =  *IDU_send_msg.b1_St_MotorOnOff	;
	pData[0] |= (*IDU_send_msg.b1_Cmd_SystemRestart&0x01)<<7	;

	pData[1] = (*IDU_send_msg.b1_Cmd_periPowK1&0x01	)		;
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK2&0x01  )<<1	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK3&0x01  )<<2	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK4&0x01  )<<3	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK5&0x01  )<<4	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK6&0x01  )<<5	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK7&0x01  )<<6	; 
	pData[1]+= (*IDU_send_msg.b1_Cmd_periPowK8&0x01  )<<7	; 

	pData[2] = (*IDU_send_msg.b1_Cmd_periPowK9&0x01	)		;
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK10&0x01  )<<1	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK11&0x01  )<<2	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK12&0x01  )<<3	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK13&0x01  )<<4	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK14&0x01  )<<5	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK15&0x01  )<<6	; 
	pData[2]+= (*IDU_send_msg.b1_Cmd_periPowK16&0x01  )<<7	; 

	pData[3] = (*IDU_send_msg.b1_Cmd_periPowK17&0x01	)		;
	pData[3]+= (*IDU_send_msg.b1_Cmd_periPowK18&0x01  )<<1	; 
	pData[3]+= (*IDU_send_msg.b1_Cmd_periPowK19&0x01  )<<2	; 
	pData[3]+= (*IDU_send_msg.b1_Cmd_periPowK20&0x01  )<<3	; 

	pData[4] = (*IDU_send_msg.b1_Cmd_SailLightPowK1&0x01	)		;
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK2&0x01  )<<1	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK3&0x01  )<<2	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK4&0x01  )<<3	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK5&0x01  )<<4	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK6&0x01  )<<5	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK7&0x01  )<<6	; 
	pData[4]+= (*IDU_send_msg.b1_Cmd_SailLightPowK8&0x01  )<<7	; 

	sendCanMsg(can_0,255,21,128,pData);
	usleep(500);//0.5ms	
}

void initIDUsendTask()
{
	IDU_sdMsg_Init();
	addTask(2,runIDUsendTask,(void*)0);
}

void runIDUsendTask(void *)
{
	IDU_sendMsg();
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IDU_SN].send_ok_number++;
}

uint8* IDU_Switch_connect( int32 connectNum )
{
	if(connectNum == -1) 
		return &U8_DEST_VALUE_1;
	if(connectNum == -2)
		return &U8_DEST_VALUE_0;
	switch(connectNum){
		case 0:
			return &command_signal.equpment_cmd.b1_periPowK1;
			break;
		case 1:
			return &command_signal.equpment_cmd.b1_periPowK2;
			break;
		case 2:
			return &command_signal.equpment_cmd.b1_periPowK3;
			break;
		case 3:
			return &command_signal.equpment_cmd.b1_periPowK4;
			break;
		case 4:
			return &command_signal.equpment_cmd.b1_periPowK5;
			break;
		case 5:
			return &command_signal.equpment_cmd.b1_periPowK6;
			break;
		case 6:
			return &command_signal.equpment_cmd.b1_periPowK7;
			break;
		case 7:
			return &command_signal.equpment_cmd.b1_periPowK8;
			break;
		case 8:
			return &command_signal.equpment_cmd.b1_periPowK9;
			break;
		case 9:
			return &command_signal.equpment_cmd.b1_periPowK10;
			break;
		case 10:
			return &command_signal.equpment_cmd.b1_periPowK11;
			break;
		case 11:
			return &command_signal.equpment_cmd.b1_periPowK12;
			break;
		case 12:
			return &command_signal.equpment_cmd.b1_periPowK13;
			break;
		case 13:
			return &command_signal.equpment_cmd.b1_periPowK14;
			break;
		case 14:
			return &command_signal.equpment_cmd.b1_periPowK15;
			break;
		case 15:
			return &command_signal.equpment_cmd.b1_periPowK16;
			break;
		case 16:
			return &command_signal.equpment_cmd.b1_periPowK17;
			break;
		case 17:
			return &command_signal.equpment_cmd.b1_periPowK18;
			break;
		case 18:
			return &command_signal.equpment_cmd.b1_periPowK19;
			break;
		case 19:
			return &command_signal.equpment_cmd.b1_periPowK20;
			break;
		default:
			return &U8_DEST_VALUE_0;
	}
}
