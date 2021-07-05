/*==========================================================*
 * ģ��˵��: lan_deal_BRadio.cpp                            *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
 * ����ʱ��: 2018��4��3��09:10:07                           *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/lan_deal_BRadio.h"
#include <stdarg.h>
#ifndef WINNT
#include <linux/sockios.h>
#include <netpacket/packet.h>
#define CloseSocket(x) close(x)
#else
#include <winsock2.h>  
#include <ws2tcpip.h>  
#pragma comment( lib, "WS2_32" )
#define CloseSocket(x) closesocket(x)
#endif

#define MAX_LAN_BUFF_LEN 200
#define BRIOP_UDP_REC_PORT	6602
#define	BRIOP_UDP_SED_PORT	6601
/******************************  Local Variable  *****************************/
//int8	re_buff[MAX_LAN_BUFF_LEN];	//���������ݻ���
COMM_SIGN		bradio_sign	= {0,0};
BRIOP_REV_MSG	BrIOP_rev_msg;
BRIOP_SEND_MSG	BrIOP_send_msg;

UDP_INF_STRUCT	BrIOP_udp_inf		;
int16			BrIOP_udp_sockid	;
uint8			log_b1_Bradio_sign=0	;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void deal_BrIOP_rec(uint8* buff,uint16 report_len);
void BRIOP_CommCal(void *);
void anayticalBrIOP(uint8 *buff);
int8 lan_bradio_rec_report(int16 sockid);
int8 checkBrIOPCrc(int8 *buff);
#define BRIOP_REV_FRAME_LEN 50
#define BRIOP_SEND_FRAME_LEN 82
#define BRIOP_SEND_MSG_LEN	70
#define BRIOP_DISCONNECT_MAX 20
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void BrIOP_udp_inf_init(void)
{
	memset((int8*)&BrIOP_udp_inf,0,sizeof(BrIOP_udp_inf));
}

void BRIOP_CommCal(void *)
{
	comm_time_cal(BRIOP_DISCONNECT_MAX,&(bradio_sign.timer),&(bradio_sign.comm_sign));

	if(log_b1_Bradio_sign != bradio_sign.comm_sign && poweron_init)
	{
		switch(bradio_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	//SysLogMsgPost("������̨ͨѶ�ָ�");
								//SysPubMsgPost("������̨ͨѶ�ָ�");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_BRADIO_TIMEOUT, WARN_OFF);
			break;
		case COMM_CONNECT_FAIL:	//SysLogMsgPost("������̨ͨѶ�ж�");
								//SysPubMsgPost("������̨ͨѶ�ж�");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_BRADIO_TIMEOUT, WARN_ON);
			break;
		default:
			break;
		}
	}

	log_b1_Bradio_sign = bradio_sign.comm_sign;
}

void* lan_deal_dradio( void *aa )
{
	//��ʼ��
	//	memset((int8*)&re_buff[0],0,sizeof(re_buff));
	BrIOP_udp_sockid = -1;

	for (;;)
	{
		if(-1 == BrIOP_udp_sockid)
		{
			BrIOP_udp_sockid = CreateUDPSocket((uint16)BRIOP_UDP_REC_PORT);	//����UDP
			comm_time_return(&(bradio_sign.timer),&(bradio_sign.comm_sign));
			if(BrIOP_udp_sockid<0){
				sleep_1(1000);
				continue;
			}
		}
		else{
			lan_bradio_rec_report(BrIOP_udp_sockid);
		}

		sleep_1(200);

	}
	
}

int8 lan_bradio_rec_report( int16 sockid )
{
	fd_set set;
	int16 ret;
	int16 iLen;
	struct timeval timeout;
	socklen_t sockaddrlen;
	int8 net_rec_buff[200];

	FD_ZERO(&set);
	FD_SET(sockid,&set);
	timeout.tv_sec=0;
	timeout.tv_usec=5;
	ret = select(sockid+1,&set,NULL,NULL,&timeout);
	if(ret<0){
		return TRUE;
	}
	if(FD_ISSET(sockid,&set)){		//UDP����
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff),0,(struct sockaddr *)&BrIOP_udp_inf.from_upd_ip, &sockaddrlen);
		if(iLen>=1){
			BrIOP_udp_inf.udp_rec_flag = SWITCH_ON;
			deal_BrIOP_rec((uint8 *)&net_rec_buff[0],iLen);
		}
	}
	return TRUE;
}

void deal_BrIOP_rec( uint8* buff,uint16 report_len)
{
	int ico,jco=0,re_sign=0;
	uint8 brIOP_Con[MAX_LAN_BUFF_LEN];
	
	for(ico= 0;ico<report_len;ico++)
	{
		if((buff[ico]=='$')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V')||(1==re_sign))
		{
			re_sign = 1;
			brIOP_Con[jco++] = buff[ico];
			if(jco >= BRIOP_REV_FRAME_LEN)
			{
				jco = 0;
				re_sign = 0;
				//checkBrIOPCRC
				if(checkBrIOPCrc((int8*)&brIOP_Con[0])==0/*brIOP_Con[BRIOP_REV_FRAME_LEN-3]!='*'*/)
				{
					monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BRIOP_SN].rec_error_number++;
					return ;
				}
				//frame analytical
				anayticalBrIOP((uint8*)&brIOP_Con[9]);
				comm_time_return(&(bradio_sign.timer),&(bradio_sign.comm_sign));
				monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BRIOP_SN].rec_ok_number++;

			}
		}

		
	}
	
}

int8 checkBrIOPCrc( int8 *buff )
{
	uint16 crcRead;
	uint16 crcCalc;
	crcRead = u8tou16((uint8*)&buff[BRIOP_REV_FRAME_LEN-2]);
	crcCalc = calcCRC(0xffff,(uint8*)buff,BRIOP_REV_FRAME_LEN-2);
	if(crcRead != crcCalc)
		return 0;

	return 1;
}

void anayticalBrIOP( uint8 *buff )
{
	BrIOP_rev_msg.b1_Wn_warn			=  ( buff[0]	& 0x01	)>>0	;
	BrIOP_rev_msg.b1_Wn_mainBoardPower	=  ( buff[0]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Wn_mainBoardTempe	=  ( buff[0]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Wn_int				=  ( buff[0]	& 0x08	)>>3	;
	BrIOP_rev_msg.b1_Wn_CANComm			=  ( buff[0]	& 0x10	)>>4	;
	BrIOP_rev_msg.b1_Wn_SCIA			=  ( buff[0]	& 0x20	)>>5	;
	BrIOP_rev_msg.b1_Wn_SCIB			=  ( buff[0]	& 0x40	)>>6	;

	BrIOP_rev_msg.b1_Wn_externSP1		=  ( buff[1]	& 0x01	)>>0	;
	BrIOP_rev_msg.b1_Wn_externSP2		=  ( buff[1]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Wn_externSP3		=  ( buff[1]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Wn_externSP4		=  ( buff[1]	& 0x08	)>>3	;
	BrIOP_rev_msg.b1_Wn_externSP5		=  ( buff[1]	& 0x10	)>>4	;
	BrIOP_rev_msg.b1_Wn_externSP6		=  ( buff[1]	& 0x20	)>>5	;
	BrIOP_rev_msg.b1_Wn_externSP7		=  ( buff[1]	& 0x40	)>>6	;
	BrIOP_rev_msg.b1_Wn_externSP8		=  ( buff[1]	& 0x80	)>>7	;

	BrIOP_rev_msg.b2_Cmd_motor			=    (buff[2] & 0x03)			;
	BrIOP_rev_msg.b1_Cmd_SystemRestart  =	 (buff[2] & 0x80)>>7		;

	BrIOP_rev_msg.b1_Cmd_emergencyMode  =  ( buff[3]	& 0x01	)>>0	;
	BrIOP_rev_msg.b1_Cmd_emergencyStop  =  ( buff[3]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Cmd_getAuthority	=  ( buff[3]	& 0x04	)>>2	;
	BrIOP_rev_msg.b2_Cmd_sailMode		=  ( buff[3]	& 0x30	)>>4	;
	BrIOP_rev_msg.b2_Cmd_sailTask		=  ( buff[3]	& 0xc0	)>>6	;

	BrIOP_rev_msg.b1_Cmd_speedConstant	=  ( buff[4]	& 0x01	)>>0	;
	BrIOP_rev_msg.b1_Cmd_headingConstant=  ( buff[4]	& 0x02	)>>1	;	
	BrIOP_rev_msg.b1_Cmd_berthMode		=  ( buff[4]	& 0x04	)>>2	;

	BrIOP_rev_msg.i16_Cmd_joy1Y	 = u8toi16(&buff[5]);
	BrIOP_rev_msg.i16_Cmd_joy1X	 = u8toi16(&buff[7]);
	BrIOP_rev_msg.i16_Cmd_joy1Z = u8toi16(&buff[9]);

	BrIOP_rev_msg.i16_Cmd_joy2Y  = u8toi16(&buff[11]);
	BrIOP_rev_msg.i16_Cmd_joy2X	= u8toi16(&buff[13]);
	BrIOP_rev_msg.i16_Cmd_joy2Z = u8toi16(&buff[15]);

	BrIOP_rev_msg.b1_Cmd_periPowerS1	=  ( buff[23]	& 0x01	)>>0	;				
	BrIOP_rev_msg.b1_Cmd_periPowerS2	=  ( buff[23]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Cmd_periPowerS3	=  ( buff[23]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Cmd_periPowerS4	=  ( buff[23]	& 0x08	)>>3	;
	BrIOP_rev_msg.b1_Cmd_periPowerS5	=  ( buff[23]	& 0x10	)>>4	;
	BrIOP_rev_msg.b1_Cmd_periPowerS6	=  ( buff[23]	& 0x20	)>>5	;
	BrIOP_rev_msg.b1_Cmd_periPowerS7	=  ( buff[23]	& 0x40	)>>6	;
	BrIOP_rev_msg.b1_Cmd_periPowerS8	=  ( buff[23]	& 0x80	)>>7	;

	BrIOP_rev_msg.b1_Cmd_periPowerS9	=  ( buff[24]	& 0x01	)>>0	;	
	BrIOP_rev_msg.b1_Cmd_periPowerS10	=  ( buff[24]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Cmd_periPowerS11	=  ( buff[24]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Cmd_periPowerS12	=  ( buff[24]	& 0x08	)>>3	;
	BrIOP_rev_msg.b1_Cmd_periPowerS13	=  ( buff[24]	& 0x10	)>>4	;
	BrIOP_rev_msg.b1_Cmd_periPowerS14	=  ( buff[24]	& 0x20	)>>5	;
	BrIOP_rev_msg.b1_Cmd_periPowerS15	=  ( buff[24]	& 0x40	)>>6	;
	BrIOP_rev_msg.b1_Cmd_periPowerS16	=  ( buff[24]	& 0x80	)>>7	;

	BrIOP_rev_msg.b1_Cmd_periPowerS17	=  ( buff[25]	& 0x01	)>>0	;	
	BrIOP_rev_msg.b1_Cmd_periPowerS18	=  ( buff[25]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Cmd_periPowerS19	=  ( buff[25]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Cmd_periPowerS20	=  ( buff[25]	& 0x08	)>>3	;

	BrIOP_rev_msg.b1_Cmd_elecWindlass1OnOff		=  ( buff[26]	& 0x01	)>>0	;	
	BrIOP_rev_msg.b1_Cmd_elecWindlass1UpDown	=  ( buff[26]	& 0x02	)>>1	;
	BrIOP_rev_msg.b1_Cmd_elecWindlass2OnOff		=  ( buff[26]	& 0x04	)>>2	;
	BrIOP_rev_msg.b1_Cmd_elecWindlass2UpDown	=  ( buff[26]	& 0x08	)>>3	;
	BrIOP_rev_msg.b1_Cmd_sprayStrip1OnOff		=  ( buff[26]	& 0x10	)>>4	;
	BrIOP_rev_msg.b1_Cmd_sprayStrip1UpDown		=  ( buff[26]	& 0x20	)>>5	;
	BrIOP_rev_msg.b1_Cmd_sprayStrip2OnOff		=  ( buff[26]	& 0x40	)>>6	;
	BrIOP_rev_msg.b1_Cmd_sprayStrip2UpDown		=  ( buff[26]	& 0x80	)>>7	;

	//32
	BrIOP_rev_msg.u16_version = u8tou16(&buff[32]);
	BrIOP_rev_msg.u16_svn	  = u8tou16(&buff[34]);
	BrIOP_rev_msg.u16_crc	  = u8tou16(&buff[36]);
}

void BRIOP_sdMsg_Init( void )
{
	memset(&BrIOP_send_msg,0,sizeof(DRIOP_SEND_MSG));
	BrIOP_send_msg.b1_Wn_IOPCommOutage	= & IOP_comm_sign.comm_sign;
	BrIOP_send_msg.b1_Wn_IOPWarn		= & IOP_rev_msg.b1_Wn_warn;
	BrIOP_send_msg.b1_Wn_IHCCommOutage  = & IHC_comm_sign.comm_sign;
	BrIOP_send_msg.b1_Wn_IHCWarn		= & IHC_rev_msg.mid_st.b1_Wn_CommonWarn;
	BrIOP_send_msg.b1_Wn_IDUCommOutage  = & IDU_comm_sign.comm_sign;
	BrIOP_send_msg.b1_Wn_IDUWarn		= & IDU_rev_msg.b1_Wn_warn;
	BrIOP_send_msg.b1_Wn_MCUCommOutage  = (uint8*)&TEST_DEST_0;
	BrIOP_send_msg.b1_Wn_MCUWarn		= (uint8*)&TEST_DEST_0;

	BrIOP_send_msg.u8_St_year			= &state_signal.time.u8_year	;///&ins_msg.u8_year;
	BrIOP_send_msg.u8_St_month			= &state_signal.time.u8_month	;//&ins_msg.u8_month;
	BrIOP_send_msg.u8_St_date			= &state_signal.time.u8_date	;//&ins_msg.u8_day;

	BrIOP_send_msg.u8_St_hour			= &state_signal.time.u8_hour	;//&ins_msg.u8_hour;
	BrIOP_send_msg.u8_St_minute			= &state_signal.time.u8_minute	;//&ins_msg.u8_minute;
	BrIOP_send_msg.u8_St_second			= &state_signal.time.u8_second	;//&ins_msg.u8_second;

	BrIOP_send_msg.u16_St_speed			= &ins_msg.u16_speed;
	BrIOP_send_msg.u16_St_heading		= &ins_msg.u16_heading;

	BrIOP_send_msg.i16_St_rot			= &ins_msg.i16_rot;
	BrIOP_send_msg.i16_St_pitch			= &ins_msg.i16_pitch;
	BrIOP_send_msg.i16_St_roll			= &ins_msg.i16_roll;
	BrIOP_send_msg.i16_St_heaving		= &ins_msg.i16_heaving;

	BrIOP_send_msg.u8_St_longiSt		= &ins_msg.u8_longiSt		;
	BrIOP_send_msg.u8_St_longiDeg		= &ins_msg.u8_longiDeg		;
	BrIOP_send_msg.u8_St_longiMin		= &ins_msg.u8_longiMin		;
	BrIOP_send_msg.u8_St_longiSec		= &ins_msg.u8_longiSec		;
	BrIOP_send_msg.u8_St_longiSecDec	= &ins_msg.u8_longiSecDec	;

	BrIOP_send_msg.u8_St_latiSt			= &ins_msg.u8_latiSt		;
	BrIOP_send_msg.u8_St_latiDeg		= &ins_msg.u8_latiDeg		;
	BrIOP_send_msg.u8_St_latiMin		= &ins_msg.u8_latiMin		;
	BrIOP_send_msg.u8_St_latiSec		= &ins_msg.u8_latiSec		;
	BrIOP_send_msg.u8_St_latiSecDec		= &ins_msg.u8_latiSecDec	;

	BrIOP_send_msg.b1_St_emergencyMode		= &command_signal.sail_mode_cmd.b1_emergencyMode	;
	BrIOP_send_msg.b1_St_emergencyStop		= &jet_system.b1_cmd_emergencyStop				;
	BrIOP_send_msg.b1_St_localAuthority		= &command_signal.b1_authority						;
	BrIOP_send_msg.b2_St_sailMode			= &command_signal.sail_mode_cmd.b2_sailMode			;
	BrIOP_send_msg.b2_St_sailTask			= &command_signal.sail_feedBack.b2_sailTask 		;

	BrIOP_send_msg.b1_St_speedConstant		= &command_signal.func_mode_cmd.b1_speedConstant	;
	BrIOP_send_msg.b1_St_headingConstant	= &command_signal.func_mode_cmd.b1_headingConstant	;	
	BrIOP_send_msg.b1_St_berthMode			= &command_signal.func_mode_cmd.b1_dock_cmd			;
	BrIOP_send_msg.b1_autoReturn			= &command_signal.func_mode_cmd.b1_autoReturn		;

	BrIOP_send_msg.b1_St_motorSt			= (uint8*)&IHC_rev_msg.mid_st.b1_St_MotorOn		;
	BrIOP_send_msg.b1_Wn_motor1Wn			= (uint8*)&IHC_rev_msg.b1_Wn_PORTMotorWarn		;
	BrIOP_send_msg.b1_Wn_motor2Wn			= (uint8*)&IHC_rev_msg.b1_Wn_STBDMotorWarn		;


	BrIOP_send_msg.u16_St_motor1Rpm			= &IHC_rev_msg.u16_St_Motor1Rpm		;	//��Ҫת��
	BrIOP_send_msg.i16_St_motor1Gear		= &IHC_rev_msg.i16_St_Motor1Gear	;
	BrIOP_send_msg.i16_St_motor1Rudder		= &IHC_rev_msg.i16_St_Motor1Rudder	;

	BrIOP_send_msg.u16_St_motor2Rpm			= &IHC_rev_msg.u16_St_Motor2Rpm		;	//��Ҫת��
	BrIOP_send_msg.i16_St_motor2Gear		= &IHC_rev_msg.i16_St_Motor2Gear	;
	BrIOP_send_msg.i16_St_motor2Rudder		= &IHC_rev_msg.i16_St_Motor2Rudder	;

	BrIOP_send_msg.u16_St_motor3Rpm			= (uint16*)&TEST_DEST_0;	//��Ҫת��
	BrIOP_send_msg.i16_St_motor3Gear		= (int16*)&TEST_DEST_0;
	BrIOP_send_msg.i16_St_motor3Rudder		= (int16* )&TEST_DEST_0;

	BrIOP_send_msg.u16_St_motor4Rpm			= (uint16*)&TEST_DEST_0;	//��Ҫת��
	BrIOP_send_msg.i16_St_motor4Gear		= (int16*)&TEST_DEST_0;
	BrIOP_send_msg.i16_St_motor4Rudder		= (int16* )&TEST_DEST_0;

	BrIOP_send_msg.b1_St_remoteKey			= &IDU_rev_msg.b1_St_remoteKey;
	BrIOP_send_msg.b1_St_PORTMotorCharge	= &IDU_rev_msg.b1_St_PORTMotorCharge;
	BrIOP_send_msg.b1_St_STBDMotorCharge	= &IDU_rev_msg.b1_St_STBDMotorCharge;
	BrIOP_send_msg.b1_St_shorePower			= &IDU_rev_msg.b1_St_shorePower;
	BrIOP_send_msg.b1_St_PORTShoreCharge	= &IDU_rev_msg.b1_St_PORTShoreCharge;
	BrIOP_send_msg.b1_St_STBDShoreCharge	= &IDU_rev_msg.b1_St_STBDShoreCharge;
	BrIOP_send_msg.b1_St_ShoreChargeEnd		= &IDU_rev_msg.b1_St_ShoreChargeEnd;
	BrIOP_send_msg.b1_St_SystemPowerOn		= &IDU_rev_msg.b1_St_SystemPowerOn;

	BrIOP_send_msg.b3_St_supplySource		= &IDU_rev_msg.b3_St_supplySource;
	BrIOP_send_msg.b1_Wn_batLow				= &IDU_rev_msg.b1_Wn_batLow;
	BrIOP_send_msg.b1_Wn_oilLow				= &IDU_rev_msg.b1_Wn_oilLow;
	BrIOP_send_msg.b1_Wn_volOver			= &IDU_rev_msg.midMsg.b1_Wn_volOver;
	BrIOP_send_msg.b1_Wn_volBelow			= &IDU_rev_msg.midMsg.b1_Wn_volBelow;
	BrIOP_send_msg.b1_Wn_curOver			= &IDU_rev_msg.midMsg.b1_Wn_curOver;

	BrIOP_send_msg.u8_St_PORTBatLvl			= &IDU_rev_msg.u8_St_PORTBatLvl;
	BrIOP_send_msg.u8_St_STBDBatLvl			= &IDU_rev_msg.u8_St_STBDBatLvl;
	BrIOP_send_msg.u8_St_PORTOilLvl			= &IDU_rev_msg.u8_St_PortOilLvl;
	BrIOP_send_msg.u8_St_STBDOilLvl			= &IDU_rev_msg.u8_St_STBDOilLvl;


	BrIOP_send_msg.u16_St_BatteryVoltage =	&IHC_rev_msg.u16_St_BatteryVoltage;
	BrIOP_send_msg.u16_St_BatteryCurrent =	&IHC_rev_msg.u16_St_BatteryCurrent;
	BrIOP_send_msg.u8_St_BatteryRemainTime_H	= (uint8*)&IHC_rev_msg.u8_St_BatteryRemainTime_H	;
	BrIOP_send_msg.u8_St_BatteryRemainTime_M	= (uint8*)&IHC_rev_msg.u8_St_BatteryRemainTime_M	;
	BrIOP_send_msg.u8_St_BatteryRemainTime_S	= (uint8*)&IHC_rev_msg.u8_St_BatteryRemainTime_S	;
	BrIOP_send_msg.u8_St_BatteryRemainPercent = &IHC_rev_msg.u8_St_BatteryLevel;


	BrIOP_send_msg.b1_St_periPowK1			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[0]);
	BrIOP_send_msg.b1_St_periPowK2			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[1]);
	BrIOP_send_msg.b1_St_periPowK3			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[2]);
	BrIOP_send_msg.b1_St_periPowK4			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[3]);
	BrIOP_send_msg.b1_St_periPowK5			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[4]);
	BrIOP_send_msg.b1_St_periPowK6			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[5]);
	BrIOP_send_msg.b1_St_periPowK7			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[6]);
	BrIOP_send_msg.b1_St_periPowK8			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[7]);

	BrIOP_send_msg.b1_St_periPowk9			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[8]);
	BrIOP_send_msg.b1_St_periPowk10			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[9]);
	BrIOP_send_msg.b1_St_periPowk11			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[10]);
	BrIOP_send_msg.b1_St_periPowk12			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[11]);
	BrIOP_send_msg.b1_St_periPowk13			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[12]);
	BrIOP_send_msg.b1_St_periPowk14			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[13]);
	BrIOP_send_msg.b1_St_periPowk15			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[14]);
	BrIOP_send_msg.b1_St_periPowk16			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[15]);

	BrIOP_send_msg.b1_St_periPowk17			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[16]);
	BrIOP_send_msg.b1_St_periPowk18			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[17]);
	BrIOP_send_msg.b1_St_periPowk19			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[18]);
	BrIOP_send_msg.b1_St_periPowk20			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[19]);

	BrIOP_send_msg.b1_St_elecWindlass1St	= &IHC_rev_msg.b1_St_elecWindlass1OnOff;
	BrIOP_send_msg.b1_St_elecWindlass2St	= &IHC_rev_msg.b1_St_elecWindlass2OnOff;
	BrIOP_send_msg.b1_St_sprayStrip1St		= &IHC_rev_msg.b1_St_sprayStrip1OnOff;
	BrIOP_send_msg.b1_St_sprayStrip2St		= &IHC_rev_msg.b1_St_sprayStrip2OnOff;
}

void initBRIOPsendTask()
{
	BRIOP_sdMsg_Init();
	addTask(3,runBRIOPSendMsg,(void*)0);	//����200ms ����
}


int8 BRIOP_sendMsg(uint16 frameCntr)
{
	uint8 *pData		;
	uint8  pFrame[BRIOP_SEND_FRAME_LEN]	;
	uint16 crc16;

	int iSendLen;
	int	DestPort;
	struct sockaddr_in server;

	DestPort = BRIOP_UDP_SED_PORT;
	server.sin_family = AF_INET;
	server.sin_port	= htons(DestPort);
	server.sin_addr.s_addr = BrIOP_udp_inf.from_upd_ip.sin_addr.s_addr;
	memset(server.sin_zero,0,8);

	memset(pFrame,0,BRIOP_SEND_FRAME_LEN);

	fill_head(pFrame,"$USV",0,BRIOP_SEND_MSG_LEN);
	pData = &pFrame[9];

	pData[0]  = (*BrIOP_send_msg.b1_Wn_IOPCommOutage	&0x01);
	pData[0] |= (*BrIOP_send_msg.b1_Wn_IOPWarn			&0x01)<<1;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_IHCCommOutage	&0x01)<<2;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_IHCWarn			&0x01)<<3;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_IDUCommOutage	&0x01)<<4;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_IDUWarn			&0x01)<<5;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_MCUCommOutage	&0x01)<<6;
	pData[0] |= (*BrIOP_send_msg.b1_Wn_MCUWarn			&0x01)<<7;

	pData[1] =  *BrIOP_send_msg.u8_St_year		;
	pData[2] =  *BrIOP_send_msg.u8_St_month		;
	pData[3] =  *BrIOP_send_msg.u8_St_date		;
	pData[4] =  *BrIOP_send_msg.u8_St_hour		;
	pData[5] =  *BrIOP_send_msg.u8_St_minute	;
	pData[6] =  *BrIOP_send_msg.u8_St_second	;

	memcpy(&(pData[7]),(uint8 *)(BrIOP_send_msg.u16_St_speed),2);
	memcpy(&(pData[9]),(uint8 *)(BrIOP_send_msg.u16_St_heading),2);
	memcpy(&(pData[11]),(uint8 *)(BrIOP_send_msg.i16_St_rot),2);
	memcpy(&(pData[13]),(uint8 *)(BrIOP_send_msg.i16_St_pitch),2);
	memcpy(&(pData[15]),(uint8 *)(BrIOP_send_msg.i16_St_roll),2);
	memcpy(&(pData[17]),(uint8 *)(BrIOP_send_msg.i16_St_heaving),2);

	pData[19] = *BrIOP_send_msg.u8_St_longiSt		;
	pData[20] = *BrIOP_send_msg.u8_St_longiDeg		;
	pData[21] = *BrIOP_send_msg.u8_St_longiMin		;
	pData[22] = *BrIOP_send_msg.u8_St_longiSec		;
	pData[23] = *BrIOP_send_msg.u8_St_longiSecDec	;

	pData[24] = *BrIOP_send_msg.u8_St_latiSt		;
	pData[25] = *BrIOP_send_msg.u8_St_latiDeg		;
	pData[26] = *BrIOP_send_msg.u8_St_latiMin		;
	pData[27] = *BrIOP_send_msg.u8_St_latiSec		;
	pData[28] = *BrIOP_send_msg.u8_St_latiSecDec	;

	pData[29]  = (*BrIOP_send_msg.b1_St_emergencyMode	& 0x01) ;
	pData[29] |= (*BrIOP_send_msg.b1_St_emergencyStop	& 0x01)<<1 ;
	pData[29] |= (*BrIOP_send_msg.b1_St_localAuthority	& 0x01)<<2 ;
	pData[29] |= (*BrIOP_send_msg.b2_St_sailMode	& 0x03)<<4 ;
	pData[29] |= (*BrIOP_send_msg.b2_St_sailTask	& 0x03)<<6 ;

	pData[30]  = (*BrIOP_send_msg.b1_St_speedConstant	& 0x01) ;
	pData[30] |= (*BrIOP_send_msg.b1_St_headingConstant	& 0x01)<<1 ;
	pData[30] |= (*BrIOP_send_msg.b1_St_berthMode		& 0x01)<<2 ;

	pData[31]  = (*BrIOP_send_msg.b1_St_motorSt		& 0x01) ;
	pData[31] |= (*BrIOP_send_msg.b1_Wn_motor1Wn	& 0x01)<<1 ;
	pData[31] |= (*BrIOP_send_msg.b1_Wn_motor2Wn	& 0x01)<<2 ;

	memcpy(&(pData[32]),(uint8 *)(BrIOP_send_msg.u16_St_motor1Rpm),2);
	memcpy(&(pData[34]),(uint8 *)(BrIOP_send_msg.i16_St_motor1Gear),2);
	memcpy(&(pData[36]),(uint8 *)(BrIOP_send_msg.i16_St_motor1Rudder),2);

	memcpy(&(pData[38]),(uint8 *)(BrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[40]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[42]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Rudder),2);

	/*memcpy(&(pData[44]),(uint8 *)(BrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[46]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[48]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Rudder),2);

	memcpy(&(pData[50]),(uint8 *)(BrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[52]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[54]),(uint8 *)(BrIOP_send_msg.i16_St_motor2Rudder),2);*/

	if(ship_version == TIANJI_VERSION){
		pData[56]  = (*BrIOP_send_msg.b1_St_remoteKey		&0x01);
		pData[56] |= (*BrIOP_send_msg.b1_St_PORTMotorCharge	&0x01)<<1;
		pData[56] |= (*BrIOP_send_msg.b1_St_STBDMotorCharge	&0x01)<<2;
		pData[56] |= (*BrIOP_send_msg.b1_St_shorePower		&0x01)<<3;
		pData[56] |= (*BrIOP_send_msg.b1_St_PORTShoreCharge	&0x01)<<4;
		pData[56] |= (*BrIOP_send_msg.b1_St_STBDShoreCharge	&0x01)<<5;
		pData[56] |= (*BrIOP_send_msg.b1_St_ShoreChargeEnd	&0x01)<<6;
		pData[56] |= (*BrIOP_send_msg.b1_St_SystemPowerOn	&0x01)<<7;

		pData[57]  = (*BrIOP_send_msg.b3_St_supplySource	&0x07);
		pData[57] |= (*BrIOP_send_msg.b1_Wn_batLow			&0x01)<<3;
		pData[57] |= (*BrIOP_send_msg.b1_Wn_oilLow			&0x01)<<4;
		pData[57] |= (*BrIOP_send_msg.b1_Wn_volOver			&0x01)<<5;
		pData[57] |= (*BrIOP_send_msg.b1_Wn_volBelow		&0x01)<<6;
		pData[57] |= (*BrIOP_send_msg.b1_Wn_curOver			&0x01)<<7;

		pData[58]  = *BrIOP_send_msg.u8_St_PORTBatLvl	;
		pData[59]  = *BrIOP_send_msg.u8_St_STBDBatLvl	;
		pData[60]  = *BrIOP_send_msg.u8_St_PORTOilLvl	;
		pData[61]  = *BrIOP_send_msg.u8_St_STBDOilLvl	;
	}
	else if(ship_version == WATER_QUALITY){

		memcpy(&(pData[56]), (uint8 *)(BrIOP_send_msg.u16_St_BatteryVoltage), 2);
		memcpy(&(pData[58]), (uint8 *)(BrIOP_send_msg.u16_St_BatteryCurrent), 2);
		pData[60] = *BrIOP_send_msg.u8_St_BatteryRemainPercent;
		pData[61] = 0;

		pData[66] = *BrIOP_send_msg.u8_St_BatteryRemainTime_H;
		pData[67] = *BrIOP_send_msg.u8_St_BatteryRemainTime_M;
		pData[68] = *BrIOP_send_msg.u8_St_BatteryRemainTime_S;
	}

	
	
	pData[62]  = (*BrIOP_send_msg.b1_St_periPowK1	&0x01);
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK2	&0x01)<<1;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK3	&0x01)<<2;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK4	&0x01)<<3;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK5	&0x01)<<4;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK6	&0x01)<<5;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK7	&0x01)<<6;
	pData[62] |= (*BrIOP_send_msg.b1_St_periPowK8	&0x01)<<7;

	pData[63]  = (*BrIOP_send_msg.b1_St_periPowk9	&0x01);
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk10	&0x01)<<1;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk11	&0x01)<<2;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk12	&0x01)<<3;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk13	&0x01)<<4;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk14	&0x01)<<5;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk15	&0x01)<<6;
	pData[63] |= (*BrIOP_send_msg.b1_St_periPowk16	&0x01)<<7;

	pData[64]  = (*BrIOP_send_msg.b1_St_periPowk17	&0x01);
	pData[64] |= (*BrIOP_send_msg.b1_St_periPowk18	&0x01)<<1;
	pData[64] |= (*BrIOP_send_msg.b1_St_periPowk19	&0x01)<<2;
	pData[64] |= (*BrIOP_send_msg.b1_St_periPowk20	&0x01)<<3;


	pData[65]  = (*BrIOP_send_msg.b1_St_elecWindlass1St	&0x01);
	pData[65] |= (*BrIOP_send_msg.b1_St_elecWindlass2St	&0x01)<<1;
	pData[65] |= (*BrIOP_send_msg.b1_St_sprayStrip1St	&0x01)<<2;
	pData[65] |= (*BrIOP_send_msg.b1_St_sprayStrip2St	&0x01)<<3;


	pFrame[BRIOP_SEND_FRAME_LEN-3] = '*';

	crc16 = calcCRC(0xffff,pFrame,BRIOP_SEND_FRAME_LEN-2);

	pFrame[BRIOP_SEND_FRAME_LEN-2] = crc16 & 0x00ff			;
	pFrame[BRIOP_SEND_FRAME_LEN-1] = (crc16 & 0xff00)>>8	;

	iSendLen=sendto(BrIOP_udp_sockid,(int8 *)&pFrame[0],BRIOP_SEND_FRAME_LEN,0,(struct sockaddr*)&server, sizeof(server));

	if(iSendLen != BRIOP_SEND_FRAME_LEN)
	{
		return FALSE;
	}
	else
	{
		monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BRIOP_SN].send_ok_number++;
		return TRUE;
	}
}


void runBRIOPSendMsg( void* )
{
	static uint16 frameCntr=0;
	frameCntr++;
	BRIOP_sendMsg(frameCntr);
}

void BRIOPCommCalInit( void )
{
	addTask(3,BRIOP_CommCal,(void*)0);
}




