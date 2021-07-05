/*==========================================================*
 * ģ��˵��: uart_deal_DRadio.cpp                             *
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
#include "../include/uart_deal_DRadio.h"
/******************************  Local Variable  *****************************/
int8    re_buff[MAX_UART_BUFF_LEN];	//���������ݻ���
uint16  re_end_ptr=0;
uint16	re_start_ptr=0;
uint8	re_sign=0;
COMM_SIGN dradio_sign={0,0};
DRIOP_REV_MSG DrIOP_rev_msg;
DRIOP_SEND_MSG DrIOP_send_msg;
uint8 logDRadio_sign = 0;
uint32 TEST_DEST_0 = 0;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
int8 uart_dradio_init( void );
int8 checkDrIOPHead(int8 *buff);
int8 checkDrIOPCrc(int8 *buff);
void DRIOP_CommCal(void *);
void anayticalDrIOP(uint8 *buff);
int8 uart_dradio_rec_report(UART_TYPE uartid);
#define DRIOP_REV_FRAME_LEN 50
#define DRIOP_SEND_FRAME_LEN 82
#define DRIOP_SEND_MSG_LEN	70
#define DRIOP_DISCONNECT_MAX 20

/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void *uart_deal_dradio( void *aa )
{
	uint8 loop_i=0;
	if(uart_dradio_init()<0)
	{
//		SysLogMsgPost("���ֵ�̨���ڳ�ʼ��ʧ��");
		return((void*)0);
	}
	else{
		comm_time_return(&(dradio_sign.timer), &(dradio_sign.comm_sign));//���ֵ�̨ͨѶ��ʱ��ʱ��������ʼ����λ
	}
	for (;;)
	{
			uart_dradio_rec_report(UART2_Fd);
			
			//	DRIOP_CommCal();
			sleep_1(100);
	}
	
}

//���ڳ�ʼ��
int8 uart_dradio_init( void )
{
	int8 iret = 0;
	memset(re_buff,0,MAX_UART_BUFF_LEN);
	UART2_Fd = open_port(COM2);
	if(UART2_Fd <0)
	{
		iret = -1;
	}
	if(set_com_config(UART2_Fd,19200,8,'N',1)<0)
	{
		iret = -1;
	}
	return iret;
}

//���մ������ݽ�֡
int8 uart_dradio_rec_report(UART_TYPE uartid)
{
	int8 buff[MAX_UART_BUFF_LEN];
	int16 ret;
	int ico;
	static int jco=0;
	int loop_i;

	ret = read_uart(uartid,buff,MAX_UART_BUFF_LEN);

	if(ret > 0)
	{
		for(ico=0;ico<ret;ico++)
		{
			if(((buff[ico]=='$')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(re_sign == 1))
			{
				re_sign = 1;
				re_buff[jco++] = buff[ico];
				if(jco>=DRIOP_REV_FRAME_LEN)
				{
					jco=0;
					re_sign = 0;
					//check frame head & check frame crc
					if(checkDrIOPHead(re_buff)==0 )
					{
						return 0;
					}
					if(checkDrIOPCrc(re_buff)== 0)
					{
						monitor_all_inf.monitor_comm_inf[MONITOR_COMM_DRIOP_SN].rec_error_number++;		//���մ������
						return 0;
					}

					//frame analytical
					anayticalDrIOP((uint8*)&re_buff[9]);
					comm_time_return(&(dradio_sign.timer),&(dradio_sign.comm_sign));
					monitor_all_inf.monitor_comm_inf[MONITOR_COMM_DRIOP_SN].rec_ok_number++;			//���ճɹ�����
				}
			}
		}
	}
	return 0;
}

//У�鱨��ͷ
int8 checkDrIOPHead(int8 *buff)
{
	if((buff[0]=='$')&&(buff[1]=='U')&&(buff[2]=='S')&&(buff[3]=='V')&&(buff[DRIOP_REV_FRAME_LEN-3] ='*'))
		return 1;
	else
		return 0;
}

//CRCУ��
int8 checkDrIOPCrc(int8 *buff)
{
	uint16 crcRead;
	uint16 crcCalc;
	crcRead = u8tou16((uint8*)&buff[DRIOP_REV_FRAME_LEN-2]);
	crcCalc = calcCRC(0xffff,(uint8*)buff,DRIOP_REV_FRAME_LEN-2);
	if(crcRead != crcCalc)
		return 0;

	return 1;

}

//���Ľ���
void anayticalDrIOP(uint8 *buff)
{
	DrIOP_rev_msg.b1_Wn_warn			=  ( buff[0]	& 0x01	)>>0	;
	DrIOP_rev_msg.b1_Wn_mainBoardPower	=  ( buff[0]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Wn_mainBoardTempe	=  ( buff[0]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Wn_int				=  ( buff[0]	& 0x08	)>>3	;
	DrIOP_rev_msg.b1_Wn_CANComm			=  ( buff[0]	& 0x10	)>>4	;
	DrIOP_rev_msg.b1_Wn_SCIA			=  ( buff[0]	& 0x20	)>>5	;
	DrIOP_rev_msg.b1_Wn_SCIB			=  ( buff[0]	& 0x40	)>>6	;

	DrIOP_rev_msg.b1_Wn_externSP1		=  ( buff[1]	& 0x01	)>>0	;
	DrIOP_rev_msg.b1_Wn_externSP2		=  ( buff[1]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Wn_externSP3		=  ( buff[1]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Wn_externSP4		=  ( buff[1]	& 0x08	)>>3	;
	DrIOP_rev_msg.b1_Wn_externSP5		=  ( buff[1]	& 0x10	)>>4	;
	DrIOP_rev_msg.b1_Wn_externSP6		=  ( buff[1]	& 0x20	)>>5	;
	DrIOP_rev_msg.b1_Wn_externSP7		=  ( buff[1]	& 0x40	)>>6	;
	DrIOP_rev_msg.b1_Wn_externSP8		=  ( buff[1]	& 0x80	)>>7	;

	DrIOP_rev_msg.b2_Cmd_motor			=    (buff[2] & 0x03)			;
	DrIOP_rev_msg.b1_Cmd_SystemRestart  =	 (buff[2] & 0x80)>>7		;

	DrIOP_rev_msg.b1_Cmd_emergencyMode  =  ( buff[3]	& 0x01	)>>0	;
	DrIOP_rev_msg.b1_Cmd_emergencyStop  =  ( buff[3]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Cmd_getAuthority	=  ( buff[3]	& 0x04	)>>2	;
	DrIOP_rev_msg.b2_Cmd_sailMode		=  ( buff[3]	& 0x30	)>>4	;
	DrIOP_rev_msg.b2_Cmd_sailTask		=  ( buff[3]	& 0xc0	)>>6	;

	DrIOP_rev_msg.b1_Cmd_speedConstant	=  ( buff[4]	& 0x01	)>>0	;
	DrIOP_rev_msg.b1_Cmd_headingConstant=  ( buff[4]	& 0x02	)>>1	;	
	DrIOP_rev_msg.b1_Cmd_berthMode		=  ( buff[4]	& 0x04	)>>2	;

	DrIOP_rev_msg.i16_Cmd_joy1Y	 = u8toi16(&buff[5]);
	DrIOP_rev_msg.i16_Cmd_joy1X	 = u8toi16(&buff[7]);
	DrIOP_rev_msg.i16_Cmd_joy1Z = u8toi16(&buff[9]);

	DrIOP_rev_msg.i16_Cmd_joy2Y  = u8toi16(&buff[11]);
	DrIOP_rev_msg.i16_Cmd_joy2X	= u8toi16(&buff[13]);
	DrIOP_rev_msg.i16_Cmd_joy2Z = u8toi16(&buff[15]);

	DrIOP_rev_msg.b1_Cmd_periPowerS1	=  ( buff[23]	& 0x01	)>>0	;				
	DrIOP_rev_msg.b1_Cmd_periPowerS2	=  ( buff[23]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Cmd_periPowerS3	=  ( buff[23]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Cmd_periPowerS4	=  ( buff[23]	& 0x08	)>>3	;
	DrIOP_rev_msg.b1_Cmd_periPowerS5	=  ( buff[23]	& 0x10	)>>4	;
	DrIOP_rev_msg.b1_Cmd_periPowerS6	=  ( buff[23]	& 0x20	)>>5	;
	DrIOP_rev_msg.b1_Cmd_periPowerS7	=  ( buff[23]	& 0x40	)>>6	;
	DrIOP_rev_msg.b1_Cmd_periPowerS8	=  ( buff[23]	& 0x80	)>>7	;

	DrIOP_rev_msg.b1_Cmd_periPowerS9	=  ( buff[24]	& 0x01	)>>0	;	
	DrIOP_rev_msg.b1_Cmd_periPowerS10	=  ( buff[24]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Cmd_periPowerS11	=  ( buff[24]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Cmd_periPowerS12	=  ( buff[24]	& 0x08	)>>3	;
	DrIOP_rev_msg.b1_Cmd_periPowerS13	=  ( buff[24]	& 0x10	)>>4	;
	DrIOP_rev_msg.b1_Cmd_periPowerS14	=  ( buff[24]	& 0x20	)>>5	;
	DrIOP_rev_msg.b1_Cmd_periPowerS15	=  ( buff[24]	& 0x40	)>>6	;
	DrIOP_rev_msg.b1_Cmd_periPowerS16	=  ( buff[24]	& 0x80	)>>7	;

	DrIOP_rev_msg.b1_Cmd_periPowerS17	=  ( buff[25]	& 0x01	)>>0	;	
	DrIOP_rev_msg.b1_Cmd_periPowerS18	=  ( buff[25]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Cmd_periPowerS19	=  ( buff[25]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Cmd_periPowerS20	=  ( buff[25]	& 0x08	)>>3	;

	DrIOP_rev_msg.b1_Cmd_elecWindlass1OnOff		=  ( buff[26]	& 0x01	)>>0	;	
	DrIOP_rev_msg.b1_Cmd_elecWindlass1UpDown	=  ( buff[26]	& 0x02	)>>1	;
	DrIOP_rev_msg.b1_Cmd_elecWindlass2OnOff		=  ( buff[26]	& 0x04	)>>2	;
	DrIOP_rev_msg.b1_Cmd_elecWindlass2UpDown	=  ( buff[26]	& 0x08	)>>3	;
	DrIOP_rev_msg.b1_Cmd_sprayStrip1OnOff		=  ( buff[26]	& 0x10	)>>4	;
	DrIOP_rev_msg.b1_Cmd_sprayStrip1UpDown		=  ( buff[26]	& 0x20	)>>5	;
	DrIOP_rev_msg.b1_Cmd_sprayStrip2OnOff		=  ( buff[26]	& 0x40	)>>6	;
	DrIOP_rev_msg.b1_Cmd_sprayStrip2UpDown		=  ( buff[26]	& 0x80	)>>7	;

	//32
	DrIOP_rev_msg.u16_version = u8tou16(&buff[32]);
	DrIOP_rev_msg.u16_svn	  = u8tou16(&buff[34]);
	DrIOP_rev_msg.u16_crc	  = u8tou16(&buff[36]);
}


void DRIOPCommCalInit(void)
{
	addTask(3,DRIOP_CommCal,(void*)0);	//200ms ����
}

//ͳ��ͨѶ��ʱ
void DRIOP_CommCal(void *)
{
	comm_time_cal(DRIOP_DISCONNECT_MAX, &(dradio_sign.timer), &(dradio_sign.comm_sign));

	if (logDRadio_sign != dradio_sign.comm_sign && poweron_init) //�ϵ��ʼ����ʼ����ͨѶ��ʱ
	{
		switch(dradio_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	//SysLogMsgPost("���ֵ�̨ͨѶ�ָ�");
								//SysPubMsgPost("���ֵ�̨ͨѶ�ָ�");
								WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_DRADIO_TIMEOUT, WARN_OFF);
			break;
		case COMM_CONNECT_FAIL:	//SysLogMsgPost("���ֵ�̨ͨѶ�ж�");
								//SysPubMsgPost("���ֵ�̨ͨѶ�ж�");
								WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_DRADIO_TIMEOUT, WARN_ON);
			break;
		default:
			break;
		}
	}
	logDRadio_sign = dradio_sign.comm_sign;
}



//����ָ���ʼ��
void DRIOP_sdMsg_Init(void)
{
	memset(&DrIOP_send_msg,0,sizeof(DRIOP_SEND_MSG));
	DrIOP_send_msg.b1_Wn_IOPCommOutage	= & IOP_comm_sign.comm_sign;
	DrIOP_send_msg.b1_Wn_IOPWarn		= & IOP_rev_msg.b1_Wn_warn;
	DrIOP_send_msg.b1_Wn_IHCCommOutage  = & IHC_comm_sign.comm_sign;
	DrIOP_send_msg.b1_Wn_IHCWarn		= & IHC_rev_msg.mid_st.b1_Wn_CommonWarn;
	DrIOP_send_msg.b1_Wn_IDUCommOutage  = & IDU_comm_sign.comm_sign;
	DrIOP_send_msg.b1_Wn_IDUWarn		= & IDU_rev_msg.b1_Wn_warn;
	DrIOP_send_msg.b1_Wn_MCUCommOutage  = (uint8*)&TEST_DEST_0;
	DrIOP_send_msg.b1_Wn_MCUWarn		= (uint8*)&TEST_DEST_0;

	DrIOP_send_msg.u8_St_year			= &state_signal.time.u8_year	;///&ins_msg.u8_year;
	DrIOP_send_msg.u8_St_month			= &state_signal.time.u8_month	;//&ins_msg.u8_month;
	DrIOP_send_msg.u8_St_date			= &state_signal.time.u8_date	;//&ins_msg.u8_day;

	DrIOP_send_msg.u8_St_hour			= &state_signal.time.u8_hour	;//&ins_msg.u8_hour;
	DrIOP_send_msg.u8_St_minute			= &state_signal.time.u8_minute	;//&ins_msg.u8_minute;
	DrIOP_send_msg.u8_St_second			= &state_signal.time.u8_second	;//&ins_msg.u8_second;

	DrIOP_send_msg.u16_St_speed			= &ins_msg.u16_speed;
	DrIOP_send_msg.u16_St_heading		= &ins_msg.u16_heading;

	DrIOP_send_msg.i16_St_rot			= &ins_msg.i16_rot;
	DrIOP_send_msg.i16_St_pitch			= &ins_msg.i16_pitch;
	DrIOP_send_msg.i16_St_roll			= &ins_msg.i16_roll;
	DrIOP_send_msg.i16_St_heaving		= &ins_msg.i16_heaving;

	DrIOP_send_msg.u8_St_longiSt		= &ins_msg.u8_longiSt		;
	DrIOP_send_msg.u8_St_longiDeg		= &ins_msg.u8_longiDeg		;
	DrIOP_send_msg.u8_St_longiMin		= &ins_msg.u8_longiMin		;
	DrIOP_send_msg.u8_St_longiSec		= &ins_msg.u8_longiSec		;
	DrIOP_send_msg.u8_St_longiSecDec	= &ins_msg.u8_longiSecDec	;

	DrIOP_send_msg.u8_St_latiSt			= &ins_msg.u8_latiSt		;
	DrIOP_send_msg.u8_St_latiDeg		= &ins_msg.u8_latiDeg		;
	DrIOP_send_msg.u8_St_latiMin		= &ins_msg.u8_latiMin		;
	DrIOP_send_msg.u8_St_latiSec		= &ins_msg.u8_latiSec		;
	DrIOP_send_msg.u8_St_latiSecDec		= &ins_msg.u8_latiSecDec	;

	DrIOP_send_msg.b1_St_emergencyMode		= &command_signal.sail_mode_cmd.b1_emergencyMode	;
	DrIOP_send_msg.b1_St_emergencyStop		= &jet_system.b1_cmd_emergencyStop				;
	DrIOP_send_msg.b1_St_localAuthority		= &command_signal.b1_authority						;
	DrIOP_send_msg.b2_St_sailMode			= &command_signal.sail_mode_cmd.b2_sailMode			;
	DrIOP_send_msg.b2_St_sailTask			= &command_signal.sail_feedBack.b2_sailTask 		;

	DrIOP_send_msg.b1_St_speedConstant		= &command_signal.func_mode_cmd.b1_speedConstant	;
	DrIOP_send_msg.b1_St_headingConstant	= &command_signal.func_mode_cmd.b1_headingConstant	;	
	DrIOP_send_msg.b1_St_berthMode			= &command_signal.func_mode_cmd.b1_dock_cmd			;
	DrIOP_send_msg.b1_autoReturn			= &command_signal.func_mode_cmd.b1_autoReturn		;

	DrIOP_send_msg.b1_St_motorSt			= (uint8*)&IHC_rev_msg.mid_st.b1_St_MotorOn		;
	DrIOP_send_msg.b1_Wn_motor1Wn			= (uint8*)&IHC_rev_msg.b1_Wn_PORTMotorWarn		;
	DrIOP_send_msg.b1_Wn_motor2Wn			= (uint8*)&IHC_rev_msg.b1_Wn_STBDMotorWarn		;


	DrIOP_send_msg.u16_St_motor1Rpm			= &IHC_rev_msg.u16_St_Motor1Rpm		;	//��Ҫת��
	DrIOP_send_msg.i16_St_motor1Gear		= &IHC_rev_msg.i16_St_Motor1Gear	;
	DrIOP_send_msg.i16_St_motor1Rudder		= &IHC_rev_msg.i16_St_Motor1Rudder	;

	DrIOP_send_msg.u16_St_motor2Rpm			= &IHC_rev_msg.u16_St_Motor2Rpm		;	//��Ҫת��
	DrIOP_send_msg.i16_St_motor2Gear		= &IHC_rev_msg.i16_St_Motor2Gear	;
	DrIOP_send_msg.i16_St_motor2Rudder		= &IHC_rev_msg.i16_St_Motor2Rudder	;

	DrIOP_send_msg.u16_St_motor3Rpm			= (uint16*)&TEST_DEST_0;	//��Ҫת��
	DrIOP_send_msg.i16_St_motor3Gear		= (int16*)&TEST_DEST_0;
	DrIOP_send_msg.i16_St_motor3Rudder		= (int16* )&TEST_DEST_0;

	DrIOP_send_msg.u16_St_motor4Rpm			= (uint16*)&TEST_DEST_0;	//��Ҫת��
	DrIOP_send_msg.i16_St_motor4Gear		= (int16*)&TEST_DEST_0;
	DrIOP_send_msg.i16_St_motor4Rudder		= (int16* )&TEST_DEST_0;

	DrIOP_send_msg.b1_St_remoteKey			= &IDU_rev_msg.b1_St_remoteKey;
	DrIOP_send_msg.b1_St_PORTMotorCharge	= &IDU_rev_msg.b1_St_PORTMotorCharge;
	DrIOP_send_msg.b1_St_STBDMotorCharge	= &IDU_rev_msg.b1_St_STBDMotorCharge;
	DrIOP_send_msg.b1_St_shorePower			= &IDU_rev_msg.b1_St_shorePower;
	DrIOP_send_msg.b1_St_PORTShoreCharge	= &IDU_rev_msg.b1_St_PORTShoreCharge;
	DrIOP_send_msg.b1_St_STBDShoreCharge	= &IDU_rev_msg.b1_St_STBDShoreCharge;
	DrIOP_send_msg.b1_St_ShoreChargeEnd		= &IDU_rev_msg.b1_St_ShoreChargeEnd;
	DrIOP_send_msg.b1_St_SystemPowerOn		= &IDU_rev_msg.b1_St_SystemPowerOn;

	DrIOP_send_msg.b3_St_supplySource		= &IDU_rev_msg.b3_St_supplySource;
	DrIOP_send_msg.b1_Wn_batLow				= &IDU_rev_msg.b1_Wn_batLow;
	DrIOP_send_msg.b1_Wn_oilLow				= &IDU_rev_msg.b1_Wn_oilLow;
	DrIOP_send_msg.b1_Wn_volOver			= &IDU_rev_msg.midMsg.b1_Wn_volOver;
	DrIOP_send_msg.b1_Wn_volBelow			= &IDU_rev_msg.midMsg.b1_Wn_volBelow;
	DrIOP_send_msg.b1_Wn_curOver			= &IDU_rev_msg.midMsg.b1_Wn_curOver;

	DrIOP_send_msg.u8_St_PORTBatLvl			= &IDU_rev_msg.u8_St_PORTBatLvl;
	DrIOP_send_msg.u8_St_STBDBatLvl			= &IDU_rev_msg.u8_St_STBDBatLvl;
	DrIOP_send_msg.u8_St_PORTOilLvl			= &IDU_rev_msg.u8_St_PortOilLvl;
	DrIOP_send_msg.u8_St_STBDOilLvl			= &IDU_rev_msg.u8_St_STBDOilLvl;

	DrIOP_send_msg.u16_St_BatteryVoltage =		&IHC_rev_msg.tmp_u8_St_BatteryVoltage;
	DrIOP_send_msg.u16_St_BatteryCurrent =		&IHC_rev_msg.tmp_u8_St_BatteryCurrent;
	DrIOP_send_msg.u8_St_BatteryRemainTime_H	=  &IHC_rev_msg.u8_St_BatteryRemainTime_H	;
	DrIOP_send_msg.u8_St_BatteryRemainTime_M	=  &IHC_rev_msg.u8_St_BatteryRemainTime_M	;
	DrIOP_send_msg.u8_St_BatteryRemainTime_S	=  &IHC_rev_msg.u8_St_BatteryRemainTime_S	;
	DrIOP_send_msg.u8_st_BatteryRemainPercent = &IHC_rev_msg.u8_St_BatteryLevel;




	DrIOP_send_msg.b1_St_periPowK1			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[0]);
	DrIOP_send_msg.b1_St_periPowK2			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[1]);
	DrIOP_send_msg.b1_St_periPowK3			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[2]);
	DrIOP_send_msg.b1_St_periPowK4			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[3]);
	DrIOP_send_msg.b1_St_periPowK5			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[4]);
	DrIOP_send_msg.b1_St_periPowK6			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[5]);
	DrIOP_send_msg.b1_St_periPowK7			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[6]);
	DrIOP_send_msg.b1_St_periPowK8			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[7]);

	DrIOP_send_msg.b1_St_periPowk9			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[8]);
	DrIOP_send_msg.b1_St_periPowk10			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[9]);
	DrIOP_send_msg.b1_St_periPowk11			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[10]);
	DrIOP_send_msg.b1_St_periPowk12			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[11]);
	DrIOP_send_msg.b1_St_periPowk13			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[12]);
	DrIOP_send_msg.b1_St_periPowk14			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[13]);
	DrIOP_send_msg.b1_St_periPowk15			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[14]);
	DrIOP_send_msg.b1_St_periPowk16			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[15]);

	DrIOP_send_msg.b1_St_periPowk17			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[16]);
	DrIOP_send_msg.b1_St_periPowk18			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[17]);
	DrIOP_send_msg.b1_St_periPowk19			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[18]);
	DrIOP_send_msg.b1_St_periPowk20			= DrIOP_Switch_connect(peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[19]);

	DrIOP_send_msg.b1_St_elecWindlass1St	= &IHC_rev_msg.b1_St_elecWindlass1OnOff;
	DrIOP_send_msg.b1_St_elecWindlass2St	= &IHC_rev_msg.b1_St_elecWindlass2OnOff;
	DrIOP_send_msg.b1_St_sprayStrip1St		= &IHC_rev_msg.b1_St_sprayStrip1OnOff;
	DrIOP_send_msg.b1_St_sprayStrip2St		= &IHC_rev_msg.b1_St_sprayStrip2OnOff;
}



void DRIOP_sendMsg(uint16 frameCntr)
{
	uint8 *pData		;
	uint8  pFrame[DRIOP_SEND_FRAME_LEN]	;
	uint16 crc16;
	memset(pFrame,0,DRIOP_SEND_FRAME_LEN);

	fill_head(pFrame,"$USV",frameCntr,DRIOP_SEND_MSG_LEN);
	pData = &pFrame[9];

	pData[0]  = (*DrIOP_send_msg.b1_Wn_IOPCommOutage	&0x01);
	pData[0] |= (*DrIOP_send_msg.b1_Wn_IOPWarn			&0x01)<<1;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_IHCCommOutage	&0x01)<<2;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_IHCWarn			&0x01)<<3;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_IDUCommOutage	&0x01)<<4;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_IDUWarn			&0x01)<<5;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_MCUCommOutage	&0x01)<<6;
	pData[0] |= (*DrIOP_send_msg.b1_Wn_MCUWarn			&0x01)<<7;

	pData[1] =  *DrIOP_send_msg.u8_St_year		;
	pData[2] =  *DrIOP_send_msg.u8_St_month		;
	pData[3] =  *DrIOP_send_msg.u8_St_date		;
	pData[4] =  *DrIOP_send_msg.u8_St_hour		;
	pData[5] =  *DrIOP_send_msg.u8_St_minute	;
	pData[6] =  *DrIOP_send_msg.u8_St_second	;

	memcpy(&(pData[7]),(uint8 *)(DrIOP_send_msg.u16_St_speed),2);
	memcpy(&(pData[9]),(uint8 *)(DrIOP_send_msg.u16_St_heading),2);
	memcpy(&(pData[11]),(uint8 *)(DrIOP_send_msg.i16_St_rot),2);
	memcpy(&(pData[13]),(uint8 *)(DrIOP_send_msg.i16_St_pitch),2);
	memcpy(&(pData[15]),(uint8 *)(DrIOP_send_msg.i16_St_roll),2);
	memcpy(&(pData[17]),(uint8 *)(DrIOP_send_msg.i16_St_heaving),2);

	pData[19] = *DrIOP_send_msg.u8_St_longiSt		;
	pData[20] = *DrIOP_send_msg.u8_St_longiDeg		;
	pData[21] = *DrIOP_send_msg.u8_St_longiMin		;
	pData[22] = *DrIOP_send_msg.u8_St_longiSec		;
	pData[23] = *DrIOP_send_msg.u8_St_longiSecDec	;

	pData[24] = *DrIOP_send_msg.u8_St_latiSt		;
	pData[25] = *DrIOP_send_msg.u8_St_latiDeg		;
	pData[26] = *DrIOP_send_msg.u8_St_latiMin		;
	pData[27] = *DrIOP_send_msg.u8_St_latiSec		;
	pData[28] = *DrIOP_send_msg.u8_St_latiSecDec	;

	pData[29]  = (*DrIOP_send_msg.b1_St_emergencyMode	& 0x01) ;
	pData[29] |= (*DrIOP_send_msg.b1_St_emergencyStop	& 0x01)<<1 ;
	pData[29] |= (*DrIOP_send_msg.b1_St_localAuthority	& 0x01)<<2 ;
	pData[29] |= (*DrIOP_send_msg.b2_St_sailMode	& 0x03)<<4 ;
	pData[29] |= (*DrIOP_send_msg.b2_St_sailTask	& 0x03)<<6 ;

	pData[30]  = (*DrIOP_send_msg.b1_St_speedConstant	& 0x01) ;
	pData[30] |= (*DrIOP_send_msg.b1_St_headingConstant	& 0x01)<<1 ;
	pData[30] |= (*DrIOP_send_msg.b1_St_berthMode		& 0x01)<<2 ;

	pData[31]  = (*DrIOP_send_msg.b1_St_motorSt		& 0x01) ;
	pData[31] |= (*DrIOP_send_msg.b1_Wn_motor1Wn	& 0x01)<<1 ;
	pData[31] |= (*DrIOP_send_msg.b1_Wn_motor2Wn	& 0x01)<<2 ;

	memcpy(&(pData[32]),(uint8 *)(DrIOP_send_msg.u16_St_motor1Rpm),2);
	memcpy(&(pData[34]),(uint8 *)(DrIOP_send_msg.i16_St_motor1Gear),2);
	memcpy(&(pData[36]),(uint8 *)(DrIOP_send_msg.i16_St_motor1Rudder),2);

	memcpy(&(pData[38]),(uint8 *)(DrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[40]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[42]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Rudder),2);

	/*memcpy(&(pData[44]),(uint8 *)(DrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[46]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[48]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Rudder),2);

	memcpy(&(pData[50]),(uint8 *)(DrIOP_send_msg.u16_St_motor2Rpm),2);
	memcpy(&(pData[52]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Gear),2);
	memcpy(&(pData[54]),(uint8 *)(DrIOP_send_msg.i16_St_motor2Rudder),2);*/

	//pData[56]  = (*DrIOP_send_msg.b1_St_remoteKey		&0x01);
	//pData[56] |= (*DrIOP_send_msg.b1_St_PORTMotorCharge	&0x01)<<1;
	//pData[56] |= (*DrIOP_send_msg.b1_St_STBDMotorCharge	&0x01)<<2;
	//pData[56] |= (*DrIOP_send_msg.b1_St_shorePower		&0x01)<<3;
	//pData[56] |= (*DrIOP_send_msg.b1_St_PORTShoreCharge	&0x01)<<4;
	//pData[56] |= (*DrIOP_send_msg.b1_St_STBDShoreCharge	&0x01)<<5;
	//pData[56] |= (*DrIOP_send_msg.b1_St_ShoreChargeEnd	&0x01)<<6;
	//pData[56] |= (*DrIOP_send_msg.b1_St_SystemPowerOn	&0x01)<<7;

	//pData[57]  = (*DrIOP_send_msg.b3_St_supplySource	&0x07);
	//pData[57] |= (*DrIOP_send_msg.b1_Wn_batLow			&0x01)<<3;
	//pData[57] |= (*DrIOP_send_msg.b1_Wn_oilLow			&0x01)<<4;
	//pData[57] |= (*DrIOP_send_msg.b1_Wn_volOver			&0x01)<<5;
	//pData[57] |= (*DrIOP_send_msg.b1_Wn_volBelow		&0x01)<<6;
	//pData[57] |= (*DrIOP_send_msg.b1_Wn_curOver			&0x01)<<7;

	//pData[58]  = *DrIOP_send_msg.u8_St_PORTBatLvl	;
	//pData[59]  = *DrIOP_send_msg.u8_St_STBDBatLvl	;
	//pData[60]  = *DrIOP_send_msg.u8_St_PORTOilLvl	;
	//pData[61]  = *DrIOP_send_msg.u8_St_STBDOilLvl	;

	//pData[56] = *DrIOP_send_msg.u16_St_BatteryVoltage;
	//pData[57] = *DrIOP_send_msg.u16_St_BatteryCurrent;
	//pData[58] = *DrIOP_send_msg.u8_St_BatteryRemainTime_H;
	//pData[59] = *DrIOP_send_msg.u8_St_BatteryRemainTime_M;
	//pData[60] = *DrIOP_send_msg.u8_St_BatteryRemainTime_S;
	//pData[61] = 0;
	
	memcpy(&(pData[56]), (uint8 *)(DrIOP_send_msg.u16_St_BatteryVoltage), 2);
	memcpy(&(pData[58]), (uint8 *)(DrIOP_send_msg.u16_St_BatteryCurrent), 2);
	pData[60] = *DrIOP_send_msg.u8_St_BatteryRemainTime_S;

	pData[66] = *DrIOP_send_msg.u8_St_BatteryRemainTime_H;
	pData[67] = *DrIOP_send_msg.u8_St_BatteryRemainTime_M;
	pData[68] = *DrIOP_send_msg.u8_St_BatteryRemainTime_S;

	
	pData[62]  = (*DrIOP_send_msg.b1_St_periPowK1	&0x01);
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK2	&0x01)<<1;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK3	&0x01)<<2;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK4	&0x01)<<3;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK5	&0x01)<<4;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK6	&0x01)<<5;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK7	&0x01)<<6;
	pData[62] |= (*DrIOP_send_msg.b1_St_periPowK8	&0x01)<<7;

	pData[63]  = (*DrIOP_send_msg.b1_St_periPowk9	&0x01);
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk10	&0x01)<<1;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk11	&0x01)<<2;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk12	&0x01)<<3;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk13	&0x01)<<4;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk14	&0x01)<<5;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk15	&0x01)<<6;
	pData[63] |= (*DrIOP_send_msg.b1_St_periPowk16	&0x01)<<7;

	pData[64]  = (*DrIOP_send_msg.b1_St_periPowk17	&0x01);
	pData[64] |= (*DrIOP_send_msg.b1_St_periPowk18	&0x01)<<1;
	pData[64] |= (*DrIOP_send_msg.b1_St_periPowk19	&0x01)<<2;
	pData[64] |= (*DrIOP_send_msg.b1_St_periPowk20	&0x01)<<3;


	pData[65]  = (*DrIOP_send_msg.b1_St_elecWindlass1St	&0x01);
	pData[65] |= (*DrIOP_send_msg.b1_St_elecWindlass2St	&0x01)<<1;
	pData[65] |= (*DrIOP_send_msg.b1_St_sprayStrip1St	&0x01)<<2;
	pData[65] |= (*DrIOP_send_msg.b1_St_sprayStrip2St	&0x01)<<3;


	pFrame[DRIOP_SEND_FRAME_LEN-3] = '*';

	crc16 = calcCRC(0xffff,pFrame,DRIOP_SEND_FRAME_LEN-2);

	pFrame[DRIOP_SEND_FRAME_LEN-2] = crc16 & 0x00ff			;
	pFrame[DRIOP_SEND_FRAME_LEN-1] = (crc16 & 0xff00)>>8	;

	write_uart(UART2_Fd,(int8*)&pFrame[0],DRIOP_SEND_FRAME_LEN);

}


void initDRIOPsendTask()
{
	DRIOP_sdMsg_Init();
	//addTask(3,runDRIOPSendMsg,(void*)0);	//����200ms ����
}

void runDRIOPSendMsg(void *)
{
	static uint16 frameCntr=0;
	frameCntr++;
	DRIOP_sendMsg(frameCntr);
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_DRIOP_SN].send_ok_number++;
}

uint8* DrIOP_Switch_connect( int32 connectNum )
{
	switch(connectNum)
	{
	case 0:
		return (uint8*)&TEST_DEST_0;
	case 1:
		return &IDU_rev_msg.b1_St_periPowK1;
		break;
	case 2:
		return &IDU_rev_msg.b1_St_periPowK2;
		break;
	case 3:
		return &IDU_rev_msg.b1_St_periPowK3;
		break;
	case 4:
		return &IDU_rev_msg.b1_St_periPowK4;
		break;
	case 5:
		return &IDU_rev_msg.b1_St_periPowK5;
		break;
	case 6:
		return &IDU_rev_msg.b1_St_periPowK6;
		break;
	case 7:
		return &IDU_rev_msg.b1_St_periPowK7;
		break;
	case 8:
		return &IDU_rev_msg.b1_St_periPowK8;
		break;
	case 9:
		return &IDU_rev_msg.b1_St_periPowk9;
		break;
	case 10:
		return &IDU_rev_msg.b1_St_periPowk10;
		break;
	case 11:
		return &IDU_rev_msg.b1_St_periPowk11;
		break;
	case 12:
		return &IDU_rev_msg.b1_St_periPowk12;
		break;
	case 13:
		return &IDU_rev_msg.b1_St_periPowk13;
		break;
	case 14:
		return &IDU_rev_msg.b1_St_periPowk14;
		break;
	case 15:
		return &IDU_rev_msg.b1_St_periPowk15;
		break;
	case 16:
		return &IDU_rev_msg.b1_St_periPowk16;
		break;
	case 17:
		return &IDU_rev_msg.b1_St_periPowk17;
		break;
	case 18:
		return &IDU_rev_msg.b1_St_periPowk18;
		break;
	case 19:
		return &IDU_rev_msg.b1_St_periPowk19;
		break;
	case 20:
		return &IDU_rev_msg.b1_St_periPowk20;
		break;
	default:
		return (uint8*)&TEST_DEST_0;
		break;
	}
}

