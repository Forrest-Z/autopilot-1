/*==========================================================*
 * ģ��˵��: uart_deal_BD.cpp                               *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
 * ����ʱ��: 2018/4/26 13:04:18                             *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/uart_deal_BD.h"
#include "../include/usv_include.h"
/******************************  Local Variable  *****************************/
#define REC_BD_MSG_LEN 48
int8	bd_re_buff[200];		//���������ݻ���
uint8	bd_re_sign=0;
BD_SAIL_TASK	bd_sailTask;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
int8 uart_BD_init(void);
int8 uart_BD_rec_report(UART_TYPE uartid);


/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

//BD���Ľ����߳�
void *uart_deal_BD(void *aa)
{
	if(uart_BD_init()<0)
	{
	//	SysLogMsgPost("�������ڳ�ʼ��ʧ��");
		return ((void*)0);
	}
	for(;;){
		uart_BD_rec_report(UART3_Fd);
		sleep_1(10);	//10ms
	}

}

//���ڳ�ʼ��
int8 uart_BD_init( void )
{
	int8 iret = 0;
	removeBDTask();
	UART3_Fd = open_port(COM3);
	if(UART5_Fd<0)
	{
		iret = -1;
	}
	if(set_com_config(UART3_Fd,19200,8,'N',1)<0)
	{
		iret = -1;
	}
	return iret;
}

int8 uart_BD_rec_report( UART_TYPE uartid )
{
	int8 buff[200];
	int ret;
	int ico;
	static int jco=0;

	ret = read_uart(uartid,buff,200);
	if(ret<=0)
		return TRUE;
	//debug start
	for(ico=0;ico<ret;ico++)
	{
		printf("0x%02x ",(uint8)buff[ico]);
	}
	printf("\n");
	//debug end

	for(ico=0;ico<ret;ico++)
	{
		if(((buff[ico]=='$')&&(buff[ico+1]=='T')&&(buff[ico+2]=='X')&&(buff[ico+3]=='X')&&(buff[ico+3]=='X'))||(1==bd_re_sign))
		{
			bd_re_sign=1;
			bd_re_buff[jco++] = buff[ico];
			if(jco>=REC_BD_MSG_LEN)
			{
				jco = 0;
				bd_re_sign = 0;
				if(CheckCRC(1,(uint8*)&bd_re_buff[0],REC_BD_MSG_LEN))	//���㱱������CRC
				{
					getBDSailMsg((uint8*)bd_re_buff);	//��������������
					monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCWARN_SN].rec_ok_number++;
				//	SysLogMsgPost("��ñ�����������");
				//	SysPubMsgPost("��ñ�����������");
				}
			}
		}
	}


}

void getBDSailMsg( uint8* recBuf )
{
	uint8	loc_latitude_sign	;
	uint8	loc_latitude_deg	;
	uint8	loc_latitude_min	;
	uint8	loc_latitude_sec	;
	uint8	loc_latitude_secDec	;
	double  locf64_latitude;

	uint8	loc_longitude_sign	;
	uint8	loc_longitude_deg	;
	uint8	loc_longitude_min	;
	uint8	loc_longitude_sec	;
	uint8	loc_longitude_secDec;
	double	locf64_longitude	;

	double  loc_speed		;
	uint16  loc_stopTime	;
	//checkCRC
	bd_sailTask.u8_St_sailMsgRev  =  1;
	//��ȡ��������
	uint8	real_msg[100];
	memcpy(&real_msg[0],&recBuf[18],26);
	
	//if(calcCRC(0xffff,real_msg,24)==(real_msg[24]|(real_msg[25]<<8)))
	{
		bd_sailTask.sailMsg.u8_msgSrc = 3;
		

		bd_sailTask.u8_cmd_sailOnOff = real_msg[9];
		
		loc_latitude_sign  = real_msg[10]	;
		loc_longitude_sign = real_msg[11]	;

		loc_latitude_deg     = real_msg[12];
		loc_latitude_min     = real_msg[13];
		loc_latitude_sec     = real_msg[14];
		loc_latitude_secDec  = real_msg[15];

		loc_longitude_deg	 = real_msg[16];
		loc_longitude_min	 = real_msg[17];
		loc_longitude_sec	 = real_msg[18];
		loc_longitude_secDec = real_msg[19];

		loc_speed		= real_msg[20]*256 + real_msg[21];
		loc_stopTime	= real_msg[22]*256 + real_msg[23];
		bd_sailTask.sailMsg.u8_sailNum = real_msg[24];
		//����ת��
		locf64_latitude  = (double)loc_latitude_deg				;
		locf64_latitude += (double)loc_latitude_min/60.0		;
		locf64_latitude += (double)loc_latitude_sec/3600.0		;
		locf64_latitude += (double)loc_latitude_secDec/360000.0	;
		if (loc_latitude_sign == 1)
			locf64_latitude = 0-locf64_latitude;

		locf64_longitude = (double)loc_longitude_deg			;
		locf64_longitude+= (double)loc_longitude_min/60.0		;
		locf64_longitude+= (double)loc_longitude_sec/3600.0		;
		locf64_longitude+= (double)loc_longitude_secDec/360000.0	;
		if(loc_longitude_sign == 1)
			locf64_longitude = 0-locf64_longitude;

		bd_sailTask.sailMsg.wayPoint.f64_latitude = locf64_latitude				;
		bd_sailTask.sailMsg.wayPoint.f64_longitude= locf64_longitude			;
		bd_sailTask.sailMsg.wayPoint.f64_expSpeed =  (double)loc_speed/100.0	;
		bd_sailTask.sailMsg.wayPoint.u16_stopTime = loc_stopTime;

		bd_sailTask.sailMsg.wayPoint.b1_type = 0;
		printf("lng = %f  lat = %f \n",bd_sailTask.sailMsg.wayPoint.f64_longitude,bd_sailTask.sailMsg.wayPoint.f64_latitude);

	}
}


void uart_BD_send(void)
{
	static uint16 msgNum=0;
	uint8 CRC;
	CRC=0;
	uint8  BD_MSG[200];
	uint16 length = 59;
	uint16 msg_length = 41*8;
	uint32 local_id = 394695;
	uint32 remote_id = 394692;
	memset(&BD_MSG[0],0,sizeof(BD_MSG));

	
	BD_MSG[0] = '$';
	BD_MSG[1] = 'T';
	BD_MSG[2] = 'X';
	BD_MSG[3] = 'S';
	BD_MSG[4] = 'Q';
	BD_MSG[5] = (length >> 8)&0x00ff;			//����
	BD_MSG[6] = length&0x00ff;				//
	BD_MSG[7] = (local_id>> 16)&0x000000ff;	//�ܿ��û���ַ
	BD_MSG[8] = (local_id>> 8)&0x000000ff;	
	BD_MSG[9] = (local_id)&0x000000ff;
	BD_MSG[10]= 0x46;					//��Ϣ���� �տ�ͨѶ
	BD_MSG[11]= (remote_id>> 16)&0x000000ff;//�û���ַ
	BD_MSG[12]= (remote_id>> 8)&0x000000ff;
	BD_MSG[13]= (remote_id)&0x000000ff;
	BD_MSG[14]= (msg_length >>8)&0x00ff;
	BD_MSG[15]= (msg_length)&0x00ff;
	BD_MSG[16]= 0;		//no answer

	BD_MSG[17] = '$';
	BD_MSG[18] = 'U';
	BD_MSG[19] = 'S';
	BD_MSG[20] = 'V';
	BD_MSG[21] = 0x4d;
	BD_MSG[22] = 1;
	BD_MSG[23] = (msgNum&0xff00)>>8	;
	BD_MSG[24] =  msgNum&0x00ff		;
	BD_MSG[25] =  29;

	BD_MSG[26] = (uint8)command_signal.sail_mode_cmd.u8_authority;	//���˴�Ȩ��λ��
	BD_MSG[27] = bd_sailTask.u8_St_BDSailState;						//��������״̬
	BD_MSG[28] = bd_sailTask.sailMsg.u8_sailNum;					//��������������
	//memcpy(&(BD_MSG[29]),(uint8*)&ins_msg.u16_speed,2);						//
	//memcpy(&(BD_MSG[31]),(uint8*)&ins_msg.u16_heading,2);

	uint16 speed_local = ins_msg.u16_speed*10;
	BD_MSG[29] = (((speed_local) & 0xff00) >> 8);//����
	BD_MSG[30] = ((speed_local) & 0x00ff);
	uint16 heading_local = ins_msg.u16_heading*10;
	BD_MSG[31] = (((heading_local) & 0xff00) >> 8);//����
	BD_MSG[32] = ((heading_local) & 0x00ff);


	BD_MSG[33] = ins_msg.u8_longiSt		;
	BD_MSG[34] = ins_msg.u8_latiSt		;

	BD_MSG[35] = ins_msg.u8_longiDeg	;
	BD_MSG[36] = ins_msg.u8_longiMin	;
	BD_MSG[37] = ins_msg.u8_longiSec	;
	BD_MSG[38] = ins_msg.u8_longiSecDec	;	

	BD_MSG[39] = ins_msg.u8_latiDeg		;
	BD_MSG[40] = ins_msg.u8_latiMin		;
	BD_MSG[41] = ins_msg.u8_latiSec		;
	BD_MSG[42] = ins_msg.u8_latiSecDec	;

	memcpy(&(BD_MSG[43]),(uint8*)&IHC_rev_msg.u16_St_Motor1Rpm,2);
	memcpy(&(BD_MSG[45]),(uint8*)&IHC_rev_msg.u16_St_Motor2Rpm,2);

	BD_MSG[47] = (uint8)(IHC_rev_msg.i16_St_Motor1Gear&0x0000ffff);	
	BD_MSG[48] = (uint8)(IHC_rev_msg.i16_St_Motor2Gear&0x0000ffff);	

	BD_MSG[49] = (uint8)(IHC_rev_msg.i16_St_Motor1Rudder*0.1);
	BD_MSG[50] = (uint8)(IHC_rev_msg.i16_St_Motor2Rudder*0.1);

	BD_MSG[51] = IDU_rev_msg.u8_St_PortOilLvl;
	BD_MSG[52] = IDU_rev_msg.u8_St_STBDOilLvl;

	BD_MSG[53] = IDU_rev_msg.u8_St_PORTBatLvl;//���ص���->
	BD_MSG[54] = IDU_rev_msg.u8_St_STBDBatLvl;//�ҵ�ص���->

	BD_MSG[55] = '*';
	BD_MSG[56] = 0x55;	//CRC
	BD_MSG[57] = 0xAA;	//CRC
	GetCheck(&CRC,BD_MSG,sizeof(BD_MSG));
	BD_MSG[58] = CRC;	//CRC
	write_uart(UART3_Fd,(int8*)&BD_MSG[0],59);

	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BD_SN].send_ok_number++;


	msgNum++;
}

void removeBDTask( void )
{
	memset((uint8*)&bd_sailTask,0,sizeof(bd_sailTask));
}
