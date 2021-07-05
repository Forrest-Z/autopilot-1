/*==========================================================*
 * ģ��˵��: sailWaypoint.cpp                               *
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
#include "../include/sailWaypoint.h"
/******************************  Local Variable  *****************************/
SAIL_TASK sailTask={0};
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void getSailMsg(uint8* recBuf)
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

	uint16	loc_speed		;
	uint16  loc_stopTime	;
	uint8	loc_usvNum;
	uint8	loop_i;

	loc_usvNum			 = recBuf[5];
	

	if(loc_usvNum == USV_NUM)
	{
		sailTask.sailMsg.u8_msgSrc = 0;				//���Կ�����̨
		sailTask.u8_St_sailMsgRev = 1;				//���մ�����
		sailTask.u8_PointNum = 0;					//ִ�е����
		sailTask.sailMsg.u8_pointSum = recBuf[8];	//��������
		sailTask.sailMsg.u8_sailNum  = recBuf[9+13*sailTask.sailMsg.u8_pointSum];	//������
		for(loop_i=0;loop_i<sailTask.sailMsg.u8_pointSum;loop_i++)
		{
			loc_latitude_sign    = (recBuf[loop_i*13+9]&0x01);
			loc_longitude_sign   = (recBuf[loop_i*13+9]&0x02)>>1;

			loc_latitude_deg     = recBuf[loop_i*13+10];
			loc_latitude_min     = recBuf[loop_i*13+11];
			loc_latitude_sec     = recBuf[loop_i*13+12];
			loc_latitude_secDec = recBuf[loop_i*13+13];

			loc_longitude_deg	 = recBuf[loop_i*13+14];
			loc_longitude_min	 = recBuf[loop_i*13+15];
			loc_longitude_sec	 = recBuf[loop_i*13+16];
			loc_longitude_secDec = recBuf[loop_i*13+17];

			loc_speed		=  recBuf[loop_i*13+18]*256+recBuf[loop_i*13+19];
			loc_stopTime	=  recBuf[loop_i*13+20]*256+recBuf[loop_i*13+21];

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

			//ת��
			sailTask.sailMsg.wayPoint[loop_i].f64_latitude  = locf64_latitude			;
			sailTask.sailMsg.wayPoint[loop_i].f64_longitude = locf64_longitude			;
			sailTask.sailMsg.wayPoint[loop_i].f64_expSpeed  = (double)loc_speed/100.0	;
			sailTask.sailMsg.wayPoint[loop_i].u16_stopTime  = loc_stopTime				;

			//����������
			sailTask.sailMsg.wayPoint[loop_i].b1_samplingComplete = 0;
			sailTask.sailMsg.wayPoint[loop_i].b1_sailArrival	  = 0;
			sailTask.sailMsg.wayPoint[loop_i].b1_samplingCommand  = 0;


			printf("lng = %f  lat = %f \n",sailTask.sailMsg.wayPoint[loop_i].f64_longitude,sailTask.sailMsg.wayPoint[loop_i].f64_latitude);

			if(loc_stopTime == 0)
			{
				sailTask.sailMsg.wayPoint[loop_i].b1_type = 1;
			}
			else
			{
				sailTask.sailMsg.wayPoint[loop_i].b1_type = 0;
			}
			//���Զ��ǲ��ǲ�����
			sailTask.sailMsg.wayPoint[loop_i].b1_type = 0;
		}
	}
}

void getSailMsgWaterQuality( uint8* recBuf )
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

	uint16	loc_speed		;
	uint16  loc_stopTime	;
	uint8	loc_usvNum;
	uint8	loop_i;
	uint64	loc_sailPointID	;
	uint8	loc_sailPointType;
	uint16	loc_sampleVolume;

	loc_usvNum			 = recBuf[5];

	if(loc_usvNum == USV_NUM)
	{
		sailTask.sailMsg.u8_msgSrc = 0;				//���Կ�����̨
		sailTask.u8_St_sailMsgRev = 1;				//���մ�����
		sailTask.u8_PointNum = 0;					//ִ�е����
		sailTask.sailMsg.u8_pointSum = recBuf[8];	//��������
		sailTask.sailMsg.u8_sailNum  = recBuf[9];	//������

		for(loop_i=0;loop_i<sailTask.sailMsg.u8_pointSum;loop_i++){
			loc_latitude_sign  = (recBuf[loop_i*24+10]&0x01);
			loc_longitude_sign = (recBuf[loop_i*24+10]&0x02)>>1;

			loc_latitude_deg	=	recBuf[loop_i*24+11];
			loc_latitude_min	=	recBuf[loop_i*24+12];
			loc_latitude_sec	=	recBuf[loop_i*24+13];
			loc_latitude_secDec	=	recBuf[loop_i*24+14];

			loc_longitude_deg		=	recBuf[loop_i*24+15];
			loc_longitude_min		=	recBuf[loop_i*24+16];
			loc_longitude_sec		=	recBuf[loop_i*24+17];
			loc_longitude_secDec	=	recBuf[loop_i*24+18];

			loc_speed				=	recBuf[loop_i*24+19]*256 + recBuf[loop_i*24+20];
			loc_stopTime			=	recBuf[loop_i*24+21]*256 + recBuf[loop_i*24+22];
			
		//	memcpy((uint8*)loc_sailPointID,(uint8*)&recBuf[loop_i*24+23],8);	//ֱ�ӿ��� �˴����Ǵ��ģʽ
			loc_sailPointID = 0;
			for(int i=0;i<8;i++){
				loc_sailPointID += recBuf[loop_i*24+23+i]<< ((7-i)*8);
			}



			loc_sailPointType = recBuf[loop_i*24+31];
			loc_sampleVolume =  recBuf[loop_i*24+32]*256+recBuf[loop_i*24+33];
			

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

			//ת��
			sailTask.sailMsg.wayPoint[loop_i].f64_latitude  = locf64_latitude			;
			sailTask.sailMsg.wayPoint[loop_i].f64_longitude = locf64_longitude			;
			sailTask.sailMsg.wayPoint[loop_i].f64_expSpeed  = (double)loc_speed/100.0	;
			sailTask.sailMsg.wayPoint[loop_i].u16_stopTime  = loc_stopTime				;

			//������������
			sailTask.sailMsg.wayPoint[loop_i].b1_type		= loc_sailPointType			;
			sailTask.sailMsg.wayPoint[loop_i].u64_sailPointID = loc_sailPointID			;
			sailTask.sailMsg.wayPoint[loop_i].u16_sampleVolume = loc_sampleVolume		;
			sailTask.sailMsg.wayPoint[loop_i].b1_samplingComplete = 0;
			sailTask.sailMsg.wayPoint[loop_i].b1_sailArrival	  = 0;
			sailTask.sailMsg.wayPoint[loop_i].b1_samplingCommand  = 0;

			//printf("lng = %f  lat = %f \n",sailTask.sailMsg.wayPoint[loop_i].f64_longitude,sailTask.sailMsg.wayPoint[loop_i].f64_latitude);
			printf("lng = %f  lat = %f  speed=%f  sampleFlag=%d  sampleVolume=%d PointId=%lld \n",sailTask.sailMsg.wayPoint[loop_i].f64_longitude,sailTask.sailMsg.wayPoint[loop_i].f64_latitude, \
				sailTask.sailMsg.wayPoint[loop_i].f64_expSpeed,sailTask.sailMsg.wayPoint[loop_i].b1_type,sailTask.sailMsg.wayPoint[loop_i].u16_sampleVolume, \
				sailTask.sailMsg.wayPoint[loop_i].u64_sailPointID);
			
		}
	//	SysPubMsgPost("�յ���������������:%d, �������:%d",sailTask.sailMsg.u8_sailNum,sailTask.sailMsg.u8_pointSum);
	}


}
