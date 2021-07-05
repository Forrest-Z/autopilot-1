/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// 
// (C) Copyright 2017,Beijing
// All Rights Reserved
//
//
// FileName:        read_usv_state.cpp
// Programmer(s):  syp ,2017-6-22
// Description: ��ȡ�ļ�����
//                  [s1] s2=value ";"��Ϊע��
*******************************************************************************************/
#include "stdafx.h"
#include <stdarg.h>

#include "../include/usv_include.h"



#define	 FLOAT_TYPE				1
#define	 INT_TYPE				2

USV_Meter_Clock USV_Meter_Clock_Data;


// #########################################################################
//			������������
// #########################################################################

int8	read_usv_state(void)
{
	int8 		*p_file_memory;		//������
	int32 	*p_buffer;
	FILE		*pFile;
	uint32	lSize;
	int32	result;
	uint32	len;
	int8		s1[32];
	int8		s2[50];
	int8		ret_val;

	ret_val = TRUE;
	if((pFile=fopen(USV_STATE_FILE_NAME,"r+"))==NULL)	//��ȡ�ļ�����
	{
		sprintf_usv(s1,"USVstate.inf");
		sprintf_usv(s2,"not found");
		input_cfg_ini_err_sub(s1,s2,0);
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);		//16K
	if(NULL == p_file_memory)
	{
		sprintf_usv(s1,"USV state file memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		fclose(pFile);
	       return FALSE;
	}

	p_buffer = (int32 *)malloc(0x10000);			//64K
	if(NULL == p_buffer)
	{
		sprintf_usv(s1,"USV state explain memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		fclose(pFile);
	       return FALSE;
	}

	//��ȡ�ļ���С
	fseek(pFile , 0 , SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile);			//��ָ��ָ��ͷ

	if(lSize>=0xffff)
	{
		sprintf_usv(s1,"USV state read file");
		sprintf_usv(s2,"too large");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

	result = fread(p_file_memory,1,lSize,pFile); 	//���ļ�������buff ��
	//if(result != lSize)
	//{
	//	sprintf_usv(s1,"USV state read file");
	//	sprintf_usv(s2,"not same");
	//	input_cfg_ini_err_sub(s1,s2,0);
	//	free(p_file_memory);
	//	free(p_buffer);
	//	fclose(pFile);
	//	return FALSE;
	//}

	if(32768 < lSize) len = 2*lSize;
	else len = 1280 + 2*lSize;
	len=len*2;
	result = ini_Initialize((char *)p_file_memory, p_buffer, len);

	if(result!=0){
		sprintf_usv(s1,"USV state  memory");
		sprintf_usv(s2,"explain error");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

	//��ʼ����״̬�ļ�
	//������̱�����
	sprintf_usv(s1,"Meter_Clock_State");
	sprintf_usv(s2,"run_time_second");
	if(read_sub_setting(s1,s2,0, (uint32 *)&USV_Meter_Clock_Data.run_time_second,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"run_total_dist");
	if(read_sub_setting(s1,s2,0, (uint32 *)&USV_Meter_Clock_Data.run_total_dist,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"run_average_speed");
	if(read_sub_setting(s1,s2,0, (uint32 *)&USV_Meter_Clock_Data.run_average_speed,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}	
	
	USV_Meter_Clock_Data.run_total_dist_this_time = 0;
	USV_Meter_Clock_Data.run_time_second_this_time = 0;
	USV_Meter_Clock_Data.run_dist_10min = 0;

	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	return ret_val;
}

int8 write_usv_state(void)	//����ʧ�ܷ���0���ɹ�����1
{
	FILE *pFile;
	int8 buff[200];
	uint16 ret_len;
	USV_Meter_Clock *p_meter_clock;

	pFile = fopen(USV_STATE_FILE_NAME,"w+");

	if(pFile == NULL)
		return FALSE;

	//ret_len = sprintf_usv((int8 *)&buff[0],";\n");
	//fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Meter_Clock_State]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	
	ret_len=sprintf_usv((int8 *)&buff[0],"run_time_second=%d;\n",USV_Meter_Clock_Data.run_time_second);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);

	ret_len=sprintf_usv((int8 *)&buff[0],"run_total_dist=%d;\n",USV_Meter_Clock_Data.run_total_dist);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);

	ret_len=sprintf_usv((int8 *)&buff[0],"run_average_speed=%d;",USV_Meter_Clock_Data.run_average_speed);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	
	fclose(pFile);
	printf("write state ok\n");
	return TRUE;
}

//������̱�����
//д���ļ��ɹ�����1�����³ɹ�����2��д���ļ�ʧ�ܷ���-1��δ���·���0
int8 Update_Meter_Clock(void)	
{
	static uint8 second_old=0;
	static uint8 minute_old=0;
	uint8	iret = 0;
	uint8	write_flash_flag = 0;

	if(USV_State.Engine_run != 0x03)
	{
		return iret;
	}

	if(second_old != Smart_Navigation_St.USV_Second)
	{
		//USV_Meter_Clock_Data.run_total_dist+= Smart_Navigation_St.USV_Speed;
		USV_Meter_Clock_Data.run_time_second ++;
		USV_Meter_Clock_Data.run_time_second_this_time ++;
		USV_Meter_Clock_Data.run_dist_10min+= Smart_Navigation_St.USV_Speed;
		
		//ʱ�����
		second_old = Smart_Navigation_St.USV_Second;
		iret = 2;
	}
	else
	{
		//ʱ�����
		second_old = Smart_Navigation_St.USV_Second;
	}

	if(minute_old <= Smart_Navigation_St.USV_Minute)
	{
		if((Smart_Navigation_St.USV_Minute- minute_old)>(WRITE_METER_CLOCK_TIME-1))
		{
			write_flash_flag = 1;
			minute_old = Smart_Navigation_St.USV_Minute;
		}
		else
		{
			write_flash_flag = 0;
			minute_old = minute_old;
		}
	}
	else
	{
		if((Smart_Navigation_St.USV_Minute+60-minute_old)>(WRITE_METER_CLOCK_TIME-1))
		{
			write_flash_flag = 1;
			minute_old = Smart_Navigation_St.USV_Minute;
		}
		else
		{
			write_flash_flag = 0;
			minute_old = minute_old;
		}
	}

	if(1 == write_flash_flag)
	{
		USV_Meter_Clock_Data.run_average_speed			 = (uint32)((double)USV_Meter_Clock_Data.run_dist_10min	/6000.0);	//ƽ������ *10 kn/h
		USV_Meter_Clock_Data.run_total_dist				+= (uint32)((double)USV_Meter_Clock_Data.run_dist_10min / 36000.0);	//�����   *10 kn
		USV_Meter_Clock_Data.run_total_dist_this_time	+= (uint32)((double)USV_Meter_Clock_Data.run_dist_10min / 36000.0); //������� *10 kn
		USV_Meter_Clock_Data.run_dist_10min  = 0;

		if(write_usv_state() == 1)
		{
			iret = 1;
		}
		else
		{
			iret = -1;
			printf("write usv state failed!\n");
		}
	}
	return iret;	
}

//������̱�����_fornew
//д���ļ��ɹ�����1,���³ɹ�����2��д���ļ�ʧ�ܷ���-1��δ���·���0
int8 updateMeterClock(void)
{
	static uint8	second_old = 0;
	static uint8	minute_old = 0;
	int8	iret = 0;
	int8	writeFlashFlag = 0;

	//������δ����ʱ��������
	if(IHC_rev_msg.mid_st.b1_St_MotorOn != 0x01)
	{
		return iret;
	}

	if(second_old != ins_msg.u8_second)
	{
		USV_Meter_Clock_Data.run_time_second++;
		USV_Meter_Clock_Data.run_time_second_this_time++;
		USV_Meter_Clock_Data.run_dist_10min += ins_msg.u16_speed*10;

		//ʱ�����
		second_old = ins_msg.u8_second;
		iret = 2;
	}
	else
	{
		//ʱ�����
		second_old = ins_msg.u8_second;
	}

	if(minute_old <= ins_msg.u8_minute)
	{
		if((ins_msg.u8_minute - minute_old)>(WRITE_METER_CLOCK_TIME-1))
		{
			writeFlashFlag = 1;
			minute_old = ins_msg.u8_minute;
		}
		else
		{
			writeFlashFlag = 0;
			minute_old = minute_old;
		}
	}
	else
	{
		if((ins_msg.u8_minute+60-minute_old)>(WRITE_METER_CLOCK_TIME-1))
		{
			writeFlashFlag = 1;
			minute_old = ins_msg.u8_minute;
		}
		else
		{
			writeFlashFlag = 0;
			minute_old = minute_old;
		}
	}

	if(1 == writeFlashFlag)
	{
		USV_Meter_Clock_Data.run_average_speed			 = (uint32)((double)USV_Meter_Clock_Data.run_dist_10min /6000.0);	//ƽ�ֺ��� *10 Kn/h
		USV_Meter_Clock_Data.run_total_dist				+= (uint32)((double)USV_Meter_Clock_Data.run_dist_10min /36000.0);	//�����   *10 Kn
		USV_Meter_Clock_Data.run_time_second_this_time	+= (uint32)((double)USV_Meter_Clock_Data.run_dist_10min /36000.0);	//������� *10 Kn
		USV_Meter_Clock_Data.run_dist_10min = 0;

		if(write_usv_state() == 1)
		{
			iret = 1;
		}
		else
		{
			iret = -1;
			SysLogMsgPost("write MeterClock failed");
		}
	}
	return iret;


}