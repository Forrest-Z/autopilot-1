/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// Building 9,Fourth Avenue,Shangdi Information Industry Base,Haidian District
// (C) Copyright 2009,Beijing
// All Rights Reserved
//
//
// FileName:        pii_readIni.c
// Programmer(s):   XiaoZQ ,2009-8-5
// Description:     ��ȡini�����ļ��ĺ�������
//                  [s1] s2=value ";"��Ϊע��
// 
// added ini_splitStr() XiaoZQ ,2009-8-11
// ini_splitStr()ȥ���ַ���ǰ�հ� XiaoZQ ,2009-9-2
*******************************************************************************************/
#include "stdafx.h"
#include <stdarg.h>

#include "../include/usv_include.h"

//�������������
//���룺�ֶΣ���ţ�
//��� FALSE--���� ��TRUE--��ȷ
int8 read_sub_cfg(int8* main_item,int8* sub_item,int8 id, uint16 *out_data)
{
int8 ret_val;
int32	len;
int8 *str;
int8 * p;
int8 split = ',';
int8 space =' ';
int32 i;
	
	ret_val =TRUE;
	if(str = ini_GetVarStr(main_item, sub_item, (int32 *)&i))
	{					
		p = ini_splitStr(str, split, id, (int32 *)&len); 
		if(p) {
			*out_data=ini_str2hex(p);			
		}
		else{
			ret_val =FALSE;
			input_cfg_ini_err_sub(main_item,sub_item,id+1);
		}
	}
	else{
		ret_val =FALSE;
		input_cfg_ini_err_sub(main_item,sub_item,id+1);
	}
	return ret_val;

}

// #########################################################################
//			���������ļ�
// #########################################################################
int8	read_usv_cfg(void)
{
int8 *p_file_memory;				//������
int32 *p_buffer;
FILE *pFile;
uint32 lSize;
int32 result;
uint32 len;
int8 s1[32];
int8 s2[50];
int8 ret_val;


	ret_val =TRUE;

	if ( (pFile = fopen(USV_CFG_FILE_NAME, "r") ) == NULL)		//�����ļ�����
	{														
		sprintf_usv(s1,"USVconfig.cfg");
		sprintf_usv(s2,"not fond");
		input_cfg_ini_err_sub(s1,s2,0);
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);				//16K
    if (NULL == p_file_memory)
    {
		sprintf_usv(s1,"USV cfg file memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		fclose(pFile);
        return FALSE;
    }

	p_buffer = (int32 *)malloc(0x10000);				//64K
    if (NULL == p_buffer)
    {
		sprintf_usv(s1,"USV cfg explain memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		fclose(pFile);
        return FALSE;
    }
	
// ��ȡ�ļ���С 
    fseek (pFile , 0 , SEEK_END);  
    lSize = ftell (pFile);  
    rewind (pFile);					//��ָ��ָ���ļ���ͷ
  
	if(lSize>=0xffff){
		sprintf_usv(s1,"USV cfg read file");
		sprintf_usv(s2,"too large");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}
  
   
    result = fread (p_file_memory,1,lSize,pFile);			 // ���ļ�������buffer��   
    if (result != lSize)  
    {  
        sprintf_usv(s1,"USV cfg read file");
		sprintf_usv(s2,"not same");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
    }  

    if(32768 < lSize) len = 2 * lSize;
    else len = 1280 + 2 * lSize;
	len=len * 2;
	result=ini_Initialize((char *)p_file_memory, p_buffer, len);

	if(result!=0){										//�ļ���ʼ������
		sprintf_usv(s1,"USV cfg  memory");
		sprintf_usv(s2,"explain error");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

// ��ʼ���������ļ�
//����Basic_Cfg
	sprintf_usv(s1,"Basic_Cfg");
	sprintf_usv(s2,"Config_Num");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&USV_State.Config_Nummber)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Hull_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Basic_Cfg.Hull_Type)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Drive_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Basic_Cfg.Drive_Type)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Propeller_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Basic_Cfg.Propeller_Type)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Basic_Cfg.Motor_Type)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Drive_Num");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Drive_Nummber_spn520252)==FALSE){
		ret_val =FALSE;
	}	

//����Board_Cfg
	sprintf_usv(s1,"Board_Cfg");
	sprintf_usv(s2,"MMSI");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.MMSI)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Board_length");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Board_length)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Board_Wide");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Board_Wide)==FALSE){
		ret_val =FALSE;
	}	
	
	sprintf_usv(s2,"Board_Deep");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Board_Deep)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Max_Static_Draft");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Max_Static_Draft)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Max_Speed");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Max_Speed)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Max_Voyage");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.Max_Voyage)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"INS_Antenna_Distance");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Board_Cfg.INS_Antenna_Distance)==FALSE){
		ret_val =FALSE;
	}	
//����Throttle_Cfg
	sprintf_usv(s1,"Throttle_Cfg");
	sprintf_usv(s2,"Motor_Speed_Max_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_MAX_Speed_L_spn520220)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Motor_Speed_Idle_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Idling_Speed_L_spn520221)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Motor_Speed_Max_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_MAX_Speed_R_spn520222)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Motor_Speed_Idle_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Idling_Speed_R_spn520223)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Trip_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Travel_L_spn520224)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Trip_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Travel_R_spn520225)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Address_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Throttle_Cfg.Motor_Address_L)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Address_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Throttle_Cfg.Motor_Address_R)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Direction_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Direction_L_spn520227)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Direction_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Direction_R_spn520228)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Motor_Fire_Time");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Motor_Fire_Time_spn520226)==FALSE){
		ret_val =FALSE;
	}
//����Throttle_Cfg
	sprintf_usv(s1,"Gear_Cfg");
	sprintf_usv(s2,"Gear_Forward_Travel_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Forward_Travel_L_spn520240)==FALSE){
		ret_val =FALSE;
	}	
	sprintf_usv(s2,"Gear_Forward_Travel_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Forward_Travel_R_spn520241)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Back_Travel_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Back_Travel_L_spn520242)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Back_Travel_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Back_Travel_R_spn520243)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Forward_Angle_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Forward_Angle_L_spn520244)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Forward_Angle_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Forward_Angle_R_spn520245)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Back_Angle_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Back_Angle_L_spn520246)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Back_Angle_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Back_Angle_R_spn520247)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Neutral_Angle_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Neutral_Angle_L_spn520248)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Neutral_Angle_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Neutral_Angle_R_spn520249)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Direction_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Direction_L_spn520250)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Gear_Direction_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Gear_Direction_R_spn520251)==FALSE){
		ret_val =FALSE;
	}
//����Rudder_Cfg
	sprintf_usv(s1,"Rudder_Cfg");
	sprintf_usv(s2,"Rudder_Angle_Max_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Angle_Max_L_spn520260)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Angle_Max_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Angle_Max_R_spn520261)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Left_Limit_Resistance_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Left_Limit_Resistance_L_spn520262)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Left_Limit_Resistance_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Left_Limit_Resistance_R_spn520263)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Middle_Limit_Resistance_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Middle_Limit_Resistance_L_spn520264)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Middle_Limit_Resistance_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Middle_Limit_Resistance_R_spn520265)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Right_Limit_Resistance_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Right_Limit_Resistance_L_spn520266)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Right_Limit_Resistance_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Right_Limit_Resistance_R_spn520267)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Control_Angle_Max_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Angle_Max_L_spn520268)==FALSE){
		ret_val =FALSE;
	}
	Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L=SR_Config_Msg.Rudder_Control_Angle_Max_L_spn520268;
	sprintf_usv(s2,"Rudder_Control_Angle_Max_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Angle_Max_R_spn520269)==FALSE){
		ret_val =FALSE;
	}
	Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R=SR_Config_Msg.Rudder_Control_Angle_Max_R_spn520269;	
	sprintf_usv(s2,"Rudder_Control_Angle_Min_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Angle_Min_L_spn520270)==FALSE){
		ret_val =FALSE;
	}		
	Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L=SR_Config_Msg.Rudder_Control_Angle_Min_L_spn520270;
	sprintf_usv(s2,"Rudder_Control_Angle_Min_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Angle_Min_R_spn520271)==FALSE){
		ret_val =FALSE;
	}
	Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R =SR_Config_Msg.Rudder_Control_Angle_Min_R_spn520271;
	sprintf_usv(s2,"Rudder_Left_Travel_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Left_Travel_L_spn520272)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Left_Travel_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Left_Travel_R_spn520273)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Right_Travel_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Right_Travel_L_spn520274)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Right_Travel_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Right_Travel_R_spn520275)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Control_Accuracy_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Accuracy_L_spn520276)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Control_Accuracy_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Control_Accuracy_R_spn520277)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Direction_L");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Direction_L_spn520278)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Rudder_Direction_R");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&SR_Config_Msg.Rudder_Direction_R_spn520279)==FALSE){
		ret_val =FALSE;
	}

//����Algorithm_Cfg0
	sprintf_usv(s1,"Algorithm_Cfg0");
	sprintf_usv(s2,"Control_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].Control_Type)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_P");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].PID_P)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_I");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].PID_I)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_D");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].PID_D)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K1");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].S_K1)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K2");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].S_K2)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K3");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].S_K3)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_A");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].Filter_A)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_B");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].Filter_B)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_C");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[0].Filter_C)==FALSE){
		ret_val =FALSE;
	}
//����Algorithm_Cfg1
	sprintf_usv(s1,"Algorithm_Cfg1");
	sprintf_usv(s2,"Control_Type");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].Control_Type)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_P");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].PID_P)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_I");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].PID_I)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"PID_D");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].PID_D)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K1");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].S_K1)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K2");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].S_K2)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"S_K3");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].S_K3)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_A");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].Filter_A)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_B");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].Filter_B)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Filter_C");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Algorithm_Cfg[1].Filter_C)==FALSE){
		ret_val =FALSE;
	}
//����Environment_Cfg
	sprintf_usv(s1,"Environment_Cfg");
	sprintf_usv(s2,"Wind_Speed");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Environment_Cfg.Wind_Speed)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Wave_Hight");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Environment_Cfg.Wave_Hight)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Flow_Speed");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&Dradio_Config.Environment_Cfg.Flow_Speed)==FALSE){
		ret_val =FALSE;
	}

//����USV_CONFIG
	sprintf_usv(s1, "USV_CONFIG");
	sprintf_usv(s2, "USV_Num");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&usv_num) == FALSE){
		ret_val = FALSE;
	}


//����CTRL_Cfg
	sprintf_usv(s1,"CTRL_Cfg");
	sprintf_usv(s2,"Rudder1_Natural_Angle");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&jet_config.rudderConfig.i16_rudder1NaturlAngle)==FALSE){
		ret_val =FALSE;
	}
	//printf("usv_nRudder1_Natural_Angleum = %d\n",jet_config.rudderConfig.i16_rudder1NaturlAngle);

	sprintf_usv(s2,"Rudder2_Natural_Angle");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&jet_config.rudderConfig.i16_rudder2NaturlAngle)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Arrival_Distance1");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_arrival_distance1)==FALSE){
		ret_val =FALSE;
	}
	printf("cfg::arrival_distance1 = %d\n",autoNaviCfg.u16_arrival_distance1);


	sprintf_usv(s2,"Arrival_Distance2");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_arrival_distance2)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Arrival_Distance3");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_arrival_distance3)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2, "Arrival_DistanceAvoid");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&autoNaviCfg.u16_arrival_distanceAvoid) == FALSE){
		ret_val = FALSE;
	}
	sprintf_usv(s2,"Arrival_Speed");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_arrival_speed)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Pid_Speed_Threshold");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_pid_speed_threshold)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2,"Roll_Heading_Threshold");
	if(read_sub_cfg(s1,s2,0, (uint16 *)&autoNaviCfg.u16_roll_heading_threshold)==FALSE){
		ret_val =FALSE;
	}
	sprintf_usv(s2, "Avoid_Speed");
	autoNaviCfg.f32_avoid_speed = ini_GetVarFloat(s1, s2);
	
	sprintf_usv(s2, "Docking_Speed"); //�����ٶ�
	docking_speed = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "LOS_N");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&n_ship) == FALSE){
		ret_val = FALSE;
	}
	sprintf_usv(s2, "Ship_Length"); //�����ٶ�
	l_ship = ini_GetVarFloat(s1, s2);

//������������
	sprintf_usv(s1, "APF_Cfg");
	sprintf_usv(s2, "LocalMininumTime");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&apfCfg.u16_apf_timeout) == FALSE){
		ret_val = FALSE;
	}
	sprintf_usv(s2, "LocalMininumDst");
	apfCfg.f32_apf_localMinDst = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "apfCoffAtt");
	apfCfg.f32_apf_coffAtt = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "apfCoffReq");
	apfCfg.f32_apf_coffRep = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "apfCoffReqImproved");
	apfCfg.f32_apf_coffRepImproved = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "apfCoffTimeScale");
	apfCfg.f32_apf_coffTimescale = ini_GetVarFloat(s1, s2);

	sprintf_usv(s2, "apfRadiusExtend");
	apfCfg.f32_apf_radius_extend = ini_GetVarFloat(s1, s2);

// parase battery configuration
	sprintf_usv(s1,"Battery Monitor");
	sprintf_usv(s2, "Battery_Number");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&batt_cfg.battery_number) == FALSE){
		ret_val = FALSE;
	}
	printf("battery number = %d\n",batt_cfg.battery_number);

	sprintf_usv(s2, "Bateery_Data_Channel");
	if (read_sub_cfg(s1, s2, 0, (uint16 *)&batt_cfg.battery_data_channel) == FALSE){
		ret_val = FALSE;
	}
	printf("battery data channel = %d\n",batt_cfg.battery_data_channel);

// parase auto contrl parameters
	sprintf_usv(s1,"Auto Control Mode");
	sprintf_usv(s2, "Arrival_Min_Speed");
	autoNaviCfg.speed_final = ini_GetVarFloat(s1, s2);
	printf("autoNaviCfg.speed_final = %f\n",autoNaviCfg.speed_final);

	autoNaviCfg.double_arrival_speedRate = (double)autoNaviCfg.u16_arrival_speed/100.0;
	autoNaviCfg.double_pid_speed_threshold = (double)autoNaviCfg.u16_pid_speed_threshold/10.0;
	autoNaviCfg.double_roll_heading_threshold = (double)autoNaviCfg.u16_roll_heading_threshold;

	Accelerator_Coefficient_L=(SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30-SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)/100.0;
	Accelerator_Coefficient_R=(SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30-SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)/100.0;

	Rudder_Coefficient_L=SR_Config_Msg.Rudder_Angle_Max_L_spn520260/100.0;			//����� /100
	Rudder_Coefficient_R=SR_Config_Msg.Rudder_Angle_Max_R_spn520261/100.0;

	Gear_Coefficient_L_F=SR_Config_Msg.Gear_Forward_Angle_L_spn520244/25.0;			//��λǰ�����Ƕ�����
	Gear_Coefficient_L_B=SR_Config_Msg.Gear_Back_Angle_L_spn520246/25.0;			//��λ���˵��Ƕ�����
	Gear_Coefficient_R_F=SR_Config_Msg.Gear_Forward_Angle_R_spn520245/25.0;
	Gear_Coefficient_R_B=SR_Config_Msg.Gear_Back_Angle_R_spn520247/25.0;


	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	return ret_val;
}

//�����ļ�����������Ĭ��ֵ
void set_usv_param_default(void)
{

	jet_config.rudderConfig.i16_rudder1NaturlAngle = 0 ;
	jet_config.rudderConfig.i16_rudder2NaturlAngle = 0 ;

	autoNaviCfg.u16_arrival_distance1 = 10	;
	autoNaviCfg.u16_arrival_distance2 = 5	;
	autoNaviCfg.u16_arrival_distance3 = 3	;
	autoNaviCfg.u16_arrival_distanceAvoid = 15;
	autoNaviCfg.u16_arrival_speed = 10		;	//1kn
	autoNaviCfg.double_arrival_speedRate  = 1.0 ;
	autoNaviCfg.u16_pid_speed_threshold = 20		;	//1kn
	autoNaviCfg.double_pid_speed_threshold    = 2.0	;
	autoNaviCfg.u16_roll_heading_threshold    = 30	;	
	autoNaviCfg.double_roll_heading_threshold = 30.0;	//30��


	apfCfg.u16_apf_timeout			=	600;
	apfCfg.f32_apf_localMinDst		=	9.0;
	apfCfg.f32_apf_coffAtt			=	1000.0;
	apfCfg.f32_apf_coffRep			=	1000000.0;
	apfCfg.f32_apf_coffRepImproved	=	10.0;
	apfCfg.f32_apf_coffTimescale	=	0.01;
	apfCfg.f32_apf_radius_extend	=	30.0;

	usv_num = 0;	
}


// ########################################################################
//		��ȡ�ļ��ӳ���
// ########################################################################

tIniHandle *    g_hIni = 0;
void *  ini_GetAddr32(char * p, int * pSize8); // ��buffȡ��32λ�����ַ
int  ini_readLine(char ** ppLine); // ���У�����1=��ͷ[s1]��2=���ݣ�0=������-1=�ռ����
int  ini_str2s2(char * str, tIniS2 * ps2); // ���ַ����ֽ�ΪtIniS2��������","�ָ��Ĳ�����Ŀ


void input_cfg_ini_err_sub(int8 *string_title,int8 *string_line,uint8 param1)
{
EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT		*p_monitor_cfg_alm_describe;
uint8 current_ID;
	
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number &=MONITOR_MAX_CFG_ERROR_ITEM;
	current_ID=monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number;
	p_monitor_cfg_alm_describe =&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[current_ID];
	memcpy(&p_monitor_cfg_alm_describe->string_title,string_title,24);
	memcpy(&p_monitor_cfg_alm_describe->string_line,string_line,24);
	p_monitor_cfg_alm_describe->param1=param1;
	
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number++;
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number &=MONITOR_MAX_CFG_ERROR_ITEM;	
}

/* ini initialize */
/* 0=success,-1=buff overflow,-2=fault */
/* ����buff�Ŀռ�Ϊ(32+strlen(pIniString)), sizeof(int) */
/* ��ɴ˺����󣬵��÷����ͷ�pIniString�ռ� */
int  ini_Initialize(const char * pIniString, int * buff, int buffLength)
{
  tIniS1 * ps1;
  tIniS2 * ps2=NULL;
  tIniS2 * p;
  char * str;
  int i, size, len;
  
  
  str = (char *)buff;
  len = sizeof(int32) * buffLength;
  // ��ʼ�����û���
  size = sizeof(tIniHandle);
  g_hIni = (tIniHandle *)ini_GetAddr32(str, &size);
  str += size;
  len -= size;
  size = sizeof(tIniS1);
  ps1 = (tIniS1 *)ini_GetAddr32(str, &size);
  str += size;
  len -= size;
  ps1->s1 = 0;
  ps1->ChildCount = 0;
  ps1->pChildren = 0;
  ps1->next = 0;
  g_hIni->iniString  = pIniString;
  g_hIni->iniLine    = pIniString;
  g_hIni->s1List     = ps1;
  g_hIni->ChildCount = 0;
  g_hIni->buff       = str;
  g_hIni->buffFree   = --len;

  // ����������Ϣ
  while(i = ini_readLine(&str)) switch(i)
  {
  case 1: // title(s1)
    if(0 != g_hIni->ChildCount ++)
    {
      size = sizeof(tIniS1);
      ps1->next = (tIniS1 *)ini_GetAddr32(g_hIni->buff, &size);
      if(size > g_hIni->buffFree){
      	 return -1;
      }
      g_hIni->buffFree -= size;
      g_hIni->buff += size;
      ps1 = ps1->next;
      ps1->ChildCount = 0;
      ps1->pChildren = 0;
      ps1->next = 0;
    }
    ps1->s1 = str;
    break;
    
  case 2: // value(s2=)
    if(0 == g_hIni->ChildCount) break;
    size = sizeof(tIniS2);
    p = (tIniS2 *)ini_GetAddr32(g_hIni->buff, &size);
  //  ps2 = p;
    if(size > g_hIni->buffFree){
    	 return -1;
    }
    g_hIni->buffFree -= size;
    g_hIni->buff += size;
    if(p->ChildCount = ini_str2s2(str, p))
    {
      if(0 != ps1->ChildCount ++)
      {
        ps2->next = p;
        ps2 = p;
      }
      else
      {
        ps1->pChildren = p;
        ps2 = p;
      }
    }
    break;
    
  default:
    return(i);
  }

  // ����
  if(0 == g_hIni->ChildCount)
  {
    g_hIni = 0;
    return -2;
  }

  return 0;
}


/* ��ȡ�ַ������� */
char *  ini_GetVarStr(const char * s1, const char * s2, int32 * pChildCount)
{
  tIniS1 * ps1;
  tIniS2 * ps2;
  int i,j, num;
  
  if(0 != pChildCount) *pChildCount = 0;
  if(0 == g_hIni) return 0;
  
  ps1 = g_hIni->s1List;
  num = g_hIni->ChildCount;
  for(i = 0; i < num; i ++)
  {
    if(0 == strcmp(s1, ps1->s1))
    {
      ps2 = ps1->pChildren;
      num = ps1->ChildCount;
      for(j = 0; j < num; j ++)
      {
        if(0 == strcmp(s2, ps2->s2))
        {
          if(0 != pChildCount) *pChildCount = ps2->ChildCount;
          return(ps2->value);
        }
        ps2 = ps2->next;
      }
      break;
    }
    ps1 = ps1->next;
  }
  return 0;
}


/* ��str��ȡ����(֧��0x��ʽ��16����) */
int  ini_str2hex(const char * str)
{
  int result = 0;
  
  if(0 != str)
  {
    if('\0' != str[0])
    {
      if(('0' == str[0]) && ('x' == str[1]))
      {
        sscanf(str, "%x", &result);
      }
      else sscanf(str, "%d", &result);
    }
  }
  return(result);
}


/* ��ȡ���Ͳ��� */
int  ini_GetVarInt(const char * s1, const char * s2)
{
  return(ini_str2hex(ini_GetVarStr(s1, s2, 0)));
}


/* ��str��ȡ������ */
float  ini_str2float(const char * str)
{
  float result = (float)0.0;
  
  if(0 != str)
  {
    if('\0' != str[0])
    {
      sscanf(str, "%f", &result);
    }
  }
  return(result);
}

/* ��str��ȡ˫���ȸ����� */
double  ini_str2double(const char * str)
{
	double result = (float)0.0;

	if (0 != str)
	{
		if ('\0' != str[0])
		{
			sscanf(str, "%lf", &result);
		}
	}
	return(result);
}

/* ��ȡ������� */
float  ini_GetVarFloat(const char * s1, const char * s2)
{
  return(ini_str2float(ini_GetVarStr(s1, s2, 0)));
}


/* �ͷ��ڴ� */
/* ��ɴ˺����󣬵��÷����ͷ�buff */
void  ini_Finalize()
{
  g_hIni = 0;
}


/* ��buffȡ��32λ�����ַ */
void *  ini_GetAddr32(char * p, int * pSize8)
{
  int i, j;
  
  i = (int)p % sizeof(int);
  if(0 != i)
  {
    j = sizeof(int) - i;
    *pSize8 += j;
    return(p + j);
  }
  return(p);
}


/* ��ȡһ����Ч�ַ��� */
/* �������м�";"�Ժ󲿷� */
/* ����1=��ͷ[s1]��2=���ݣ�0=������-1=�ռ���� */
int  ini_readLine(char ** ppLine)
{
  int num, i = 0;
  const char * str;
  char * p;
  char ch;
  
  if(0 == g_hIni) return 0;
  str = (char *)g_hIni->iniLine;
  if(0 == str) return 0;
  num = g_hIni->buffFree;
  p = g_hIni->buff;
  *ppLine = p;
  
  while(ch = *str ++)
  {
  	if('\0' == ch) break;
  	if(ch==0xff) break;
    if(';' == ch) // ����ע������
    {
      while(ch = *str ++)
      {
        if(('\r' == ch) || ('\n' == ch))
          break;
      }
      if('\0' == ch) break;
    }
    if(('\r' == ch) || ('\n' == ch))
    {
      while(0 != i)
      {
        ch = p[--i];
        if(' ' < (unsigned char)ch) // ��ǰ����հ�
        {
          if(('[' == p[0]) && (']' == ch))
          {
            if(2 > i)
            {
              i = 0;
              break;
            }
            (*ppLine)++;
            p[i] = '\0';
            g_hIni->buffFree -= ++i;
            g_hIni->buff = &p[i];
            g_hIni->iniLine = str;
            return 1;
          }
          p[++i] = '\0';
          g_hIni->buffFree -= ++i;
          g_hIni->buff = &p[i];
          g_hIni->iniLine = str;
          return 2;
        }
      }
      continue;
    }
   
    // ������ǰ�հ�
    if((0 == i) && (' ' >= (unsigned char)ch)) continue;
    p[i ++] = ch; // �����ַ�
    if(i >= num){
    	 return -1;
    }
  }
  
  while(0 != i)
  {
    ch = p[--i];
    if(' ' < (unsigned char)ch) // ��ǰ����հ�
    {
      if(('[' == p[0]) && (']' == ch))
      {
        if(2 > i) break;
        (*ppLine)++;
        p[i] = '\0';
        g_hIni->buffFree -= ++i;
        g_hIni->buff = &p[i];
        g_hIni->iniLine = str;
        return 1;
      }
      p[++i] = '\0';
      g_hIni->buffFree -= ++i;
      g_hIni->buff = &p[i];
      g_hIni->iniLine = str;
      return 2;
    }
  }
  g_hIni->iniLine = str;
  return 0;
}

/* ���ַ����ֽ�ΪtIniS2 */
/* ������","�ָ��Ĳ�����Ŀ */
int  ini_str2s2(char * str, tIniS2 * ps2)
{
  unsigned char * p;
  unsigned char ch;
  int i = 0;
  int j = 1;
  int k;
  
  p = (unsigned char *)str;
  ps2->ChildCount = 0;
  ps2->next = 0;
  // ��ȡs2
  ps2->s2 = str;
  while(ch = p[i ++])
  {
    if('=' == ch)
    {
      for(k = --i; k > 0; )
      {
        if(' ' < p[--k])
        {
          str[++k] = '\0';
          break;
        }
      }
      if(0 == k) return 0;
      while(ch = p[++i])
      {
        if(' ' < ch) break;
      }
      break;
    }
  }
  if('\0' == ch) return 0;
  // ��ȡvalue
  ps2->value = str + i;
  while(ch = p[i ++])
  {
    if(',' == ch) j ++;
  }
  ps2->ChildCount = j;
  return(j);
}




/* ��ȡ���ַ�split���еĵ�n���ֶ� */
char *  ini_splitStr(const char * str, char split, int n, int32 * pLength)
{
  int i = 0;
  char * p;
  char ch;
  
  p = (char *)str;
  if(0 < n) // �ҵ���n���ֶ�
  {
    while(ch = *p ++)
    {
      if(ch == split)
      {
        if(n == ++i) break;
      }
    }
    if(i != n)
    {
      *pLength = 0;
      return 0;
    }
    i = 0;
  }
  
  while(ch = p[i ++])
  {
    if(ch == split) break;
    if('\0' == ch)
    {
      if(0 < i) break;
      *pLength = 0;
      return 0;
    }
  }
  
#if 1
  while(' ' >= *p) // ȥ���ַ���ǰ�հ�
  {
    if(1 >= i) break;
    p ++;
    i --;
  }
#endif
  *pLength = --i;
  return(p);
}

