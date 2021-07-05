/*
* can_ctrl.c --CAN线程
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#include "stdafx.h"
#include "../include/usv_include.h"
#ifdef WINNT
#include "../../win_prj/win_prj/win_drive.h"
#endif

CAN_Struct 	CAN_Msg;
uint8 FAULT_CODE_Last[2][10];
uint8 Sealight_Count,Sealight_Sign;

// #########################################################################
//				CAN接口函数
// #########################################################################


#ifdef WINNT
uint16 read_can(HANDLE hCom,can_frame *buff,uint16 len)
{
	return can_read_win(hCom,buff,len);
	return 1;
}

int8 write_can(HANDLE hCom,can_frame *lpOutBuffer,uint16 write_len)
{

	return (int8)(can_write_win(hCom,lpOutBuffer,write_len));
	return 1;
}

#else
uint16 read_can(int hCom,struct can_frame *buff,uint16 len)
{
	return read(hCom,buff,len);
}

int8 write_can(int hCom,struct can_frame *lpOutBuffer,uint16 write_len)
{
	return write(hCom,lpOutBuffer,write_len);
}
#endif



// ##########################################################################
//DSP报文解析
void DSP_Analyzing(uint16 pgn,uint8 *data)
{
#ifdef	debug_print
	printf("DSP CAN %s\n",data);
#endif
	return	;
}
//无人机平台CAN报文解析
void UAV_Analyzing(uint16 pgn,uint8 *data)
{
	int monitor_cnt;
	if(UAV_PGN==pgn)	
	{
		UAV_CAN_State.Platform_Hatch_spn520576=data[0]&0x07;
		UAV_CAN_State.Platform_Lift_spn520577=((data[0]&0x38)>>3);
		UAV_CAN_State.Platform_Open_spn520578=((data[0]&0xc0)>>6);
		UAV_CAN_State.UAV_Electricity_spn520579=data[1];
		UAV_CAN_State.UAV_Con_spn520580=data[2]&0x0f;
		UAV_CAN_State.UAV_Run_spn520581=(data[2]&0x70)>>4;
		UAV_CAN_State.UAV_Charging_spn520582=((data[2]&0x80)>>7);
		UAV_CAN_State.System_Power_5_spn520583=data[3]&0x01;
		UAV_CAN_State.System_Power_12_spn520584=(data[3]&0x02)>>1;
		UAV_CAN_State.System_Power_3_spn520585=(data[3]&0x04)>>2;
		UAV_CAN_State.System_Power_24_spn520586=(data[3]&0x10)>>4;		
		UAV_CAN_State.System_Check_spn520587=(data[3]&0x08)>>3;
		UAV_CAN_State.System_TEMP_spn502588=data[4];
		
		UAV_Detail_St.UAV_Heatbeat=1;
		USV_State.Conrtol_System_Msg.UAV_platform=0;
/*		printf("Platform_Hatch_spn520576=%d\n",UAV_CAN_State.Platform_Hatch_spn520576);
		printf("Platform_Lift_spn520577=%d\n",UAV_CAN_State.Platform_Lift_spn520577);
		printf("Platform_Open_spn520578=%d\n",UAV_CAN_State.Platform_Open_spn520578);
		printf("UAV_Electricity_spn520579=%d\n",UAV_CAN_State.UAV_Electricity_spn520579);
		printf("UAV_Con_spn520580=%d\n",UAV_CAN_State.UAV_Con_spn520580);
		printf("UAV_Run_spn520581=%d\n",UAV_CAN_State.UAV_Run_spn520581);
		printf("UAV_Charging_spn520582=%d\n",UAV_CAN_State.UAV_Charging_spn520582);
		printf("System_Power_5_spn520583=%d\n",UAV_CAN_State.System_Power_5_spn520583);
		printf("System_Power_12_spn520584=%d\n",UAV_CAN_State.System_Power_12_spn520584);
		printf("System_Power_3_spn520585=%d\n",UAV_CAN_State.System_Power_3_spn520585);
		printf("System_Power_24_spn520586=%d\n",UAV_CAN_State.System_Power_24_spn520586);
		printf("System_Check_spn520587=%d\n",UAV_CAN_State.System_Check_spn520587);
		printf("System_TEMP_spn502588=%d\n",UAV_CAN_State.System_TEMP_spn502588);

		printf("UAV_Heatbeat=%d\n",UAV_Detail_St.UAV_Heatbeat);*/

		//监控数据转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1 ; monitor_cnt++)
			monitor_all_inf.module_report_inf[MONITOR_UAV_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];
		
	}
	else if(65321==pgn)
	{
		sprintf_usv(Version[3],"UAV_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		UAVVersion_sign=1;
	}
#ifdef	debug_print
	else
		printf("UAV CAN %s\n",data);
#endif
	return	;
}
//智能舵CAN报文解析
void SR_Analyzing(uint16 pgn,uint8 *data)
{
//	SR_CAN_State.Rudder_Angle_L_spn520766=500;
//	SR_CAN_State.Rudder_Angle_R_spn520767=500;
	int monitor_cnt;
	if(SR_Speed_PGN==pgn)	
	{
		SR_CAN_State.Rudder_Angle_L_spn520766=data[1]<<8;//注意高字节在前
		SR_CAN_State.Rudder_Angle_L_spn520766+=data[0];
		SR_CAN_State.Rudder_Angle_R_spn520767=data[3]<<8;
		SR_CAN_State.Rudder_Angle_R_spn520767+=data[2];
		SR_CAN_State.Motor_Gear_L_spn520768=data[4];
		SR_CAN_State.Motor_Gear_R_spn520769=data[5];
		SR_CAN_State.Rudder_Major_Failure = data[6];
		SR_CAN_State.Rudder_Minor_Failure = data[7];
		Rudder_Detail_St.Heatbeat_Rudder=1;
		if(SR_CAN_State.Rudder_Angle_L_spn520766<250)
			SR_CAN_State.Rudder_Angle_L_spn520766=250;//舵角最小为负25°
		if(SR_CAN_State.Rudder_Angle_L_spn520766>750)
			SR_CAN_State.Rudder_Angle_L_spn520766=750;//舵角最大为正75°
		if(SR_CAN_State.Rudder_Angle_R_spn520767<250)
			SR_CAN_State.Rudder_Angle_R_spn520767=250;//舵角最小为负25°
		if(SR_CAN_State.Rudder_Angle_R_spn520767>750)
			SR_CAN_State.Rudder_Angle_R_spn520767=750;//舵角最大为正75°
		
		//2017年5月23日17:17:33 临时修改
		SR_CAN_State.Rudder_Angle_R_spn520767 = 1000 - SR_CAN_State.Rudder_Angle_R_spn520767;

		Rudder_Detail_St.Heatbeat_Rudder=1;
		USV_State.Conrtol_System_Msg.Intelligent_rudder=0;
/*		printf("Motor_Speed_L_spn520766=0x%2x\n",SR_CAN_State.Rudder_Angle_L_spn520766);
		printf("Motor_Speed_R_spn520767=0x%2x\n",SR_CAN_State.Rudder_Angle_R_spn520767);
		printf("Motor_Gear_L_spn520768=0x%2x\n",SR_CAN_State.Motor_Gear_L_spn520768);
		printf("Motor_Gear_R_spn520769=0x%2x\n",SR_CAN_State.Motor_Gear_R_spn520769);*/

		//监控数据转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
			monitor_all_inf.module_report_inf[MONITOR_SR_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];


	}
	else if(SR_Rudder_PGN==pgn)	
	{
		SR_CAN_State.Motor_Speed_L_spn520770=data[1]<<8;
		SR_CAN_State.Motor_Speed_L_spn520770+=data[0];
		SR_CAN_State.Motor_Speed_R_spn520771=data[3]<<8;
		SR_CAN_State.Motor_Speed_R_spn520771+=data[2];
		SR_CAN_State.Damper_st_L_spn520774=(data[4]&0x04)>>2;
		SR_CAN_State.Damper_st_R_spn520775=(data[4]&0x08)>>3;
		SR_CAN_State.Outboard_Engine_st_L_spn520776=(data[4]&0x30)>>4;
		SR_CAN_State.Outboard_Engine_st_R_spn520777=(data[4]&0xc0)>>6;
		SR_CAN_State.Config_Answer_spn520778=data[5]&0x01;
		SR_CAN_State.System_Power_5_spn520779=(data[5]&0x02)>>1;
		SR_CAN_State.System_Power_12_spn520780=(data[5]&0x02)>>1;
		SR_CAN_State.System_Power_3_spn520781=(data[5]&0x04)>>2;
		SR_CAN_State.System_Power_24_spn520782=(data[5]&0x08)>>3;
		SR_CAN_State.System_Check_spn520783=(data[5]&0x10)>>4;
		SR_CAN_State.System_TEMP_spn520784=data[6];
		Rudder_Detail_St.Heatbeat_Rudder=1;
		USV_State.Conrtol_System_Msg.Intelligent_rudder=0;

		//监控数据转存
		for(monitor_cnt = CAN_PART1; monitor_cnt < CAN_PART2; monitor_cnt++)
			monitor_all_inf.module_report_inf[MONITOR_SR_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART1];
	}
//	printf("motor=%d-%d\n",SR_CAN_State.Motor_Speed_L_spn520770,SR_CAN_State.Motor_Speed_R_spn520771);
	else if(65332==pgn)
	{
		sprintf_usv(Version[4],"SR_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		SRVersion_sign=1;
	}
#ifdef	debug_print
	else
		printf("SR CAN %s\n",data);
#endif
	return	;
}
//电源管理系统CAN报文解析
void POW_CON_Analyzing(uint16 pgn,uint8 *data)
{
	uint8 monitor_cnt;
	if(POW_CON_PGN==pgn)	
	{
		POW_CAN_State.Stable_Platform_Power_spn520958=(data[0]&0x01);
		POW_CAN_State.UAV_Power_spn520959=((data[0]&0x02)>>1);
		POW_CAN_State.Horn_Power_spn520960=((data[0]&0x04)>>2);
		POW_CAN_State.Navigationlight_Power_spn520961=((data[0]&0x08)>>3);
		POW_CAN_State.D_radar_Power_spn520962=((data[0]&0x10)>>4);
		POW_CAN_State.Camera_main_Power_spn520963=((data[0]&0x20)>>5);
		POW_CAN_State.Application_24V_spn520964=((data[0]&0x40)>>6);
		POW_CAN_State.Searchlight_Power_spn520965=((data[0]&0x80)>>7);
		POW_CAN_State.Camera_ahead_Power_spn520966=(data[1]&0x01);
		POW_CAN_State.Camera_lesser_Power_spn520967=((data[1]&0x02)>>1);
		POW_CAN_State.Camera_tail_Power_spn520968=((data[1]&0x04)>>2);
		POW_CAN_State.Application_12V_spn520969=((data[1]&0x08)>>3);
		POW_CAN_State.System_Power_5_spn520970=((data[1]&0x10)>>4);
		POW_CAN_State.System_Power_12_spn520971=((data[1]&0x20)>>5);
		POW_CAN_State.System_Power_3_spn520972=((data[1]&0x40)>>6);
		POW_CAN_State.System_Check_spn520973=((data[1]&0x80)>>7);
		POW_CAN_State.System_Voltage_spn520974=data[2];
		POW_CAN_State.System_Electricity_spn520975=data[3];
		POW_CAN_State.DIN1_Connection_St_spn520976=(data[4]&0x01);
		POW_CAN_State.DIN2_Connection_St_spn520977=((data[4]&0x02)>>1);
		POW_CAN_State.DIN3_Connection_St_spn520978=((data[4]&0x04)>>2);
		POW_CAN_State.DIN4_Connection_St_spn520979=((data[4]&0x08)>>3);
		POW_CAN_State.System_TEMP_spn502980=data[5];		
		POW_CAN_State.System_Heatbeat=1;
		USV_State.Conrtol_System_Msg.Power_management=0;

		//监控数据转存
		for(monitor_cnt = 0; monitor_cnt < 8 ; monitor_cnt++)
			monitor_all_inf.module_report_inf[MONITOR_POW_MSG].report_detail[monitor_cnt] = data[monitor_cnt];

		
/*		if((1==POW_CAN_State.System_Voltage_spn520974)||(1==POW_CAN_State.System_Electricity_spn520975))
			USV_State.Conrtol_System_Msg.Power_management=1;
		else
			USV_State.Conrtol_System_Msg.Power_management=0;*/
/*		for(count=0;count<8;count++)
			printf(" 0x%.2x",data[count]);	
		printf("\n");			
		printf("Stable_Platform_Power_spn520958=%d\n",POW_CAN_State.Stable_Platform_Power_spn520958);
		printf("UAV_Power_spn520959=%d\n",POW_CAN_State.UAV_Power_spn520959);
		printf("Horn_Power_spn520960=%d\n",POW_CAN_State.Horn_Power_spn520960);
		printf("Navigationlight_Power_spn520961=%d\n",POW_CAN_State.Navigationlight_Power_spn520961);
		printf("D_radar_Power_spn520962=%d\n",POW_CAN_State.D_radar_Power_spn520962);
		printf("Camera_main_Power_spn520963=%d\n",POW_CAN_State.Camera_main_Power_spn520963);
		printf("Application_24V_spn520964=%d\n",POW_CAN_State.Application_24V_spn520964);
		printf("Searchlight_Power_spn520965=%d\n",POW_CAN_State.Searchlight_Power_spn520965);
		printf("Camera_ahead_Power_spn520966=%d\n",POW_CAN_State.Camera_ahead_Power_spn520966);
		printf("Camera_lesser_Power_spn520967=%d\n",POW_CAN_State.Camera_lesser_Power_spn520967);
		printf("Camera_tail_Power_spn520968=%d\n",POW_CAN_State.Camera_tail_Power_spn520968);
		printf("Application_12V_spn520969=%d\n",POW_CAN_State.Application_12V_spn520969);
		printf("System_Power_5_spn520970=%d\n",POW_CAN_State.System_Power_5_spn520970);
		printf("System_Power_12_spn520971=%d\n",POW_CAN_State.System_Power_12_spn520971);
		printf("System_Power_3_spn520972=%d\n",POW_CAN_State.System_Power_3_spn520972);
		printf("System_Check_spn520973=%d\n",POW_CAN_State.System_Check_spn520973);
		printf("System_Voltage_spn520974=%d\n",POW_CAN_State.System_Voltage_spn520974);
		printf("System_Electricity_spn520975=%d\n",POW_CAN_State.System_Electricity_spn520975);
		printf("DIN1_Connection_St_spn520976=%d\n",POW_CAN_State.DIN1_Connection_St_spn520976);
		printf("DIN2_Connection_St_spn520977=%d\n",POW_CAN_State.DIN2_Connection_St_spn520977);
		printf("DIN3_Connection_St_spn520978=%d\n",POW_CAN_State.DIN3_Connection_St_spn520978);
		printf("DIN4_Connection_St_spn520979=%d\n",POW_CAN_State.DIN4_Connection_St_spn520979);
		printf("System_TEMP_spn502980=%d\n",POW_CAN_State.System_TEMP_spn502980);
		*/
//		printf("Power_Heatbeat=%d\n",POW_CAN_State.System_Heatbeat);

	}
	else if(65341==pgn)
	{
		sprintf_usv(Version[5],"POW_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		POWVersion_sign=1;
	}

#ifdef	debug_print
	else
		printf("POW CAN %s\n",data);
#endif
	return	;
}
//智能面板CAN报文解析cxy
//处理智能面板下发的报文
void SP_Analyzing(uint16 pgn,uint8 *data)
{
	int	monitor_cnt;
	if(SP_Model_PGN==pgn)											//智能面板控制模式 65353
	{
		SP_CAN_Model_Con.Emergency_spn521192=data[0]&0x03;			//应急模式
		SP_CAN_Model_Con.Get_USV_Con_spn521193=((data[0]&0x0c)>>2);			//船端模式 ，获取无人船控制权限 1--面板控制权限 ，0--放弃
		SP_CAN_Model_Con.System_Heatbeat=1;
		if(SP_CAN_Model_Con.Get_USV_Con_spn521193==1)					//船体智能面板获取控制权限
			SP_CAN_Count++;
		else															//船体智能面板放弃控制权限
			SP_CAN_Count=0;	
		if(SP_CAN_Count>=5)												//连续5次
		{
			SP_CON_Sign=1;						//船体控制	智能面板控制					
			Get_Control_Sign=1;
			SP_CAN_Count=5;
		}
		else
			SP_CON_Sign=0;       //船体控制放弃	 面板放弃控制	

		USV_Control.USV_Control_Message[4].trans_rot_cmd.translation_mode = data[1]&0x03;
		USV_Control.USV_Control_Message[4].trans_rot_cmd.translation_command = ((data[1]&0x0c)>>2);
		USV_Control.USV_Control_Message[4].trans_rot_cmd.rotation_mode = ((data[1]&0x30)>>4);
		USV_Control.USV_Control_Message[4].trans_rot_cmd.rotation_command = ((data[1]&0xc0)>>6);

		SP_CAN_Model_Con.translation_mode    = data[1]&0x03;
		SP_CAN_Model_Con.translation_command = ((data[1]&0x0c)>>2);
		SP_CAN_Model_Con.rotation_mode       = ((data[1]&0x30)>>4);
		SP_CAN_Model_Con.rotation_command	 = ((data[1]&0xc0)>>6);

	//	printf("trans mode = %d\t trans cmd = %d\t rotate mode = %d\t rotate cmd = %d\n",SP_CAN_Model_Con.translation_mode,SP_CAN_Model_Con.translation_command,SP_CAN_Model_Con.rotation_mode,SP_CAN_Model_Con.rotation_command);

//		printf("SP_CON_Sign=%d-0x%.2x\n",SP_CON_Sign,data[0]);		
	}
	if(SP_CON1_PGN==pgn)						//智能面板控制PGN1
	{
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.UAV_Power=(data[0]&0x03);
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Stable_Platform_Power=(data[0]&0x0c)>>2;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Camera_ahead_Power=(data[0]&0x30)>>4;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.D_radar_Power=data[0]>>6;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Camera_main_Power=(data[1]&0x03);
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Horn_Power=(data[1]&0x0c)>>2;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Searchlight_Power=(data[1]&0x30)>>4;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Camera_lesser_Power=data[1]>>6;
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Navigationlight_Power=(data[2]&0x03);
		USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Camera_tail_Power=(data[2]&0x0c)>>2;
		USV_Control.USV_Control_Message[4].Navigation_Tsak_sign=(data[2]&0xf0)>>4;
		USV_Control.USV_Control_Message[4].Engine_Run_L=(data[3]&0x03);
		USV_Control.USV_Control_Message[4].Engine_Run_R=(data[3]&0x0C)>>2;
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Sailing_Mod=(data[3]&0x30)>>4;
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Differential_Mod=(data[3]&0xc0)>>6;
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Set_Return_Point_Mod=(data[4]&0x03);
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Speed_Constant_Mod=(data[4]&0x0c)>>2;
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Direction_Constant_Mod=(data[4]&0x30)>>4;
		USV_Control.USV_Control_Message[4].Dradio_USV_Model.Return_Mod=(data[4]&0xc0)>>6;
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Gear_Left=data[5];
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Gear_Right=data[6];
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Accelerator_Left=data[7];
		SP_CAN_Model_Con.System_Heatbeat=1;		

		//监控转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_SP_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];
				
	}
	if(SP_CON2_PGN==pgn)										//智能面板控制PGN2
	{
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Accelerator_Right=data[0];
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Rudder_Angle_Left=data[1];
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Rudder_Angle_Right=data[2];
		USV_Control.USV_Control_Message[4].Speed_Limit=data[3];
		USV_Control.USV_Control_Message[4].Dradio_USV_Stable_Platform.Stable_Platform_AT=data[4];
		USV_Control.USV_Control_Message[4].Dradio_USV_Stable_Platform.Stable_Platform_RL=data[5];
		USV_Control.USV_Control_Message[4].Dradio_USV_UAV_Control.Platform_Hatch=(data[6]&0x03);
		USV_Control.USV_Control_Message[4].Dradio_USV_UAV_Control.Platform_Lift=(data[6]&0x0c)>>2;
		USV_Control.USV_Control_Message[4].Dradio_USV_UAV_Control.Platform_Open=(data[6]&0x30)>>4;
		USV_Control.USV_Control_Message[4].Dradio_USV_UAV_Control.UAV_Charging=(data[6]&0xc0)>>6;
		USV_Control.USV_Control_Message[4].Hull_Fan_Con=data[7]&0x03;
		USV_Control.USV_Control_Message[4].Hull_Pump_Con=(data[7]&0x3C)>>2;
		USV_Control.USV_Control_Message[4].Oil_Con=(data[7]&0xC0)>>6;
		SP_CAN_Model_Con.System_Heatbeat=1;		

		//监控转存
		for(monitor_cnt = CAN_PART1; monitor_cnt < CAN_PART2; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_SP_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART1];
		
//		printf("Oil_Con=%d\n",USV_Control.USV_Control_Message[4].Oil_Con);
	}
	if(SP_CON3_PGN==pgn)
	{
		USV_Control.USV_Control_Message[4].Application_24=(data[0]&0x03);
		USV_Control.USV_Control_Message[4].Application_12=((data[0]&0x0C)>>2);
		USV_Control.USV_Control_Message[4].Sealight_Speed=((data[0]&0x10)>>4);
		USV_Control.USV_Control_Message[4].Sealight_Direction=((data[0]&0xe0)>>5);
		USV_Control.USV_Control_Message[4].Emergency_Stop=(data[1]&0x01);
		USV_Control.USV_Control_Message[4].Outboard_Engine_L=((data[1]&0x06)>>1);
		USV_Control.USV_Control_Message[4].Outboard_Engine_R=((data[1]&0x18)>>3);
		SP_CAN_Model_Con.System_Heatbeat=1;		

		//监控转存
		for(monitor_cnt = CAN_PART2; monitor_cnt < CAN_PART3; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_SP_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART2];
		
	}
	
/*	printf("Gear_Left=%d-%d-%d-%d-%d-%d\n",USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Gear_Left,
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Gear_Right,
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Rudder_Angle_Left,
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Rudder_Angle_Right,
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Accelerator_Left,
		USV_Control.USV_Control_Message[4].Dradio_USV_Drive.Accelerator_Right);*/
	if(65354==pgn)
	{
		sprintf_usv(Version[6],"SP_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		SPVersion_sign=1;
//		printf("Version5=%s",Version[5]);
	}
#ifdef	debug_print
	else
		printf("SP CAN %s\n",data);
#endif

	return	;
}
//稳定平台CAN报文解析
void ST_PL_Analyzing(uint16 pgn,uint8 *data)
{
	int monitor_cnt;
	if(ST_PL_PGN==pgn)	
	{
		ST_PL_CAN_State.Stable_Platform_RL_spn521342	=data[0];
		ST_PL_CAN_State.Stable_Platform_AT_spn521343=data[1];
		ST_PL_CAN_State.System_TEMP_spn520344=data[2];
		ST_PL_CAN_State.System_Power_5_spn521345=data[3]&0x01;
		ST_PL_CAN_State.System_Power_12_spn521346=(data[3]&0x02)>>1;
		ST_PL_CAN_State.System_Power_3_spn521347=(data[3]&0x04)>>1;
		ST_PL_CAN_State.System_Power_24_spn521348=(data[3]&0x08)>>3;
		ST_PL_CAN_State.System_Check_spn521349=(data[3]&0x10)>>4;
		Stable_Platform_St.Stable_Platform_Heatbeat=1;
		USV_State.Conrtol_System_Msg.Stabilized_platform=0;
//		printf("Stable_Platform_AT=%d-%d\n",data[0],data[1]);

		//监控转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_ST_PL_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];

	}
	else if(65361==pgn)
	{
		sprintf_usv(Version[7],"ST_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		STVersion_sign=1;
	}
#ifdef	debug_print
	else
		printf("ST_PL CAN %s\n",data);
#endif
	return;
}
//能源管理系统CAN报文解析
void BAT_CON_Analyzing(uint16 pgn,uint8 *data)
{
	int monitor_cnt;
	if(BAT_ST_PGN==pgn)	
	{
		BAT_CON_CAN_State.USV_Pow_spn521534=data[0]&0x0f;
		BAT_CON_CAN_State.Battery_Charge_L_spn521535=(data[1]&0x01);
		BAT_CON_CAN_State.Battery_Chager_R_spn521536=((data[1]&0x02)>>1);
		BAT_CON_CAN_State.Battery_TEMP_Alarm_L_spn521537=((data[1]&0x04)>>2);
		BAT_CON_CAN_State.Battery_TEMP_ALarm_R_spn521538=((data[1]&0x08)>>3);
		BAT_CON_CAN_State.Battery_Capacity_L_spn521539=((data[1]&0x10)>>4);
		BAT_CON_CAN_State.Battery_Capacity_R_spn521540=((data[1]&0x20)>>5);
		BAT_CON_CAN_State.Plug_Connection_St_spn521541=(data[2]&0x01);
		BAT_CON_CAN_State.Plug_Connection_St_spn521542=((data[2]&0x02)>>1);
		BAT_CON_CAN_State.Plug_Connection_St_spn521543=((data[2]&0x04)>>2);
		BAT_CON_CAN_State.Plug_Connection_St_spn521544=((data[2]&0x08)>>3);
		BAT_CON_CAN_State.Plug_Connection_St_spn521545=((data[2]&0x10)>>4);
		BAT_CON_CAN_State.Plug_Connection_St_spn521546=((data[2]&0x20)>>5);
		BAT_CON_CAN_State.Plug_Connection_St_spn521547=((data[2]&0x40)>>6);
		BAT_CON_CAN_State.Plug_Connection_St_spn521548=((data[2]&0x80)>>7);
		BAT_CON_CAN_State.System_Power_5_spn521549=(data[3]&0x01);
		BAT_CON_CAN_State.System_Power_12_spn521550=((data[3]&0x02)>>1);
		BAT_CON_CAN_State.System_Power_3_spn521551=((data[3]&0x04)>>2);
		BAT_CON_CAN_State.System_Power_24_spn521552=((data[3]&0x08)>>3);
		BAT_CON_CAN_State.System_Check_spn521553=((data[3]&0x10)>>4);		
		BAT_CON_CAN_State.System_TEMP_spn520566=data[4];		
		USV_State.Conrtol_System_Msg.Energy_management=0;
		Energy_Control_St.Power_Control_Heatbeat=1;

		//监控转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_BAT_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];
		
/*		for(count=0;count<8;count++)
			printf(" 0x%.2x",data[count]);	
		printf("\n");
		printf("USV_Pow_spn521534=%d\n",BAT_CON_CAN_State.USV_Pow_spn521534);
		printf("Battery_Charge_L_spn521535=%d\n",BAT_CON_CAN_State.Battery_Charge_L_spn521535);
		printf("Battery_Chager_R_spn521536=%d\n",BAT_CON_CAN_State.Battery_Chager_R_spn521536);
		printf("Battery_TEMP_Alarm_L_spn521537=%d\n",BAT_CON_CAN_State.Battery_TEMP_Alarm_L_spn521537);
		printf("Battery_TEMP_ALarm_R_spn521538=%d\n",BAT_CON_CAN_State.Battery_TEMP_ALarm_R_spn521538);
		printf("Battery_Capacity_L_spn521539=%d\n",BAT_CON_CAN_State.Battery_Capacity_L_spn521539);
		printf("Battery_Capacity_R_spn521540=%d\n",BAT_CON_CAN_State.Battery_Capacity_R_spn521540);
		printf("Plug_Connection_St_spn521541=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521541);
		printf("Plug_Connection_St_spn521542=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521542);
		printf("Plug_Connection_St_spn521543=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521543);
		printf("Plug_Connection_St_spn521544=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521544);
		printf("Plug_Connection_St_spn521545=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521545);
		printf("Plug_Connection_St_spn521546=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521546);
		printf("Plug_Connection_St_spn521547=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521547);
		printf("Plug_Connection_St_spn521548=%d\n",BAT_CON_CAN_State.Plug_Connection_St_spn521548);
		printf("System_Power_5_spn521549=%d\n",BAT_CON_CAN_State.System_Power_5_spn521549);
		printf("System_Power_12_spn521550=%d\n",BAT_CON_CAN_State.System_Power_12_spn521550);
		printf("System_Power_3_spn521551=%d\n",BAT_CON_CAN_State.System_Power_3_spn521551);
		printf("System_Power_24_spn521552=%d\n",BAT_CON_CAN_State.System_Power_24_spn521552);
		printf("System_Check_spn521553=%d\n",BAT_CON_CAN_State.System_Check_spn521553);
		printf("System_TEMP_spn520566=%d\n",BAT_CON_CAN_State.System_TEMP_spn520566);*/
		
//		printf("Energy_Heatbeat=%d\n",Energy_Control_St.Power_Control_Heatbeat);
		
		
	}
	else if(BAT_L_PGN==pgn)
	{
		BAT_CON_CAN_State.Battery_Left_spn521554=data[0];
		BAT_CON_CAN_State.Battery_TEMP_L_spn521555=data[1];
		BAT_CON_CAN_State.Battery_Power_L_spn521556=data[2];
		BAT_CON_CAN_State.Battery_Current_L_spn521557=data[3];
		BAT_CON_CAN_State.Battery_Voltage_L_spn521558=data[4];
		BAT_CON_CAN_State.Oil_L_spn521559=(uint8)(data[5]/1.9);		
		USV_State.Conrtol_System_Msg.Energy_management=0;		
		Energy_Control_St.Power_Control_Heatbeat=1;

		//监控转存
		for(monitor_cnt = CAN_PART1; monitor_cnt < CAN_PART2; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_BAT_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART1];

		
/*		printf("Battery_Left_spn521554=%d\n",BAT_CON_CAN_State.Battery_Left_spn521554);
		printf("Battery_TEMP_L_spn521555=%d\n",BAT_CON_CAN_State.Battery_TEMP_L_spn521555);
		printf("Battery_Power_L_spn521556=%d\n",BAT_CON_CAN_State.Battery_Power_L_spn521556);
		printf("Battery_Current_L_spn521557=%d\n",BAT_CON_CAN_State.Battery_Current_L_spn521557);
		printf("Battery_Voltage_L_spn521558=%d\n",BAT_CON_CAN_State.Battery_Voltage_L_spn521558);
		printf("Oil_L_spn521559=%d\n",BAT_CON_CAN_State.Oil_L_spn521559);*/
//		printf("Oil_L_spn521559=%d\n",BAT_CON_CAN_State.Oil_L_spn521559);
//		printf("Battery_Left_spn521554=%d\n",BAT_CON_CAN_State.Battery_Left_spn521554);
		
	}
	else if(BAT_R_PGN==pgn)
	{
		BAT_CON_CAN_State.Battery_Right_spn521560=data[0];
		BAT_CON_CAN_State.Battery_TEMP_R_spn521561=data[1];
		BAT_CON_CAN_State.Battery_Power_R_spn521562=data[2];
		BAT_CON_CAN_State.Battery_Current_R_spn521563=data[3];
		BAT_CON_CAN_State.Battery_Voltage_R_spn521564=data[4];
		BAT_CON_CAN_State.Oil_R_spn521565=(uint8)(data[5]/1.9);
		USV_State.Conrtol_System_Msg.Energy_management=0;		
		Energy_Control_St.Power_Control_Heatbeat=1;

		//监控转存
		for(monitor_cnt = CAN_PART2; monitor_cnt < CAN_PART3; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_BAT_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART2];
		
/*		printf("Battery_Right_spn521560=%d\n",BAT_CON_CAN_State.Battery_Right_spn521560);
		printf("Battery_TEMP_R_spn521561=%d\n",BAT_CON_CAN_State.Battery_TEMP_R_spn521561);
		printf("Battery_Power_R_spn521562=%d\n",BAT_CON_CAN_State.Battery_Power_R_spn521562);
		printf("Battery_Current_R_spn521563=%d\n",BAT_CON_CAN_State.Battery_Current_R_spn521563);
		printf("Battery_Voltage_R_spn521564=%d\n",BAT_CON_CAN_State.Battery_Voltage_R_spn521564);
		printf("Oil_R_spn521565=%d\n",BAT_CON_CAN_State.Oil_R_spn521565);*/
//		printf("Oil_R_spn521565=%d\n",BAT_CON_CAN_State.Oil_R_spn521565);
//		printf("Battery_Right_spn521560=%d\n",BAT_CON_CAN_State.Battery_Right_spn521560);
		
	}
	else if(65373==pgn)
	{
		sprintf_usv(Version[8],"BAT_V0.%d_R%d_%04XH",data[1]*256+data[0],data[3]*256+data[2],data[4]*256+data[5]);	
		BATVersion_sign=1;
	}
#ifdef	debug_print
	else
		printf("BAT_CON CAN %s\n",data);
#endif
	return	;
}
//左发动机CAN报文解析
void Motor_L_Analyzing(uint16 pgn,uint8 *data)
{
//	int count;
	uint8	msg_rev=0;	
	int monitor_cnt;
//	USV_State.Engine_power=USV_State.Engine_power|0x01;//收到左发动机报文，左发动机已上电
	if(61444==pgn)
	{
/*		for(count=0;count<8;count++)
			printf("Lmotor=0x%.2x",data[count]);
		printf("\n");*/
		Motor_CAN_State.Motor_PGN61444_State[0].Torque_Mod_spn899=(data[0]&0x0f);
		Motor_CAN_State.Motor_PGN61444_State[0].Torque_Percent_EXP_spn512=data[1];
		Motor_CAN_State.Motor_PGN61444_State[0].Torque_Percent_ACT_spn513=data[2];
		Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190=(data[4]<<8);
		Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190+=data[3];
		Motor_CAN_State.Motor_PGN61444_State[0].Motor_CON_ADD_spn1483=data[5];
		Motor_CAN_State.Motor_PGN61444_State[0].Mortor_Starter_Mod_spn1675=(data[6]&0x0f);
		if(Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190>500*8)
			USV_State.Engine_run=USV_State.Engine_run|0x01;//左发动机转速不为0，已启动

		//监控转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];
		msg_rev=1;
	}
	else if(65244==pgn)
	{
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Fuel_Consume_spn236=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Fuel_Consume_spn236+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Fuel_Consume_spn236+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Fuel_Consume_spn236+=data[0];//cxy
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART1; monitor_cnt < CAN_PART2; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART1];
		msg_rev=1;
	}
	else if(65253==pgn)
	{
		Motor_CAN_State.Motor_PGN65253_State[0].Working_Time_spn247=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65253_State[0].Working_Time_spn247+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65253_State[0].Working_Time_spn247+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65253_State[0].Working_Time_spn247+=data[0];
		Motor_CAN_State.Motor_PGN65253_State[0].Total_turn_spn249=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65253_State[0].Total_turn_spn249+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65253_State[0].Total_turn_spn249+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65253_State[0].Total_turn_spn249+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART2; monitor_cnt < CAN_PART3; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART2];
		msg_rev=1;
	}
	else if(65257==pgn)
	{
		Motor_CAN_State.Motor_PGN65257_State[0].Single_Fuel_Consume_spn182=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65257_State[0].Single_Fuel_Consume_spn182+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65257_State[0].Single_Fuel_Consume_spn182+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65257_State[0].Single_Fuel_Consume_spn182+=data[0];
		Motor_CAN_State.Motor_PGN65257_State[0].Total_Fuel_Consume_spn250=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65257_State[0].Total_Fuel_Consume_spn250+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65257_State[0].Total_Fuel_Consume_spn250+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65257_State[0].Total_Fuel_Consume_spn250+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART3; monitor_cnt < CAN_PART4; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART3];
		msg_rev=1;
	}
	else if(65262==pgn)
	{
		Motor_CAN_State.Motor_PGN65262_State[0].Cool_Temp_spn110=data[0];
		Motor_CAN_State.Motor_PGN65262_State[0].Fuel_Temp_spn174=data[1];
		Motor_CAN_State.Motor_PGN65262_State[0].Oil_Temp_spn175=(data[3]<<8);
		Motor_CAN_State.Motor_PGN65262_State[0].Oil_Temp_spn175+=data[2];
		Motor_CAN_State.Motor_PGN65262_State[0].Tuibine_Oil_Temp_spn176=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65262_State[0].Tuibine_Oil_Temp_spn176+=data[4];
		Motor_CAN_State.Motor_PGN65262_State[0].Motor_Cooler_Temp_spn52=data[6];
		Motor_CAN_State.Motor_PGN65262_State[0].Mortor_Cooler_Opening_spn1134=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART4; monitor_cnt < CAN_PART5; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART4];
		msg_rev=1;
	}
	else if(65263==pgn)
	{
		Motor_CAN_State.Motor_PGN65263_State[0].Fuel_Delivery_Pressure_spn94=data[0];
		Motor_CAN_State.Motor_PGN65263_State[0].Extension_Crankcase_Leakage_spn22=data[1];
		Motor_CAN_State.Motor_PGN65263_State[0].Oil_Level_spn98=data[2];
		Motor_CAN_State.Motor_PGN65263_State[0].Oil_Pressure_spn100=data[3];
		Motor_CAN_State.Motor_PGN65263_State[0].Crankcase_Pressure_spn101=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65263_State[0].Crankcase_Pressure_spn101+=data[4];
		Motor_CAN_State.Motor_PGN65263_State[0].Cool_Pressure_spn109=data[6];
		Motor_CAN_State.Motor_PGN65263_State[0].Cool_Level_spn111=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART5; monitor_cnt < CAN_PART6; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART5];
		msg_rev=1;
	}
	else if(65266==pgn)
	{
		Motor_CAN_State.Motor_PGN65266_State[0].Fuel_Usage_spn183=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65266_State[0].Fuel_Usage_spn183+=data[0];
		Motor_CAN_State.Motor_PGN65266_State[0].Instant_Fuel_Economy_spn184=(data[3]<<8);
		Motor_CAN_State.Motor_PGN65266_State[0].Instant_Fuel_Economy_spn184+=data[2];
		Motor_CAN_State.Motor_PGN65266_State[0].Average_Fuel_Economy_spn185=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65266_State[0].Average_Fuel_Economy_spn185+=data[4];
		Motor_CAN_State.Motor_PGN65266_State[0].Throttle_Position_spn51=(data[7]<<8);
		Motor_CAN_State.Motor_PGN65266_State[0].Throttle_Position_spn51+=data[6];

		//监控转存
		for(monitor_cnt = CAN_PART6; monitor_cnt < CAN_PART7; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART6];
		msg_rev=1;		
	}
	else if(655269==pgn)
	{
		Motor_CAN_State.Motor_PGN65269_State[0].Air_Pressure_spn108=data[0];
		Motor_CAN_State.Motor_PGN65269_State[0].AIr_Temp_spn171=(data[4]<<8);
		Motor_CAN_State.Motor_PGN65269_State[0].AIr_Temp_spn171+=data[3];
		Motor_CAN_State.Motor_PGN65269_State[0].Air_inlet_Temp_spn172=data[5];

		//监控转存
		for(monitor_cnt = CAN_PART7; monitor_cnt < CAN_PART8; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART7];
		msg_rev=1;		
	}
	else if(655270==pgn)
	{
		Motor_CAN_State.Motor_PGN65270_State[0].Supercharger_Pressure_spn102=data[1];
		Motor_CAN_State.Motor_PGN65270_State[0].Inlet_Air_Temp_spn105=data[2];
		Motor_CAN_State.Motor_PGN65270_State[0].Air_inlet_Pressure_spn106=data[3];
		Motor_CAN_State.Motor_PGN65270_State[0].Air_fillter_Diff_Pressure_spn107=data[4];
		Motor_CAN_State.Motor_PGN65270_State[0].Exhaust_Temp_spn173=(data[6]<<8);
		Motor_CAN_State.Motor_PGN65270_State[0].Exhaust_Temp_spn173+=data[5];
		Motor_CAN_State.Motor_PGN65270_State[0].Cool_Fillter_Diff_Pressure_spn112=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART8; monitor_cnt < CAN_PART9; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART8];
		msg_rev=1;
	}	
	else if(655279==pgn)
	{
		Motor_CAN_State.Motor_PGN65279_State[0].Fuel_Moisture_spn97=(data[0]&0x03);

		//监控转存
		for(monitor_cnt = CAN_PART9; monitor_cnt < CAN_PART10; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART9];		
		msg_rev=1;
	}
	else if(60416==pgn)//多包传输，DM1用
	{
		memset((uint8 *)&Receive_Status[0].PGN,0,sizeof(Receive_Status));
		
		if(data[0]==TP_CM_BAM)
		{
			Receive_Status[0].byte_count+=data[1];
			Receive_Status[0].byte_count=(data[2]<<8);
			Receive_Status[0].total_packet_number=data[3];
			Receive_Status[0].Dest_PGN=data[5];
			Receive_Status[0].Dest_PGN+=(data[6]<<8);
		}
#ifdef	debug_print
		else if(TP_CM_RTS==data[0])
		{
			printf("Motor_L_60416 CAN %s\n",data);
		}
		else
			printf("Motor_L_60416 CAN %s\n",data);
#endif

		//监控转存
		for(monitor_cnt = CAN_PART10; monitor_cnt < CAN_PART11; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART10];	
		msg_rev=1;
	}
	else if((65226==pgn)&&(65226==Receive_Status[0].Dest_PGN))//更新故障码
	{
		uint8 count,count_old,count_new,last_num,j;
		uint16 i;
		count=0;
		count_old=0;
		count_new=0;
		last_num=0;
		
		count=data[0];
		last_num=(Receive_Status[0].byte_count/7);
		if(1==count)
		{
			count_old=0;
			count_new=5;
		}
		if(count<Receive_Status[0].total_packet_number)
		{
			count_new=count*7-2;
			count_old=count_new-7;
			FAULT_CODE_Last[0][0]=data[3];
			FAULT_CODE_Last[0][1]=data[4];
			FAULT_CODE_Last[0][2]=data[5];
			FAULT_CODE_Last[0][3]=data[6];
			FAULT_CODE_Last[0][4]=data[7];
		}
		if(count==Receive_Status[0].total_packet_number)
		{
			count_new=count*7-9+last_num;
			count_old=count_new-7;
		}
		for(i=(count_old/4);i<(count_new/4);i++)
		{
			Fault_Code[0].count=i+1;
			Fault_Code[0].Fault_Code_T[i+1].SPN=FAULT_CODE_Last[0][i-count/4+0];
			Fault_Code[0].Fault_Code_T[i+1].SPN+=(FAULT_CODE_Last[0][i-count/4+1]<<8);
			Fault_Code[0].Fault_Code_T[i+1].SPN+=((FAULT_CODE_Last[0][i-count/4+2]&0xe0)<<11);
			Fault_Code[0].Fault_Code_T[i+1].FMI=FAULT_CODE_Last[0][i-count/4+3]&0x1f;
			Fault_Code[0].Fault_Code_T[i+1].CM=FAULT_CODE_Last[0][i-count/4+4]&0x80;
			Fault_Code[0].Fault_Code_T[i+1].OC=FAULT_CODE_Last[0][i-count/4+4]&0x7f;
			if(i==(count_new/4))
			{
				if(count>1)
				{
					for(j=0;j<(count_new%4);j++)
					{
						FAULT_CODE_Last[0][j]=data[8-count_new%4];
					}
				}
				if(1==count)
					FAULT_CODE_Last[0][0]=data[7];
			}
		}
		
//		if(Receive_Status[0].total_packet_number==data[0])//多包传输结束

	
/*		uint8	DTC[J1939_MAX_MESSAGE_LENGTH];
		uint8	i,count,last_num,end_sign;
		
		count=0;
		last_num=0;
		end_sign=0;
		memset(DTC,0,sizeof(DTC));
		count=data[0];
		last_num=(Receive_Status[0].byte_count/7);
		if(count==1)//第一包包含两字节灯状态
		{
			DTC[0]=data[3];
			DTC[1]=data[4];
			DTC[2]=data[5];
			DTC[3]=data[6];
			DTC[4]=data[7];
		}
		else if(count<Receive_Status[0].total_packet_number)
		{
			for(i=1;i<8;i++)
			{
				DTC[count*7-9]=data[i];
			}
		}
		else if(count==Receive_Status[0].total_packet_number)
		{
			for(i=1;i<last_num;i++)
			{
				DTC[count*7-9]=data[i];
			}
			end_sign=1;
		}
		else
			printf("Motor_L_65226 CAN %s\n",data);
		
		if(end_sign==1)
		{
			

		}
*/
		//监控转存
		for(monitor_cnt = CAN_PART11; monitor_cnt < CAN_PART12; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_L_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART11];	
		msg_rev=1;
	}
	else if(65235==pgn)//清除所有故障码
	{
		memset((uint8 *)&Fault_Code[0].count,0,sizeof(Fault_Code));
		/*cxy清除上送状态中的故障码*/
	}
#ifdef	debug_print
	else
		printf("Motor_L CAN %d,%s\n",pgn,data);
#endif
	if(msg_rev==1)
	{
		msg_rev=0;
		USV_State.Conrtol_System_Msg.Engine_L=0;
	}

	return	;
}
//发右动机CAN报文解析
void Motor_R_Analyzing(uint16 pgn,uint8 *data)
{
//	int count;
	uint8	msg_rev=0;	
	int monitor_cnt;
	uint8	Engine_run_old;

//	USV_State.Engine_power=USV_State.Engine_power|0x02;//收到右发动机报文，右发动机已上电
	if(61444==pgn)
	{
/*		for(count=0;count<8;count++)
			printf("Rmotor=0x%.2x",data[count]);
		printf("\n");
*/
		/*Engine_run_old = USV_State.Engine_run&0x02;*/

		Motor_CAN_State.Motor_PGN61444_State[1].Torque_Mod_spn899=(data[0]&0x0f);
		Motor_CAN_State.Motor_PGN61444_State[1].Torque_Percent_EXP_spn512=data[1];
		Motor_CAN_State.Motor_PGN61444_State[1].Torque_Percent_ACT_spn513=data[2];
		Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190=(data[4]<<8);
		Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190+=data[3];
		Motor_CAN_State.Motor_PGN61444_State[1].Motor_CON_ADD_spn1483=data[5];
		Motor_CAN_State.Motor_PGN61444_State[1].Mortor_Starter_Mod_spn1675=(data[6]&0x0f);
		if(Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190>500*8)
			USV_State.Engine_run=USV_State.Engine_run|0x02;//右发动机转速不为0，已启动


		//if((Engine_run_old == 0)&&(USV_State.Engine_run ))	//发动机点火
		//{
		//	SysLogMsgPost("发动机点火,点火来源:%d(0-数字电台,1-数字冗余电台 2-宽带电台 3-北斗电台  4-面板)",Radio_Sign);
		//}

		//监控转存
		for(monitor_cnt = CAN_PART0; monitor_cnt < CAN_PART1; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART0];	
		msg_rev=1;
	}                                        
	else if(65244==pgn)                      
	{                                        
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Fuel_Consume_spn236=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Fuel_Consume_spn236+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Fuel_Consume_spn236+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Fuel_Consume_spn236+=data[0];
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART1; monitor_cnt < CAN_PART2; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART1];
		msg_rev=1;
	}
	else if(65253==pgn)
	{
		Motor_CAN_State.Motor_PGN65253_State[1].Working_Time_spn247=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65253_State[1].Working_Time_spn247+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65253_State[1].Working_Time_spn247+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65253_State[1].Working_Time_spn247+=data[0];
		Motor_CAN_State.Motor_PGN65253_State[1].Total_turn_spn249+=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65253_State[1].Total_turn_spn249+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65253_State[1].Total_turn_spn249+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65253_State[1].Total_turn_spn249+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART2; monitor_cnt < CAN_PART3; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART2];
		msg_rev=1;
	}
	else if(65257==pgn)
	{
		Motor_CAN_State.Motor_PGN65257_State[1].Single_Fuel_Consume_spn182=(data[3]<<24);
		Motor_CAN_State.Motor_PGN65257_State[1].Single_Fuel_Consume_spn182+=(data[2]<<16);
		Motor_CAN_State.Motor_PGN65257_State[1].Single_Fuel_Consume_spn182+=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65257_State[1].Single_Fuel_Consume_spn182+=data[0];
		Motor_CAN_State.Motor_PGN65257_State[1].Total_Fuel_Consume_spn250=(data[7]<<24);
		Motor_CAN_State.Motor_PGN65257_State[1].Total_Fuel_Consume_spn250+=(data[6]<<16);
		Motor_CAN_State.Motor_PGN65257_State[1].Total_Fuel_Consume_spn250+=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65257_State[1].Total_Fuel_Consume_spn250+=data[4];

		//监控转存
		for(monitor_cnt = CAN_PART3; monitor_cnt < CAN_PART4; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART3];
		msg_rev=1;
	}
	else if(65262==pgn)
	{
		Motor_CAN_State.Motor_PGN65262_State[1].Cool_Temp_spn110=data[0];
		Motor_CAN_State.Motor_PGN65262_State[1].Fuel_Temp_spn174=data[1];
		Motor_CAN_State.Motor_PGN65262_State[1].Oil_Temp_spn175=(data[3]<<8);
		Motor_CAN_State.Motor_PGN65262_State[1].Oil_Temp_spn175+=data[2];
		Motor_CAN_State.Motor_PGN65262_State[1].Tuibine_Oil_Temp_spn176=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65262_State[1].Tuibine_Oil_Temp_spn176+=data[4];
		Motor_CAN_State.Motor_PGN65262_State[1].Motor_Cooler_Temp_spn52=data[6];
		Motor_CAN_State.Motor_PGN65262_State[1].Mortor_Cooler_Opening_spn1134=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART4; monitor_cnt < CAN_PART5; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART4];
		msg_rev=1;
	}
	else if(65263==pgn)
	{
		Motor_CAN_State.Motor_PGN65263_State[1].Fuel_Delivery_Pressure_spn94=data[0];
		Motor_CAN_State.Motor_PGN65263_State[1].Extension_Crankcase_Leakage_spn22=data[1];
		Motor_CAN_State.Motor_PGN65263_State[1].Oil_Level_spn98=data[2];
		Motor_CAN_State.Motor_PGN65263_State[1].Oil_Pressure_spn100=data[3];
		Motor_CAN_State.Motor_PGN65263_State[1].Crankcase_Pressure_spn101=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65263_State[1].Crankcase_Pressure_spn101+=data[4];
		Motor_CAN_State.Motor_PGN65263_State[1].Cool_Pressure_spn109=data[6];
		Motor_CAN_State.Motor_PGN65263_State[1].Cool_Level_spn111=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART5; monitor_cnt < CAN_PART6; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART5];
		msg_rev=1;
	}
	else if(65266==pgn)
	{
		Motor_CAN_State.Motor_PGN65266_State[1].Fuel_Usage_spn183=(data[1]<<8);
		Motor_CAN_State.Motor_PGN65266_State[1].Fuel_Usage_spn183+=data[0];
		Motor_CAN_State.Motor_PGN65266_State[1].Instant_Fuel_Economy_spn184=(data[3]<<8);
		Motor_CAN_State.Motor_PGN65266_State[1].Instant_Fuel_Economy_spn184+=data[2];
		Motor_CAN_State.Motor_PGN65266_State[1].Average_Fuel_Economy_spn185=(data[5]<<8);
		Motor_CAN_State.Motor_PGN65266_State[1].Average_Fuel_Economy_spn185+=data[4];
		Motor_CAN_State.Motor_PGN65266_State[1].Throttle_Position_spn51=(data[7]<<8);
		Motor_CAN_State.Motor_PGN65266_State[1].Throttle_Position_spn51+=data[6];

		//监控转存
		for(monitor_cnt = CAN_PART6; monitor_cnt < CAN_PART7; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART6];
		msg_rev=1;
	}
	else if(655269==pgn)
	{
		Motor_CAN_State.Motor_PGN65269_State[1].Air_Pressure_spn108=data[0];
		Motor_CAN_State.Motor_PGN65269_State[1].AIr_Temp_spn171=(data[4]<<8);
		Motor_CAN_State.Motor_PGN65269_State[1].AIr_Temp_spn171+=data[3];
		Motor_CAN_State.Motor_PGN65269_State[1].Air_inlet_Temp_spn172=data[5];

		//监控转存
		for(monitor_cnt = CAN_PART7; monitor_cnt < CAN_PART8; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART7];
		msg_rev=1;
	}
	else if(655270==pgn)
	{
		Motor_CAN_State.Motor_PGN65270_State[1].Supercharger_Pressure_spn102=data[1];
		Motor_CAN_State.Motor_PGN65270_State[1].Inlet_Air_Temp_spn105=data[2];
		Motor_CAN_State.Motor_PGN65270_State[1].Air_inlet_Pressure_spn106=data[3];
		Motor_CAN_State.Motor_PGN65270_State[1].Air_fillter_Diff_Pressure_spn107=data[4];
		Motor_CAN_State.Motor_PGN65270_State[1].Exhaust_Temp_spn173=(data[6]<<8);
		Motor_CAN_State.Motor_PGN65270_State[1].Exhaust_Temp_spn173+=data[5];
		Motor_CAN_State.Motor_PGN65270_State[1].Cool_Fillter_Diff_Pressure_spn112=data[7];

		//监控转存
		for(monitor_cnt = CAN_PART9; monitor_cnt < CAN_PART10; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART9];
		msg_rev=1;
	}	
	else if(655279==pgn)
	{
		Motor_CAN_State.Motor_PGN65279_State[1].Fuel_Moisture_spn97=(data[0]&0x03);

		//监控转存
		for(monitor_cnt = CAN_PART10; monitor_cnt < CAN_PART11; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART10];
		msg_rev=1;
	}
	else if(60416==pgn)//多包传输，DM1用
	{
		memset((uint8 *)&Receive_Status[1].PGN,0,sizeof(Receive_Status));
		
		if(TP_CM_BAM==data[0])
		{
			Receive_Status[1].byte_count=data[1];
			Receive_Status[1].byte_count+=(data[2]<<8);
			Receive_Status[1].total_packet_number=data[3];
			Receive_Status[1].Dest_PGN=data[5];
			Receive_Status[1].Dest_PGN+=(data[6]<<8);
		}
#ifdef	debug_print
		else if(TP_CM_RTS==data[0])
		{
			printf("Motor_R_60416 CAN %s\n",data);
		}
		else
			printf("Motor_R_60416 CAN %s\n",data);
#endif

		//监控转存
		for(monitor_cnt = CAN_PART10; monitor_cnt < CAN_PART11; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART10];
		msg_rev=1;
	}
	else if((65226==pgn)&&(65226==Receive_Status[1].Dest_PGN))//更新故障码
	{
		uint8 count,count_old,count_new,last_num,j;
		uint16 i;
		count=0;
		count_old=0;
		count_new=0;
		last_num=0;
		
		count=data[0];
		last_num=(Receive_Status[1].byte_count/7);
		if(1==count)
		{
			count_old=0;
			count_new=5;
		}
		if(count<Receive_Status[1].total_packet_number)
		{
			count_new=count*7-2;
			count_old=count_new-7;
			FAULT_CODE_Last[1][0]=data[3];
			FAULT_CODE_Last[1][1]=data[4];
			FAULT_CODE_Last[1][2]=data[5];
			FAULT_CODE_Last[1][3]=data[6];
			FAULT_CODE_Last[1][4]=data[7];
		}
		if(count==Receive_Status[1].total_packet_number)
		{
			count_new=count*7-9+last_num;
			count_old=count_new-7;
		}
		for(i=(count_old/4);i<(count_new/4);i++)
		{
			Fault_Code[1].count=i+1;
			Fault_Code[1].Fault_Code_T[i+1].SPN=FAULT_CODE_Last[1][i-count/4+0];
			Fault_Code[1].Fault_Code_T[i+1].SPN+=(FAULT_CODE_Last[1][i-count/4+1]<<8);
			Fault_Code[1].Fault_Code_T[i+1].SPN+=((FAULT_CODE_Last[1][i-count/4+2]&0xe0)<<11);
			Fault_Code[1].Fault_Code_T[i+1].FMI=FAULT_CODE_Last[1][i-count/4+3]&0x1f;
			Fault_Code[1].Fault_Code_T[i+1].CM=FAULT_CODE_Last[1][i-count/4+4]&0x80;
			Fault_Code[1].Fault_Code_T[i+1].OC=FAULT_CODE_Last[1][i-count/4+4]&0x7f;
			if(i==(count_new/4))
			{
				if(count>1)
				{
					for(j=0;j<(count_new%4);j++)
					{
						FAULT_CODE_Last[1][j]=data[8-count_new%4];
					}
				}
				if(1==count)
					FAULT_CODE_Last[1][0]=data[7];
			}
		}
		
	
/*		uint8	DTC[J1939_MAX_MESSAGE_LENGTH];
		uint8	i,count,last_num,end_sign;
		
		count=0;
		last_num=0;
		end_sign=0;
		memset(DTC,0,sizeof(DTC));
		count=data[0];
		last_num=(Receive_Status[0].byte_count/7);
		if(count==1)//第一包包含两字节灯状态
		{
			DTC[0]=data[3];
			DTC[1]=data[4];
			DTC[2]=data[5];
			DTC[3]=data[6];
			DTC[4]=data[7];
		}
		else if(count<Receive_Status[0].total_packet_number)
		{
			for(i=1;i<8;i++)
			{
				DTC[count*7-9]=data[i];
			}
		}
		else if(count==Receive_Status[0].total_packet_number)
		{
			for(i=1;i<last_num;i++)
			{
				DTC[count*7-9]=data[i];
			}
			end_sign=1;
		}
		else
			printf("Motor_L_65226 CAN %s\n",data);
		
		if(end_sign==1)
		{
			

		}
*/
		//监控转存
		for(monitor_cnt = CAN_PART11; monitor_cnt < CAN_PART12; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART11];
		msg_rev=1;
	}
	else if(65235==pgn)//清除所有故障码
	{
		memset((uint8 *)&Fault_Code[1].count,0,sizeof(Fault_Code));
		/*cxy清除上送状态中的故障码*/
		//监控转存
		for(monitor_cnt = CAN_PART12; monitor_cnt < CAN_PART13; monitor_cnt++)
		monitor_all_inf.module_report_inf[MONITOR_MOTOR_R_MSG].report_detail[monitor_cnt] = data[monitor_cnt-CAN_PART12];
		msg_rev=1;
	}
#ifdef	debug_print
	else
		printf("Motor_R CAN %d,%s\n",pgn,data);
#endif
	if(msg_rev==1)
	{
		msg_rev=0;
		USV_State.Conrtol_System_Msg.Engine_R=0;
	}
	return	;
}
//更新发动机报警信息
void Update_Motor_Alarm(FAULT_CODE_T *fault_code)
{
	int count,num;
	num=(J1939_MAX_MESSAGE_LENGTH-2)/4;
	for(count=0;count<num;count++)
	{
		switch (fault_code[0].Fault_Code_T[count].SPN)
		{
			case 0xBE:
				Motor_Detail_St.SuperLoad_Alarm_Left=1;//发动机转速异常
				break;
			case 0x6E:
				Motor_Detail_St.Cooling_TEMP_Alarm_Left=1;//冷却液温度异常
				break;
			case 0x6F:
				Motor_Detail_St.Cooling_Level_Alarm_Left=1;//冷却液液位异常
				break;
			case 0x64:
				Motor_Detail_St.OilPressure_Alarm_Left=1;//机油压力异常
				break;
			case 0xAF:
				Motor_Detail_St.Oil_TEMP_Alarm_L=1;//机油温度异常
				break;
			case 0x66:
				Motor_Detail_St.Supercharger_Pressure_Alarm_L=1;//进气歧管压力异常
				break;
			case 0x69:
				Motor_Detail_St.InletTEMP_Alarm_Left=1;//进气歧管温度异常
				break;
			case 0x5E:
				Motor_Detail_St.Fuel_Pressure_Alarm_L=1;//燃油压力异常
				break;
			case 0x61:
				Motor_Detail_St.Fuel_Moisture_Alarm_L=1;//燃油含水异常
				break;
			case 0x9D:
				Motor_Detail_St.Nozzle_Pressure_L=1;//喷嘴压力异常
				break;
			case 0xA8:
				Motor_Detail_St.Electricity_Alarm_L=1;//发动机电池电量低
				break;
			default:
				break;
		}
		switch (fault_code[1].Fault_Code_T[count].SPN)
		{
			case 0xBE:
				Motor_Detail_St.SuperLoad_Alarm_Right=1;//发动机转速异常
				break;
			case 0x6E:
				Motor_Detail_St.Cooling_TEMP_Alarm_Right=1;//冷却液温度异常
				break;
			case 0x6F:
				Motor_Detail_St.Cooling_Level_Alarm_Right=1;//冷却液液位异常
				break;
			case 0x64:
				Motor_Detail_St.OilPressure_Alarm_Right=1;//机油压力异常
				break;
			case 0xAF:
				Motor_Detail_St.Oil_TEMP_Alarm_R=1;//机油温度异常
				break;
			case 0x66:
				Motor_Detail_St.Supercharger_Pressure_Alarm_R=1;//进气歧管压力异常
				break;
			case 0x69:
				Motor_Detail_St.InletTEMP_Alarm_Right=1;//进气歧管温度异常
				break;
			case 0x5E:
				Motor_Detail_St.Fuel_Pressure_Alarm_R=1;//燃油压力异常
				break;
			case 0x61:
				Motor_Detail_St.Fuel_Moisture_Alarm_R=1;//燃油含水异常
				break;
			case 0x9D:
				Motor_Detail_St.Nozzle_Pressure_R=1;//喷嘴压力异常
				break;
			case 0xA8:
				Motor_Detail_St.Electricity_Alarm_R=1;//发动机电池电量低
				break;
			default:
				break;
		}
	}
	return	;
}

void CAN_Analyzing(CAN_Struct  *frame)
{
	uint8 CAN_add,CAN_Priority,CAN_DATA[8],count;
	uint16 CAN_PGN;
	CAN_add=0;
	CAN_Priority=0;
	CAN_PGN=0;
	memset(CAN_DATA,0,sizeof(CAN_DATA));
	CAN_add=(frame->CAN_add);
	CAN_PGN=(frame->CAN_PGN);
	CAN_Priority=(frame->CAN_Priority);
//	printf("add=%d\n",CAN_add);
//	printf("Priority=%d\n",CAN_Priority);
//	printf("PGN=%d\n",CAN_PGN);
	for(count=0;count<8;count++)
		CAN_DATA[count]=frame->CAN_Data[count];
//		strncpy(CAN_DATA,frame->data,8);
	switch(CAN_add)
	{
		case CAN_DSP_ADD:
			DSP_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_DSP_50ms=0;
			break;
		case CAN_UAV_ADD:
			UAV_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_UAV_50ms=0;
			break;
		case CAN_SR_ADD:
			SR_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_SR_50ms=0;
			break;
		case CAN_POW_CON_ADD:
			POW_CON_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_POW_50ms=0;
			break;
		case CAN_SP_ADD:
			SP_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_SP_50ms=0;		
			break;
		case CAN_ST_PL_ADD:
			ST_PL_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_ST_PL_50ms=0;
			break;
		case CAN_BAT_CON_ADD:
			BAT_CON_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_BAT_50ms=0;
			break;
		case CAN_Motor_ADD_L:
			Motor_L_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_Motor_L_10ms=0;
			break;
		case CAN_Motor_ADD_R:
			Motor_R_Analyzing(CAN_PGN,CAN_DATA);
			task_time.CAN_Motor_R_10ms=0;
			break;
		default:
			break;
	}
	Update_Motor_Alarm(Fault_Code);
	return	;
}

void CAN_Init(void)
{
	memset((uint8 *)&UAV_CAN_State.Platform_Hatch_spn520576,0,sizeof(UAV_CAN_State));	
	memset((uint8 *)&SR_CAN_State.Rudder_Angle_L_spn520766,0,sizeof(SR_CAN_State));
	memset((uint8 *)&POW_CAN_State.Stable_Platform_Power_spn520958,0,sizeof(POW_CAN_State));
	memset((uint8 *)&SP_CAN_Model_Con.Emergency_spn521192,0,sizeof(SP_CAN_Model_Con));
	memset((uint8 *)&SP_Sail_CAN_Con.Gear_Left_spn521178,0,sizeof(SP_Sail_CAN_Con));
	memset((uint8 *)&SP_Equipment_CAN_Con.Stable_Platform_RL_spn521189,0,sizeof(SP_Equipment_CAN_Con));
	memset((uint8 *)&ST_PL_CAN_State.Stable_Platform_RL_spn521342,0,sizeof(ST_PL_CAN_State));
	memset((uint8 *)&BAT_CON_CAN_State.USV_Pow_spn521534,0,sizeof(BAT_CON_CAN_State));
	memset((uint8 *)&Motor_CAN_State.Motor_PGN61444_State[0].Torque_Mod_spn899,0,sizeof(Motor_CAN_State));
	memset((uint8 *)&Motor_CAN_State.Motor_PGN61444_State[1].Torque_Mod_spn899,0,sizeof(Motor_CAN_State));
	USV_State.Conrtol_System_Msg.Intelligent_rudder=0x01;
	USV_State.Conrtol_System_Msg.Stabilized_platform=0x01;		
	USV_State.Conrtol_System_Msg.Energy_management=0x01;		
	USV_State.Conrtol_System_Msg.Power_management=0x01;		
	USV_State.Conrtol_System_Msg.UAV_platform=0x01;		
	USV_State.Conrtol_System_Msg.Fire_fighting_system=0x00;	//cxy	
	USV_State.Conrtol_System_Msg.Engine_L=0x01;		
	USV_State.Conrtol_System_Msg.Engine_R=0x01;		
}

