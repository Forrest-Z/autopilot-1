#include "stdafx.h"                                                                                                                                                     
#include "../include/usv_include.h" 

#define MSG_TYPE 0x46 //普快通讯
#define MSG_ACK 0
#define BD_MSG_LEN 83
#define BD_LOCAL_ID 0
#define BD_REMOTE_ID 307420
#define BD_MSG_MAX 100




//void BD_Msg_Send(uint8* pMsg,uint16 m_length)
//{
//	uint8 CRC;
//	CRC=0;
//	uint8 BD_MSG[BD_MSG_MAX];
//	uint16 length = BD_MSG_LEN;
//	uint32 local_id = BD_LOCAL_ID;
//	uint32 remote_id= BD_REMOTE_ID;
//	uint16 msg_length = m_length*8;
//	memset(&BD_MSG[0],0,BD_MSG_MAX);
//	
//	BD_MSG[0] = '$';
//	BD_MSG[1] = 'T';
//	BD_MSG[2] = 'X';
//	BD_MSG[3] = 'S';
//	BD_MSG[4] = 'Q';
//	BD_MSG[5] = (length >> 8)&0x00ff;			//长度
//	BD_MSG[6] = length&0x00ff;				//
//	BD_MSG[7] = (local_id>> 16)&0x000000ff;	//受控用户地址
//	BD_MSG[8] = (local_id>> 8)&0x000000ff;	
//	BD_MSG[9] = (local_id)&0x000000ff;
//	BD_MSG[10]= MSG_TYPE;					//信息类型
//	BD_MSG[11]= (remote_id>> 16)&0x000000ff;//用户地址
//	BD_MSG[12]= (remote_id>> 8)&0x000000ff;
//	BD_MSG[13]= (remote_id)&0x000000ff;
//	BD_MSG[14]= (msg_length >>8)&0x00ff;
//	BD_MSG[15]= (msg_length)&0x00ff;
//	BD_MSG[16]= MSG_ACK;
//	memcpy(&BD_MSG[17],pMsg,m_length);
//	GetCheck(&CRC,BD_MSG,BD_MSG_LEN);
//	BD_MSG[BD_MSG_LEN-1]= CRC;
//	write_uart(UART3_Fd, (int8 *)&BD_MSG[0], BD_MSG_LEN);
//}
//
//
//
//
//void Send_BD_State( void )
//{
//	uint8 CRC;
//	CRC=0;
//	USV_State_Current[0]='!';                                              
//	GetCheck(&CRC,USV_State_Current,(sizeof(USV_State_Current)));          
//	USV_State_Current[State_Current_Num-1]=CRC;  
//	BD_Msg_Send(&USV_State_Current[0],State_Current_Num);	
//}


void BD_test(void)
{
	USV_Control.USV_Control_Message[3].USV_Num = 0;
	USV_Control.USV_Control_Message[3].Msg_Num = 0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.UAV_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Stable_Platform_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_ahead_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.D_radar_Power=0;

	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_main_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Horn_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Searchlight_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_lesser_Power=0;

	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Navigationlight_Power=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_tail_Power=0;
	USV_Control.USV_Control_Message[3].Navigation_Tsak_sign=1;


	//USV_Control.USV_Control_Message[3].Engine_Run_L=(real_msg[12]&0x03);
	//USV_Control.USV_Control_Message[3].Engine_Run_R=(real_msg[12]&0x0C)>>2;


	BD_Engine_Run_L = 1;			//北斗电台点火临时处理
	BD_Engine_Run_R = 1;		//北斗电台点火临时处理

	USV_Control.USV_Control_Message[3].Dradio_USV_Model.Sailing_Mod=2;

	USV_Control.USV_Control_Message[3].Dradio_USV_Stable_Platform.Stable_Platform_RL=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_Stable_Platform.Stable_Platform_AT=0;

	USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Hatch=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Lift=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Open=0;
	USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.UAV_Charging=0;

	USV_Control.USV_Control_Message[3].Hull_Fan_Con=0;
	USV_Control.USV_Control_Message[3].Hull_Pump_Con=0;
	USV_Control.USV_Control_Message[3].Oil_Con=0;

	USV_Control.USV_Control_Message[3].Application_24=0;
	USV_Control.USV_Control_Message[3].Application_12=0;

	USV_Control.USV_Control_Message[3].Emergency_Stop=0;
	USV_Control.USV_Control_Message[3].Speed_Limit = 40;
	//航行任务部分
	Sailing_Cnt_Old = 1;
	USV_State.USV_Sailing_Intel_Sign=2;//收到航行任务，待开启
	USV_State.Sailing_Nummber = 2;

	//
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].USV_Num=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber= 2;	//BD仅下发一个应急航点
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Sign=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Sign=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Degree=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Minute=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Second=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Decimal=0;		
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Degree=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Minute=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Second=0;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Decimal=0;	

	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_speed=700;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_speed=700;
	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[2].Waypoint_speed=700;

	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Stop_Time=0;

	USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber=255;	//北斗航行任务编号
}





void BD_Msg_Analytical(uint8 *BD_Com_Msg)
{
	uint8 real_msg[52];
	memcpy(&real_msg[0],&BD_Com_Msg[18],35);
	if(CheckCRC(1,&real_msg[0],35))	//BD电文CRC验证
	{
		if(real_msg[5]==USV_NUM)//是本船命令
		{
			//控制命令部分
			USV_Control.USV_Control_Message[3].USV_Num = real_msg[5];
			USV_Control.USV_Control_Message[3].Msg_Num=real_msg[6]*256+real_msg[7];
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.UAV_Power=(real_msg[9]&0x03);
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Stable_Platform_Power=(real_msg[9]&0x0c)>>2;
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_ahead_Power=(real_msg[9]&0x30)>>4;
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.D_radar_Power=real_msg[9]>>6;

			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_main_Power=(real_msg[10]&0x03);
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Horn_Power=(real_msg[10]&0x0c)>>2;
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Searchlight_Power=(real_msg[10]&0x30)>>4;
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_lesser_Power=real_msg[10]>>6;
			
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Navigationlight_Power=(real_msg[11]&0x03);
			USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Camera_tail_Power=(real_msg[11]&0x0c)>>2;
			USV_Control.USV_Control_Message[3].Navigation_Tsak_sign=(real_msg[11]&0xf0)>>4;

			//USV_Control.USV_Control_Message[3].Engine_Run_L=(real_msg[12]&0x03);
			//USV_Control.USV_Control_Message[3].Engine_Run_R=(real_msg[12]&0x0C)>>2;

			BD_Engine_Run_L = (real_msg[12]&0x03);			//北斗电台点火临时处理
			BD_Engine_Run_R = (real_msg[12]&0x0C)>>2;		//北斗电台点火临时处理

			USV_Control.USV_Control_Message[3].Dradio_USV_Model.Sailing_Mod=(real_msg[12]&0x30)>>4;

			USV_Control.USV_Control_Message[3].Dradio_USV_Stable_Platform.Stable_Platform_RL=real_msg[13];
			USV_Control.USV_Control_Message[3].Dradio_USV_Stable_Platform.Stable_Platform_AT=real_msg[14];

			USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Hatch=(real_msg[15]&0x03);
			USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Lift=(real_msg[15]&0x0c)>>2;
			USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.Platform_Open=(real_msg[15]&0x30)>>4;
			USV_Control.USV_Control_Message[3].Dradio_USV_UAV_Control.UAV_Charging=(real_msg[15]&0xc0)>>6;
			
			USV_Control.USV_Control_Message[3].Hull_Fan_Con=real_msg[16]&0x03;
			USV_Control.USV_Control_Message[3].Hull_Pump_Con=(real_msg[16]&0x3C)>>2;
			USV_Control.USV_Control_Message[3].Oil_Con=(real_msg[16]&0xC0)>>6;

			USV_Control.USV_Control_Message[3].Application_24=(real_msg[17]&0x03);
			USV_Control.USV_Control_Message[3].Application_12=((real_msg[17]&0x0C)>>2);

			USV_Control.USV_Control_Message[3].Emergency_Stop=real_msg[18]&0x01;
			USV_Control.USV_Control_Message[3].Speed_Limit = 40;		//限速值40节
			//航行任务部分
			Sailing_Cnt_Old = 1;
			USV_State.USV_Sailing_Intel_Sign=2;//收到航行任务，待开启
			USV_State.Sailing_Nummber = 2;
			
			//
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].USV_Num=real_msg[5];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber= 2;	//BD仅下发一个应急航点
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Sign=(real_msg[19]&0x01);
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Sign=(real_msg[20]&0x02)>>1;
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Degree=real_msg[21];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Minute=real_msg[22];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Second=real_msg[23];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Latitude_Decimal=real_msg[24];		
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Degree=real_msg[25];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Minute=real_msg[26];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Second=real_msg[27];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Longitude_Decimal=real_msg[28];	

			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_speed=real_msg[29]*256+real_msg[30];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_speed=real_msg[29]*256+real_msg[30];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[2].Waypoint_speed=real_msg[29]*256+real_msg[30];

			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[1].Waypoint_Stop_Time=real_msg[31]*256+real_msg[32];

			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber=255;	//北斗航行任务编号
		}
	}
}

void Send_BD_State( void )
{
	uint8 CRC;
	CRC=0;
	uint8 BD_MSG[BD_MSG_LEN+10];
	uint16 length = BD_MSG_LEN;
	uint16 msg_length = State_Current_Num*8;
	uint32 local_id = BD_LOCAL_ID;
	uint32 remote_id = BD_REMOTE_ID;
	memset(&BD_MSG[0],0,sizeof(BD_MSG));

	USV_State_Current[0]='!';                                              
	GetCheck(&CRC,USV_State_Current,(sizeof(USV_State_Current)));          
	USV_State_Current[State_Current_Num-1]=CRC;                            

	BD_MSG[0] = '$';
	BD_MSG[1] = 'T';
	BD_MSG[2] = 'X';
	BD_MSG[3] = 'S';
	BD_MSG[4] = 'Q';
	BD_MSG[5] = (length >> 8)&0x00ff;			//长度
	BD_MSG[6] = length&0x00ff;				//
	BD_MSG[7] = (local_id>> 16)&0x000000ff;	//受控用户地址
	BD_MSG[8] = (local_id>> 8)&0x000000ff;	
	BD_MSG[9] = (local_id)&0x000000ff;
	BD_MSG[10]= MSG_TYPE;					//信息类型
	BD_MSG[11]= (remote_id>> 16)&0x000000ff;//用户地址
	BD_MSG[12]= (remote_id>> 8)&0x000000ff;
	BD_MSG[13]= (remote_id)&0x000000ff;
	BD_MSG[14]= (msg_length >>8)&0x00ff;
	BD_MSG[15]= (msg_length)&0x00ff;
	BD_MSG[16]= MSG_ACK;
	memcpy(&BD_MSG[17],USV_State_Current,State_Current_Num);
	GetCheck(&CRC,BD_MSG,sizeof(BD_MSG));
	BD_MSG[BD_MSG_LEN-1]= CRC;
	write_uart(UART3_Fd, (int8 *)&BD_MSG[0], BD_MSG_LEN);
	
}
