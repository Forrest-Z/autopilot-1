//BradioUdpThread.cpp
//2017/9/23 22:29:08
//author:shaoyuping

#include "stdafx.h"
#include <stdarg.h>
#include <string.h>
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
#include "../include/usv_include.h"



UDP_INF_STRUCT bradio_udp_inf;
int16 udp_sockid;			//与后台通讯用sockid
char MoniforIpCfg[30];		//服务器IP配置

void bradio_udp_inf_init(void)
{
	memset((int8 *)&bradio_udp_inf,0,sizeof(bradio_udp_inf));
}




void * Lan1_Bradio_UDP_thread( void *aa )
{
	//int16	sockid;		//与串口服务器连接用端口
	uint16  state_num=0;	//报文编号
	//初始化
	
	udp_sockid = -1;
	for(;;)
	{
		if(-1 == udp_sockid){
			udp_sockid = CreateUDPSocket((uint16)BRADIO_UDP_REC_PORT);	//创建UDP
			if(udp_sockid<0){
				sleep_1(1000);
				continue;
			}
		}
		else{
			Lan1_Bradio_UDP_rec(udp_sockid);
		}
		//sleep_1(200);

		//state_num++;
		////报文发送
		//deal_bradio_udp_send(sockid,state_num);
			
		sleep_1(100);
			//200ms
	}
}



int8 Lan1_Bradio_UDP_rec( int sockid )
{
	fd_set set;
	int16 ret;
	int16 iLen;
	struct timeval timeout;
	socklen_t sockaddrlen;
	int8 net_rec_buff[4096];

	FD_ZERO(&set);
	FD_SET(sockid,&set);
	timeout.tv_sec=0;
	timeout.tv_usec=5;
	ret = select(sockid+1,&set,NULL,NULL,&timeout);
	if(ret<0){
		return TRUE;
	}
	if(FD_ISSET(sockid,&set)){		//UDP接收
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff),0,(struct sockaddr *)&bradio_udp_inf.from_upd_ip, &sockaddrlen);
		if(iLen>=1){
			bradio_udp_inf.udp_rec_flag = SWITCH_ON;
			deal_bradio_udp_rec((uint8 *)&net_rec_buff[0],iLen);
		}
	}
	return TRUE;
}

void deal_bradio_udp_rec( uint8 *net_rec_buff,uint16 report_len )
{
	int ico,jco=0,re_sign=0;
	uint8 bradio_sail[Dradio_Sal_Msg];
	uint16 sail_len;
	for(ico=0;ico<report_len;ico++)
	{
		if((net_rec_buff[ico]=='#')&&(net_rec_buff[ico+1]=='U')&&(net_rec_buff[ico+2]=='S')&&(net_rec_buff[ico+3]=='V')||(1==re_sign))
		{
			re_sign = 1;
			bradio_sail[jco++] = net_rec_buff[ico];
			if(jco>7)
				sail_len = bradio_sail[6]*256+bradio_sail[7];
			if(jco==sail_len)
			{
				jco=0;
				re_sign=0;
				if(CheckCRC(1,bradio_sail,sail_len))
				{	
					//WAIT DELET
					Sailing_Sign = 2;
					USV_Sailing.Radio_Nummber =2;
				//	Dradio_Sal_Analytical(bradio_sail);
					USV_State.Sailing_Nummber = USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber;
					USV_State.USV_Sailing_Intel_Sign = 1; //收到航行任务，待开启
				//	Get_Return_Point();
					Sailing_Cnt_Old = 1;
				//	Get_Compensating_Dst();
					//WAIT DELET END

				//	getSailMsg(bradio_sail);

					getSailMsgWaterQuality(bradio_sail);
					monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BTASK_SN].rec_ok_number++;			//接收成功计数
	
				}
				memset(bradio_sail,0,Dradio_Sal_Msg);
			}
		}
	}
}

int8 Lan1_Bradio_UDP_send( int sockid ,int8 * msgBuff,uint16 msgLen)
{
	int iSendLen;
	int DestPort;
	uint8 CRC;
	struct sockaddr_in server;

	DestPort = BRADIO_UDP_SEND_PORT;
	server.sin_family = AF_INET;
	server.sin_port = htons(DestPort);

	// 取消使用接收报文地址，全部使用指定地址
	//if(bradio_udp_inf.udp_rec_flag == SWITCH_ON)
	//	server.sin_addr.s_addr = bradio_udp_inf.from_upd_ip.sin_addr.s_addr;
	//else
		server.sin_addr.s_addr =inet_addr(MoniforIpCfg);
	//如果没有发起
	//printf("sendcfg %s : %d\n",MoniforIpCfg,DestPort);
	memset(server.sin_zero,0,8);

	iSendLen = sendto(sockid,(int8 *)msgBuff,msgLen,0,(struct sockaddr*)&server, sizeof(server));

	if(iSendLen != msgLen)
		return FALSE;
	else
		return TRUE;
}



void deal_bradio_udp_send( int sockid ,uint16 State_Num)
{

	uint8 chd;
	//发送状态报文
	Send_Bradio_State_udp(sockid,State_Num);
	//发送AIS报文
//	Send_Bradio_Ais_udp(sockid);
	sleep_1(20);
	chd = State_Num%10;
	Send_Bradio_Detail_Msg_udp(sockid,chd);
}


//宽带电台上送状态
void Send_Bradio_State_udp(int sockid,uint16 State_Num)
{
	//uint8 CRC;
	//CRC=0;
	uint8	USV_State_UDP[State_Current_Num];
	//memset(USV_State_UDP,0,State_Current_Num);
	//memcpy(USV_State_UDP,USV_State_Current,State_Current_Num);

	//USV_State_UDP[0]='$';
	//GetCheck(&CRC,USV_State_UDP,sizeof(USV_State_UDP));
	//USV_State_UDP[64]=CRC;

	uint8 CRC;
	CRC=0;
	USV_State_UDP[0]='$';
	USV_State_UDP[1]='U';
	USV_State_UDP[2]='S';
	USV_State_UDP[3]='V';
	USV_State_UDP[4]='R';
	USV_State_UDP[5]=USV_State.USV_Num;
	USV_State_UDP[6]=((State_Num&0xff00)>>8);
	USV_State_UDP[7]=(State_Num&0x00ff);
	USV_State_UDP[8]=State_Current_Num;
	USV_State_UDP[9]=(((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0xff00)>>8);
	USV_State_UDP[10]=((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0x00ff);
	USV_State_UDP[11]=(((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0xff00)>>8);
	USV_State_UDP[12]=((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0x00ff);
	USV_State_UDP[13]=(((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0xff00)>>8);
	USV_State_UDP[14]=((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0x00ff);
	USV_State_UDP[15]=(((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0xff00)>>8);
	USV_State_UDP[16]=((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0x00ff);
	USV_State_UDP[17]=(uint8)USV_State.Dradio_USV_Sailing_State.USV_Heave;
	USV_State_UDP[18]=USV_State.Dradio_USV_Model_State.Latitude_Sign_St;
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Longitude_Sign_St)<<1);
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Sailing_Model_St)<<2);
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Differential_Model_St)<<4);
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Set_Return_Point_Model_St)<<5);
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Speed_Constant_Model_St)<<6);
	USV_State_UDP[18]+=((USV_State.Dradio_USV_Model_State.Direction_Constant_Model_St)<<7);
	USV_State_UDP[19]=USV_State.Dradio_USV_Location.USV_Latitude_Degree;
	USV_State_UDP[20]=USV_State.Dradio_USV_Location.USV_Latitude_Minute;
	USV_State_UDP[21]=USV_State.Dradio_USV_Location.USV_Latitude_Second;
	USV_State_UDP[22]=USV_State.Dradio_USV_Location.USV_Latitude_Decimal;
	USV_State_UDP[23]=USV_State.Dradio_USV_Location.USV_Longitude_Degree;
	USV_State_UDP[24]=USV_State.Dradio_USV_Location.USV_Longitude_Minute;
	USV_State_UDP[25]=USV_State.Dradio_USV_Location.USV_Longitude_Second;
	USV_State_UDP[26]=USV_State.Dradio_USV_Location.USV_Longitude_Decimal;
	USV_State_UDP[27]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0xff00)>>8);
	USV_State_UDP[28]=((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0x00ff);
	USV_State_UDP[29]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0xff00)>>8);
	USV_State_UDP[30]=((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0x00ff);
	USV_State_UDP[31]=USV_State.Dradio_USV_Drive_State.Gear_Left_St;
	USV_State_UDP[32]=USV_State.Dradio_USV_Drive_State.Gear_Right_St;
	USV_State_UDP[33]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St;
	USV_State_UDP[34]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St;
	USV_State_UDP[35]=USV_State.USV_Oil;

	USV_State_UDP[36]=USV_State.Dradio_USV_Battery.Battery_Left;
	USV_State_UDP[37]=USV_State.Dradio_USV_Battery.Battery_TEMP_Left;
	//	printf("Battery_Left=%d-%d\n",USV_State_Current[36],USV_State_Current[37]);
	USV_State_UDP[38]=USV_State.Dradio_USV_Battery.Battery_Right;
	USV_State_UDP[39]=USV_State.Dradio_USV_Battery.Battery_TEMP_Right;
	USV_State_UDP[40]=USV_State.USV_Sailing_Intel_Sign;								//USV航行任务执行状态信息
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.UAV_Power_St)<<2);
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.Stable_Platform_Power_St)<<3);
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.Camera_ahead_Power_St)<<4);
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.D_radar_Power_St)<<5);
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.Camera_main_Power_St)<<6);
	USV_State_UDP[40]+=((USV_State.Dradio_USV_Device_State.Horn_Power_St)<<7);
	USV_State_UDP[41]=USV_State.Dradio_USV_Device_State.Searchlight_Power_St;
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Device_State.Camera_lesser_Power_St)<<1);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Device_State.Navigationlight_Power_St)<<2);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Device_State.Camera_tail_Power_St)<<3);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Left_Charging)<<4);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Right_Charging)<<5);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_L)<<6);
	USV_State_UDP[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_R)<<7);
	USV_State_UDP[42]=USV_State.Dradio_USV_Fire.Outfire_St;
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Fire.Water_Level_St)<<1);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Fire.Ventilation_St)<<2);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Left)<<3);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Right)<<4);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletPressure_Alarm_Left)<<5);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right)<<6);
	USV_State_UDP[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Left)<<7);
	USV_State_UDP[43]=USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right;
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Left)<<1);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Right)<<2);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Left)<<3);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Right)<<4);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Left)<<5);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Right)<<6);
	USV_State_UDP[43]+=((USV_State.Dradio_USV_Engine_Alarm.Fuel_Moisture_Alarm)<<7);


	USV_State_UDP[44]=USV_State.Hull_Fan_st;
	USV_State_UDP[44]+=((USV_State.Hull_Pump_st)<<2);
	USV_State_UDP[44]+=((USV_State.Dradio_USV_Model_State.Panel_Control_Sign_ST)<<6);
	USV_State_UDP[44]+=((USV_State.Dradio_USV_Model_State.Emergency_St)<<7);
	USV_State_UDP[45]=USV_State.APP_12V;
	USV_State_UDP[45]+=((USV_State.APP_24V)<<1);	
	USV_State_UDP[45]+=((USV_State.Outboard_Engine_st_L)<<2);	
	USV_State_UDP[45]+=((USV_State.Outboard_Engine_st_R)<<4);	
	USV_State_UDP[45]+=(E_Stop<<6);		
	USV_State_UDP[46]=(((USV_State.Dradio_USV_Time.Date_Inside)&0xff0000)>>16);
	USV_State_UDP[47]=(((USV_State.Dradio_USV_Time.Date_Inside)&0x00ff00)>>8);
	USV_State_UDP[48]=((USV_State.Dradio_USV_Time.Date_Inside)&0x0000ff);
	USV_State_UDP[49]=(((USV_State.Dradio_USV_Time.Time_Inside)&0xff0000)>>16);
	USV_State_UDP[50]=(((USV_State.Dradio_USV_Time.Time_Inside)&0x00ff00)>>8);
	USV_State_UDP[51]=((USV_State.Dradio_USV_Time.Time_Inside)&0x0000ff);

	USV_State_UDP[52]=((USV_State.USV_ROT&0xff00)>>8);
	USV_State_UDP[53]=USV_State.USV_ROT&0x00ff;
	USV_State_UDP[54]=USV_State.Dradio_USV_Engine_State.Fuel_ConRate;
	USV_State_UDP[55]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_RL_St;
	USV_State_UDP[56]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_AT_St;
	USV_State_UDP[57]=USV_State.Dradio_USV_UAV_State.Platform_Hatch_St;
	USV_State_UDP[57]+=((USV_State.Dradio_USV_UAV_State.Platform_Lift_St)<<3);
	USV_State_UDP[57]+=((USV_State.Dradio_USV_UAV_State.Platform_Open_St)<<6);
	USV_State_UDP[58]=USV_State.Dradio_USV_UAV_State.UAV_Charging_St;
	USV_State_UDP[58]+=USV_State.USV_Current_Sailing;
	USV_State_UDP[58]+=((USV_State.Sailing_Nummber)<<4);
	USV_State_UDP[59]=USV_State.Engine_power;
	USV_State_UDP[59]+=((USV_State.Engine_run)<<2);

	USV_State_UDP[59]+=((USV_State.USV_oil_sign)<<4);
	USV_State_UDP[60]=USV_State.USV_Speed_limit;
	USV_State_UDP[61]=USV_State.Conrtol_System_Msg.Intelligent_rudder;
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Stabilized_platform)<<1;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Energy_management)<<2;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Power_management)<<3;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.UAV_platform)<<4;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Fire_fighting_system)<<5;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Engine_L)<<6;		
	USV_State_UDP[61]+=(USV_State.Conrtol_System_Msg.Engine_R)<<7;	

	//平移旋转状态
	USV_State_UDP[62]  =	USV_State.Dradio_USV_Model_State.translation_mode_St;	
	USV_State_UDP[62] +=	USV_State.Dradio_USV_Model_State.translation_command_St<<2;
	USV_State_UDP[62] +=    USV_State.Dradio_USV_Model_State.rotation_mode_St<<4;
	USV_State_UDP[62] +=	USV_State.Dradio_USV_Model_State.rotation_command_St<<6;


	GetCheck(&CRC,USV_State_UDP,sizeof(USV_State_UDP));
	USV_State_UDP[63]='*';
	USV_State_UDP[64]=CRC;


	Lan1_Bradio_UDP_send(sockid,(int8 *)&USV_State_UDP[0],State_Current_Num);
	return	;
}




//宽带电台UDP上送AIS报文
void Send_Bradio_Ais_udp(int sockid)
{
	uint8 aisMsgBuff[AIS_Buff_Num];
	uint8 Msg_Sign,Msg_Num,CRC,count;
	int length;
	StruAisMsg AisMsgTmp;

	Msg_Sign = AIS_Sign	;	//AIS设备号
	Msg_Num =  AIS_Buff_Num;
	CRC=0;
	if(AisMsgCost(&AisMsgTmp) == 0)
		return;
	memset((char *)&aisMsgBuff[0],0,sizeof(aisMsgBuff));
	Fill_Msg_Header(aisMsgBuff,Msg_Sign,Msg_Num);
	memcpy((char *)&aisMsgBuff[12],(char *)&AisMsgTmp.frame_buff[0],AisMsgTmp.frame_length);
	aisMsgBuff[125] = 1;
	aisMsgBuff[126] = '*';
	GetCheck(&CRC,aisMsgBuff,sizeof(aisMsgBuff));
	aisMsgBuff[127] = CRC;

	Lan1_Bradio_UDP_send(sockid,(int8 *)aisMsgBuff,AIS_Buff_Num);
	
	return;
}

void Send_Bradio_Detail_Msg_udp( int sockid,uint8 chd)
{

	switch(chd){
		case(0):	Send_Motor_Detail_State_udp(sockid);
					break;
		case(1):	Send_Rudder_Detail_State_udp(sockid);
					break;
		case(2):	Send_Stable_Platform_Detail_State_udp(sockid);
					break;
		case(3):	Send_UAV_Detail_State_udp(sockid);
					break;
		case(4):	Send_MCU_State_udp(sockid);
					break;
		case(5):	Send_Hull_State_udp(sockid);
					break;
		case(6):	Send_Smart_Navigation_Msg_udp(sockid);
					break;
		case(7):	Send_Panel_Control_Msg_udp(sockid);
					break;
		case(8):	Send_Energy_Control_Msg_udp(sockid);
					break;
		case(9):	Send_Power_Control_Msg_udp(sockid);
					break;
		default:	break;
	}
	return;
}


//发送发动机详细信息
void Send_Motor_Detail_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Motor_Detail_Buff[Motor_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Motor_Detail_Sign;
	Msg_Num=Motor_Detail_Buff_Num;
	CRC=0;
	memset(Motor_Detail_Buff,0,Motor_Detail_Buff_Num);
	Update_Motor_Detail_State();
	Fill_Msg_Header(Motor_Detail_Buff,Msg_Sign, Msg_Num );
	Motor_Detail_Buff[12]=(((Motor_Detail_St.Motor_Spd_L)&0xff00)>>8);	
	Motor_Detail_Buff[13]=((Motor_Detail_St.Motor_Spd_L)&0x00ff);	
	Motor_Detail_Buff[14]=(((Motor_Detail_St.Motor_Spd_R)&0xff00)>>8);	
	Motor_Detail_Buff[15]=((Motor_Detail_St.Motor_Spd_R)&0x00ff);	
	Motor_Detail_Buff[16]=Motor_Detail_St.Cool_Temp_L;	
	Motor_Detail_Buff[17]=Motor_Detail_St.Cool_Temp_R;	
	Motor_Detail_Buff[18]=Motor_Detail_St.Cool_Lev_L;	
	Motor_Detail_Buff[19]=Motor_Detail_St.Cool_Lev_R;	
	Motor_Detail_Buff[20]=(((Motor_Detail_St.Oil_Temp_L)&0xff00)>>8);	
	Motor_Detail_Buff[21]=((Motor_Detail_St.Oil_Temp_L)&0x00ff);	
	Motor_Detail_Buff[22]=(((Motor_Detail_St.Oil_Temp_R)&0xff00)>>8);	
	Motor_Detail_Buff[23]=((Motor_Detail_St.Oil_Temp_R)&0x00ff);	
	Motor_Detail_Buff[24]=Motor_Detail_St.Fuel_Temp_L;	
	Motor_Detail_Buff[25]=Motor_Detail_St.Fuel_Temp_R;	
	Motor_Detail_Buff[26]=Motor_Detail_St.Inlet_Air_Temp_L;	
	Motor_Detail_Buff[27]=Motor_Detail_St.Inlet_Air_Temp_R;	
	Motor_Detail_Buff[28]=Motor_Detail_St.Oil_Pressure_L;	
	Motor_Detail_Buff[29]=Motor_Detail_St.Oil_Pressure_R;	
	Motor_Detail_Buff[30]=Motor_Detail_St.Supercharger_Pressure_L;	
	Motor_Detail_Buff[31]=Motor_Detail_St.Supercharger_Pressure_R;	
	Motor_Detail_Buff[32]=(((Motor_Detail_St.Fuel_Usage_L)&0xff00)>>8);	
	Motor_Detail_Buff[33]=((Motor_Detail_St.Fuel_Usage_L)&0x00ff);	
	Motor_Detail_Buff[34]=(((Motor_Detail_St.Fuel_Usage_R)&0xff00)>>8);	
	Motor_Detail_Buff[35]=((Motor_Detail_St.Fuel_Usage_R)&0x00ff);	
	Motor_Detail_Buff[36]=(((Motor_Detail_St.Average_Fuel_Usage_L)&0xff00)>>8);	
	Motor_Detail_Buff[37]=((Motor_Detail_St.Average_Fuel_Usage_L)&0x00ff);	
	Motor_Detail_Buff[38]=(((Motor_Detail_St.Average_Fuel_Usage_R)&0xff00)>>8);	
	Motor_Detail_Buff[39]=((Motor_Detail_St.Average_Fuel_Usage_R)&0x00ff);	
	Motor_Detail_Buff[40]=(((Motor_Detail_St.Working_Time_L)&0xff000000)>>24);	
	Motor_Detail_Buff[41]=(((Motor_Detail_St.Working_Time_L)&0x00ff0000)>>16);	
	Motor_Detail_Buff[42]=(((Motor_Detail_St.Working_Time_L)&0x0000ff00)>>8);	
	Motor_Detail_Buff[43]=((Motor_Detail_St.Working_Time_L)&0x000000ff);	
	Motor_Detail_Buff[44]=(((Motor_Detail_St.Working_Time_R)&0xff000000)>>24);	
	Motor_Detail_Buff[45]=(((Motor_Detail_St.Working_Time_R)&0x00ff0000)>>16);	
	Motor_Detail_Buff[46]=(((Motor_Detail_St.Working_Time_R)&0x0000ff00)>>8);	
	Motor_Detail_Buff[47]=((Motor_Detail_St.Working_Time_R)&0x000000ff);	
	Motor_Detail_Buff[48]=(((Motor_Detail_St.Total_turn_L)&0xff000000)>>24);	
	Motor_Detail_Buff[49]=(((Motor_Detail_St.Total_turn_L)&0x00ff0000)>>16);	
	Motor_Detail_Buff[50]=(((Motor_Detail_St.Total_turn_L)&0x0000ff00)>>8);	
	Motor_Detail_Buff[51]=((Motor_Detail_St.Total_turn_L)&0x000000ff);	
	Motor_Detail_Buff[52]=(((Motor_Detail_St.Total_turn_R)&0xff000000)>>24);	
	Motor_Detail_Buff[53]=(((Motor_Detail_St.Total_turn_R)&0x00ff0000)>>16);	
	Motor_Detail_Buff[54]=(((Motor_Detail_St.Total_turn_R)&0x0000ff00)>>8);	
	Motor_Detail_Buff[55]=((Motor_Detail_St.Total_turn_R)&0x000000ff);	
	Motor_Detail_Buff[56]=Motor_Detail_St.Air_preeure_L;	
	Motor_Detail_Buff[57]=(((Motor_Detail_St.Single_Mileage)&0xff00)>>8);	
	Motor_Detail_Buff[58]=((Motor_Detail_St.Single_Mileage)&0x00ff);	
	Motor_Detail_Buff[59]=(((Motor_Detail_St.Total_Mileage)&0x00ff0000)>>16);	
	Motor_Detail_Buff[60]=(((Motor_Detail_St.Total_Mileage)&0x0000ff00)>>8);	
	Motor_Detail_Buff[61]=((Motor_Detail_St.Total_Mileage)&0x000000ff);	
	Motor_Detail_Buff[62]=(((Motor_Detail_St.Single_Fuel_Consume)&0xff00)>>8);	
	Motor_Detail_Buff[63]=((Motor_Detail_St.Single_Fuel_Consume)&0x00ff);	
	Motor_Detail_Buff[64]=(((Motor_Detail_St.Total_Fuel_Consume)&0x00ff0000)>>16);	
	Motor_Detail_Buff[65]=(((Motor_Detail_St.Total_Fuel_Consume)&0x0000ff00)>>8);	
	Motor_Detail_Buff[66]=((Motor_Detail_St.Total_Fuel_Consume)&0x000000ff);	
	Motor_Detail_Buff[67]=(((Motor_Detail_St.Ldle_Spd_L)&0xff00)>>8);	
	Motor_Detail_Buff[68]=((Motor_Detail_St.Ldle_Spd_L)&0x00ff);	
	Motor_Detail_Buff[69]=(((Motor_Detail_St.Ldle_Spd_R)&0xff00)>>8);	
	Motor_Detail_Buff[70]=((Motor_Detail_St.Ldle_Spd_R)&0x00ff);	
	Motor_Detail_Buff[71]=Motor_Detail_St.Ldle_Time_Left;	
	Motor_Detail_Buff[72]=Motor_Detail_St.Ldle_Time_Right;	
	
	Motor_Detail_Buff[73]+=Motor_Detail_St.SuperLoad_Alarm_Left;	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.SuperLoad_Alarm_Right)<<1);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.ECU_Alarm_L)<<2);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.ECU_Alarm_R)<<3);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_TEMP_Alarm_Left)<<4);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_TEMP_Alarm_Right)<<5);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_Level_Alarm_Left)<<6);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_Level_Alarm_Right)<<7);	
	Motor_Detail_Buff[74]+=Motor_Detail_St.OilPressure_Alarm_Left;	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.OilPressure_Alarm_Right)<<1);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Oil_TEMP_Alarm_L)<<2);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Oil_TEMP_Alarm_R)<<3);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Supercharger_Pressure_Alarm_L)<<4);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Supercharger_Pressure_Alarm_R)<<5);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.InletTEMP_Alarm_Left)<<6);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.InletTEMP_Alarm_Right)<<7);	
	Motor_Detail_Buff[75]+=Motor_Detail_St.Fuel_Pressure_Alarm_L;	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Pressure_Alarm_R)<<1);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Moisture_Alarm_L)<<2);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Moisture_Alarm_R)<<3);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Nozzle_Pressure_L)<<4);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Nozzle_Pressure_R)<<5);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Electricity_Alarm_L)<<6);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Electricity_Alarm_R)<<7);	
	Motor_Detail_Buff[76]+=Motor_Detail_St.Whole_Alarm_L;	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Whole_Alarm_R)<<1);	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Heatbeat_L)<<2);	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Heatbeat_R)<<3);	
	

	GetCheck(&CRC,Motor_Detail_Buff,sizeof(Motor_Detail_Buff));
	Motor_Detail_Buff[94]='*';
	Motor_Detail_Buff[95]=CRC; 
	   Lan1_Bradio_UDP_send(sockid,(char *)Motor_Detail_Buff,Msg_Num);

	return	;
}


void Send_Rudder_Detail_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Rudder_Detail_Buff[Rudder_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Rudder_Detail_Sign;
	Msg_Num=Rudder_Detail_Buff_Num;
	CRC=0;
	memset(Rudder_Detail_Buff,0,Rudder_Detail_Buff_Num);
	Update_Rudder_Detail_State();
	Fill_Msg_Header(Rudder_Detail_Buff,Msg_Sign, Msg_Num );
	Rudder_Detail_Buff[12]=(((Rudder_Detail_St.Rudder_Angle_Left_St)&0xff00)>>8);	
	Rudder_Detail_Buff[13]=((Rudder_Detail_St.Rudder_Angle_Left_St)&0x00ff);	
	Rudder_Detail_Buff[14]=(((Rudder_Detail_St.Rudder_Angle_Right_St)&0xff00)>>8);	
	Rudder_Detail_Buff[15]=((Rudder_Detail_St.Rudder_Angle_Right_St)&0x00ff);	
	Rudder_Detail_Buff[16]=(((Rudder_Detail_St.Rudder_Angle_Left_Con)&0xff00)>>8);	
	Rudder_Detail_Buff[17]=((Rudder_Detail_St.Rudder_Angle_Left_Con)&0x00ff);	
	Rudder_Detail_Buff[18]=(((Rudder_Detail_St.Rudder_Angle_Right_Con)&0xff00)>>8);	
	Rudder_Detail_Buff[19]=((Rudder_Detail_St.Rudder_Angle_Right_Con)&0x00ff);	
	Rudder_Detail_Buff[20]=Rudder_Detail_St.Speed_Limit;	
	Rudder_Detail_Buff[21]=(Rudder_Detail_St.Motor_Speed_L&0xff00)<<8;	
	Rudder_Detail_Buff[22]=Rudder_Detail_St.Motor_Speed_L&0x00ff;	
	Rudder_Detail_Buff[23]=(Rudder_Detail_St.Motor_Speed_R&0xff00)<<8;	
	Rudder_Detail_Buff[24]=Rudder_Detail_St.Motor_Speed_R&0x00ff;	
	Rudder_Detail_Buff[25]=Rudder_Detail_St.System_TEMP;	
	Rudder_Detail_Buff[26]=Rudder_Detail_St.System_Power_5;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_12)<<1;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_3)<<2;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_24)<<3;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Check)<<4;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.Heatbeat_Rudder)<<5;	
	

	GetCheck(&CRC,Rudder_Detail_Buff,sizeof(Rudder_Detail_Buff));
	Rudder_Detail_Buff[30]='*';
	Rudder_Detail_Buff[31]=CRC;
		
       Lan1_Bradio_UDP_send(sockid,(char *)Rudder_Detail_Buff,Msg_Num);
	Rudder_Detail_St.Heatbeat_Rudder=0;
	return	;
}


//发送稳定平台详细信息
void Send_Stable_Platform_Detail_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Stable_Platform_Detail_Buff[Stable_Platform_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Stable_Platform_Sign;
	Msg_Num=Stable_Platform_Detail_Buff_Num;
	CRC=0;
	memset(Stable_Platform_Detail_Buff,0,Stable_Platform_Detail_Buff_Num);
	Update_Stable_Platform_Detail_State();
	Fill_Msg_Header(Stable_Platform_Detail_Buff,Msg_Sign, Msg_Num );
	Stable_Platform_Detail_Buff[12]=Stable_Platform_St.Stable_Platform_RL_St;	
	Stable_Platform_Detail_Buff[13]=Stable_Platform_St.Stable_Platform_AT_St;	
	Stable_Platform_Detail_Buff[14]=Stable_Platform_St.System_TEMP;	
	Stable_Platform_Detail_Buff[15]=Stable_Platform_St.System_Power_5;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_12)<<1;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_3)<<2;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_24)<<3;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Check)<<4;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.Stable_Platform_Heatbeat)<<5;	
//	for(count=12;count<15;count++)
//		Stable_Platform_Detail_Buff[count]=count;
	GetCheck(&CRC,Stable_Platform_Detail_Buff,sizeof(Stable_Platform_Detail_Buff));
	Stable_Platform_Detail_Buff[30]='*';
	Stable_Platform_Detail_Buff[31]=CRC;

		 Lan1_Bradio_UDP_send(sockid,(char *)Stable_Platform_Detail_Buff,Msg_Num);

	Stable_Platform_St.Stable_Platform_Heatbeat=0;
	return	;
}


//发送无人机平台详细信息
void Send_UAV_Detail_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 UAV_Detail_Buff[UAV_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=UAV_Detail_Sign;
	Msg_Num=UAV_Detail_Buff_Num;
	CRC=0;
	memset(UAV_Detail_Buff,0,UAV_Detail_Buff_Num);
	Update_UAV_Detail_State();
	Fill_Msg_Header(UAV_Detail_Buff,Msg_Sign, Msg_Num );
	UAV_Detail_Buff[12]=UAV_Detail_St.Platform_Hatch_St;	
	UAV_Detail_Buff[12]+=((UAV_Detail_St.Platform_Lift_St)<<3);	
	UAV_Detail_Buff[12]+=((UAV_Detail_St.Platform_Open_St)<<6);	
	UAV_Detail_Buff[13]=UAV_Detail_St.UAV_Electricity;	
	UAV_Detail_Buff[14]=UAV_Detail_St.UAV_Con_Mod;	
	UAV_Detail_Buff[14]+=((UAV_Detail_St.UAV_Run_Mod)<<2);	
	UAV_Detail_Buff[14]+=((UAV_Detail_St.UAV_Charging_St)<<5);		
	UAV_Detail_Buff[15]=UAV_Detail_St.UAV_TEMP;		
	UAV_Detail_Buff[16]=UAV_Detail_St.UAV_5V_St;	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_12V_St)<<1);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_3V_St)<<2);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_24V_St)<<3);		
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_Check_St)<<4);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_Heatbeat)<<5);	
//	for(count=12;count<15;count++)
//		UAV_Detail_Buff[count]=count;
	GetCheck(&CRC,UAV_Detail_Buff,sizeof(UAV_Detail_Buff));
	UAV_Detail_Buff[30]='*';
	UAV_Detail_Buff[31]=CRC;

	  Lan1_Bradio_UDP_send(sockid,(char *)UAV_Detail_Buff,Msg_Num);
	UAV_Detail_St.UAV_Heatbeat=0;

	return	;
}

//发送控制器监测详细信息
void Send_MCU_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 MCU_Buff[MCU_Buff_Num];
 
	Msg_Sign=MCU_Sign;
	Msg_Num=MCU_Buff_Num;
	CRC=0;
	memset(MCU_Buff,0,MCU_Buff_Num);
	Update_MCU_State();
	Fill_Msg_Header(MCU_Buff,Msg_Sign, Msg_Num );
	MCU_Buff[12]=MCU_St.MCU_TEMP_1;	
	MCU_Buff[13]=MCU_St.MCU_TEMP_2;	
	MCU_Buff[14]=MCU_St.MCU_TEMP_3;	
	MCU_Buff[15]=MCU_St.RAM_Memory;	
	MCU_Buff[16]=MCU_St.RAM_CPU;		//cxy获取系统资源
	MCU_Buff[17]=MCU_St.MPC_Disk;	
	MCU_Buff[18]=MCU_St.MPC_Memory;	
	MCU_Buff[19]=MCU_St.MCP_CPU;	
	MCU_Buff[20]=MCU_St.Mesh_Bandwidth;	//cxy获取带宽

	GetCheck(&CRC,MCU_Buff,sizeof(MCU_Buff));
	MCU_Buff[30]='*';
	MCU_Buff[31]=CRC;
	 Lan1_Bradio_UDP_send(sockid,(char *)MCU_Buff,Msg_Num);
	return	;
}

//发送船体监测详细信息
void Send_Hull_State_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Hull_Buff[Hull_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Hull_Sign;
	Msg_Num=Hull_Buff_Num;
	CRC=0;
	memset(Hull_Buff,0,Hull_Buff_Num);
	Update_Hull_State();
	Fill_Msg_Header(Hull_Buff,Msg_Sign, Msg_Num );
	Hull_Buff[12]=Hull_St.Hull_TEMP_1;	
	Hull_Buff[13]=Hull_St.Hull_TEMP_2;	
	Hull_Buff[14]=Hull_St.Hull_TEMP_3;	
	Hull_Buff[15]+=Hull_St.Outfire_St;	
	Hull_Buff[15]+=((Hull_St.Ventilation_St)<<2);	
	Hull_Buff[15]+=((Hull_St.Water_Pump_St)<<4);
	Hull_Buff[15]+=((Hull_St.Water_Level_St)<<6);
	Hull_Buff[16]=(((Hull_St.Motor_Vibration_1)&0xff000000)>>24);	
	Hull_Buff[17]=(((Hull_St.Motor_Vibration_1)&0x00ff0000)>>16);	
	Hull_Buff[18]=(((Hull_St.Motor_Vibration_1)&0x0000ff00)>>8);	
	Hull_Buff[19]=((Hull_St.Motor_Vibration_1)&0x000000ff);	
	Hull_Buff[20]=(((Hull_St.Motor_Vibration_2)&0xff000000)>>24);	
	Hull_Buff[21]=(((Hull_St.Motor_Vibration_2)&0x00ff0000)>>16);	
	Hull_Buff[22]=(((Hull_St.Motor_Vibration_2)&0x0000ff00)>>8);	
	Hull_Buff[23]=((Hull_St.Motor_Vibration_2)&0x000000ff);	
	Hull_Buff[24]=(((Hull_St.Motor_Vibration_3)&0xff000000)>>24);	
	Hull_Buff[25]=(((Hull_St.Motor_Vibration_3)&0x00ff0000)>>16);	
	Hull_Buff[26]=(((Hull_St.Motor_Vibration_3)&0x0000ff00)>>8);	
	Hull_Buff[27]=((Hull_St.Motor_Vibration_3)&0x000000ff);	
	Hull_Buff[28]=(((Hull_St.Motor_Vibration_4)&0xff000000)>>24);	
	Hull_Buff[29]=(((Hull_St.Motor_Vibration_4)&0x00ff0000)>>16);	
	Hull_Buff[30]=(((Hull_St.Motor_Vibration_4)&0x0000ff00)>>8);	
	Hull_Buff[31]=((Hull_St.Motor_Vibration_4)&0x000000ff);	
	Hull_Buff[32]=(((Hull_St.Motor_Vibration_5)&0xff000000)>>24);	
	Hull_Buff[33]=(((Hull_St.Motor_Vibration_5)&0x00ff0000)>>16);	
	Hull_Buff[34]=(((Hull_St.Motor_Vibration_5)&0x0000ff00)>>8);	
	Hull_Buff[35]=((Hull_St.Motor_Vibration_5)&0x000000ff);	
	Hull_Buff[36]=(((Hull_St.Motor_Vibration_6)&0xff000000)>>24);	
	Hull_Buff[37]=(((Hull_St.Motor_Vibration_6)&0x00ff0000)>>16);	
	Hull_Buff[38]=(((Hull_St.Motor_Vibration_6)&0x0000ff00)>>8);	
	Hull_Buff[39]=((Hull_St.Motor_Vibration_6)&0x000000ff);	
 
	GetCheck(&CRC,Hull_Buff,sizeof(Hull_Buff));
	Hull_Buff[62]='*';
	Hull_Buff[63]=CRC;
	Lan1_Bradio_UDP_send(sockid,(char *)Hull_Buff,Msg_Num);
	return	;

}


//发送多功能惯导详细信息
void Send_Smart_Navigation_Msg_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Smart_Navigation_Buff[Smart_Navigation_Buff_Num];
//	int ret=0;
	Msg_Sign=Smart_Navigation_Sign;
	Msg_Num=Smart_Navigation_Buff_Num;
	CRC=0;
	memset(Smart_Navigation_Buff,0,Smart_Navigation_Buff_Num);
	Fill_Msg_Header(Smart_Navigation_Buff,Msg_Sign, Msg_Num );
	Smart_Navigation_Buff[12]=(((Smart_Navigation_St.USV_Heading)&0xff00)>>8);	
	Smart_Navigation_Buff[13]=((Smart_Navigation_St.USV_Heading)&0x00ff);	
	Smart_Navigation_Buff[14]=(((Smart_Navigation_St.USV_Speed)&0xff00)>>8);	
	Smart_Navigation_Buff[15]=((Smart_Navigation_St.USV_Speed)&0x00ff);	
	Smart_Navigation_Buff[16]=(((Smart_Navigation_St.USV_Roll)&0xff00)>>8);	
	Smart_Navigation_Buff[17]=((Smart_Navigation_St.USV_Roll)&0x00ff);	
	Smart_Navigation_Buff[18]=(((Smart_Navigation_St.USV_Pitch)&0xff00)>>8);	
	Smart_Navigation_Buff[19]=((Smart_Navigation_St.USV_Pitch)&0x00ff);	
	Smart_Navigation_Buff[20]=(((Smart_Navigation_St.USV_Bow)&0xff00)>>8);	
	Smart_Navigation_Buff[21]=((Smart_Navigation_St.USV_Bow)&0x00ff);	
	Smart_Navigation_Buff[22]=(((Smart_Navigation_St.USV_Transverse)&0xff00)>>8);	
	Smart_Navigation_Buff[23]=((Smart_Navigation_St.USV_Transverse)&0x00ff);	
	Smart_Navigation_Buff[24]=(((Smart_Navigation_St.USV_Surge)&0xff00)>>8);	
	Smart_Navigation_Buff[25]=((Smart_Navigation_St.USV_Surge)&0x00ff);	
	Smart_Navigation_Buff[26]=(((Smart_Navigation_St.USV_Heave)&0xff00)>>8);	
	Smart_Navigation_Buff[27]=((Smart_Navigation_St.USV_Heave)&0x00ff);	
	Smart_Navigation_Buff[28]=(((Smart_Navigation_St.USV_Yaw)&0xff00)>>8);	
	Smart_Navigation_Buff[29]=((Smart_Navigation_St.USV_Yaw)&0x00ff);	
	Smart_Navigation_Buff[30]=(((Smart_Navigation_St.USV_ROT)&0xff00)>>8);	
	Smart_Navigation_Buff[31]=((Smart_Navigation_St.USV_ROT)&0x00ff);	
	Smart_Navigation_Buff[32]=(((Smart_Navigation_St.USV_Height)&0x00ff0000)>>16);	
	Smart_Navigation_Buff[33]=(((Smart_Navigation_St.USV_Height)&0x0000ff00)>>8);
	Smart_Navigation_Buff[34]=((Smart_Navigation_St.USV_Height)&0x000000ff);	
	Smart_Navigation_Buff[35]+=Smart_Navigation_St.Latitude_Sign_St;	
	Smart_Navigation_Buff[35]+=((Smart_Navigation_St.Longitude_Sign_St)<<2);	
	Smart_Navigation_Buff[36]=Smart_Navigation_St.USV_Latitude_Degree;	
	Smart_Navigation_Buff[37]=Smart_Navigation_St.USV_Latitude_Minute;	
	Smart_Navigation_Buff[38]=Smart_Navigation_St.USV_Latitude_Second;	
	Smart_Navigation_Buff[39]=Smart_Navigation_St.USV_Latitude_Decimal_2;	
	Smart_Navigation_Buff[40]=Smart_Navigation_St.USV_Latitude_Decimal_4;	
	Smart_Navigation_Buff[41]=Smart_Navigation_St.USV_Longitude_Degree;	
	Smart_Navigation_Buff[42]=Smart_Navigation_St.USV_Longitude_Minute;	
	Smart_Navigation_Buff[43]=Smart_Navigation_St.USV_Longitude_Second;	
	Smart_Navigation_Buff[44]=Smart_Navigation_St.USV_Longitude_Decimal_2;	
	Smart_Navigation_Buff[45]=Smart_Navigation_St.USV_Longitude_Decimal_4;	
	Smart_Navigation_Buff[46]=Smart_Navigation_St.USV_Year;	
	Smart_Navigation_Buff[47]=Smart_Navigation_St.USV_Month;	
	Smart_Navigation_Buff[48]=Smart_Navigation_St.USV_Date;	
	Smart_Navigation_Buff[49]=Smart_Navigation_St.USV_Hour;	
	Smart_Navigation_Buff[50]=Smart_Navigation_St.USV_Minute;	
	Smart_Navigation_Buff[51]=Smart_Navigation_St.USV_Second;	
	Smart_Navigation_Buff[52]=Smart_Navigation_St.Satellite_Num_1;	
	Smart_Navigation_Buff[53]=Smart_Navigation_St.Satellite_Num_2;	
	Smart_Navigation_Buff[54]+=Smart_Navigation_St.Sys_State_1;	
	Smart_Navigation_Buff[54]+=((Smart_Navigation_St.Sys_State_2)<<4);	
	Smart_Navigation_Buff[55]+=Smart_Navigation_St.Power_Light;	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Serial_Light)<<2);	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Main_Antenna_Light)<<4);	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Minor_Antenna_Light)<<6);	
	Smart_Navigation_Buff[56]+=Smart_Navigation_St.Differerntial_Signal_Light;	
	Smart_Navigation_Buff[56]+=((Smart_Navigation_St.Differential_Position_Light)<<2);	
	Smart_Navigation_Buff[56]+=((Smart_Navigation_St.Smart_Navigation_Heatbeat)<<4);	
	Smart_Navigation_Buff[57]=Smart_Navigation_St.Latitude_Sign_St;
	Smart_Navigation_Buff[57]+=(Smart_Navigation_St.Longitude_Sign_St)<<2;
	Smart_Navigation_Buff[58]=Point_Return[1].Waypoint_Latitude_Degree;
	Smart_Navigation_Buff[59]=Point_Return[1].Waypoint_Latitude_Minute;
	Smart_Navigation_Buff[60]=Point_Return[1].Waypoint_Latitude_Second;
	Smart_Navigation_Buff[61]=Point_Return[1].Waypoint_Latitude_Decimal;
	Smart_Navigation_Buff[62]=Point_Return[1].Waypoint_Longitude_Degree;
	Smart_Navigation_Buff[63]=Point_Return[1].Waypoint_Longitude_Minute;
	Smart_Navigation_Buff[64]=Point_Return[1].Waypoint_Longitude_Second;
	Smart_Navigation_Buff[65]=Point_Return[1].Waypoint_Longitude_Decimal;	

	GetCheck(&CRC,Smart_Navigation_Buff,sizeof(Smart_Navigation_Buff));
	Smart_Navigation_Buff[78]='*';
	Smart_Navigation_Buff[79]=CRC;
	Lan1_Bradio_UDP_send(sockid,(char *)Smart_Navigation_Buff,Msg_Num);
	return	;
}

//发送船载控制面板信息
void Send_Panel_Control_Msg_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Panel_Control_Buff[Panel_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Panel_Control_Sign;
	Msg_Num=Panel_Control_Buff_Num;
	CRC=0;
	memset(Panel_Control_Buff,0,Panel_Control_Buff_Num);
	Update_Panel_Control_Msg();
	Fill_Msg_Header(Panel_Control_Buff,Msg_Sign, Msg_Num );
	Panel_Control_Buff[12]=Panel_Control_St.Panel_Control_St;	

	GetCheck(&CRC,Panel_Control_Buff,sizeof(Panel_Control_Buff));
	Panel_Control_Buff[30]='*';
	Panel_Control_Buff[31]=CRC;
	Lan1_Bradio_UDP_send(sockid,(char *)Panel_Control_Buff,Msg_Num);
	return	;
}

//发送能源管理系统详细信息
void Send_Energy_Control_Msg_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Energy_Control_Buff[Energy_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Energy_Control_Sign;
	Msg_Num=Energy_Control_Buff_Num;
	CRC=0;
	memset(Energy_Control_Buff,0,Energy_Control_Buff_Num);
	Update_Power_Control_Msg();
	Fill_Msg_Header(Energy_Control_Buff,Msg_Sign, Msg_Num );
	Energy_Control_Buff[12]=Energy_Control_St.Battery_Left;	
	Energy_Control_Buff[13]=Energy_Control_St.Battery_TEMP_Left;	
	Energy_Control_Buff[14]=Energy_Control_St.Battery_Power_Left;	
	Energy_Control_Buff[15]=Energy_Control_St.Battery_Current_Left;	
	Energy_Control_Buff[16]=Energy_Control_St.Battery_Voltage_Left;	
	Energy_Control_Buff[17]=Energy_Control_St.Battery_Right;	
	Energy_Control_Buff[18]=Energy_Control_St.Battery_TEMP_Right;	
	Energy_Control_Buff[19]=Energy_Control_St.Battery_Power_Right;	
	Energy_Control_Buff[20]=Energy_Control_St.Battery_Current_Right;	
	Energy_Control_Buff[21]=Energy_Control_St.Battery_Voltage_Right;	
	Energy_Control_Buff[22]=Energy_Control_St.Battery_Charge_Left;	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Chager_Right)<<1);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_TEMP_Alarm_Left)<<2);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_TEMP_ALarm_Right)<<3);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Capacity_Left)<<4);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Capacity_Right)<<5);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Power_Control_Heatbeat)<<6);	
	Energy_Control_Buff[23]=Energy_Control_St.SYS_DIN1_St;	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN2_St)<<1);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN3_St)<<2);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN4_St)<<3);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN5_St)<<4);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN6_St)<<5);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN7_St)<<6);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN8_St)<<7);	
	Energy_Control_Buff[24]=Energy_Control_St.SYS_5V_St;	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_12V_St)<<1);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_3V_St)<<2);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_24V_St)<<3);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_Check_St)<<4);	
	Energy_Control_Buff[25]=Energy_Control_St.SYS_TEMP;	

	GetCheck(&CRC,Energy_Control_Buff,sizeof(Energy_Control_Buff));
	Energy_Control_Buff[46]='*';
	Energy_Control_Buff[47]=CRC;
	Lan1_Bradio_UDP_send(sockid,(char *)Energy_Control_Buff,Msg_Num);
	Energy_Control_St.Power_Control_Heatbeat=0;

	return	;
}

//向能源管理发送控制命令
void Send_Power_Control_Msg_udp(int sockid)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Power_Control_Buff[Power_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Power_Control_Sign;
	Msg_Num=Power_Control_Buff_Num;
	CRC=0;
	memset(Power_Control_Buff,0,Power_Control_Buff_Num);
	Update_Power_Control_Msg();
	Fill_Msg_Header(Power_Control_Buff,Msg_Sign, Msg_Num );
	Power_Control_Buff[12]=Power_Control_St.Stable_Platform_Power;	
	Power_Control_Buff[12]+=((Power_Control_St.UAV_Power)<<1);	
	Power_Control_Buff[12]+=((Power_Control_St.Horn_Power)<<2);	
	Power_Control_Buff[12]+=((Power_Control_St.Navigationlight_Power)<<3);	
	Power_Control_Buff[12]+=((Power_Control_St.D_radar_Power)<<4);	
	Power_Control_Buff[12]+=((Power_Control_St.Camera_main_Power)<<5);	
	Power_Control_Buff[12]+=((Power_Control_St.Application_24V)<<6);	
	Power_Control_Buff[12]+=((Power_Control_St.Searchlight_Power)<<7);	
	Power_Control_Buff[13]=Power_Control_St.Camera_ahead_Power;	
	Power_Control_Buff[13]+=((Power_Control_St.Camera_lesser_Power)<<1);	
	Power_Control_Buff[13]+=((Power_Control_St.Camera_tail_Power)<<2);	
	Power_Control_Buff[13]+=((Power_Control_St.Application_12V)<<3);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_5)<<4);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_12)<<5);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_3)<<6);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Check)<<7);	
	Power_Control_Buff[14]=Power_Control_St.System_Voltage;	
	Power_Control_Buff[15]=Power_Control_St.System_Electricity;	
	Power_Control_Buff[16]=Power_Control_St.DIN1_Connection_St;
	Power_Control_Buff[16]+=((Power_Control_St.DIN2_Connection_St)<<1);
	Power_Control_Buff[16]+=((Power_Control_St.DIN3_Connection_St)<<2);
	Power_Control_Buff[16]+=((Power_Control_St.DIN4_Connection_St)<<3);
	Power_Control_Buff[16]+=((Power_Control_St.System_Heatbeat)<<4);
	GetCheck(&CRC,Power_Control_Buff,sizeof(Power_Control_Buff));
	Power_Control_Buff[30]='*';
	Power_Control_Buff[31]=CRC;

	Lan1_Bradio_UDP_send(sockid,(char *)Power_Control_Buff,Msg_Num);
	Power_Control_St.System_Heatbeat=0;

	return	;

}

//上送到后台的状态量，其中未获取的数据统一为0
//udp发送函数
void send_state_msg_udp(int sockid,uint16 State_Num)
{
	uint8	USV_State_UDP[State_Current_Num];
	memset(USV_State_UDP,0,State_Current_Num);
	uint8 CRC;
	CRC=0;
	USV_State_UDP[0]='$';
	USV_State_UDP[1]='U';
	USV_State_UDP[2]='S';
	USV_State_UDP[3]='V';
	USV_State_UDP[4]='R';
	USV_State_UDP[5]=USV_State.USV_Num; //船号
	USV_State_UDP[6]=((State_Num&0xff00)>>8);
	USV_State_UDP[7]=(State_Num&0x00ff);
	USV_State_UDP[8]=State_Current_Num;//长度
	//add&modify by foo 
	//Smart_Navigation_St.USV_Speed = 20;//test
	//memcpy(&(USV_State_UDP[9]), (uint8*)&(Smart_Navigation_St.USV_Speed), 2);//航速
	//Smart_Navigation_St.USV_Heading = 70;//test
	//memcpy(&(USV_State_UDP[11]), (uint8*)&(Smart_Navigation_St.USV_Heading), 2);//航向
	//memcpy(&(USV_State_UDP[13]), (uint8*)&(Smart_Navigation_St.USV_Pitch), 2);//俯仰
	//memcpy(&(USV_State_UDP[15]), (uint8*)&(Smart_Navigation_St.USV_Roll), 2);//横滚
	//memcpy(&(USV_State_UDP[17]), (uint8*)&(Smart_Navigation_St.USV_Heave), 2);//深沉
	uint16 speed_local = ins_msg.u16_speed*10;
	USV_State_UDP[9] = (((speed_local) & 0xff00) >> 8);//航速
	USV_State_UDP[10] = ((speed_local) & 0x00ff);
	uint16 heading_local = ins_msg.u16_heading*10;
	USV_State_UDP[11] = (((heading_local) & 0xff00) >> 8);//航向
	USV_State_UDP[12] = ((heading_local) & 0x00ff);
	uint16 pitch_local = ins_msg.i16_pitch*10+8999;
	USV_State_UDP[13] = (((pitch_local) & 0xff00) >> 8);//俯仰
	USV_State_UDP[14] = ((pitch_local) & 0x00ff);
	uint16 roll_local = ins_msg.i16_roll *10+8999;
	USV_State_UDP[15] = (((roll_local) & 0xff00) >> 8);//横滚
	USV_State_UDP[16] = ((roll_local) & 0x00ff);
	uint8 heave_local = (uint8)(ins_msg.i16_heaving + 500);
	USV_State_UDP[17] = (uint8)heave_local;//升沉


	//定位 更新报文数据
	USV_State_UDP[18] = ins_msg.u8_longiSt &0x01;
	USV_State_UDP[18] += (ins_msg.u8_latiSt &0x01) << 1;
	USV_State_UDP[18] += (command_signal.sail_mode_cmd.b2_sailMode&0x03)<<2;

	USV_State_UDP[19] = ins_msg.u8_longiDeg;
	USV_State_UDP[20] = ins_msg.u8_longiMin;
	USV_State_UDP[21] = ins_msg.u8_longiSec;
	USV_State_UDP[22] = ins_msg.u8_longiSecDec;

	USV_State_UDP[23] = ins_msg.u8_latiDeg;
	USV_State_UDP[24] = ins_msg.u8_latiMin;
	USV_State_UDP[25] = ins_msg.u8_latiSec;
	USV_State_UDP[26] = ins_msg.u8_latiSecDec;


	//更新转速 更新报文数据
	USV_State_UDP[27] = (((IHC_rev_msg.u16_St_Motor1Rpm) & 0xff00) >> 8);
	USV_State_UDP[28] = ((IHC_rev_msg.u16_St_Motor1Rpm) & 0x00ff);
	USV_State_UDP[29] = (((IHC_rev_msg.u16_St_Motor2Rpm) & 0xff00) >> 8);
	USV_State_UDP[30] = ((IHC_rev_msg.u16_St_Motor2Rpm) & 0x00ff);
	//更新档位
	//gear_state_msg.i16_Gear_Angle_Left_Act = IHC_rev_msg.i16_St_Motor1Gear;
	//gear_state_msg.i16_Gear_Angle_Right_Act = IHC_rev_msg.i16_St_Motor2Gear;
	//USV_State_UDP[31] = (uint8)(IHC_rev_msg.i16_St_Motor1Gear);//左档位->
	USV_State_UDP[31] = (IHC_rev_msg.i16_St_Motor1Gear+128);
	//USV_State_UDP[32] = (uint8)(IHC_rev_msg.i16_St_Motor2Gear);//右档位->
	USV_State_UDP[32] = (IHC_rev_msg.i16_St_Motor2Gear+128);
	//更新舵角
	//USV_State_UDP[33] = (uint8)(IHC_rev_msg.i16_St_Motor1Rudder);//左舵->
	USV_State_UDP[33] = (IHC_rev_msg.i16_St_Motor1Rudder*0.1+128);
	//USV_State_UDP[34] = (uint8)(IHC_rev_msg.i16_St_Motor2Rudder);//右舵->
	USV_State_UDP[34] = (IHC_rev_msg.i16_St_Motor2Rudder*0.1+128);
	//更新油量
	USV_State_UDP[35] = (IDU_rev_msg.u8_St_PortOilLvl+IDU_rev_msg.u8_St_STBDOilLvl)/2;

	//更新电池
	USV_State_UDP[36] = IDU_rev_msg.u8_St_PORTBatLvl;//左电池电量->

	USV_State_UDP[38] = IDU_rev_msg.u8_St_STBDBatLvl;//右电池电量->

	//更新急停信息
	USV_State_UDP[45] = jet_system.b1_cmd_emergencyStop<<6;
	USV_State_UDP[45] |= (autoNaviSt.b1_st_apf && (command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_AUTO)) << 7;		//2019年3月7日11:19:46 增加避障标志


	uint32 insideDate = (uint32)state_signal.time.u8_year*10000;
	insideDate += (uint32)state_signal.time.u8_month*100;
	insideDate += (uint32)state_signal.time.u8_date;

	uint32 insideTime = (uint32)state_signal.time.u8_hour*10000;
	insideTime += (uint32)state_signal.time.u8_minute*100;
	insideTime += (uint32)state_signal.time.u8_second;

	//日期 更新惯导报文数据
	//USV_State_UDP[46] = ins_msg.u8_year;
	//USV_State_UDP[47] = ins_msg.u8_month;
	//USV_State_UDP[48] = ins_msg.u8_date;

	USV_State_UDP[46] = (insideDate&0xff0000)>>16	;
	USV_State_UDP[47] = (insideDate&0x00ff00)>>8	;
	USV_State_UDP[48] = (insideDate&0x0000ff)		;

	////时间 更新更新惯导报文数据
	//USV_State_UDP[49] = ins_msg.u8_hour;
	//USV_State_UDP[50] = ins_msg.u8_minute;
	//USV_State_UDP[51] = ins_msg.u8_second;

	USV_State_UDP[49] = (insideTime&0xff0000)>>16	;
	USV_State_UDP[50] = (insideTime&0x00ff00)>>8	;
	USV_State_UDP[51] =	(insideTime&0x0000ff)		;
 

	//更新转向率
	uint16 rot_local = ins_msg.i16_rot + 10000;
	USV_State_UDP[52] = ((rot_local & 0xff00) >> 8);//转向率
	USV_State_UDP[53] = rot_local & 0x00ff;//转向率
	//更新油耗率
	//USV_State_UDP[54] = USV_State.Dradio_USV_Engine_State.Fuel_ConRate;
	USV_State_UDP[58]+= (sailTask.sailMsg.u8_sailNum &0x0f)<<4;		//航行任务编号
	GetCheck(&CRC,USV_State_UDP,sizeof(USV_State_UDP));
	USV_State_UDP[63]='*';
	USV_State_UDP[64]=CRC;

	Lan1_Bradio_UDP_send(sockid,(int8 *)&USV_State_UDP[0],State_Current_Num);

	//commented @foo 2019-05-24
	//write_uart(UART4_Fd,(int8 *)&USV_State_UDP[0],State_Current_Num);	//串口发送报文
	return;
}


extern void initLanUdpSendTask( void )
{
	addTask(3,runLanUdpSendTask,(void*)0);
}

void runLanUdpSendTask( void* )
{
	static uint16 SendNum=0;
	SendNum++;
	send_state_msg_udp(udp_sockid,SendNum);
}
