/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/lan_deal_CtrlCenter.h"
#include "util/easylogging++.h"
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

using std::cout;
using std::endl;

#define BRCC_DISCONNECT_MAX 20
#define BRCC_SEND_FRAME_LEN 200
/******************************  Local Variable  *****************************/
char CtrlCenterIP[30];
uint32 BRCC_UDP_RECV_PORT;
uint32 BRCC_UDP_SEND_PORT;

COMM_SIGN		brCc_sign = { 0, COMM_CONNECT_FAIL};

UDP_INF_STRUCT	BrCC_udp_inf;
int16			BrCC_udp_sockid;
uint8			log_b1_BrCC_sign = 0;


BRADIO_USV_STATE	br_usv_state;

WARN_MSG			br_warn_confirm;
WARN_MSG			br_warn_pub;
BRADIO_CC_CMD		br_usv_cmd;
SAIL_TASK_BUFF		br_sailTaskRcv;


/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void BRCC_CommCal(void *);
void BrCC_SendUsvState(void);			
void BrCC_SendTaskConfirm(void);		
void BrCC_SendWarnMsg(void);			
int8 BrCC_rec_report(int16 sockid);
int get_CC_frameType(int8 *buff, uint16 len);
int8 deal_CC_all_parsing(int8 *buff, uint16 len);
int8 deal_CC_cmd_parsing(int8 *buff, uint16 len);
int8 deal_CC_sailTask_parsing(int8 *buff, uint16 len);
int8 deal_CC_warnConfirm_parsing(int8 *buff, uint16 len);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
void BrCC_udp_inf_init(void)
{

	memset((int8*)&BrCC_udp_inf, 0, sizeof(BrCC_udp_inf));
}

void BRCC_CommCal(void *)
{
	comm_time_cal(BRCC_DISCONNECT_MAX, &(brCc_sign.timer), &(brCc_sign.comm_sign));
}

void BrCC_CommCalInit(void)
{
	addTask(3, BRCC_CommCal, (void*)0);
}

void * lan_deal_CtrlCenter(void *aa)
{
	BrCC_udp_sockid = -1;
	memset((char*)&br_usv_cmd, 0, sizeof(br_usv_cmd));
	brCc_sign.comm_sign = COMM_CONNECT_FAIL;
	//test warnnig
	//for (int i = 1; i <= 38; i++)
	//{

	//	//WarnMsgQueuePut(WARN_SRC_IHC, i , 1);
	//	WarnMsgQueuePut(WARN_SRC_IHC, i , 0);
	//}
	//for (int j = 1; j <= 13; j++)
	//{
	//	WarnMsgQueuePut(WARN_SRC_ARM, 13, 1);
	//	WarnMsgQueuePut(WARN_SRC_ARM, 13, 0);
	//}

	for (;;)
	{
		if (-1 == BrCC_udp_sockid)
		{
			BrCC_udp_sockid = CreateUDPSocket((uint16)BRCC_UDP_RECV_PORT);	
			if (BrCC_udp_sockid < 0){
				sleep_1(1000);
				continue;
			}
		}
		else{
			//printf("upd receive data...\n");
			BrCC_rec_report(BrCC_udp_sockid);
		}
		sleep_1(100);
	}
}


int8 BrCC_rec_report(int16 sockid)
{
	fd_set set;
	int16 ret;
	int16 iLen;
	struct timeval timeout;
	socklen_t sockaddrlen;
	int8 net_rec_buff[1024];
	uint16 i = 0;

	FD_ZERO(&set);
	FD_SET(sockid, &set);
	timeout.tv_sec = 0;
	timeout.tv_usec = 5;
	ret = select(sockid + 1, &set, NULL, NULL, &timeout);
	if (ret < 0)
	{
		return false;
	}
	if (FD_ISSET(sockid, &set))
	{
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff), 0, (struct sockaddr*)&BrCC_udp_inf.from_upd_ip, &sockaddrlen);
		if (iLen >= 10)
		{
			BrCC_udp_inf.udp_rec_flag = SWITCH_ON;
			if (net_rec_buff[5] == usv_num)		
			{
				deal_CC_all_parsing(net_rec_buff, iLen);
			}

		}
	}
	return true;
}


int8 deal_CC_all_parsing(int8 *buff,uint16 len)
{
	int8 iret;
	int8 frameType = get_CC_frameType(buff,len);

	switch (frameType)
	{
		case FRAME_TYPE_CONTROL:
			if ((iret = deal_CC_cmd_parsing(buff, len)) == 1)
			{
				monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCCMD_SN].rec_ok_number++;			//���ճɹ�����
			}
			break;
		case FRAME_TYPE_SAILTASK:
			if ((iret = deal_CC_sailTask_parsing(buff, len)) == 1)
			{
				monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCTASK_SN].rec_ok_number++;			//���ճɹ�����
			}
			break;
		case FRAME_TYPE_WARNCONFIRM:
			if ((iret = deal_CC_warnConfirm_parsing(buff, len)) == 1)
			{
				monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCWARN_SN].rec_ok_number++;			//���ճɹ�����
			}
			break;
		default:
			iret = 0;
			break;
	}

	return iret;
}

int get_CC_frameType(int8 *buff, uint16 len)
{
	int8 iret_frameType = FRAME_TYPE_NULL;

	

	if ((buff[0] == '$') && (buff[1] == 'U') && (buff[2] == 'S') && (buff[3] == 'V') && (buff[4] == 'T'))
	{
		iret_frameType = FRAME_TYPE_CONTROL;	
	}
	if ((buff[0] == '#') && (buff[1] == 'U') && (buff[2] == 'S') && (buff[3] == 'V') && (buff[4] == 'T'))
	{
		iret_frameType = FRAME_TYPE_SAILTASK;	
	}
	if ((buff[0] == '$') && (buff[1] == 'W') && (buff[2] == 'N') && (buff[3] == 'A') && (buff[4] == 'R'))
	{
		iret_frameType = FRAME_TYPE_WARNCONFIRM;	
	}

	return iret_frameType;
}

int8 deal_CC_cmd_parsing(int8 *buff, uint16 len)
{
	int8  iret = 0;
	uint8 frameLen = buff[8];
	uint8 temp[1024];
	memcpy(temp,buff,sizeof(temp));
	if (CheckCRC(1, (uint8*)buff, frameLen + 2))	//У��
	{
		memcpy((char*)&br_usv_cmd, (char*)&(buff[9]), sizeof(br_usv_cmd));
		comm_time_return(&(brCc_sign.timer), &(brCc_sign.comm_sign));	
		iret = 1;
	//	printf("br_usv_cmd.u8_cmd_getAuthority = %d",br_usv_cmd.u8_cmd_getAuthority);
	}

	return iret;
}

int8 deal_CC_sailTask_parsing(int8 *buff, uint16 len)
{
	int8 iret = 0;
	uint16 frameLen = *((uint16*)&buff[8]);
	double temp=0;
	//UDP_SAILTASK_REPORT_STRUCT *p_udp_sailTask_report;
	UDP_SAILTASK_REPORT_STRUCT udp_sailTask_report;

	if (CheckCRC(1, (uint8*)buff, frameLen + 2))	//У��
	{
		udp_sailTask_report.socket_ID_h = buff[6];
		udp_sailTask_report.socket_ID_l = buff[7];
		udp_sailTask_report.frame_len = u8tou16((uint8*)&buff[8]);
		udp_sailTask_report.point_sum = buff[10];
		udp_sailTask_report.task_squence = buff[11];
		udp_sailTask_report.point_num = buff[12];

		for (int i = 0; i < udp_sailTask_report.point_num; i++)
		{
			udp_sailTask_report.arrayTaskPoint[i].u8_lat_st			=   buff[13 + i*25 + 0];
			udp_sailTask_report.arrayTaskPoint[i].u8_lat_deg		=   buff[13 + i*25 + 1];
			udp_sailTask_report.arrayTaskPoint[i].u8_lat_min		=   buff[13 + i*25 + 2];
			udp_sailTask_report.arrayTaskPoint[i].u8_lat_sec		=   buff[13 + i*25 + 3];
			udp_sailTask_report.arrayTaskPoint[i].u8_lat_secDec		=   buff[13 + i*25 + 4];
			udp_sailTask_report.arrayTaskPoint[i].u8_lon_st			=  	buff[13 + i*25 + 5];
			udp_sailTask_report.arrayTaskPoint[i].u8_lon_deg		=  	buff[13 + i*25 + 6];
			udp_sailTask_report.arrayTaskPoint[i].u8_lon_min		=  	buff[13 + i*25 + 7];
			udp_sailTask_report.arrayTaskPoint[i].u8_lon_sec		=   buff[13 + i*25 + 8];
			udp_sailTask_report.arrayTaskPoint[i].u8_lon_secDec		=  	buff[13 + i*25 + 9];
			udp_sailTask_report.arrayTaskPoint[i].u16_speed = u8tou16((uint8*)&buff[13 + i * 25 + 10]);
			udp_sailTask_report.arrayTaskPoint[i].u16_stopTime = u8tou16((uint8*)&buff[13 + i * 25 + 12]);
			udp_sailTask_report.arrayTaskPoint[i].u64_pointId = u8tou64((uint8*)&buff[13 + i * 25 + 14]);
			udp_sailTask_report.arrayTaskPoint[i].u8_pointType		=   buff[13 + i*25 + 22];
			udp_sailTask_report.arrayTaskPoint[i].u16_sampleVolume = u8tou16((uint8*)&buff[13 + i * 25 + 23]);

			printf("WP_LAT(num-st-deg-min-sec-secdec)::%d,%d", (i+1),udp_sailTask_report.arrayTaskPoint[i].u8_lat_st);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lat_deg);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lat_min);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lat_sec);
			printf("-%d\n", udp_sailTask_report.arrayTaskPoint[i].u8_lat_secDec);

			printf("WP_LON(num-st-deg-min-sec-secdec)::%d,%d",(i+1),udp_sailTask_report.arrayTaskPoint[i].u8_lon_st);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lon_deg);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lon_min);
			printf("-%d", udp_sailTask_report.arrayTaskPoint[i].u8_lon_sec);
			printf("-%d\n", udp_sailTask_report.arrayTaskPoint[i].u8_lon_secDec);


		}

		if (udp_sailTask_report.socket_ID_l == 0){
			memset((char*)&br_sailTaskRcv, 0, sizeof(br_sailTaskRcv));
		}
		if (udp_sailTask_report.socket_ID_l >= 10){
			return iret;
		}
	
		br_sailTaskRcv.savedPointNum += udp_sailTask_report.point_num;
		for (int i = 0; i < udp_sailTask_report.point_num; i++)
		{
			temp  = (double)udp_sailTask_report.arrayTaskPoint[i].u8_lat_deg;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lat_min / 60.0;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lat_sec / 3600.0;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lat_secDec / 360000.0;
			if (udp_sailTask_report.arrayTaskPoint[i].u8_lat_st == 1)
				temp = 0 - temp;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].f64_latitude = temp;

			temp =  (double)udp_sailTask_report.arrayTaskPoint[i].u8_lon_deg;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lon_min / 60.0;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lon_sec / 3600.0;
			temp += (double)udp_sailTask_report.arrayTaskPoint[i].u8_lon_secDec / 360000.0;
			if (udp_sailTask_report.arrayTaskPoint[i].u8_lon_st == 1)
				temp = 0 - temp;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].f64_longitude = temp;
			
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].f64_expSpeed = udp_sailTask_report.arrayTaskPoint[i].u16_speed / 10.0;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].u16_stopTime = udp_sailTask_report.arrayTaskPoint[i].u16_stopTime;

			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].b1_type = udp_sailTask_report.arrayTaskPoint[i].u8_pointType;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].u64_sailPointID = udp_sailTask_report.arrayTaskPoint[i].u64_pointId;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].u16_sampleVolume = udp_sailTask_report.arrayTaskPoint[i].u16_sampleVolume;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].b1_samplingComplete = 0;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].b1_sailArrival     = 0;
			br_sailTaskRcv.sailTask.sailMsg.wayPoint[udp_sailTask_report.socket_ID_l * 20 + i].b1_samplingCommand = 0;
			
		}

	
		if (udp_sailTask_report.socket_ID_h == 0x80){
			if (br_sailTaskRcv.savedPointNum == udp_sailTask_report.point_sum){	
				br_sailTaskRcv.updateOk = 1;
				br_sailTaskRcv.sailTask.sailMsg.u8_msgSrc = 0;	
				br_sailTaskRcv.sailTask.u8_St_sailMsgRev = 1;
				br_sailTaskRcv.sailTask.u8_PointNum = 0;		
				br_sailTaskRcv.sailTask.sailMsg.u8_pointSum = udp_sailTask_report.point_sum;
				br_sailTaskRcv.sailTask.sailMsg.u8_sailNum = udp_sailTask_report.task_squence;
				iret = 1;

				
				memcpy(&sailTask, &br_sailTaskRcv.sailTask, sizeof(sailTask));
			
			}
		}


	}
	return iret;
}


int8 deal_CC_warnConfirm_parsing(int8 *buff, uint16 len)
{
	int8  iret = 0;
	uint8 frameLen = buff[8];
	uint16 pb_warn_squence;
	if (CheckCRC(1, (uint8*)buff, frameLen + 2))	//У��
	{
		br_warn_confirm.warn_squence = u8tou16((uint8*)&buff[15]);
		br_warn_confirm.warn_stat = buff[17];

		if (br_warn_pub.warn_source == WARN_SRC_IHC){
			pb_warn_squence = br_warn_pub.warn_squence + IHC_WARN_SQ_BASE;
		}
		else{
			pb_warn_squence = br_warn_pub.warn_squence;
		}

		if ((br_warn_confirm.warn_squence == pb_warn_squence) && (br_warn_confirm.warn_stat == br_warn_pub.warn_stat))
		{
			delWarnMsgFromUdpQueue();
			iret = 1;
		}

	
	}

	return iret;
}

//void updateBrCC_usvStateTest(void)
//{
//	br_usv_state.u8_st_year = 1;
//	br_usv_state.u8_st_month = 2;
//	br_usv_state.u8_st_date = 3;
//	br_usv_state.u8_st_hour = 4;
//	br_usv_state.u8_st_minute = 5;
//	br_usv_state.u8_st_second = 6;
//	br_usv_state.u8_st_latSt = 7;
//	br_usv_state.u8_st_latDeg = 8;
//	br_usv_state.u8_st_latMin = 9;
//	br_usv_state.u8_st_latSec = 10;
//	br_usv_state.u8_st_latSecDec = 11;
//	br_usv_state.u8_st_lonSt = 12;
//	br_usv_state.u8_st_lonDeg = 13;
//	br_usv_state.u8_st_lonMin = 14;
//	br_usv_state.u8_st_lonSec = 15;
//	br_usv_state.u8_st_lonSecDec = 16;
//	br_usv_state.u16_st_speed = 17;
//	br_usv_state.u16_st_heading = 18;
//	br_usv_state.u16_st_velocityDir = 19;
//	br_usv_state.i16_st_pitch = -20;
//	br_usv_state.i16_st_roll = -21;
//	br_usv_state.i16_st_heaving = -22;
//	br_usv_state.i16_st_rot = -23;
//	br_usv_state.u8_st_emergencyStop = 24;
//	br_usv_state.u8_st_authority = 25;
//	br_usv_state.u8_st_sailMode = 26;
//	br_usv_state.u8_st_sailTaskState = 27;
//	br_usv_state.u8_st_speedConstant = 28;
//	br_usv_state.u8_st_headingConstant = 29;
//	br_usv_state.u8_st_berthMode = 30;
//	br_usv_state.u8_st_motorState = 31;
//	br_usv_state.u16_st_moter1Rpm = 32;
//	br_usv_state.i16_st_moter1Gear = -33;
//	br_usv_state.i16_st_moter1Rudder = -34;
//	br_usv_state.u16_st_moter2Rpm = 35;
//	br_usv_state.i16_st_moter2Gear = -36;
//	br_usv_state.i16_st_moter2Rudder = -37;
//	br_usv_state.u16_st_moter3Rpm = 38;
//	br_usv_state.i16_st_moter3Gear = -39;
//	br_usv_state.i16_st_moter3Rudder = -40;
//	br_usv_state.u16_st_moter4Rpm = 41;
//	br_usv_state.i16_st_moter4Gear = -42;
//	br_usv_state.i16_st_moter4Rudder = -43;
//	br_usv_state.u16_st_voltage = 44;
//	br_usv_state.u16_st_current = 45;
//	br_usv_state.u8_remainTime_h = 46;
//	br_usv_state.u8_remainTime_m = 47;
//	br_usv_state.u8_remainTime_s = 48;
//	br_usv_state.u8_powerPercent = 49;
//}



void updateBrCC_usvState(void)
{

	static double last_lat=0;
	static double last_long=0;
	br_usv_state.u8_st_year  = state_signal.time.u8_year;
	br_usv_state.u8_st_month = state_signal.time.u8_month;
	br_usv_state.u8_st_date  = state_signal.time.u8_date;

	br_usv_state.u8_st_hour   = state_signal.time.u8_hour;
	br_usv_state.u8_st_minute = state_signal.time.u8_minute;
	br_usv_state.u8_st_second = state_signal.time.u8_second;

	br_usv_state.u8_st_latSt = ins_msg.u8_latiSt ^ 0x01;
	br_usv_state.u8_st_latDeg = ins_msg.u8_latiDeg;
	br_usv_state.u8_st_latMin = ins_msg.u8_latiMin;
	br_usv_state.u8_st_latSec = ins_msg.u8_latiSec;
	br_usv_state.u8_st_latSecDec = ins_msg.u8_latiSecDec;

	br_usv_state.u8_st_lonSt = ins_msg.u8_longiSt ^ 0x01;
	br_usv_state.u8_st_lonDeg = ins_msg.u8_longiDeg;
	br_usv_state.u8_st_lonMin = ins_msg.u8_longiMin;
	br_usv_state.u8_st_lonSec = ins_msg.u8_longiSec;
	br_usv_state.u8_st_lonSecDec = ins_msg.u8_longiSecDec;



	br_usv_state.u16_st_speed   = ins_msg.u16_speed;
	br_usv_state.u16_st_heading = ins_msg.u16_heading;
	br_usv_state.u16_st_velocityDir = ins_msg.u16_volecityDir;

	br_usv_state.i16_st_pitch	= ins_msg.i16_pitch		;
	br_usv_state.i16_st_roll	= ins_msg.i16_roll		;
	br_usv_state.i16_st_heaving	= ins_msg.i16_heaving	;
	br_usv_state.i16_st_rot		= ins_msg.i16_rot		;

	br_usv_state.u8_st_emergencyStop = jet_system.b1_cmd_emergencyStop;

	if (IHC_rev_msg.mid_st.b1_St_Authority == 1)
	{
		br_usv_state.u8_st_authority = USV_RM_AUTHROTY;
	}
	else if (command_signal.sail_mode_cmd.u8_authority == BRCC_AUTHORITY)
	{
		br_usv_state.u8_st_authority = USV_CC_AUTHROTY;
	}
	else if(command_signal.sail_mode_cmd.u8_authority == NO_AUTHORITY)
	{
		br_usv_state.u8_st_authority = USV_NO_AUTHROTY;
	}
	br_usv_state.u8_st_sailMode = command_signal.sail_mode_cmd.b2_sailMode;
	br_usv_state.u8_st_sailTaskState = command_signal.sail_feedBack.b2_sailTask;
	br_usv_state.u8_st_speedConstant = command_signal.func_mode_cmd.b1_speedConstant;
	br_usv_state.u8_st_headingConstant = command_signal.func_mode_cmd.b1_headingConstant;

	br_usv_state.u8_st_berthMode = usv_sign.log_b1_dock_state;						
	br_usv_state.u8_dockcmd_feedback = usv_sign.cmd_feedback;
	br_usv_state.u8_st_motorState = IHC_rev_msg.mid_st.b1_St_MotorOn;
	
<<<<<<< HEAD
    //fabsf(jet_system.jetL.u8_Cmd_MotorOpenDeg)* 1000.f/255.0f ;//
	br_usv_state.u16_st_moter1Rpm    = fabsf(jet_system.jetL.u8_Cmd_MotorOpenDeg)* 1000.f/255.0f;
	br_usv_state.i16_st_moter1Gear   = IHC_rev_msg.i16_St_Motor1Gear;
	br_usv_state.i16_st_moter1Rudder = IHC_rev_msg.i16_St_Motor1Rudder;
										
	//fabsf(jet_system.jetR.i16_Cmd_MotorRudderDeg)* 1000.0f/255.0f ;//
	br_usv_state.u16_st_moter2Rpm    = fabsf(jet_system.jetR.i16_Cmd_MotorRudderDeg)* 1000.0f/255.0f ;
=======

	br_usv_state.u16_st_moter1Rpm    = fabsf(jet_system.jetL.u8_Cmd_MotorOpenDeg)*1000.f/255.0f;//IHC_rev_msg.u16_St_Motor1Rpm;
	br_usv_state.i16_st_moter1Gear   = IHC_rev_msg.i16_St_Motor1Gear;
	br_usv_state.i16_st_moter1Rudder = IHC_rev_msg.i16_St_Motor1Rudder;
										

	br_usv_state.u16_st_moter2Rpm    = fabsf(jet_system.jetL.i16_Cmd_MotorRudderDeg)*1000.f/255.0f;//IHC_rev_msg.u16_St_Motor2Rpm;
>>>>>>> 4ee0903dae03683f62fc1fcf9c95ac370491883a
	br_usv_state.i16_st_moter2Gear   = IHC_rev_msg.i16_St_Motor2Gear;
	br_usv_state.i16_st_moter2Rudder = IHC_rev_msg.i16_St_Motor2Rudder;

	//printf("br_usv_state.u16_st_moter1Rpm == %d\n", br_usv_state.u16_st_moter1Rpm);
	//printf("br_usv_state.u16_st_moter2Rpm == %d\n", br_usv_state.u16_st_moter2Rpm);

	br_usv_state.u16_st_moter3Rpm    =	 IHC_rev_msg.u16_St_Motor1Rpm;
	br_usv_state.i16_st_moter3Gear   =	 0;
	br_usv_state.i16_st_moter3Rudder =	 0;

	br_usv_state.u16_st_moter4Rpm    =	IHC_rev_msg.u16_St_Motor2Rpm;
	br_usv_state.i16_st_moter4Gear   =	 0;
	br_usv_state.i16_st_moter4Rudder =	 0;


	// Battery information
	if(batt_cfg.battery_data_channel == 0){
		br_usv_state.u16_st_voltage = IHC_rev_msg.u16_St_BatteryVoltage;
		br_usv_state.u16_st_current = IHC_rev_msg.u16_St_BatteryCurrent;

		br_usv_state.u8_remainTime_h = IHC_rev_msg.u8_St_BatteryRemainTime_H  ;
		br_usv_state.u8_remainTime_m = IHC_rev_msg.u8_St_BatteryRemainTime_M  ;
		br_usv_state.u8_remainTime_s = IHC_rev_msg.u8_St_BatteryRemainTime_S  ;
		br_usv_state.u8_powerPercent = IHC_rev_msg.u8_St_BatteryLevel		  ;
	}
	else if(batt_cfg.battery_data_channel == 1){
		br_usv_state.u16_st_voltage  = bms1_msg.voltage;
		br_usv_state.u16_st_current  = fabsf(bms1_msg.current);
		
		if(batt_cfg.battery_number >1){
			uint16 tmp = fabsf(bms2_msg.current);
			br_usv_state.u8_remainTime_h = bms2_msg.voltage &0x00ff  ;
			br_usv_state.u8_remainTime_m = bms2_msg.voltage >> 8 ;
			br_usv_state.u8_remainTime_s = tmp &0x00ff;
			br_usv_state.u8_powerPercent = tmp >>8;
		}
	}else{

	}


	br_usv_state.u16_pseudoRangeError = (uint16)((rtk_locating_fuc == 1 ? irtk_msg.pseudorRangeError : ins_msg.pseudorRangeError) * 100);//rtk / ins ��λ
	br_usv_state.u8_locationState = rtk_locating_fuc == 1 ? irtk_msg.rtk_state.b1_diffSignalValid : ins_msg.insState.b1_diffSignalValid;


	if(br_usv_state.u8_st_latDeg!=0&&br_usv_state.u8_st_lonSt!=0)
		{
			double distance = fabsf(Get_distance(ins_msg.latitude,ins_msg.longitude,last_lat,last_long));
			if(distance >=10)
				{
					LOG(ERROR)<<"center send::ins_msg jump from "<<last_lat<<" ,"<<last_long<<" to "
					  <<ins_msg.latitude<<" , "<<ins_msg.longitude 
					  <<" RangeError = "<< br_usv_state.u16_pseudoRangeError 
					  << " locationState = "<<	br_usv_state.u8_locationState 
					  << " distance = " <<distance
					  <<std::endl;
				}
		}

	last_lat = ins_msg.latitude;
	last_long = ins_msg.longitude;

	//printf("locationState = %d\n",br_usv_state.u8_locationState );

	br_usv_state.u8_readyGetOutDock = jet_system.b1_cmd_emergencyStop == 0 && (false == pAutoReturnInst->IsLowPowerAutoReturnNeeded());
	br_usv_state.u8_sailPointSq = sailTask.u8_PointNum;
	br_usv_state.u8_avoidControlState = autoNaviSt.b1_st_apf;

	br_usv_state.u16_expSpeed = (pPidAutoSpeed->getSetingValue()*10);
	br_usv_state.u16_expHeading = (pPidHeading->getSetingValue() * 10);
	
	//����״̬
	br_usv_state.u8_return_state = usv_sign.log_b1_return_state;
	//���÷�����״̬
	br_usv_state.u8_returnPointSetOn = usv_sign.set_retpos_on;

	br_usv_state.u8_sateliteNum_ins = ins_msg.insState.u8_sateliteNum1;//ins ������
	br_usv_state.u8_sateliteNum_irtk = irtk_sn_msg.Satellite_Num_1;//rtk ������
//	printf("ins_sateliteNum = %d,rtk_sateliteNum = %d\n",br_usv_state.u8_sateliteNum_ins,br_usv_state.u8_sateliteNum_irtk);

	br_usv_state.u8_c_rmcValid_ins = (ins_msg.insState.c_rmcValid == 'A'? 1:0);
	br_usv_state.u8_c_rmcValid_irtk = (irtk_msg.rtk_state.c_rmcValid == 'A'?1:0);
	//printf("ins_valid = %d,rtk_valid = %d\n",br_usv_state.u8_c_rmcValid_ins,br_usv_state.u8_c_rmcValid_irtk);
	
	if(batt_cfg.battery_data_channel == 1){
		br_usv_state.u8_reserve18 = (100 * (float)(bms1_msg.batt_remain)/bms1_msg.batt_total);
		br_usv_state.u8_reserve19 = (100 * (float)(bms2_msg.batt_remain)/bms2_msg.batt_total);
	}else{
		br_usv_state.u8_reserve18 = 0;
		br_usv_state.u8_reserve19 = 0;
	}
	
	br_usv_state.u8_reserve20 = IHC_rev_msg.b1_water_level;

}


void runBrCC_SendMsgFast(void*)	//fast 200ms
{
	updateBrCC_usvState();
	//updateBrCC_usvStateTest();
	BrCC_SendUsvState();
	//BrCC_SendTaskConfirm();
	//BrCC_SendWarnMsg();
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCCMD_SN].send_ok_number++;			//���ճɹ�����

}

void runBrCC_SendMsgSlow(void*)	//slow 1s
{
	BrCC_SendTaskConfirm();
	BrCC_SendWarnMsg();
	
}

void initBrCCSendTask(void)
{
	addTask(3, runBrCC_SendMsgFast, (void *)0);		//����200ms ����
	addTask(4, runBrCC_SendMsgSlow, (void *)0);		//����1s ����
}

void BrCC_SendUsvState(void)
{
	uint8 *pData;
	uint8 pFrame[BRCC_SEND_FRAME_LEN];
	uint8 CRC;
	uint16 cnt = 0;
	uint8 msgLen = 0;
	static uint16 frameNo = 0;
	int iSendlen = 0;

	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(BRCC_UDP_SEND_PORT);
	server.sin_addr.s_addr = inet_addr(CtrlCenterIP);
	memset(server.sin_zero, 0, 8);

	frameNo++;
	msgLen = 9 + sizeof(br_usv_state);
	memset(pFrame, 0, BRCC_SEND_FRAME_LEN);
	pFrame[cnt++] = '$';
	pFrame[cnt++] = 'U';
	pFrame[cnt++] = 'S';
	pFrame[cnt++] = 'V';
	pFrame[cnt++] = 'R';
	pFrame[cnt++] = usv_num;
	pFrame[cnt++] = frameNo & 0x00ff;
	pFrame[cnt++] = (frameNo & 0xff00) >> 8;
	pFrame[cnt++] = msgLen;

	memcpy((char *)&pFrame[cnt], (char *)&br_usv_state, sizeof(br_usv_state));
	cnt += sizeof(br_usv_state);

	pFrame[cnt++] = '*';
	GetCheck(&CRC, pFrame, cnt+1);
	pFrame[cnt++] = CRC;
	iSendlen = sendto(BrCC_udp_sockid, (char *)pFrame, cnt, 0, (struct sockaddr*)&server, sizeof(server));
}



void BrCC_SendTaskConfirm(void)
{
	uint8 *pData;
	uint8 pFrame[BRCC_SEND_FRAME_LEN];
	uint8 CRC;
	uint16 cnt = 0;
	uint16 msgLen = 0;
	static uint16 frameTaskNo = 0;
	static uint16 frameWarnNo = 0;
	int iSendlen = 0;

	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(BRCC_UDP_SEND_PORT);
	server.sin_addr.s_addr = inet_addr(CtrlCenterIP);
	memset(server.sin_zero, 0, 8);

	if (br_sailTaskRcv.updateOk == 0)
	{
		return;
	}
	else
	{
		br_sailTaskRcv.updateOk = 0;
		frameTaskNo++;
		msgLen = 12;
		memset(pFrame, 0, BRCC_SEND_FRAME_LEN);
		pFrame[cnt++] = '#';
		pFrame[cnt++] = 'U';
		pFrame[cnt++] = 'S';
		pFrame[cnt++] = 'V';
		pFrame[cnt++] = 'R';
		pFrame[cnt++] = usv_num;
		pFrame[cnt++] = frameTaskNo & 0x00ff;
		pFrame[cnt++] = (frameTaskNo & 0xff00) >> 8;
		pFrame[cnt++] = msgLen & 0x00ff;
		pFrame[cnt++] = (msgLen & 0xff00) >> 8;
		pFrame[cnt++] = br_sailTaskRcv.sailTask.sailMsg.u8_pointSum;
		pFrame[cnt++] = br_sailTaskRcv.sailTask.sailMsg.u8_sailNum;
		pFrame[cnt++] = '*';
		GetCheck(&CRC, pFrame, cnt + 1);
		pFrame[cnt++] = CRC;
		iSendlen = sendto(BrCC_udp_sockid, (char *)pFrame, cnt, 0, (struct sockaddr*)&server, sizeof(server));
		monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCTASK_SN].send_ok_number++;
	}
}


void BrCC_SendWarnMsg(void)
{
	uint8 *pData;
	uint8 pFrame[BRCC_SEND_FRAME_LEN];
	uint8 CRC;
	uint16 cnt = 0;
	uint16 msgLen = 0;
	uint16 warn_squence = 0;
	static uint16 frameNo = 0;
	int iSendlen = 0;


	if (isWarnMsgUdpEmpty())
	{
		//printf("warn message is empty\n");
		return;
	}
	else
	{

		getWarnMsgFromUdpQueue(&br_warn_pub);//�澯�����л�ȡ�澯ֵ
		struct sockaddr_in server;
		server.sin_family = AF_INET;
		server.sin_port = htons(BRCC_UDP_SEND_PORT);
		server.sin_addr.s_addr = inet_addr(CtrlCenterIP);
		memset(server.sin_zero, 0, 8);

		frameNo++;
		msgLen = 18;
		memset(pFrame, 0, BRCC_SEND_FRAME_LEN);
		pFrame[cnt++] = '$';
		pFrame[cnt++] = 'W';
		pFrame[cnt++] = 'N';
		pFrame[cnt++] = 'A';
		pFrame[cnt++] = 'T';
		pFrame[cnt++] = usv_num;
		pFrame[cnt++] = frameNo & 0x00ff;
		pFrame[cnt++] = (frameNo & 0xff00) >> 8;
		pFrame[cnt++] = msgLen;
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.year - 2000);
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.month);
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.day);
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.hour);
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.minute);
		pFrame[cnt++] = (uint8)(br_warn_pub.warn_time.second);
		if (1 == br_warn_pub.warn_source){ 

			warn_squence = br_warn_pub.warn_squence + IHC_WARN_SQ_BASE;
		}
		else if (0 == br_warn_pub.warn_source){
			warn_squence = br_warn_pub.warn_squence;
		}
		memcpy(&pFrame[15], &warn_squence, 2);
		cnt += 2;

		pFrame[cnt++] = br_warn_pub.warn_stat;
		pFrame[cnt++] = '*';
		GetCheck(&CRC, pFrame, cnt + 1);
		pFrame[cnt++] = CRC;
		iSendlen = sendto(BrCC_udp_sockid, (char *)pFrame, cnt, 0, (struct sockaddr*)&server, sizeof(server));
		monitor_all_inf.monitor_comm_inf[MONITOR_COMM_CCWARN_SN].send_ok_number++;
	}
}
