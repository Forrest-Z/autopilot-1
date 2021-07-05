/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_param_tuning.cpp
-	Author	   : fushuai
-	Date	   : 2019/06/02 9:44
-   Version    : 1.0
-	Description:该部分只临时用于调试船坞进坞部分，参数设定暂时用上位机工具PID设定值部分 后期直接集成到仿真工具中 此部分为临时文件
-	Others:
*********************************************************************/
#include "stdafx.h"
#include"../include/docking_sumlink_tuning.h"

uint32 SUMLINK_PORT;
uint32 SUMLINK_RECVPORT;
UDP_INF_STRUCT	sumlink_udp_inf;
char docking_contrl_cfg[30];
SumLinkDocking sumlink_dockin_state; //控制状态值
SumlinkDockingLidar sumlink_lidardockin_state;//激光雷达进坞状态值
SumlinkCmd sumlink_cmd;
int16 sumlink_udp_sockid;
/****local function***/

static int8 recvSumlinkUSVCmd(int16 sockid);
static void sendSumlinkUSVState(void);
static void updateSumlinkUSVState(void);
static void updateSumlinkUSVStateTest(void);
static int8 dealFrameParing(int8 *buff, uint16 len);

void *tuningParsmsSumlinkTempThread(void *aa)
{
	sumlink_udp_sockid = -1;
	for (;;)
	{
		if (-1 == sumlink_udp_sockid)
		{
			sumlink_udp_sockid = CreateUDPSocket(SUMLINK_RECVPORT);	//创建UDP
			if (sumlink_udp_sockid < 0){
				sleep_1(1000);
				continue;
			}
		}
		else{
			//updateSumlinkUSVState();
			//updateSumlinkUSVStateTest();
			updateSumlinkUSVState();
			sendSumlinkUSVState();
			recvSumlinkUSVCmd(sumlink_udp_sockid);
		}
		sleep_1(100);
	}
	return ((void *)0);
}
int8 recvSumlinkUSVCmd(int16 sockid)
{
	fd_set set;
	int16 ret;
	int16 iLen;
	struct timeval timeout;
	socklen_t sockaddrlen;
	int8 net_rec_buff[1024];

	FD_ZERO(&set);
	FD_SET(sockid, &set);
	timeout.tv_sec = 0;
	timeout.tv_usec = 5;
	ret = select(sockid + 1, &set, NULL, NULL, &timeout);
	if (ret < 0)
	{
		return true;
	}
	if (FD_ISSET(sockid, &set))
	{
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff), 0, (struct sockaddr*)&sumlink_udp_inf.from_upd_ip, &sockaddrlen);
		if (iLen >= 10)
		{
			sumlink_udp_inf.udp_rec_flag = SWITCH_ON;

			if (net_rec_buff[5] == usv_num)		//是本船命令
			{
				//报文解析
				dealFrameParing(net_rec_buff, iLen);
				//printf("recv sumlink port=======================\n");
			}

		}
	}
	return true;
}

int8 dealFrameParing(int8 *buff, uint16 len)
{
	int8  iret = 0;
	uint8 frameLen = buff[8]+9;
	uint8 temp[17];
	memcpy(temp, buff, sizeof(temp));

	if ((buff[0] == '#') && (buff[1] == 'D') && (buff[2] == 'O') && (buff[3] == 'C') && (buff[4] == 'K')) //解析帧头
	{
		if (CheckCRC(1, (uint8*)buff, frameLen + 2))	//校验
		{
			memcpy((char*)&sumlink_cmd, (char*)&(buff[9]), sizeof(sumlink_cmd));
			iret = 1;
			//printf("sumlink_cmd  u8_reset	 == %d \n", sumlink_cmd.u8_reset);
			//printf("sumlink_cmd  u8_return_set	 == %d \n", sumlink_cmd.u8_return_set);
			//printf("sumlink_cmd  u8_dockin	     == %d \n", sumlink_cmd.u8_dockin);
			//printf("sumlink_cmd  u8_dockout		 == %d \n", sumlink_cmd.u8_dockout);
			//printf("sumlink_cmd	 u8_ready_in	 == %d \n", sumlink_cmd.u8_ready_in);
			//printf("sumlink_cmd  u8_already_in	 == %d \n", sumlink_cmd.u8_already_in);
			//printf("sumlink_cmd	 u8_ready_out	 == %d \n", sumlink_cmd.u8_ready_out);

		}

	}


	return iret;
}

//初始化程序
void sendSumlinkUSVState(void)
{
	uint8 *pData;
	uint8 pFrame[200];
	uint8 CRC;
	uint16 cnt = 0;
	uint8 msgLen = 0;
	static uint16 frameNo = 0;
	int iSendlen = 0;

	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_port = htons(SUMLINK_PORT);
	server.sin_addr.s_addr = inet_addr(docking_contrl_cfg);
	memset(server.sin_zero, 0, 8);

	frameNo++;
	msgLen = sizeof(sumlink_dockin_state);
	memset(pFrame, 0, 200);
	pFrame[cnt++] = '$';
	pFrame[cnt++] = 'D';
	pFrame[cnt++] = 'O';
	pFrame[cnt++] = 'C';
	pFrame[cnt++] = 'K';
	pFrame[cnt++] = usv_num;
	pFrame[cnt++] = frameNo & 0x00ff;
	pFrame[cnt++] = (frameNo & 0xff00) >> 8;
	pFrame[cnt++] = msgLen;
#ifndef LIDAR_TRACK
	memcpy((char *)&pFrame[cnt], (char *)&sumlink_dockin_state, sizeof(sumlink_dockin_state));
	cnt += sizeof(sumlink_dockin_state);
#else
	memcpy((char *)&pFrame[cnt], (char *)&sumlink_lidardockin_state, sizeof(sumlink_lidardockin_state));
	cnt += sizeof(sumlink_lidardockin_state);
#endif // !LIDAR_TRACK
	pFrame[cnt++] = '*';
	GetCheck(&CRC, pFrame, cnt + 1);
	pFrame[cnt++] = CRC;
	iSendlen = sendto(sumlink_udp_sockid, (char *)pFrame, cnt, 0, (struct sockaddr*)&server, sizeof(server));
	return;
}

void updateSumlinkUSVState(void)
{
	sumlink_dockin_state.u8_st_oil1 = jet_system.jetL.u8_Cmd_MotorOpenDeg;
	sumlink_dockin_state.u8_st_oil2 = jet_system.jetR.u8_Cmd_MotorOpenDeg;

	sumlink_dockin_state.u16_st_moter1Rpm = IHC_rev_msg.u16_St_Motor1Rpm;
	sumlink_dockin_state.u16_st_moter2Rpm = IHC_rev_msg.u16_St_Motor2Rpm;
	sumlink_dockin_state.i16_st_moter1Rudder = IHC_rev_msg.i16_St_Motor1Rudder;
	sumlink_dockin_state.i16_st_moter2Rudder = IHC_rev_msg.i16_St_Motor2Rudder;

	sumlink_dockin_state.u16_st_speed = ins_msg.u16_speed;
	sumlink_dockin_state.u16_st_heading = ins_msg.u16_heading;
	sumlink_dockin_state.i16_st_rot = ins_msg.i16_rot;

	//纬度
	sumlink_dockin_state.u8_st_latSt = ins_msg.u8_latiSt ^ 0x01;
	sumlink_dockin_state.u8_st_latDeg = ins_msg.u8_latiDeg;
	sumlink_dockin_state.u8_st_latMin = ins_msg.u8_latiMin;
	sumlink_dockin_state.u8_st_latSec = ins_msg.u8_latiSec;
	sumlink_dockin_state.u8_st_latSecDec = ins_msg.u8_latiSecDec;

	//经度
	sumlink_dockin_state.u8_st_lonSt = ins_msg.u8_longiSt ^ 0x01;
	sumlink_dockin_state.u8_st_lonDeg = ins_msg.u8_longiDeg;
	sumlink_dockin_state.u8_st_lonMin = ins_msg.u8_longiMin;
	sumlink_dockin_state.u8_st_lonSec = ins_msg.u8_longiSec;
	sumlink_dockin_state.u8_st_lonSecDec = ins_msg.u8_longiSecDec;

	sumlink_dockin_state.i16_cur_cx = t_pos.t_x;
	sumlink_dockin_state.i16_cur_cy = t_pos.t_y;
	sumlink_dockin_state.u8_cur_lock = t_pos.lock_on;
	sumlink_dockin_state.u16_ex_cx = pix_ex_out;//期望x方向像素值输出

	////控制命令返回
	//sumlink_dockin_state.u8_return_set = sumlink_cmd.u8_return_set;
	//sumlink_dockin_state.u8_dockin = sumlink_cmd.u8_dockin;
	//sumlink_dockin_state.u8_dockout = sumlink_cmd.u8_dockout;
	//sumlink_dockin_state.u8_ready_in = sumlink_cmd.u8_ready_in;
	//sumlink_dockin_state.u8_ready_out = sumlink_cmd.u8_ready_out;
	//sumlink_dockin_state.u8_already_in = sumlink_cmd.u8_already_in;
}

void updateSumlinkUSVStateTest(void)
{
	sumlink_dockin_state.u8_st_oil1 = 125;
	sumlink_dockin_state.u8_st_oil2 = 126;

	sumlink_dockin_state.u16_st_moter1Rpm = 1000 * 4;
	sumlink_dockin_state.u16_st_moter2Rpm = 1000 * 4;
	sumlink_dockin_state.i16_st_moter1Rudder = jet_system.jetL.i16_Cmd_MotorRudderDeg*2.196;
	sumlink_dockin_state.i16_st_moter2Rudder = jet_system.jetL.i16_Cmd_MotorRudderDeg*2.196;

	sumlink_dockin_state.u16_st_speed = 200;
	sumlink_dockin_state.u16_st_heading = 2700;
	sumlink_dockin_state.i16_st_rot = 100;

	//纬度
	sumlink_dockin_state.u8_st_latSt = ins_msg.u8_latiSt ^ 0x01;
	sumlink_dockin_state.u8_st_latDeg = ins_msg.u8_latiDeg;
	sumlink_dockin_state.u8_st_latMin = ins_msg.u8_latiMin;
	sumlink_dockin_state.u8_st_latSec = ins_msg.u8_latiSec;
	sumlink_dockin_state.u8_st_latSecDec = ins_msg.u8_latiSecDec;

	//经度
	sumlink_dockin_state.u8_st_lonSt = ins_msg.u8_longiSt ^ 0x01;
	sumlink_dockin_state.u8_st_lonDeg = ins_msg.u8_longiDeg;
	sumlink_dockin_state.u8_st_lonMin = ins_msg.u8_longiMin;
	sumlink_dockin_state.u8_st_lonSec = ins_msg.u8_longiSec;
	sumlink_dockin_state.u8_st_lonSecDec = ins_msg.u8_longiSecDec;

	sumlink_dockin_state.i16_cur_cx = t_pos.t_x;
	sumlink_dockin_state.i16_cur_cy = t_pos.t_y;
	sumlink_dockin_state.u8_cur_lock = t_pos.lock_on;
	sumlink_dockin_state.u16_ex_cx = pix_ex_out;//期望x方向像素值输出
}