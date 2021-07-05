/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_visual_guid.cpp
-	Author	   : fushuai
-	Date	   : 2019/05/28 11:34
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#include "stdafx.h"
#include "../include/docking_visual_guid.h"
#include <stdio.h>
#define REQUEST_TIMEOUT 3000	//毫秒
#define REQUEST_RETRIES 3		//重连次数
#define DOCKING_TIMES 10	//10ms

using std::cout;
using std::endl;

/**本地变量**/
char docking_zmq_cfg[30];


DockCmd dock_zmq_cmd;
static ImageInfo_t  imageInfo;			//目标图像信息
static pidCtlSigs_t   imagePidCtlSigs;	//



//初始化配置参数
void initImageControlParms()
{

	memset(&imageInfo, 0, sizeof(ImageInfo_t));
	memset(&imagePidCtlParams, 0, sizeof(pidCtlParams_t));
	memset(&imagePidCtlSigs, 0, sizeof(pidCtlSigs_t));

	imagePidRefMode.dt = 0.05;
	imagePidCtlParams.outLowerLimit = -255;
	imagePidCtlParams.outUpperLimt = 255;
	imagePidCtlParams.ctlSign = -1;

	//图像跟踪控制PID参数13~15 用最后8个值
	//docking_params_PID.cmd_r = monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].P;
	imagePidRefMode.wn = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].I;//指令跟随速度
	imagePidRefMode.yita = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].D;//PID 控制符号

	//PID
	imagePidCtlParams.proportional = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].P;	//P
	imagePidCtlParams.integral = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].I;		//I
	imagePidCtlParams.derivative = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].D;		//D

	//
	imagePidCtlParams.integralZone = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].P;//PID 积分误差区间
	imagePidCtlParams.antiWinup_Kb = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].I;//PID 饱和溢出反馈增益
	imagePidCtlParams.outZeroBias = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].D;//PID 零点偏置
}

void * cameraComm(void *aa)
{
	int ix;
	int roll = randof(1000);
	char *cmd = NULL;
	char request[128];
	char respone[128];
	intptr_t id = 0;
	void *ctx = zmq_ctx_new();

	void *client = zmq_socket(ctx, ZMQ_REQ);
#ifndef WINNT
	s_set_id(client);
#else
	s_set_id(client, id);
#endif

	zmq_connect(client, docking_zmq_cfg);
	printf("client %s start\n", id);

	int reconnect = REQUEST_RETRIES;
	int tracker_start = 0;
	int req_flag = 0;
	for (;;){
		if (req_flag != dock_zmq_cmd.dockin_cmd)
		{
			switch (dock_zmq_cmd.dockin_cmd)
			{
			case 1://开始跟踪命令
				//zmq_msg_init_size(&request,5);
				/*		cmd = ()malloc*/
				memcpy(request, "start", 6);//开始跟踪命令
				s_send(client, request);
				zmq_send(client, &request, sizeof(request), 0);
				memcpy(respone, s_recv(client), sizeof(respone));//收响应
				printf("clinet %s recv:%s\n", id, respone);
				break;
			case 2://结束跟踪命令
				memcpy(request, "stop", 5);//停止跟踪命令
				s_send(client, request);
				zmq_send(client, &request, sizeof(request), 0);
				memcpy(respone, s_recv(client), sizeof(respone));//收响应
				printf("clinet %s recv:%s\n", id, respone);
				break;
			default:
				break;
			}
			req_flag = dock_zmq_cmd.dockin_cmd;
		}
		//发送开始跟踪命令
		sleep_1(DOCKING_TIMES);
	}
	zmq_close(client);
	zmq_ctx_destroy(ctx);
}



// 返回：0 表示失败，1表示成功，-1 表示异常
int8 imageServo_yawControl(int16 *rudder_deg)
{
	/*
	imagePidRefMode.wn = &monitor_al
	imagePidRefMode.yita = &monitor_

	//PID
	imagePidCtlParams.proportional =
	imagePidCtlParams.integral = &mo
	imagePidCtlParams.derivative = &

	//
	imagePidCtlParams.integralZone =
	imagePidCtlParams.antiWinup_Kb =
	imagePidCtlParams.outZeroBias =
	*/
	//printf("wn == %f\n", *imagePidRefMode.wn);
	//printf("yt == %f\n", *imagePidRefMode.yita);
	//printf("p  == %f\n", *imagePidCtlParams.proportional);
	//printf("i  == %f\n", *imagePidCtlParams.integral);
	//printf("d  == %f\n", *imagePidCtlParams.derivative);
	//printf("z  == %f\n", *imagePidCtlParams.integralZone);
	//printf("kb == %f\n", *imagePidCtlParams.antiWinup_Kb);
	//printf("zb == %f\n", *imagePidCtlParams.outZeroBias);

	//方案1
#if 0
	int8_t returnFlag = 0;
	static uint16_t timeoutCnt = 0;                     /*!< 传感器通信超时计数器 */
	static bool  firstEnter = TRUE;
	memcpy(&imageInfo, &t_pos,sizeof(ImageInfo_t));  //获取当前目标值
	if (imageInfo.b_locking == TRUE)					//目标锁定
	{
		timeoutCnt = 0;
		returnFlag = 1;

		if (firstEnter == TRUE)
		{
			firstEnter = FALSE;
			imagePidRefMode.v[0] = imageInfo.t_x;
			imagePidRefMode.v[1] = 0;
		}
		else
		{
			imagePidCtlSigs.desired = 350;
			imagePidCtlSigs.measured = imageInfo.t_x;
			PidControlUpdate(&imagePidRefMode, &imagePidCtlParams, &imagePidCtlSigs);
			*rudder_deg = imagePidCtlSigs.aftSatCtlOut;

		}
	}
	else
	{
		// firstEnter  = TRUE
		//timeoutCnt++;
		//if (timeoutCnt >= 10)
		//{
		//	timeoutCnt = 0;
		//	returnFlag = -1;
		//}
	}
	return returnFlag;
#endif
	//方案2
	imagePidCtlSigs.desired = 350;
	imagePidCtlSigs.measured = imageInfo.t_x;
	memcpy(&imageInfo, &t_pos, sizeof(ImageInfo_t));  //获取当前跟踪目标值
	float delta_pix = imagePidCtlSigs.measured - imagePidCtlSigs.desired;
	float delta_heading = 100 * delta_pix / imagePidCtlSigs.desired;

	if ((delta_heading > 20 || delta_heading < -20) || !imageInfo.b_locking){
		*rudder_deg = ins_msg.heading * 10;
		printf("不修改delta_heading == %f\n", delta_heading);
		return 0;
	}
	else{
		*rudder_deg = delta_heading + ins_msg.heading * 10;
		printf("修改delta_heading == %f\n", delta_heading);
		return 1;
	}
}

float imageServo(float target_dis)
{
	float delt_pix_scale = 0.0;
	float iret_heading = 0.0;
	float target_dis_square = 0.0;
	float pix_coeff = image_servo_pix_multi;

	float delta_pix = imageInfo.cols / 2 - imageInfo.t_x;
	float delta_heading = image_servo_pix_multi * delta_pix / (imageInfo.cols / 2);

	//iret_heading = 57.0*delt_pix_scale * pix_coeff / pow(target_dis, 2);
	printf(" imageInfo.cols / 2  == %d\n", imageInfo.cols / 2);
	printf(" imageInfo.t_x		 == %d\n", imageInfo.t_x);
	printf(" delta_pix           == %f\n", delta_pix);
	cout << "fix_heading = " << delta_heading << endl;
	if (imageInfo.b_locking)
	{
		return delta_heading;
	}
	else
	{
		return 0;
	}

}

