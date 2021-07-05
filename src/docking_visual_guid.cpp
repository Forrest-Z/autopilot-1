/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
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
#define REQUEST_TIMEOUT 3000	//����
#define REQUEST_RETRIES 3		//��������
#define DOCKING_TIMES 10	//10ms

using std::cout;
using std::endl;

/**���ر���**/
char docking_zmq_cfg[30];


DockCmd dock_zmq_cmd;
static ImageInfo_t  imageInfo;			//Ŀ��ͼ����Ϣ
static pidCtlSigs_t   imagePidCtlSigs;	//



//��ʼ�����ò���
void initImageControlParms()
{

	memset(&imageInfo, 0, sizeof(ImageInfo_t));
	memset(&imagePidCtlParams, 0, sizeof(pidCtlParams_t));
	memset(&imagePidCtlSigs, 0, sizeof(pidCtlSigs_t));

	imagePidRefMode.dt = 0.05;
	imagePidCtlParams.outLowerLimit = -255;
	imagePidCtlParams.outUpperLimt = 255;
	imagePidCtlParams.ctlSign = -1;

	//ͼ����ٿ���PID����13~15 �����8��ֵ
	//docking_params_PID.cmd_r = monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].P;
	imagePidRefMode.wn = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].I;//ָ������ٶ�
	imagePidRefMode.yita = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[13].D;//PID ���Ʒ���

	//PID
	imagePidCtlParams.proportional = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].P;	//P
	imagePidCtlParams.integral = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].I;		//I
	imagePidCtlParams.derivative = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[14].D;		//D

	//
	imagePidCtlParams.integralZone = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].P;//PID �����������
	imagePidCtlParams.antiWinup_Kb = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].I;//PID ���������������
	imagePidCtlParams.outZeroBias = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[15].D;//PID ���ƫ��
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
			case 1://��ʼ��������
				//zmq_msg_init_size(&request,5);
				/*		cmd = ()malloc*/
				memcpy(request, "start", 6);//��ʼ��������
				s_send(client, request);
				zmq_send(client, &request, sizeof(request), 0);
				memcpy(respone, s_recv(client), sizeof(respone));//����Ӧ
				printf("clinet %s recv:%s\n", id, respone);
				break;
			case 2://������������
				memcpy(request, "stop", 5);//ֹͣ��������
				s_send(client, request);
				zmq_send(client, &request, sizeof(request), 0);
				memcpy(respone, s_recv(client), sizeof(respone));//����Ӧ
				printf("clinet %s recv:%s\n", id, respone);
				break;
			default:
				break;
			}
			req_flag = dock_zmq_cmd.dockin_cmd;
		}
		//���Ϳ�ʼ��������
		sleep_1(DOCKING_TIMES);
	}
	zmq_close(client);
	zmq_ctx_destroy(ctx);
}



// ���أ�0 ��ʾʧ�ܣ�1��ʾ�ɹ���-1 ��ʾ�쳣
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

	//����1
#if 0
	int8_t returnFlag = 0;
	static uint16_t timeoutCnt = 0;                     /*!< ������ͨ�ų�ʱ������ */
	static bool  firstEnter = TRUE;
	memcpy(&imageInfo, &t_pos,sizeof(ImageInfo_t));  //��ȡ��ǰĿ��ֵ
	if (imageInfo.b_locking == TRUE)					//Ŀ������
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
	//����2
	imagePidCtlSigs.desired = 350;
	imagePidCtlSigs.measured = imageInfo.t_x;
	memcpy(&imageInfo, &t_pos, sizeof(ImageInfo_t));  //��ȡ��ǰ����Ŀ��ֵ
	float delta_pix = imagePidCtlSigs.measured - imagePidCtlSigs.desired;
	float delta_heading = 100 * delta_pix / imagePidCtlSigs.desired;

	if ((delta_heading > 20 || delta_heading < -20) || !imageInfo.b_locking){
		*rudder_deg = ins_msg.heading * 10;
		printf("���޸�delta_heading == %f\n", delta_heading);
		return 0;
	}
	else{
		*rudder_deg = delta_heading + ins_msg.heading * 10;
		printf("�޸�delta_heading == %f\n", delta_heading);
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

