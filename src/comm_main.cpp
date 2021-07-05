/*==========================================================*
 * ģ��˵��: comm_main.cpp                                    *
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
#include "../include/comm_main.h"
#include "../include/usv_include.h"
#include <util/easylogging++.h>
/******************************  Local Variable  *****************************/
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/


void * comm_main_thread( void *aa )
{
	uint8 comm_task_loop=0;
	uint8 comm_task1s_loop=0;
	if(initlevel2TaskQue()<0)
	{
		return ((void *)0);
	}
	else if(initlevel3TaskQue()<0)
	{
		return ((void *)0);
	}
	else if(initlevel4TaskQue()<0)
	{
	
	}
	else
	{
		comm_main_init();
	}
	for(;;)
	{
		level2TaskSchedue();	//50ms����
		comm_task_loop++;
		comm_task1s_loop++;
		if(comm_task_loop>=4)
		{
			comm_task_loop=0;
			level3TaskSchedue();	//200ms����
		}
		if(comm_task1s_loop>=20)	//1s����
		{
			comm_task1s_loop=0;
			level4TaskSchedue();
		}
		sleep_1(50);
	}
}


void comm_main_init( void )
{
	initDRIOPsendTask();
	initBRIOPsendTask();
	initIOPsendTask();
	initIHCsendTask();
	initIDUsendTask();
	initLanUdpSendTask();
	initBrCCSendTask();
	
	initDocksendTask(); //���뽻������

	canCommCalInit();	//canͨѶ��ʱͳ��
	//DRIOPCommCalInit();	//���ֵ�̨ͨѶ��ʱͳ��
	//BRIOPCommCalInit(); //������̨ͨѶ��ʱͳ��
	insCommCalInit();	//�ߵ�ͨѶ��ʱͳ��
	obsCommCalInit();	//mpc �ϰ���ͨѶ�¹�Լ
	BrCC_CommCalInit();	//��������ͨѶ��ʱͳ��

}