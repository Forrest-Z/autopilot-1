/*==========================================================*
 * ģ��˵��: control_BD.cpp                                 *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա: shaoyuping                                     *
 * ����ʱ��: 2018/4/26 11:39:54                             *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_BD.h"
/******************************  Local Variable  *****************************/
BD_NAVI_ST BDNaviSt;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void BD_operation();
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void BD_operation( void )
{
	//�ر��������
	if(bd_sailTask.u8_cmd_sailOnOff == 0)
		idlingSpeedCtrl();	
	else if(bd_sailTask.u8_cmd_sailOnOff == 1)
		BDNavigation();

}

void BDcalSpeedHeading(void)
{
	BDNaviSt.double_dst = Get_distance(	ins_msg.latitude,							\
										ins_msg.longitude,							\
										bd_sailTask.sailMsg.wayPoint.f64_latitude,	\
										bd_sailTask.sailMsg.wayPoint.f64_longitude	\
										);
	BDNaviSt.double_speed_exp   = bd_sailTask.sailMsg.wayPoint.f64_expSpeed;
	BDNaviSt.double_heading_exp = Get_heading(ins_msg.latitude,							\
											  ins_msg.longitude,							\
											  bd_sailTask.sailMsg.wayPoint.f64_latitude,	\
											  bd_sailTask.sailMsg.wayPoint.f64_longitude	\
											  );
}

void BDJudgeDstArrive(void)
{
	if (BDNaviSt.double_dst < 70.0)	//С��70m ��Ϊ����
	{
		//todo�����߼�
		bd_sailTask.u8_St_BDSailState = 2;//ִ�����
		bd_sailTask.u8_St_sailMsgRev  = 0;//�������
	}
	return;
}

void BDRudderOpenDeg(void)
{
	double openDeg;
	double rudder;
	openDeg = simu_speed_PID(BDNaviSt.double_speed_exp,ins_msg.speed);
	rudder  = simu_heading_PID(BDNaviSt.double_heading_exp,ins_msg.heading);
	//���ſ���
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
	//���
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudder;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudder;
	//��λ
	jetGearFoward();	//ǰ��
	//
	bd_sailTask.u8_St_BDSailState = 1;	//ִ����
}


void BDNavigation( void )
{
	BDcalSpeedHeading();		//������������,�������٣�����
	BDRudderOpenDeg();
	BDJudgeDstArrive();
}
