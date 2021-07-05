/*==========================================================*
 * ģ��˵��: control_assistance.cpp                         *
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
#include "../include/usv_include.h"
#include "../include/control_assistance.h"
/******************************  Local Variable  *****************************/
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

int8 calc_obs_collision( OBSTACLE_LOCATE_INF_STRUCT* pObs ,int32 *rtTcpa,int32 *rtDcpa)
{
	int		iret = 0;
	int32	safe_dcpa,dcpa,tcpa;
	*rtDcpa = 0;
	*rtTcpa = 0;
	//����Ŀ��DCPA/TCPA
	double Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double Dst;
	double Qu_Deg,Qt_Deg,Qut_Deg;
	double AUT;

	double obs_lat;
	double obs_lng;
	double loc_lat;
	double loc_lng;

	uint8 collision_type = 0; //��ײ���� 0����ײ 1��׷��	2������	3���󽻲� 4���ҽ���

	//
	obs_lat = pObs->obstacle_locate.lat/360000.0;
	obs_lng = pObs->obstacle_locate.lng/360000.0;

	loc_lat = ins_msg.latitude	;
	loc_lng = ins_msg.longitude	;

	Dst = Get_distance(loc_lat,loc_lng,obs_lat,obs_lng);
	AUT = Get_heading(loc_lat,loc_lng,obs_lat,obs_lng);

	//��������ٶ�
	Vu = ins_msg.speed*Nmile / 360000.0;			//���� m/s
	//ʹ������
	Qu_Deg = ins_msg.heading;						//���� ��

	Qu  = Radian(Qu_Deg);
	VXu = Vu*sin(Qu);		//x���ٶ�
	VYu = Vu*cos(Qu);		//y���ٶ�

	Vt = pObs->obstacle_speed;
	Qt_Deg = pObs->obstacle_direction;

	Qt = Radian(Qt_Deg);
	VXt = Vt * sin(Qt);
	VYt = Vt * cos(Qt);

	VXut = VXu - VXt;
	VYut = VYu - VXt;
	AQ = Get_AQ(VXut,VYut);
	Vut = sqrt(VXut*VXut + VYut*VYut);		//����ٶ�
	Qut= atan(VXut/VYut)*180.0/Pi + AQ;			//����ٶȷ��� ��Ҫ������

	dcpa = (int32)(fabs(Dst*sin(Radian(Qut-AUT))));
	tcpa = (int32)(Dst*cos(Radian(Qut-AUT))/Vut);

	safe_dcpa = (int32)(pObs->obstacle_radius + 30.0);	//��ȫ����+30m

	if((dcpa < safe_dcpa)&&(tcpa > 0)&&(tcpa < TCPA_MIN))
	{
		Qut_Deg = (Qu_Deg-Qt_Deg);
		if(Qut_Deg > 180)
			Qut_Deg = Qut_Deg-360;
		if(Qut_Deg<-180)
			Qut_Deg=360+Qut_Deg;

		if(Qut_Deg<=30 && Qut_Deg>= -30)	//׷��
			collision_type = 1;
		else if(Qut_Deg>=150 || Qut_Deg<=-150 )//����
			collision_type = 2;
		else
		{
			if(Qut_Deg < 0) //�󽻲�
				collision_type = 3;
			else			//�ҽ���
				collision_type = 4;
		}

		//switch(collision_type)
		//{
		//case 1: SysPubMsgPost("��ײ�澯 ׷�� DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 2: SysPubMsgPost("��ײ�澯 ���� DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 3: SysPubMsgPost("��ײ�澯 �󽻲� DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 4: SysPubMsgPost("��ײ�澯 �ҽ��� DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//default:
		//		break;
		//}
		*rtDcpa = dcpa;
		*rtTcpa = tcpa;
		iret = collision_type;
	}
	else
		iret = 0;

	return iret;

}
