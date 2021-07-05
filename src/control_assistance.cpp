/*==========================================================*
 * 模块说明: control_assistance.cpp                         *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员:                                                *
 * 创建时间: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
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
	//计算目标DCPA/TCPA
	double Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double Dst;
	double Qu_Deg,Qt_Deg,Qut_Deg;
	double AUT;

	double obs_lat;
	double obs_lng;
	double loc_lat;
	double loc_lng;

	uint8 collision_type = 0; //碰撞类型 0无碰撞 1：追赶	2：相遇	3：左交叉 4：右交叉

	//
	obs_lat = pObs->obstacle_locate.lat/360000.0;
	obs_lng = pObs->obstacle_locate.lng/360000.0;

	loc_lat = ins_msg.latitude	;
	loc_lng = ins_msg.longitude	;

	Dst = Get_distance(loc_lat,loc_lng,obs_lat,obs_lng);
	AUT = Get_heading(loc_lat,loc_lng,obs_lat,obs_lng);

	//计算相对速度
	Vu = ins_msg.speed*Nmile / 360000.0;			//航速 m/s
	//使用首向
	Qu_Deg = ins_msg.heading;						//航向 °

	Qu  = Radian(Qu_Deg);
	VXu = Vu*sin(Qu);		//x轴速度
	VYu = Vu*cos(Qu);		//y轴速度

	Vt = pObs->obstacle_speed;
	Qt_Deg = pObs->obstacle_direction;

	Qt = Radian(Qt_Deg);
	VXt = Vt * sin(Qt);
	VYt = Vt * cos(Qt);

	VXut = VXu - VXt;
	VYut = VYu - VXt;
	AQ = Get_AQ(VXut,VYut);
	Vut = sqrt(VXut*VXut + VYut*VYut);		//相对速度
	Qut= atan(VXut/VYut)*180.0/Pi + AQ;			//相对速度方向 需要做补偿

	dcpa = (int32)(fabs(Dst*sin(Radian(Qut-AUT))));
	tcpa = (int32)(Dst*cos(Radian(Qut-AUT))/Vut);

	safe_dcpa = (int32)(pObs->obstacle_radius + 30.0);	//安全距离+30m

	if((dcpa < safe_dcpa)&&(tcpa > 0)&&(tcpa < TCPA_MIN))
	{
		Qut_Deg = (Qu_Deg-Qt_Deg);
		if(Qut_Deg > 180)
			Qut_Deg = Qut_Deg-360;
		if(Qut_Deg<-180)
			Qut_Deg=360+Qut_Deg;

		if(Qut_Deg<=30 && Qut_Deg>= -30)	//追赶
			collision_type = 1;
		else if(Qut_Deg>=150 || Qut_Deg<=-150 )//相遇
			collision_type = 2;
		else
		{
			if(Qut_Deg < 0) //左交叉
				collision_type = 3;
			else			//右交叉
				collision_type = 4;
		}

		//switch(collision_type)
		//{
		//case 1: SysPubMsgPost("碰撞告警 追赶 DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 2: SysPubMsgPost("碰撞告警 相遇 DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 3: SysPubMsgPost("碰撞告警 左交叉 DCPA=%d\tTCPA=%d",dcpa,tcpa);
		//		break;
		//case 4: SysPubMsgPost("碰撞告警 右交叉 DCPA=%d\tTCPA=%d",dcpa,tcpa);
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
