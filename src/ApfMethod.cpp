/*==========================================================*
 * ģ��˵��: can_deal_IDU.cpp                                *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)						*
 * ������Ա: shaoyuping                                      *
 * ����ʱ��: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):								*
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>						*
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/ApfMethod.h"
#include "../include/nanoObstacleSender.h"



/******************************  Local Variable  *****************************/
#define APF_TIMECOUNT_20s 600	//50ms*600 = 30s
#define APF_LOCALMINI_COFF 9	//局部最优距离系数	0.6*  1.8/3.6 (m/s)/(kn/h) * 30s
//#define APF_COFF_ATT	2		//引力系数
#define APF_COFF_ATT	10000	//引力系数	modify 2019年3月20日14:14:01

#define APF_COFF_REP	1000000		//斥力系数
//#define APF_COFF_REP1	30000		//�Ľ��ĳ���ϵ��
//#define APF_COFF_REP1	100000		//�Ľ��ĳ���ϵ��
#define APF_COFF_REP1	10		//改进的斥力系数	modify 2019年3月21日10:18:49 小空间穿越

#define APF_COFF_T		0.01			//时间积分系数
//#define APF_RADIUS_EXTEND 50.0			//�ϰ�������뾶
#define APF_RADIUS_EXTEND 10.0			//障碍物扩大半径		modify 2019年3月21日10:29:20 for 窄路径穿越


APF_CFG apfCfg;


vector <StruApfObstacle> apf_obs;	//参与APF计算的障碍物列表
vector <StruApfForce>	 apf_rep;	//斥力列表
StruApfForce			 apf_att;	//引力
POSITION				 apf_loc;	//自身位置
POSITION				 apf_dst;	//目标航点
POSITION				 apf_dstCalc;	//APF计算得到的航点
double                   apf_heading_cd = ins_msg.motionDirection;
double					 apf_speed_cd = ins_msg.speed;
double                   apf_fence_lat = 0;
double                   apf_fence_lng = 0;
int8	apf_valid=0;	//人工势场算法有效标志

HMUTEX apf_mutex;


// 左/右侧障碍物个数
static uint8_t left_obs_num = 0;
static uint8_t right_obs_num = 0;
 bool    is_fence = false;

nanoObstacleSender senderTestV1;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

/**
  * @return length of vector
  */
 float Loc_get_vector_length(Vector2f_t p)
 {
	 return sqrtf(p.x*p.x + p.y*p.y);
 }


 /**
  * @normalizes this vector
  */
Vector2f_t Loc_normalize_vector(Vector2f_t p)
{
	float len = sqrtf(p.x*p.x + p.y*p.y);
	p.x = p.x/len;
	p.y = p.y/len;
    return p;
}


/**
 * @return reverise this vector 
 */
Vector2f_t Loc_reverse_vector(Vector2f_t p)
{
    p.x = -p.x;
    p.y = -p.y;
    return p;
}


 /**
  * @return the projection of vector s1 on vector s2
  */
float Loc_get_vector_dot(Vector2f_t *s1,Vector2f_t *s2)
{
 return s1->x*s2->x + s1->y*s2->y;
}

 /**
  * @return the vector productor from vector s1 to vector s2
  */
float Loc_get_vector_cross(Vector2f_t *s1,Vector2f_t *s2)
{
	return s1->x*s2->y - s1->y*s2->x;
}



void APF_locDstInit(POSITION dst)
{

	apf_loc.lat = ins_msg.latitude;
	apf_loc.lng = ins_msg.longitude;

	apf_dst.lat = dst.lat;
	apf_dst.lng = dst.lng;

	apf_valid = 0;

	left_obs_num = 0;
	right_obs_num = 0;

	apf_fence_lat = 0;
	apf_fence_lng = 0;
	
	apf_heading_cd = ins_msg.heading;
	apf_speed_cd   = ins_msg.speed;
}

int8 APF_calc(POSITION dst_postion)
{
	int8 iret = 0;
	int32 loop_i = 0;

	StruApfForce forceAtt,forceRep,forceResultant;

	if((ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A')||ins_msg.insState.u8_sysState1==3 ){
		return -1;
	}

	APF_locDstInit(dst_postion);
	
	if((iret=APF_obsInputZmq())<=0)
	{
		return iret;
	}

	forceAtt = APF_attForceCalc();
	forceRep = APF_repForceCalc();

	forceResultant = APF_forceResultant(forceAtt,forceRep);
	if(forceRep.magnitude > 0)
	{
		Vector2f_t att,rep;
		att.x = forceAtt.magnitude * cosf(Pi*forceAtt.direction/180.0);
		att.y = forceAtt.magnitude * sinf(Pi*forceAtt.direction/180.0);
		rep.x = forceRep.magnitude * cosf(Pi*forceRep.direction/180.0);
		rep.y = forceRep.magnitude * sinf(Pi*forceRep.direction/180.0);
		
		float dot = Loc_get_vector_length(rep) * Loc_get_vector_length(att);
		if(fabsf(dot) <= FLT_EPSILON)
		{
			dot = FLT_EPSILON;
		}

		float ang = Loc_get_vector_dot(&rep,&att)/dot;
		
		if(fabsf(ang + 1) <= sinf(Pi * 10/180))
		{
			if(left_obs_num > right_obs_num)
				forceResultant.direction = wrap_360_cd(ins_msg.heading + 45);
			else
				forceResultant.direction = wrap_360_cd(ins_msg.heading - 45);
		}
		apf_heading_cd = forceResultant.direction;
		apf_speed_cd   = ins_msg.speed;
	}
	else
	{
		apf_heading_cd = ins_msg.heading;
		apf_speed_cd   = 0;
	}
	
	// apf_dstCalc = APF_DstCalc(forceResultant);

	apf_valid = 1;
	
	iret = 1;
	return iret;
}


int8 APF_obsInputZmq( void )
{
	int i;
	int8 obsFluence;
	double dist = 0;
	float angle = 0;
	POSITION ps,p0;
	Vector2f_t Vobs,Vbase,Vor,Por;
	POSITION ps_dt,p0_dt;
	
	
	apf_obs.clear();

	StruApfObstacle obs_temp;

	if(obs_var.obsInsValid == -1 || obs_var.obsSenserValid== -1)
		return -1;

	usv_mutex_lock(&apf_mutex);
	p0.lat = ins_msg.latitude;
	p0.lng = ins_msg.longitude;

	
	Vbase.x = kn2ms(ins_msg.speed) * cosf(ins_msg.motionDirection *Pi/180);
	Vbase.y = kn2ms(ins_msg.speed) * sin(ins_msg.motionDirection *Pi/180);

	for(i=0;i<obs_var.obsNum;i++)
	{
	
		dist = obs_var.obsAttr[i].lat;
		angle = wrap_360_cd(obs_var.obsAttr[i].lng + ins_msg.heading);
		ps =  APF_PositionCalc(p0,dist,angle);
		obs_temp.lat			= ps.lat;
		obs_temp.lng			= ps.lng;
	
	
		float vx = obs_var.obsAttr[i].velocityMag;
		float vy = obs_var.obsAttr[i].velocityDir;

	
		Vor.x = vx * cosf(Radian(angle)) - vy * sinf(Radian(angle));
		Vor.y = vx * sinf(Radian(angle)) + vy * cosf(Radian(angle));


	    Vobs.x = Vbase.x + Vor.x;
		Vobs.y = Vbase.y + Vor.y;
		obs_temp.speed   = Loc_get_vector_length(Vobs);
		obs_temp.heading = wrap_360_cd(atan2f(Vobs.y,Vobs.x)/Pi * 180);
		
		if(obs_temp.speed <= 0.5f || dist <= 5.0)
		{
			obs_temp.speed = 0;
			Vobs.x = 0;
			Vobs.y = 0;
		}
			

		Por.x = dist * cosf(angle / 180 *Pi);
		Por.y = dist * sinf(angle / 180 *Pi);
		float v1 =  Loc_get_vector_dot(&Vbase,&Por);
		float v2 =  Loc_get_vector_dot(&Vobs,&Por);
		float dv = v1-v2;

		
		obs_temp.radius			= MAX(obs_var.obsAttr[i].radius,3.0);
		obs_temp.infactRadius   = apfCfg.f32_apf_radius_extend;

		obs_temp.type           = obs_var.obsAttr[i].type;

		obs_temp.dst = Get_distance( ins_msg.latitude,ins_msg.longitude,obs_temp.lat,obs_temp.lng);
		obsFluence   = APF_ObsInfluence(ins_msg.latitude,ins_msg.longitude,obs_temp);

		if ((obs_temp.dst > (obs_temp.infactRadius + obs_temp.radius) || obs_temp.dst <= 0.0 || std::isnan(obs_temp.dst)))
		{
			continue;
		}
		else
		{
			//printf("obs_temp.speed = %f,obs_heading = %f\n",obs_temp.speed,obs_temp.heading);

			if(obs_temp.type == STOPED)
			{
				is_fence = true;
				apf_fence_lat = obs_temp.lat;
				apf_fence_lng = obs_temp.lng;
				return 0;
			}
			else{is_fence = false;}
			
			if(obs_var.obsAttr[i].lng <= 180)
			{
				right_obs_num ++;
			}
			else
			{
				left_obs_num ++;
			}
			apf_obs.push_back(obs_temp);
			
		}
	}
	usv_mutex_unlock(&apf_mutex);

//	senderTestV1.obstacleEnqueue(apf_obs);

	return apf_obs.size();
}

StruApfForce APF_attCalcFunc( POSITION locPos,POSITION dstPos )
{
	StruApfForce iret_force;
	iret_force.direction = Get_heading(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng);
	float distance = Get_distance(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng);

	float apf_att_d0 = apfCfg.f32_apf_coffTimescale; 

	if(distance <= apf_att_d0 )
	{
		iret_force.magnitude = apfCfg.f32_apf_coffAtt * distance;
	}
	else
	{
		iret_force.magnitude = apfCfg.f32_apf_coffAtt * apf_att_d0;
	}
	return iret_force;
}


StruApfForce APF_repCalcFunc( POSITION locPos,StruApfObstacle obs )
{
	StruApfForce iret_force;
	double rho0_inv,rho_inv;
	double rou = obs.dst - obs.radius;
	double rou0 = obs.infactRadius;
	
	if (rou < 0 || rou > rou0 ){
		return iret_force;
	}

	iret_force.direction = Get_heading(obs.lat,obs.lng,locPos.lat,locPos.lng);
	rho0_inv = 1.0/rou0;
	rho_inv  = 1.0/rou;
	iret_force.magnitude = apfCfg.f32_apf_coffRep * (rho_inv - rho0_inv)*pow(rho_inv,2);
	return iret_force;
}


StruApfForce APF_repCalcFunc1( POSITION locPos,POSITION dstPos,StruApfObstacle obs )
{
	StruApfForce Frep1;		//Frep1 的方向为从障碍物指向机器人
	StruApfForce Frep2;		//Frep2 的方向为从机器人指向目标点
	StruApfForce iret_force;
	double rho0_inv,rho_inv,dst_length;

	if (obs.dst < 0.01){
		return iret_force;
	}
	rho0_inv = 1.0/obs.infactRadius;
	rho_inv  = 1.0/obs.dst;
	dst_length = Get_distance(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng);

	Frep1.direction = Get_heading(obs.lat,obs.lng,locPos.lat,locPos.lng);
	Frep1.magnitude = apfCfg.f32_apf_coffRepImproved * (rho_inv - rho0_inv)*pow(rho_inv,2) * dst_length;


	Frep2.direction = Get_heading(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng);
	Frep2.magnitude = 0.5*apfCfg.f32_apf_coffRepImproved * pow((rho_inv - rho0_inv), 2);

	iret_force = APF_forceResultant(Frep1,Frep2);

	return iret_force;	
}


StruApfForce APF_attForceCalc( void )
{
	StruApfForce attForce;
	attForce = APF_attCalcFunc(apf_loc,apf_dst);
	return attForce;
}


StruApfForce APF_repForceCalc( void )
{
	StruApfForce repForce;
	POSITION loc;
	int i;

	loc.lat = ins_msg.latitude;
	loc.lng = ins_msg.longitude;

	for(i=0;i<apf_obs.size();i++)
	{
		if(i==0)
		{
			//经典斥力
			repForce = APF_repCalcFunc(loc,apf_obs[i]);
			//改进斥力
			//repForce = APF_repCalcFunc1(apf_loc,apf_dst,apf_obs[i]);
		}
		else
		{
			repForce = APF_forceResultant(repForce,APF_repCalcFunc(loc,apf_obs[i]));
		}
	}
	return repForce;	
}

StruApfForce APF_forceResultant( StruApfForce f1,StruApfForce f2 )	
{
	unsigned char bGoEast = FALSE,bGoNorth = FALSE;
	StruApfForce resultant;
		
	double forceNorth ;
	double forceEast	;
	double dbDir;

	forceNorth    =  f1.magnitude * cos(Pi*f1.direction/180.0) + f2.magnitude * cos(Pi*f2.direction/180.0);
	forceEast     =  f1.magnitude * sin(Pi*f1.direction/180.0) + f2.magnitude * sin(Pi*f2.direction/180.0);
	
	if(forceNorth > 0.0)
		bGoNorth = TRUE;
	else
		bGoNorth = FALSE;
	if(forceEast > 0.0)
		bGoEast = TRUE;
	else
		bGoEast = FALSE;
	
	resultant.magnitude = sqrt(pow(forceNorth,2)+pow(forceEast,2));

	if(forceNorth == 0)
	{
		if(forceEast == 0) 
			dbDir = 0;
		else
			dbDir = bGoEast?90:270;
	}
	else
	{
		dbDir = atan(forceEast/forceNorth)*180/Pi;
		if(!bGoEast&&bGoNorth)
			dbDir=360+dbDir;
		if(!bGoEast&&!bGoNorth)
			dbDir=180+dbDir;
		if(bGoEast&&!bGoNorth)
			dbDir=180+dbDir;
	}
	if(dbDir>=360.0)
		dbDir = dbDir - 360.0;
	resultant.direction = dbDir;
	return resultant;
}

POSITION APF_DstCalc( StruApfForce forceResultant )
{
	POSITION iret_pos;
	double forceDst = forceResultant.magnitude * apfCfg.f32_apf_coffTimescale;
	if (forceDst > 1000)
		forceDst = 1000;
	if(forceDst <= 10)
		forceDst = 10;

	iret_pos = APF_PositionCalc(apf_loc,forceDst,forceResultant.direction);
	cout << "dst length: " << forceDst << endl;
	return iret_pos;
}

POSITION APF_PositionCalc( POSITION loc,double dst,double heading )
{
	POSITION iret_destination;
	double c=dst/Earth_Radius;
	double a=acos(cos(Pi*(90.0-loc.lat)/180.0)*cos(c)+sin(Pi*(90-loc.lat)/180.0)*sin(c)*cos(Pi*(heading)/180.0));
	double C=asin(sin(c)*sin(Pi*(heading)/180.0)/sin(a));

	iret_destination.lat = 90-a*180.0/Pi;
	iret_destination.lng = loc.lng + C*180.0/Pi;
	return iret_destination;
}



void apf_test( void )
{
	//���Ժ���
	StruApfForce a,b,c;
	a.direction = 45.0;
	a.magnitude = 1;
	b.direction = 225.0;
	b.magnitude = 2;
	c = APF_forceResultant(a,b);
	c = c;

	//���Ժ���
	StruApfForce attForce;
	POSITION loc,dst;
	loc.lat = 36.0;
	loc.lng = 114.0;
	dst.lat = 36.00;
	dst.lng = 114.001;

	//��������
	attForce = APF_attCalcFunc(loc,dst);

	//���Գ���
	StruApfForce repForce;
	POSITION repLoc;
	StruApfObstacle obs;
	repLoc.lat = 30.40650;	repLoc.lng = 114.32519;
	obs.lat = 30.40530;	obs.lng = 114.32523;
	obs.infactRadius = 30; obs.dst = 25.0;
	repForce = APF_repCalcFunc(repLoc,obs);
}

//返回值
//0-无碰撞 1-追赶 2-相遇 3-左交叉 4-右交叉
int8 APF_ObsInfluence( double lat,double lng,StruApfObstacle obs )
{
	int8 iret;
	int32 safe_dcpa,dcpa,tcpa;
	//计算目标DCPA/TCPA
	double Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double Dst;
	double Qu_Deg,Qt_Deg,Qut_Deg;
	double AUT;

	double obs_lat;
	double obs_lng;
	double loc_lat;
	double loc_lng;

	int8 collision_type = 0; //碰撞类型 0无碰撞 1：追赶	2：相遇	3：左交叉 4：右交叉

	obs_lat = obs.lat;
	obs_lng = obs.lng;

	loc_lat = lat;
	loc_lng = lng;

	Dst = Get_distance(loc_lat,loc_lng,obs_lat,obs_lng);
	AUT = Get_heading(loc_lat,loc_lng,obs_lat,obs_lng);

	//计算相对速度
	Vu = ins_msg.speed*Nmile / 360000.0;			//航速 m/s
	//使用首向
	Qu_Deg = ins_msg.heading;						//航向 °

	Qu  = Radian(Qu_Deg);
	VXu = Vu*sin(Qu);		//x轴速度
	VYu = Vu*cos(Qu);		//y轴速度

	Vt = obs.speed;
	Qt_Deg = obs.heading;

	Qt = Radian(Qt_Deg);
	VXt = Vt * sin(Qt);
	VYt = Vt * cos(Qt);

	VXut = VXu - VXt;
	VYut = VYu - VXt;
	AQ = Get_AQ(VXut,VYut);
	Vut = sqrt(VXut*VXut + VYut*VYut);		//相对速度
	Qut= atan(VXut/VYut)*180.0/Pi + AQ;			//相对速度方向 需要做补偿

	dcpa = (int32)((Dst*sin(Radian(Qut-AUT))));
	if (dcpa < 0)
		dcpa = -dcpa;
	tcpa = (int32)(Dst*cos(Radian(Qut-AUT))/Vut);

	safe_dcpa = (int32)(obs.radius + 30.0);	//安全距离+30m

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
		iret = collision_type;
	//	SysPubMsgPost("碰撞预警,预警值 会遇时间 %d s ,会遇距离 %d m",tcpa,dcpa);
	}
	else
		iret = 0;

	return iret;


}

//返回值
//1 检测位置未变化
int8 APF_locMiniumDistance(const POSITION pos,const double miniRadius,const int cntrMax)
{
	static POSITION lastPos = {0.0,0.0};	//
	static int cntr = 0;
	double distanceThisTime;

	distanceThisTime = Get_distance(lastPos.lat,lastPos.lng,pos.lat,pos.lng);
	
	lastPos = pos;

	if(distanceThisTime < miniRadius)
	{
		if(cntr < cntrMax)
			cntr++;
		else
			;
	}
	else{
		if(cntr > 0)
			cntr--;
		else
			;
	}

	if(cntr >= cntrMax)
		return 1;
	else
		return 0;

}

POSITION APF_MiniumEscapeDst( const POSITION locPos, POSITION const dstPos )
{
	POSITION iretPos;
	double dest = 0.0;	//航点距离
	double angle = 0.0;	

	dest  = Get_distance(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng);
	angle = Get_heading(locPos.lat,locPos.lng,dstPos.lat,dstPos.lng) + 90.0;	//顺时针旋转90°

	if(angle > 360.0)
		angle-=360.0;

	iretPos = APF_PositionCalc(locPos,dest,angle);
	return iretPos;

}

void APF_locMiniumCalc( void )
{
	//判断是否是局部最优点
	static int timecount = 0;
	static int8 isLocalMinimun = 0;
	timecount++;
	if(timecount >= apfCfg.u16_apf_timeout)
	{
		POSITION locPos;
		locPos.lat = ins_msg.latitude;
		locPos.lng = ins_msg.longitude;
		if((isLocalMinimun=APF_locMiniumDistance(locPos,apfCfg.f32_apf_localMinDst*ins_msg.speed,2))){	//3�׷�Χ 2������
		//	SysPubMsgPost("���˴�����ֲ����ŵ�");
		}
		timecount = 0;
	}

	//计算临时航点
	if(isLocalMinimun == 1){
		apf_dst = APF_MiniumEscapeDst(apf_loc,apf_dst);
	}
}








