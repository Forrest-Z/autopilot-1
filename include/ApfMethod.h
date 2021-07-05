#ifndef __APF_METHOD__H_
#define __APF_METHOD__H_

#include "usv_include.h"

using namespace std;


typedef struct {
    float x;
    float y;
} Vector2f_t;

typedef enum
{
	OBS_MOVING = 0,
	OBS_STATIC,
	OBS_ON_COMMING,
    RESERVER1,
	RESERVER2,
	RESERVER3,
	RESERVER4,
	STOPED = 7,
}obs_type;

//��̬����
struct MotionInputDataMsg
{
	double yaw; //heading
	double motion_direction; //groud speed direction
	double speed; //+ - m/s	//groud speed
	double yaw_rate;//deg/s	//ROT
};

//�ϰ�����Ϣ
struct StruApfObstacle
{
	double	lat;			//γ��
	double	lng;			//����
	double	radius;			//�뾶
	double	infactRadius;	//Ӱ��뾶 ��0
	double	dst;		    //�ϰ���౾������
	double	speed;			//�ٶ�
	double	heading;		//�˶�����
	uint8   type;

	StruApfObstacle()
	{
		lat = 0.0;
		lng = 0.0;
		radius = 0.0;
		infactRadius = 0.0;
		dst = 0.0;
		speed = 0.0;
		heading = 0.0;
		type = OBS_STATIC;
	}
};

//APF二维力场计算	
//基本力
struct StruApfForce{
	double magnitude;
	double direction;
	
	StruApfForce()
	{
		magnitude = 0.0;
		direction = 0.0;
	}
};

typedef struct{
	uint16 u16_apf_timeout;			//局部陷阱时间系数 单位 秒
	float  f32_apf_localMinDst;		//局部陷阱距离系数 单位 米
	float  f32_apf_coffAtt;			//引力系数
	float  f32_apf_coffRep;			//斥力系数
	float  f32_apf_coffRepImproved;	//改进的斥力系数
	float  f32_apf_coffTimescale;	//时间积分系数
	float  f32_apf_radius_extend;	//障碍物扩大半径
}APF_CFG;




extern APF_CFG apfCfg;

extern HMUTEX apf_mutex;

extern POSITION	 apf_dstCalc;	
extern double apf_heading_cd;
extern double apf_speed_cd;

extern int8	apf_valid;
extern vector <StruApfObstacle> apf_obs;

extern bool    is_fence;// 判别是否为不可达点

extern double    apf_fence_lat;
extern double    apf_fence_lng;

int8 APF_ObsInfluence(double lat,double lng,StruApfObstacle obs);
void APF_locDstInit(POSITION dst);		//初始化输入 目标点

int8 APF_obsInputZmq(void);	//障碍物输入
extern int8 APF_calc(POSITION dst_postion);		//APF计算结果 返回值 0:无障碍物影响 1:有障碍物影响

StruApfForce APF_attCalcFunc(POSITION locPos,POSITION dstPos);	//计算到目的地的引力
StruApfForce APF_repCalcFunc(POSITION locPos,StruApfObstacle obs);		//计算单个障碍物的斥力

StruApfForce APF_attForceCalc(void); //计算引力
StruApfForce APF_repForceCalc(void); //计算斥力

POSITION	APF_PositionCalc(POSITION loc,double dst,double heading);	//位置计算函数
POSITION	APF_DstCalc(StruApfForce forceResultant);					//计算目标航点

StruApfForce APF_forceResultant(StruApfForce f1,StruApfForce f2);	//力的合成

int8 APF_locMiniumDistance(const POSITION pos,const double miniRadius,const int cntrMax);	//检测是否陷入局部最小点-距离检测
POSITION APF_MiniumEscapeDst(const POSITION locPos, const POSITION dstPos);	//计算解围目标航点
void APF_locMiniumCalc(void);

//改进的斥力函数
StruApfForce APF_repCalcFunc1(POSITION locPos,POSITION dstPos,StruApfObstacle obs);	//计算单个障碍物斥力(改进)取n=1

extern void apf_test(void);

 float Loc_get_vector_length(Vector2f_t p);
 Vector2f_t Loc_normalize_vector(Vector2f_t p);
Vector2f_t Loc_reverse_vector(Vector2f_t p);
float Loc_get_vector_dot(Vector2f_t *s1,Vector2f_t *s2);
float Loc_get_vector_cross(Vector2f_t *s1,Vector2f_t *s2);


#endif	/*__APF_METHOD__H_*/