//control_auto.h
// #ifndef __CONTROL_AUTO__H_
// #define __CONTROL_AUTO__H_

#pragma once

#include "usv_include.h"

typedef struct{
	double   double_heading_exp;
	double   double_speed_exp;		
	double   exp_rot;				
	double   double_dst;			
	int8_t   i8_st_collision;		
	int8_t   b1_st_apf;			
	int8_t   b1_st_apf_old;		
}AUTO_NAVI_ST;

typedef struct{
	uint16_t	u16_arrival_distance1;			//	
	uint16_t	u16_arrival_distance2;			//	
	uint16_t	u16_arrival_distance3;			//	
	uint16_t	u16_arrival_distanceAvoid;		//	
	uint16_t	u16_arrival_speed;				//	0.1Kn/bit
	double      double_arrival_speedRate;		//
	uint16_t	u16_pid_speed_threshold;		//	0.1Kn/bit
	double	    double_pid_speed_threshold;		//	pid
	uint16_t	u16_roll_heading_threshold;		//	1m/bit
	double	    double_roll_heading_threshold;	//  
	float	    f32_avoid_speed;				//	
	float       speed_final;
}AUTO_NAVI_CFG;


#define SAIL_TASK_NONE	0
#define SAIL_TASK_GET	1
#define SAIL_TASK_ON	2
#define SAIL_TASK_PAUSE 3


#define FIX_TH1M							1
#define FIX_TH1M2D							2
#define FIX_TH3M							3
#define FIX_TH3M3D							3
#define FIX_TH5M							5
#define FIX_TH5M1D							5
#define FIX_TH10M							10
#define FIX_TH10M1D							9
#define	FIX_TH20M							20
#define FIX_TH20M3D							10
#define FIX_TH30M							30
#define FIX_TH30M5D							13
#define FIX_TH70M							70
#define	FIX_TH70M10D						20


#define AVOID_SPEED  0.5	

extern AUTO_NAVI_ST  autoNaviSt;
extern AUTO_NAVI_CFG autoNaviCfg;

void	autoNavigation(void);
void	autoAvoidNavigation(void);
extern void auto_operation(void);

void check_mode(void);

void calArrivalSpeedHeading(void);	
void calSpeedHeading(void);
void calRudderOpenDeg(void);


void calRotSpd(void);

void nCalRudderOpenDegOverload(float exp_rot, float exp_spd);
void nCalRudderOpenDeg(float expHeading, float expSpeed);
void nCalRudder(float expHeading);
void nCalOpenDeg(float expSpeed);
int8 autoJudgeTaskEnd(void);
void autoJudgeDstArrive(void);

void autoSampleNavigaion(void);
int8 autoModeJudge(void);
void samplingTask(void);


void calAvoidSpeedHeading(void);
void autoJudgeAvoidArrive(void);
void autoAvoidNavigation(void);

float trackingPathLos(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng);//LOS��������
void trackingPathL1(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng);//L1��������
void trackingPathL1On(double lastLat, double lastLng, double nextLat, double nextLng, double thirdLat1, double thirdLng1, double curLat, double curLng);//L1�������� ��������Զ���ȡ
int8_t trackingCircle(double center_Lat, double center_Lng, double curLat, double curLng, float radius, int8_t loiter_direction);//L1+PD����Բ

int8 arriveStandby(double lat,double lng);
int8 arriveStandby2(double lat,double lng);	

int8 standby(double lat,double lng);	

int8 sampleStandby();

float sqrt_controller(float error, float p, float second_ord_lim,float dt );

//#endif /*__CONTROL_AUTO__H_*/