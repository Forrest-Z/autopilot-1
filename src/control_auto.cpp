
/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_auto.h"
#ifndef WINNT
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#else
#include <sys/timeb.h>
#endif
#include "../include/adrc.h"
using namespace std;

#include <boat/boat.h>


/******************************  Local Variable  *****************************/
AUTO_NAVI_ST autoNaviSt;
AUTO_NAVI_CFG autoNaviCfg;
float last_nu = 0.0;
float target_bearing = 0.0;
float arrive_dis = 1; //初始一米
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/

static void speedHeadFocusOn(float expHeading, float expSpeed);
static void speedHeadFocusOff(float expHeading, float expSpeed);
static void trackingSpeedFuc(double lastLat, double lastLng, double curLat, double curLng, double nextLat, double nextLng, double exp_speed, double *output_speed);
//static double constrain_value(const double amt, const double low, const double high);
static void Emergency_brake_ctrl();
static float smootherSpeed(float expspeed);
static float prevent_indecision(float nu);

static float turnPath(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng);
static float turn_distance(float turn_angle, float dist_l1);
static uint8_t twistCircle(float exp_heading);
static void twistCircleOff();
// static void set_steering(float steering);
// static void set_throttle(float throttle);
void circleOn();

static double check_heading_step(double last_lat,double last_lng,double next_lat,double next_lng);
static double check_angle_step(double last_lat,double last_lng,double next_lat,double next_lng);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/


  float wrap_360(const float angle)
 {
	 float res = fmodf(angle, 360.0f);
	 if (res < 0) {
		 res += 360.0f;
	 }
	 return res;
 }


float wrap_180(const float angle)
{
     float res = wrap_360(angle);
    if (res > float(180)) {
        res -= float(360);
    }
    return res;
}


float Clamp(const float value,float bound1, float bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}


float wrap_180_cd(float angle)
{
	return wrap_180(angle, 1);
}

float wrap_180(float angle, float unit_mod)
{
	auto res = wrap_360(angle, unit_mod);
	if (res > 180.f * unit_mod) {
		res -= 360.f * unit_mod;
	}
	return res;
}

float wrap_360(float angle, float unit_mod)
{
	const float ang_360 = 360.f * unit_mod;
	float res = fmodf(static_cast<float>(angle), ang_360); //除360的余数 对于本系统本来就是0~360 没反应
	if (res < 0) {
		res += ang_360;
	}
	return res;
}

float wrap_360_cd(float angle)
{
	return wrap_360(angle,1);
}


static uint8_t fence_state = 0;
 double start_lat = 0;
 double start_lng = 0;

void auto_operation( void )
{
	if(sailTask.u8_St_sailMsgRev == 0 )	//无航行任务
	{
		command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
		//todo 怠速 
		idlingSpeedCtrl();
		return;
	}
	else
	{
		if(command_signal.sail_mode_cmd.b2_sailTask == 0)//航行任务状态—缺省
		{
			command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_GET;
			//todo 怠速 
			idlingSpeedCtrl();
			return;
		}
		else if (command_signal.sail_mode_cmd.b2_sailTask == 1)	//航行任务状态—关闭
		{
			//如果正在采样，则取消任务
			if (isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID) == FALSE 
			/*&& sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type == 1*/){
				sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
			}

			//航行任务复归
			sailTask.u8_St_sailMsgRev = 0;
			sailTask.u8_PointNum = 0;
			sailTask.sailMsg.u8_sailNum = 0;
			command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
		}
		else if(command_signal.sail_mode_cmd.b2_sailTask == 3)	//航行任务状态—暂停
		{
			command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_PAUSE;
			//todo 手动航行
			manual_operation();
			return;
		}
		else if(command_signal.sail_mode_cmd.b2_sailTask == 2)	//航行任务状态—开启
		{

			autoNaviSt.i8_st_collision = 0;

			POSITION dst_postion;
			dst_postion.lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
			dst_postion.lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;
			autoNaviSt.b1_st_apf = APF_calc(dst_postion);

			switch(fence_state)
			{
				case 0:
					if(autoNaviSt.b1_st_apf==1)	{
						autoAvoidNavigation();
					}
					else if(autoNaviSt.b1_st_apf == -1){	//障碍物传感异常
						
						idlingSpeedCtrl();
						printf("radar failure, automatic navigation termination, stop and wait for command\n");
					}
					else{
						if(AP::conf()->old_code_test_enable == true){
							autoSampleNavigaion();
						}else{		
							boat.loop();
						}
					}

					if(is_fence == true && command_signal.sail_feedBack.b2_sailTask != SAIL_TASK_NONE)
					{
						fence_state = 1;
						
						idlingSpeedCtrl();
					    printf("Got to fence mode!\n");

						autoNaviSt.double_speed_exp = 0;
						autoNaviSt.double_heading_exp = ins_msg.heading;

						if (isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID) == FALSE 
						/*&& sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type == 1*/){
							sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
							printf("Cancel sampling task because boat in fence mode!\n");
						}

					    break;
					}

				break;
				case 1:
				{
				// 减速，同时保持航向,同时保持船舶位于围栏一定距离开外
				double dis = 999;
				if(apf_fence_lat != 0 && apf_fence_lng != 0)
				{
					dis = Get_distance(ins_msg.latitude,ins_msg.longitude,apf_fence_lat,apf_fence_lng);
				}

				// 如果当前船舶距离围栏太近，那么进行适当的后退，保持5m间距
				if( dis<= 3 && fabsf(ins_msg.rotRate) <= 2)
				{
					Emergency_brake_ctrl();
					nCalRudder(autoNaviSt.double_heading_exp);
				}
				else
				{
					nCalRudderOpenDeg(autoNaviSt.double_heading_exp, 0.0); //LOS L1航向
					if(fabsf(ins_msg.speed) < 1.0f)
					{
						printf("Start searching the fence exit\n");	
						fence_state = 2;
					}
				}

					break;
				}
				case 2:
				{
						uint8_t num = 0;
					// 如果航点被围栏包围，那么什么都不做吧！
					if(sailTask.u8_PointNum  >= sailTask.sailMsg.u8_pointSum)
					{
						// 悬停
						idlingSpeedCtrl();	//怠速
						is_fence = false;
						fence_state = 0;
						command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
						printf("The navigation points are surrounded by fences. Stop and idle!\n");
						break;
					}

#if 0		
			/* 按照每次探索距离进行探索 */
				double origin_lat,origin_lng,dest_lat,dest_lng;

				if(sailTask.u8_PointNum == 0)
				{
					origin_lat = ins_msg.latitude;
					origin_lng = ins_msg.longitude;
					dest_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
					dest_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;
				}
				else
				{
					origin_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude;
					origin_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude;
					dest_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
					dest_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;
				}

				autoNaviSt.double_heading_exp =  check_heading_step(origin_lat,origin_lng,dest_lat,dest_lng);
#else
			/* 按照每次探索角度进行探索 */
			double origin_lat,origin_lng,dest_lat,dest_lng;

			if(sailTask.u8_PointNum == 0)
			{
				origin_lat = ins_msg.latitude;;
				origin_lng = ins_msg.longitude;
				dest_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
				dest_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;
			}
			else
			{
				origin_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude;
				origin_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude;
				dest_lat = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
				dest_lng = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;
			}
			autoNaviSt.double_heading_exp =  check_angle_step(origin_lat,origin_lng,dest_lat,dest_lng);
#endif
			
					fence_state = 3;
					printf("Aim at desired waypoint azimuth...\n");
					nCalRudderOpenDeg(autoNaviSt.double_heading_exp, 0); //LOS L1航向
				}
				break;
				case 3:
	
			//	if(fabsf(wrap_180_cd(autoNaviSt.double_heading_exp - ins_msg.heading)) <= 15 /*&& fabsf(ins_msg.rotRate)<=2*/)
			//	{

					fence_state = 0;
					is_fence = false;	
					printf("Try to exit the fence!\n");
			//	}
				
				nCalRudderOpenDeg(autoNaviSt.double_heading_exp, 0);
				
				break;
				default:

				break;
			}

			if(autoNaviSt.b1_st_apf_old != autoNaviSt.b1_st_apf){
				if(autoNaviSt.b1_st_apf == 1){
					printf("APF obstacle avoidance start\n");
				}
				if(autoNaviSt.b1_st_apf == 0){
					printf("APF obstacle avoidance stop\n");
				}
				if(autoNaviSt.b1_st_apf == -1){
					printf("Sense the equipment failure, stop the automatic navigation, and wait for the command\n");
				}
			}
			autoNaviSt.b1_st_apf_old = autoNaviSt.b1_st_apf;
		}
	}
}

/* 每前进一步，试探返回目标航向 */
#define CHECK_STEP_MIN   10
static double check_heading_step(double last_lat,double last_lng,double next_lat,double next_lng)
{
	double heading = 0;
	static uint16_t  i = 0;
	Vector2f_t AB,AC;

	// 计算航线航向
	float wp_heading_cd =  Get_heading(last_lat, \
					last_lng, \
					next_lat, \
					next_lng\
					);

	float wp_distance_cd = Get_distance(last_lat,last_lng,next_lat,next_lng);

	AB.x = wp_distance_cd * cosf(wp_heading_cd * Pi/180);
	AB.y = wp_distance_cd * sinf(wp_heading_cd * Pi/180);

	float AC_heading_cd  = Get_heading(last_lat,last_lng,ins_msg.latitude,ins_msg.longitude);
	float AC_distance_cd = Get_distance(last_lat,last_lng,ins_msg.latitude,ins_msg.longitude);

	AC.x = AC_distance_cd * cosf(AC_heading_cd *Pi/180);
	AC.y = AC_distance_cd * sinf(AC_heading_cd *Pi/180);


	double dist = Loc_get_vector_dot(&AB,&AC)/(wp_distance_cd);
	if(dist <= 0 || dist >= wp_distance_cd)
		dist = 0;

	// 根据步长计算替代航点
	double Lat = 0,Lng = 0;
	if(sailTask.u8_PointNum == 0){
		i++;
		Get_lat_lng(last_lat,last_lng,i*CHECK_STEP_MIN,wp_heading_cd,&Lat,&Lng);
	}
	else{
		i =0;
		Get_lat_lng(last_lat,last_lng,CHECK_STEP_MIN + dist,wp_heading_cd,&Lat,&Lng);
	}

	// 计算原地旋转期望航向
	double remain_distance = Get_distance(Lat,Lng,next_lat,next_lng);
	//printf("sailTask.u8_PointNum = %d\n",sailTask.u8_PointNum);
	if(remain_distance <= CHECK_STEP_MIN)
	{
		sailTask.u8_PointNum++;
		i=0;

		heading = Get_heading(ins_msg.latitude, \
				ins_msg.longitude, \
				next_lat, \
				next_lng\
				);
		if(sailTask.sailMsg.u8_pointSum == 1){
			heading = wrap_360_cd(heading + 180);
		}
	}
	else
	{
		heading = Get_heading(ins_msg.latitude, \
				ins_msg.longitude, \
				Lat, \
				Lng\
				);
		if(sailTask.u8_PointNum >=1){
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_latitude = Lat;
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_longitude = Lng;
		}
	}

	return heading;
}

// 按照视角进行搜索
#define CHECK_ANGLE_MIN   15 // [deg]
#define CHECK_DIST_MIN    12 // [m]
static double check_angle_step(double last_lat,double last_lng,double next_lat,double next_lng)
{

	double heading_cd = ins_msg.heading;

	// 根据输入航线计算投影点。如果当前点距离航线较近，则将投影点向前迁移一定距离。
	Vector2f_t AB,AC;

	// 计算AB
	float AB_heading_cd =  Get_heading(last_lat, last_lng,next_lat, next_lng);
	float AB_distance_cd = Get_distance(last_lat,last_lng,next_lat,next_lng);
	AB.x = AB_distance_cd * cosf(AB_heading_cd * Pi/180);
	AB.y = AB_distance_cd * sinf(AB_heading_cd * Pi/180);

	// 计算AC
	float AC_heading_cd  = Get_heading(last_lat,last_lng,ins_msg.latitude,ins_msg.longitude);
	float AC_distance_cd = Get_distance(last_lat,last_lng,ins_msg.latitude,ins_msg.longitude);
	AC.x = AC_distance_cd * cosf(AC_heading_cd *Pi/180);
	AC.y = AC_distance_cd * sinf(AC_heading_cd *Pi/180);

	
	// 计算投影长度以及投影点坐标
	double prj_dist = Loc_get_vector_dot(&AB,&AC)/(AB_distance_cd);
	if(prj_dist <= 0 || prj_dist >= AB_distance_cd)
		prj_dist = 0;

	// 计算当前点距离当前航线距离,修正投影距离
	double per_dist = Loc_get_vector_cross(&AB,&AC)/(AB_distance_cd);
	if(fabs(per_dist) <= 2)
	{
		prj_dist += CHECK_DIST_MIN;
	}
	
	// 超出航线则进行缩放
	if(prj_dist >= AB_distance_cd)
	{
		prj_dist = AB_distance_cd;
	}

	// 计算投影点坐标
	double prj_Lat = 0,prj_Lng = 0;
	Get_lat_lng(last_lat,last_lng,prj_dist,AB_heading_cd,&prj_Lat,&prj_Lng);
	
	// 判断当前船航向是否与投影点连线方向重合，如果重合则先旋转到投影连线，同时替换当前投影点为新起点
	// 否则判断AB航线视角，如果视角小于CHECK_ANGLE_MIN，则整个进行放弃，同时输出航向指定到B点(如果只有一个航点，输出反向)
	// 如果视角大于CHECK_ANGLE_MIN，则旋转CHECK_ANGLE_MIN，同时替换起点
	float PP_heading = Get_heading(ins_msg.latitude, ins_msg.longitude,prj_Lat, prj_Lng);
	float PB_heading = Get_heading(ins_msg.latitude, ins_msg.longitude,next_lat, next_lng);

	Vector2f_t PA_B,PA_P;
	float s1 = Get_distance(prj_Lat,prj_Lng,next_lat,next_lng);
	float t1 = Get_heading(prj_Lat,prj_Lng,next_lat,next_lng);
	PA_B.x = s1 * cosf(Radian(t1));PA_B.y = s1 * sinf(Radian(t1));

	float s2 = Get_distance(prj_Lat,prj_Lng,ins_msg.latitude,ins_msg.longitude);
	float t2 = Get_heading(prj_Lat,prj_Lng,ins_msg.latitude,ins_msg.longitude);
	PA_P.x = s2 * cosf(Radian(t2));PA_P.y = s2 * sinf(Radian(t2));

	if(fabsf(wrap_180_cd(PP_heading - ins_msg.heading)) <= CHECK_ANGLE_MIN)
	{
		// 切换航线,如果视角很小或者航线距离已经很小
		if(fabsf(wrap_180_cd(PP_heading - PB_heading)) <= CHECK_ANGLE_MIN || AB_distance_cd <= CHECK_DIST_MIN||fabs(per_dist) <= 2)
		{
		  
			if(sailTask.sailMsg.u8_pointSum == 1){
				heading_cd = wrap_360_cd(PB_heading + 180);
			}else{
				heading_cd = PB_heading;
			}
				sailTask.u8_PointNum ++;
		}
		else
		{
			// 计算夹角PPB
			float dot = Loc_get_vector_length(PA_B) * Loc_get_vector_length(PA_P);
			if(fabsf(dot) <= FLT_EPSILON)
			{
				dot = FLT_EPSILON;
			}
			float ang = acosf(Loc_get_vector_dot(&PA_B,&PA_P)/dot) + Radian(CHECK_ANGLE_MIN);
			float ds = s2 / sinf(ang) * sinf(Radian(CHECK_ANGLE_MIN));

			// 计算下一个起点和航向
			double Lat = 0,Lng = 0;
			Get_lat_lng(prj_Lat,prj_Lng,ds,t1,&Lat,&Lng);
			heading_cd =  Get_heading(ins_msg.latitude, ins_msg.longitude,Lat, Lng);
			if(sailTask.u8_PointNum >=1)
			{
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_latitude  = Lat;
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_longitude = Lng;
			}
		}
	}
	else
	{
		// 更新起点
		if(sailTask.u8_PointNum >=1){
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_latitude = prj_Lat;
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1].f64_longitude = prj_Lng;
		}
		heading_cd = PP_heading;
	}

	return heading_cd;
}



void check_mode(void)
{
	if(command_signal.sail_mode_cmd.b2_sailMode != SAIL_MODE_AUTO)
	{
		fence_state = 0;
	}
	else
	{
		if(command_signal.sail_mode_cmd.b2_sailTask != 2)
		{
			fence_state = 0;
		}
	}
}


// 平方根算法
float sqrt_controller(float error, float p, float second_ord_lim,float dt )
{
	 float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = sqrtf(2.0f * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -sqrtf(2.0f * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = sqrtf(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            correction_rate = -sqrtf(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
        } else {
            correction_rate = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_value(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

static const float second_ord_limt = 0.25;
static const float p = 0.1;
static bool switch_wp = true;
static const float pivot_command = 0.222;

static adrc_td_filter_t td = 
{
    .h  = 0.05f,
    .v1 = 0,
    .v2 = 0,
    .r0 = 0.25,
    .h0 = 0.05f,
};

void calSpeedHeading(void)
{
	static uint32 tlast = 0;
	double exp_speed;
	double origin_lat = 0,origin_lng = 0;
	double curr_leg_bearing_cd = 0;
	double next_leg_bearing_cd = 0;
	double desired_speed_final = 0.0f;

	uint32 tnow = millis();
	float final_speed = autoNaviCfg.speed_final;
	float wp_heading = 0;

	// 超时检测
	float dt = (tnow - tlast)*0.001f;
	if((tnow - tlast) >= 500 || tlast == 0)
	{
		// 第一次启动任务时候保证对准启动
		start_lat = ins_msg.latitude;
		start_lng = ins_msg.longitude;
		switch_wp = true;
		adrc_td_filter_reset(&td,ins_msg.speed,0);
		dt = 0.05f;
	}
	tlast = tnow;

	//计算航点距离单位 米
	autoNaviSt.double_dst = Get_distance(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);

	if(sailTask.u8_PointNum  >= sailTask.sailMsg.u8_pointSum)
	{
		// 悬停
		idlingSpeedCtrl();	//怠速
		command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
		return;
	}

	exp_speed   = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;
	if (sailTask.u8_PointNum >= 1)
	{
		 wp_heading = Get_heading(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum -1 ].f64_latitude, \
									sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_longitude, \
									sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
									sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
	    );
		
		float  yaw_err = wrap_180_cd(wp_heading - ins_msg.heading);
		float  abs_yaw_err = fabsf(yaw_err);
		float ratio =  1- constrain_value(abs_yaw_err/90,0,1);
		
		// Heading control using LOS track alogrithm
		autoNaviSt.double_heading_exp = trackingPathLos(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude,
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude,
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
		ins_msg.latitude,
		ins_msg.longitude);

		// Speed control using sqrt alogrithm
		if(switch_wp == true)
		{
			if(fabsf(yaw_err) <= 15.0f /*&& fabsf(ins_msg.rotRate)<=2*/)
			{
				switch_wp = false;
			}
			float tmep_value = pivot_command*(yaw_err > 0 ? 1:-1);
			set_steering(tmep_value);
			set_throttle(0);
		}
		else{
			float speed_temp  = constrain_value(sqrt_controller(autoNaviSt.double_dst,p,second_ord_limt,dt),final_speed,exp_speed);
			autoNaviSt.double_speed_exp = MAX(speed_temp * ratio,final_speed);
			nCalRudderOpenDeg(autoNaviSt.double_heading_exp, autoNaviSt.double_speed_exp); //LOS L1航向
		}
	}
	else
	{
		 wp_heading = Get_heading(start_lat, \
			start_lng, \
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
	       );

		float  err = wrap_180_cd(wp_heading - ins_msg.heading);
	    float ratio =  1- constrain_value(fabsf(err)/90,0,1);

		if(switch_wp == true)
		{
			if(fabsf(err) <= 15.0f /* && fabsf(ins_msg.rotRate)<= 2*/)
			{
				switch_wp = false;
				start_lat = ins_msg.latitude;
				start_lng = ins_msg.longitude;
			}

			float tmep_value = pivot_command*(err > 0?1:-1);
			set_steering(tmep_value);
			set_throttle(0);
		}
		else
		{
			float speed_temp = constrain_value(sqrt_controller(autoNaviSt.double_dst,p,second_ord_limt,dt),final_speed,exp_speed);
			autoNaviSt.double_speed_exp = MAX(speed_temp * ratio,final_speed);

			// Heading control using LOS track alogrithm
			autoNaviSt.double_heading_exp = trackingPathLos(start_lat,start_lng,
											sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,
											sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
											ins_msg.latitude,
											ins_msg.longitude);
			nCalRudderOpenDeg(autoNaviSt.double_heading_exp, autoNaviSt.double_speed_exp); //LOS L1航向
		}
		
	}

}

void nCalRudder(float expHeading)
{
	float rotExp;		//期望转向率
	float rudderExp;	//期望舵角

	if(ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A' ){
		set_throttle(0.0);
		set_steering(0.0);
		return;
	}

	rotExp = pPidHeading->pidHeadingCalc(expHeading, ins_msg.heading);
	rudderExp = pPidRot->pidRealize(rotExp, ins_msg.rotRate);

	//舵角
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudderExp;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudderExp;
}

void nCalOpenDeg(float expSpeed)
{
	double openDeg;
	
	if(ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A' ){
		set_throttle(0.0);
		set_steering(0.0);
		return;
	}

	openDeg = pPidAutoSpeed->pidIncrease(expSpeed, ins_msg.speed);
	//油门开度
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
	//档位
	//jetGearFoward();	//前进
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
}



void nCalRudderOpenDeg(float expHeading, float expSpeed)
{
	static float last_throttle = 0;
	static uint32 tlast = 0;
	uint32 tnow = millis();

	double openDeg;
	float rotExp;		//期望转向率
	float rudderExp;	//期望舵角

	if(ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A' ){
		set_throttle(0.0);
		set_steering(0.0);
		return;
	}

	// reduce desired speed if yaw_error is large
	// 45deg of error reduces speed to 75%,90deg of error reduces speed to 50%
	float relative_angle = wrap_180( expHeading - ins_msg.heading);
	double yaw_error_ratio = 1.0 - Clamp(abs(relative_angle / 180.0),0.0,1.0)*0.25;
	expSpeed *= yaw_error_ratio;

	
	openDeg = pPidAutoSpeed->pidIncrease(expSpeed, ins_msg.speed);
	rotExp = pPidHeading->pidHeadingCalc(expHeading, ins_msg.heading);
	rudderExp = pPidRot->pidRealize(rotExp, ins_msg.rotRate);


	//油门开度
	jet_system.jetL.u8_Cmd_MotorOpenDeg = fabsf(openDeg);
	jet_system.jetR.u8_Cmd_MotorOpenDeg = fabsf(openDeg);

	//舵角
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudderExp;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudderExp;

	//档位
	//jetGearFoward();	//前进
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;

}


void calRudderOpenDeg(void)
{
	double openDeg;
	double rudder;
	double speed_exp;
	float rotExp;		//期望转向率
	float rudderExp;	//期望舵角

	//rudder  = simu_heading_PID(autoNaviSt.double_heading_exp,ins_msg.heading);
	
	//修改为左右PID参数分离模式
	rudder  = simu_heading_TJ_PID(autoNaviSt.double_heading_exp,ins_msg.heading);

	//修改为多速度独立PID
	if(autoNaviSt.double_speed_exp > autoNaviCfg.double_pid_speed_threshold) 
		rudder  = simu_heading_TJ_PID(autoNaviSt.double_heading_exp,ins_msg.heading);
	else
		rudder  = simu_heading_TJ_PID_LowSpeed(autoNaviSt.double_heading_exp,ins_msg.heading);


	//openDeg = simu_speed_PID(autoNaviSt.double_speed_exp,ins_msg.speed);
	openDeg = pPidAutoSpeed->pidIncrease(autoNaviSt.double_speed_exp, ins_msg.speed);
	rotExp = pPidHeading->pidHeadingCalc(autoNaviSt.double_heading_exp, ins_msg.heading);
	rudderExp = pPidRot->pidRealize(rotExp, ins_msg.rotRate);
		
	//油门开度
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
	//舵角
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudderExp;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudderExp;
	//档位
	jetGearFoward();	//前进

}


int8 autoJudgeTaskEnd(void)
{
	int8 i8_endVaild;
	if(sailTask.u8_PointNum >= sailTask.sailMsg.u8_pointSum)	//任务执行完毕，
	{
		//idlingSpeedCtrl();	//怠速

		i8_endVaild =  arriveStandby2(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_latitude,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_longitude);

		if(i8_endVaild == -1)
		{
			//航行任务复归
			sailTask.u8_St_sailMsgRev = 0;
			//sailTask.u8_PointNum = 0;
			sailTask.sailMsg.u8_sailNum = 0;
		}
		else
		{
			//航行任务结束
			//if(sailTask.u8_St_sailMsgRev == 1)
				sailTask.u8_St_sailMsgRev = 2;

		}
		command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
		return 1;
	}
	else
	{
		command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_ON;
		return 0;
	}
}

void autoJudgeDstArrive(void)
{
	if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed>=4.5)
	{
		if (autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance1)	//航速大于5.0节情况下 4个船身转向
		{
			sailTask.u8_PointNum++;
		}
		if (autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance2)	//提前减速 怠速转弯
		{
			autoNaviSt.double_speed_exp = 2.0;							
		}
		
	}
	else
	{
		if (autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance3)	//小于21米 判定到达 3个船身到达
		{
			sailTask.u8_PointNum++;
		}
	}

	return;
}

void autoNavigation(void)
{
	if(autoJudgeTaskEnd()) return;	//判断任务时候完成
		
	calSpeedHeading();		//计算期望航向、期望航速、距离
	
	autoJudgeDstArrive();	//判断是否到达
		
	//calRudderOpenDeg();		//计算舵角及右面开度
	nCalRudderOpenDeg(autoNaviSt.double_heading_exp, autoNaviSt.double_speed_exp);

}

void calAvoidSpeedHeading(void)
{	
	//人工势场计算
	autoNaviSt.double_dst = Get_distance(		ins_msg.latitude,										\
		ins_msg.longitude,										\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,	\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude   \
		);

		// 如何很靠近目标航点，但又在避障，此时停船切换航点
	  if( autoNaviSt.double_dst <= 10)
	  {
		  autoNaviSt.double_speed_exp = 0;
	  }
	  else
	  {
		  autoNaviSt.double_speed_exp = apf_speed_cd;
	  }

	autoNaviSt.double_heading_exp  = apf_heading_cd;

	
	//autoNaviSt.double_speed_exp = autoNaviCfg.f32_avoid_speed * sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;

	
	#if 0
	autoNaviSt.double_heading_exp = Get_heading(ins_msg.latitude,												\
		ins_msg.longitude,												\
		apf_dstCalc.lat,	\
		apf_dstCalc.lng		\
		);
	#endif
}

void autoJudgeAvoidArrive(void)
{
	
	if(autoNaviSt.double_dst < 10.0 ){	//如果采样点10米范围内仍然有障碍物，则去下一个点	
		if(sailTask.u8_PointNum < sailTask.sailMsg.u8_pointSum-1){	
			//采样任务执行中则取消任务
			if(isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID)==FALSE && sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type == 1){
				sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
				sailTask.u8_PointNum++;
				printf("Avoidance: waypoint with sampling,cancel current sampling task,and go to next waypoint!\n");
			}
			else{
				sailTask.u8_PointNum++;
				printf("Avoidance: waypoint without sampling,,directly go to next waypoint!\n");
			}
		}
		else{
			printf("Avoidance: Cancel sampling task at last waypoint!\n");
			sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
			sailTask.u8_PointNum++;
		}
	}
	return;
}


void autoAvoidNavigation(void)
{
	if(autoJudgeTaskEnd()) return;	//判断任务时候完成
	calAvoidSpeedHeading();			//计算避障航向航速
	autoJudgeAvoidArrive();			//判断到达
	nCalRudderOpenDeg(autoNaviSt.double_heading_exp, autoNaviSt.double_speed_exp);
}



#if 0
float trackingPathLos(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng)
{
	double angle_exp, angle_real, angle_diff;
	double local_distance, track_error;
	float tracker_angle;
	uint32 tnow = millis();
	static float eta = 0;
	static uint32 tlast = 0;

	if(tlast == 0 || (tnow - tlast) >= 500)
	{
		eta = 0;
	}

	tlast = tnow;

	angle_exp = Get_heading(lastLat, lastLng, nextLat, nextLng);	 //计算上一航点到下一航点航向角
	angle_real = Get_heading(curLat, curLng, nextLat, nextLng);	     //计算上一航点到现有位置的航向角
	float angle_AC = Get_heading(lastLat, lastLng, curLat, curLng);	 //AC方位角
	float angle_CB = Get_heading(curLat, curLng, nextLat, nextLng);	 //CB方位角

	float dis_AB = Get_distance(lastLat, lastLng, nextLat, nextLng);	//AB距离
	float dis_AC = Get_distance(lastLat, lastLng, curLat, curLng);		//AC距离
	float  along_track_dis = dis_AC * cosf(((angle_exp - angle_AC))/180 * Pi);
	
	if(along_track_dis > (dis_AB + kn2ms(ins_msg.speed) * 3))
	{
		return (wrap_360_cd(angle_CB));
	}
	
	#if 1
	float L1_distance =  0.3183099f * 0.85f * 20 * kn2ms(ins_msg.speed);
	if( fabsf(wrap_180_cd(angle_exp - angle_AC)) >= 135 && dis_AC >= L1_distance)
	{
		return(wrap_360_cd(angle_AC + 180));
	}
	#endif

	angle_diff = angle_exp - angle_real;		//计算角度差
	local_distance = Get_distance(curLat, curLng, nextLat, nextLng);	//计算当前点到目标点距离
	track_error = local_distance * sin(Radian(angle_diff));  //包括了跟踪误差的负号
	double delta_los = n_ship*l_ship;                        // l_ship n_ship配置参数
	
	if(track_error > delta_los){track_error = delta_los;}
	else if(track_error < -delta_los){track_error = -delta_los;}
	else{}

	double los_angle = atan(track_error / delta_los);
	double beta = wrap_180_cd(ins_msg.motionDirection - ins_msg.heading);
	if(fabsf(beta) >  90.0f){
		beta = wrap_180_cd(180.0f + ins_msg.motionDirection  - ins_msg.heading);
	}
//	beta = 0;

	double tracking_path_sigma = Radian(angle_exp) - los_angle; //弧度
	tracker_angle = wrap_2PI(tracking_path_sigma);
	tracker_angle = tracker_angle * 180 / Pi;//转成角度
	track_control.track_error = track_error;
	return wrap_360_cd(tracker_angle- beta);
}
#endif

float trackingPathLos(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng)
{
	uint32 tnow = millis();
	static uint32_t _los_track_last_time = 0;
	static float _los_track_integral = 0;
	static float _los_track_i_gain_prev = 0;
	const float ki = 0;

	float dt = (tnow - _los_track_last_time)*1.0e-3f;

	if (dt > 0.1 || _los_track_last_time == 0){
		dt = 0.1;
		_los_track_integral = 0;
	}
	_los_track_last_time = tnow;

	double smax     = l_ship*5;
	double smin     = n_ship*l_ship;
	double kgain    = 80;

    double angle_last2Cur  = Get_heading(lastLat, lastLng, curLat, curLng);
	double angle_last2Next = Get_heading(lastLat, lastLng, nextLat, nextLng);
	double angle_cur2Next  = Get_heading(curLat, curLng, nextLat, nextLng);
	double angle_cur2Last  = Get_heading(curLat, curLng, lastLat, lastLng);

	double diff1 = wrap_180_cd(angle_last2Next - angle_last2Cur);
	double diff2 = wrap_180_cd(angle_cur2Next - angle_last2Next);
	double distLast2Cur = Get_distance(lastLat, lastLng, curLat, curLng);
	if ((diff1 > 135 || diff1 < -135.0) && (distLast2Cur > smin))
	{
		return angle_cur2Last;
	}

	if (diff2 > 135 || diff2 < -135) //目标点外
	{
		return angle_cur2Next;
	}

	double trackErr = distLast2Cur * sin(Radian(diff1));
	double sightLen  = smin;

	if (trackErr > sightLen )
	{
		trackErr = sightLen;
	}
	else if (trackErr < -sightLen)
	{
		trackErr = -sightLen;
	}
	if((_los_track_i_gain_prev != ki) || ki <= 0)
	{
		_los_track_integral = 0;
		_los_track_i_gain_prev = ki;
	}
	else if(fabsf(trackErr) <= 1)
	{
		_los_track_integral += trackErr * ki *dt;
	}
	else
	{
		_los_track_integral = 0;
	}

	double angleFix = (atan((trackErr + _los_track_integral)/ sightLen)) * 180 /Pi;
	double beta = wrap_180_cd(ins_msg.motionDirection - ins_msg.heading);
	if(fabsf(beta) >  90.0f){
		beta = wrap_180_cd(180.0f + ins_msg.motionDirection  - ins_msg.heading);
	}
	beta = 0;
	if (ins_msg.insState.c_rmcValid != 'A' || ins_msg.latitude == 0 || ins_msg.longitude == 0)
	{
		return angle_last2Next;
	}
	return wrap_360_cd(angle_last2Next + angleFix - beta);
}

#define DST_MIN  10.0
#define DST_MAX  100.0
#define HEADING_DT_MAX  30.0
#define HEADING_DT_MIN -30.0
int8 arriveStandby(double lat,double lng)
{
	//计算位置距离 holdPos 距离
	double holdPos_heading;
	double holdPos_speed;
	double openDeg;
	double rudder;
	double holdPos_dst = Get_distance(ins_msg.latitude,ins_msg.longitude,lat,lng);
	//计算航向航速
	if(holdPos_dst > DST_MAX)
	{
		return -1;
	}
	else if(holdPos_dst > DST_MIN)
	{
		holdPos_heading = Get_heading(ins_msg.latitude,ins_msg.longitude,lat,lng);
		holdPos_speed = 1.0;

		//计算舵角油门
		rudder = simu_heading_TJ_PID(holdPos_heading,ins_msg.heading);
		openDeg= simu_speed_PID(holdPos_speed,ins_msg.speed);
		//油门
		jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
		jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
		//舵角
		jet_system.jetL.i16_Cmd_MotorRudderDeg = rudder;
		jet_system.jetR.i16_Cmd_MotorRudderDeg = rudder;
		//档位
		jetGearFoward();
		return 1;
	}
	else
	{
		idlingSpeedCtrl();	//怠速
		return 0;
	}

}

#define MIN(A,B) (A)<(B)?(A):(B)
int8 arriveStandby2( double lat,double lng )
{
	double headingDelt; 
	double holdPos_dst     = Get_distance(ins_msg.latitude,ins_msg.longitude,lat,lng);
	double holdPos_heading = Get_heading(ins_msg.latitude,ins_msg.longitude,lat,lng);

	if(holdPos_dst > (autoNaviCfg.u16_arrival_distance3*10)) //离开距离超过十倍保持距离 判断任务已经结束，手动已经开走
	{
		idlingSpeedCtrl();	
		return -1;
	}

	 float _distance_to_destination = holdPos_dst;
	 float _desired_speed = 0;
	 float _desired_yaw_cd = holdPos_heading;
	 const float loiter_radius = autoNaviCfg.u16_arrival_distance3;
	 int8 ret = 0;
	 const float loiter_reverse_max_throttle = -0.2;

	// if within loiter radius slew desired speed towards zero and use existing desired heading
	if(_distance_to_destination <= loiter_radius){

		 // run steering and throttle controllers
		 //nCalRudderOpenDeg(_desired_yaw_cd, 0);
		 set_throttle(0.0);
		 set_steering(0.0);
		 ret = 0;
	}else{
		 // P controller with hard-coded gain to convert distance to desired speed
		  _desired_speed = MIN((_distance_to_destination - loiter_radius) * 0.5, 0.5);

			float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ins_msg.heading);
  			 // if destination is behind vehicle, reverse towards it
			if (fabsf(yaw_error_cd) > 90 ) {
				
			    _desired_yaw_cd = wrap_360_cd(_desired_yaw_cd + 180);
				 yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ins_msg.heading);

				float throttle = loiter_reverse_max_throttle;
				 // reduce desired speed if yaw_error is large
				// 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
				float yaw_error_ratio = 1.0f - constrain_value(fabsf(yaw_error_cd / 90.0f), 0.0f, 1.0f) * 0.5f;
				throttle *= yaw_error_ratio;

				set_throttle(throttle);
			}
			else
			{
				nCalOpenDeg(_desired_speed);
			}
		nCalRudder(_desired_yaw_cd);
		ret = 1;
	}
	return ret;
}



int8 standby( double lat,double lng )
{
	//计算位置距离 holdPos 距离
	double headingDelt; //航向偏差
	double holdPos_dst = Get_distance(ins_msg.latitude,ins_msg.longitude,lat,lng);
	double holdPos_heading = Get_heading(ins_msg.latitude,ins_msg.longitude,lat,lng);


	if(holdPos_dst> autoNaviCfg.u16_arrival_distance3)
	{
		//	nCalRudderOpenDeg(holdPos_heading,1/* sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_expSpeed * autoNaviCfg.double_arrival_speedRate*/); //modify @foo 2019-09-07
		headingDelt = wrap_180_cd(holdPos_heading - ins_msg.heading);

		// P controller with hard-coded gain to convert distance to desired speed       
		float _desired_speed = MIN((holdPos_dst - autoNaviCfg.u16_arrival_distance3) * 0.5, 1.0f);

		// reduce desired speed if yaw_error is large
		// 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
		float yaw_error_ratio = 1.0f - constrain_value(fabsf(headingDelt / 90.0f), 0.0f, 1.0f) * 0.5f;
		_desired_speed *= yaw_error_ratio;

		nCalRudderOpenDeg(holdPos_heading,_desired_speed);

		return 1;
	}
	else
	{
		idlingSpeedCtrl();	//怠速
		return 0;
	}



/* revise by xianglunkai*/
#if 0 
	if(holdPos_dst> autoNaviCfg.u16_arrival_distance3){
		headingDelt = holdPos_heading - ins_msg.heading;
		if(headingDelt>180.0)
			headingDelt = headingDelt - 360.0;
		else if(headingDelt<-180)
			headingDelt = headingDelt + 360.0;

		//曲折前行
		if(headingDelt > autoNaviCfg.double_roll_heading_threshold){
			idlingRollRight();
			return 1;
		}
		else if(headingDelt < (-autoNaviCfg.double_roll_heading_threshold)){
			idlingRollLeft();
			return 2;
		}
		else{
			idlingForward();
			return 3;
		}
	}
	else{
		idlingSpeedCtrl();	//怠速
		return 0;
	}
	#endif
}


int8 sampleStandby()
{
	 // get distance (in meters) to destination
	 float _desired_speed = 0;
	 float _desired_yaw_cd = 0;
	 const float loiter_radius = autoNaviCfg.u16_arrival_distance3;
	 int8 ret = 0;
	 const float loiter_reverse_max_throttle = -0.35;

	 // calculate bearing to destination
	 _desired_yaw_cd = Get_heading(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);

	 float _distance_to_destination = Get_distance(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude);

     // if within loiter radius slew desired speed towards zero and use existing desired heading
	if(_distance_to_destination <= loiter_radius){
		
		// run steering and throttle controllers
		// nCalRudderOpenDeg(_desired_yaw_cd, 0);
		set_steering(0.0);
		set_throttle(0.0);
		  ret = 0;
	}else{
		 // P controller with hard-coded gain to convert distance to desired speed
		  _desired_speed = MIN((_distance_to_destination - loiter_radius) * 0.5, 0.5);

			float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ins_msg.heading);
  			 // if destination is behind vehicle, reverse towards it
			if (fabsf(yaw_error_cd) > 90 ) {
				
			    _desired_yaw_cd = wrap_360_cd(_desired_yaw_cd + 180);
				float err = wrap_180_cd(_desired_yaw_cd - ins_msg.heading);

				float throttle = loiter_reverse_max_throttle;
				 // reduce desired speed if yaw_error is large
				// 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
				float yaw_error_ratio = 1.0f - constrain_value(fabsf(err / 90.0f), 0.0f, 1.0f) * 0.5f;
				throttle *= yaw_error_ratio;

				set_throttle(throttle);
			}
			else
			{
				nCalOpenDeg(_desired_speed);
			}
		nCalRudder(_desired_yaw_cd);
		ret = 1;
	}

	return ret;
}

//用于判断无人船是否达到航点及附近	返回值 0未到达	1到达附近	2到达航点
int8 autoModeJudge( void )
{
	int8 iret = 0;

	#if 0
	if(autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance2)
	{
		iret = 1;
	}
	#endif
//	printf("autoNaviSt.double_dst  = %f,autoNaviCfg.u16_arrival_distance1 = %f\n",autoNaviSt.double_dst,autoNaviCfg.u16_arrival_distance1);
	if((autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance1) || (sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival == 1))
	{
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival = 1;
		iret = 2;
	}

	return iret;
}

void autoSampleNavigaion( void )
{
	if (autoJudgeTaskEnd()) return;	//航行任务结束

		autoNaviSt.double_dst = Get_distance(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);
	
	switch (autoModeJudge())
	{
	case 0:
		calSpeedHeading();			//LOS跟踪航迹设定
		break;
	case 1:
		calArrivalSpeedHeading();	//到达航速航行
		nCalRudderOpenDeg(autoNaviSt.double_heading_exp, autoNaviSt.double_speed_exp);//LOS L1航向
		break;
	case 2:
		//到达采样处理
		samplingTask();
	default:
		break;
	}
}


void calArrivalSpeedHeading( void )
{
	//计算航点距离单位 米
	autoNaviSt.double_dst = Get_distance(		ins_msg.latitude,		\
		ins_msg.longitude,												\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,	\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);
	autoNaviSt.double_speed_exp =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed * autoNaviCfg.double_arrival_speedRate;	//低速
	autoNaviSt.double_heading_exp = Get_heading(ins_msg.latitude,												\
		ins_msg.longitude,												\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,	\
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);
}

void samplingTask( void )
{
	if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type != 0)
	{
		//航点保持
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u16_sampleVolume > 0){
			sampleStandby();
		}
		
		//发送航点任务
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 0){
			//@20201211
			sampleStart2(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u16_sampleVolume);
			//sampleStart(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u16_sampleVolume);
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand = 2;
		}
		
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 2){
			if(isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID)){
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete = 1;
			}

		}
		//接收采样结束	//采样结束标识在航点任务中体现
		//判断是否采样结束
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 2 &&sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete == 1)
		{
			printf("Arrival of waypoint[%d],waypoint with sampling, follow-up task",sailTask.u8_PointNum+1);
			sailTask.u8_PointNum++;
			printf("waypoint with sampling ++\n");
		}

	}
	else	//执行下一个航点
	{
		printf("Arrival of waypoint[%d],waypoint without sampling, follow-up task",sailTask.u8_PointNum+1);
		sailTask.u8_PointNum++;
		switch_wp = true;
		printf("waypoint without sampling++\n");
	}
}

//DSP控制指令
uint8_t twistCircle(float exp_heading)
{
	uint8_t focus_on = 0;;
	if ((ins_msg.heading - exp_heading < 15) && (ins_msg.heading - exp_heading > -15)){
		focus_on = 1;
		docking_control_cmd.cmd_state = 0;
	}
	else{

		docking_control_cmd.cmd_state = 1;
		docking_control_cmd.vx = 0; //发个IHC期望航速命令
		docking_control_cmd.heading = exp_heading * 10; //期望航向
		focus_on = 0;
	}
	return focus_on;
}

void twistCircleOff()
{
	docking_control_cmd.cmd_state = 0;
}
void speedHeadFocusOn(float expHeading, float expSpeed)
{
	docking_control_cmd.cmd_state = 1;
	docking_control_cmd.vx = expSpeed * 10; //发IHC期望航速命令 后退
	docking_control_cmd.heading = expHeading * 10; //期望航向
}

void speedHeadFocusOff(float expHeading, float expSpeed)
{
	docking_control_cmd.cmd_state = 0;
	docking_control_cmd.vx = 0;      //发IHC期望航速命令 后退
	docking_control_cmd.heading = 0; //期望航向
}

float smootherSpeed(float expspeed) //速度平滑
{
	static float old_speed;
	float speed_err;
	uint64 elapsed_ms;
	static uint64 lasttime_ms;
	float dt_s;
#ifndef WINNT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	elapsed_ms = tv.tv_usec / 1000 + tv.tv_sec * 1000;
#else
	struct timeb tv;
	ftime(&tv);
	elapsed_ms = tv.millitm + tv.time * 1000;
#endif
	dt_s = (elapsed_ms - lasttime_ms) * 1.0e-6f;
	if (dt_s > 100)
	{
		dt_s = 0.1;
		old_speed = 0.0f;//
	}
	lasttime_ms = elapsed_ms;
	speed_err = expspeed - ins_msg.speed;
	printf("speed dt_s == %f\n", dt_s);
	if (speed_err >= 0.5 && dt_s >= 1){
		old_speed += ins_msg.speed + 0.5;
	}
	else
	{
		old_speed = ins_msg.speed;
	}
	constrain_value(old_speed, old_speed - 0.5, expspeed);
	return old_speed;
}

//速度曲线
//T = (dis*a+v^2) / (a*v)
//ta = v/a <= T/2 a>0
//指定了 a和目标位置 那么通过将输出速度离散化
//输出速度单位kn/s
//计算过程中用m单位
void trackingSpeedFuc(double lastLat, double lastLng, double curLat, double curLng, double nextLat, double nextLng, double exp_speed, double *output_speed)
{
	static uint64 lasttime_ms;
	uint64 elapsed_ms;
	uint64 dt;
#ifndef WINNT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	elapsed_ms = tv.tv_usec / 1000 + tv.tv_sec * 1000;
#else
	struct timeb tv;
	ftime(&tv);
	elapsed_ms = tv.millitm + tv.time * 1000;
#endif

	double target_dis = Get_distance(curLat, curLng, nextLat, nextLng);	//计算目标点航行距离
	double elapsed_dis = Get_distance(lastLat, lastLng, curLat, curLng);
	double acc_value = 0.1; //0.2m/s^2
	double S = Get_distance(lastLat, lastLng, nextLat, nextLng);
	float Ts = 0.05;//20HZ
	float s_acc = pow(exp_speed / 2.0, 2) / (2.0*acc_value);//最小加速距离;
	float s_dec = pow(ins_msg.speed / 2.0, 2) / (2.0*acc_value);//最小减速距离
	float T = (S*acc_value + exp_speed*exp_speed / 4.0) / (acc_value*exp_speed / 2.0); //到达耗费时间
	float T_a = (exp_speed / 2.0) / acc_value; //根据给定加速度达到期望速度所需时间
	float T_d = (ins_msg.speed / 2.0) / acc_value;//根据根据现有速度减速到所需时间
	float tick = 0.0;
	int tick_count = 0;
	static int time = 0; //触发计数
	tick_count = (int)(T / Ts);//离散计数

	//if (time == 0){
	//	lasttime_ms = elapsed_ms;
	//}
	dt = elapsed_ms - lasttime_ms; //调用时间差
	lasttime_ms = elapsed_ms;
	if (0 < (dt - 100)){
		time++;
	}
	if (dt > 500){ //调用时间差
		time = 0;
	}
	if (s_acc > S && target_dis < s_dec){ //没法达到期望速度
		*output_speed = sqrt(acc_value*S) > 0.5 ? sqrt(acc_value*S) : 0.5;
		printf("be quit 2Ta == %f \t T == %f\n", 2 * T_a, T);
		return;
	}
	if (target_dis <= 1){ //已经到达目标
		time = 0;
		return;
	}
	//else{
	tick = time * Ts;
	if (elapsed_dis < s_acc && target_dis > s_acc && ins_msg.speed <= exp_speed){
		*output_speed = (ins_msg.speed + tick*acc_value) < exp_speed ? (ins_msg.speed + tick*acc_value) : exp_speed;
		printf("++++++Acc Phase == %f\n", *output_speed);
	}
	else if (target_dis <= s_acc && ins_msg.speed >= 0){
		*output_speed = (ins_msg.speed - tick*acc_value) >= 0.5 ? (ins_msg.speed - tick*acc_value) : 0.5;
		printf("------Dec Phase == %f\n", *output_speed);
	}
	else if (target_dis > s_acc && elapsed_dis > s_acc){
		time = 0;
		*output_speed = exp_speed;
		printf("*******Uni Phase == %f\n", *output_speed);
	}
	else{
		time = 0;
		*output_speed = 0.5;
		printf("2Ta == %f \t T == %f\n", 2 * T_a, T);
	}
	/*if (time <= tick_count){
	tick = time * Ts;
	if(tick <= T_a && tick > 0){
	*output_speed = tick*acc_value;
	printf("++++++Acc Phase == %f\n",*output_speed);
	}
	if (tick > T_a && tick  <= T-T_a){
	*output_speed = exp_speed;
	printf("*******Uni Phase == %f\n", *output_speed);
	}
	if (tick > T - T_a && tick <= T){
	*output_speed = exp_speed - (T_a + tick - T)*acc_value;
	printf("------Dec Phase == %f\n", *output_speed);
	}
	}*/
	//}
	printf("sample times == %d\n", time);
}

void nCalRudderOpenDegOverload(float exp_rot, float exp_spd)
{
	double openDeg;
	float rudderExp;	//期望舵角

	//openDeg = pPidAutoSpeed->pidIncrease(exp_spd, ins_msg.speed);
	openDeg = pPidAutoSpeed->pidRealize(exp_spd, ins_msg.speed); //绝对式PID
	rudderExp = pPidRot->pidRealize(exp_rot, ins_msg.rotRate);

	//printf("expHeading = %f, rotExp = %f \n", expHeading, rotExp);

	//油门开度
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;

	//舵角
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudderExp;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudderExp;
	//档位
	jetGearFoward();	//前进
	printf("L == %d \t R == %d\n", jet_system.jetL.i16_Cmd_MotorRudderDeg, jet_system.jetR.i16_Cmd_MotorRudderDeg);
}


float prevent_indecision(float nu)
{
	const float Nu_limit = 0.9f * Pi;
	if (fabsf(nu) > Nu_limit &&
		fabsf(last_nu) > Nu_limit &&
		labs(wrap_180_cd(target_bearing - ins_msg.heading)) > 120 &&
		nu * last_nu < 0.0f)
	{
		nu = last_nu;
	}
	return nu;
}



//eta1为USV和参考点连线与AB连线的夹角
//eta2为艏向与AB连线的夹角
//angle_eta = eta1 + eta2 近似认为
//L1_dist = 1/pi * dmping*period*speed
//AccDem = 4*damping^2 * speed^2*sin(eta)/L1_dist
//radius = L1_dist / (2 * sin(Radian(angle_eta)))
//w_cmd = sqrt(AccDem/radius);
//w_cmd = Acc/V;
void trackingPathL1(double lastLat, double lastLng, double nextLat, double nextLng, double curLat, double curLng)
{
	double a_cmd;//向心加速度
	double angle_eta, eta1, eta2, nu, nu1, nu2;//η角 单位度
	double radius_loiter;//旋转半径
	double angle_AB, angle_AC, angle_CB, turn_angle, nav_bearing;
	double dis_AB, dis_AC, dis_CB, cross_track_error, along_track_dis;
	double tracker_angle, l1_angle;
	double dist_L1, K_L1; // L1
	double w_cmd;
	float xtrackVel;
	float ltrackVel;
	float damping, period;

	float _L1_xtrack_i = 0.0f; //I
	float _L1_xtrack_i_gain = 0.0f; //P
	static float _L1_xtrack_i_gain_prev = 0;
	int8_t turn_direction = 1;
	uint64 elapsed_ms;
	static uint64 lasttime_ms;
	float dt_s;
#ifndef WINNT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	elapsed_ms = tv.tv_usec / 1000 + tv.tv_sec * 1000;
#else
	struct timeb tv;
	ftime(&tv);
	elapsed_ms = tv.millitm + tv.time * 1000;
#endif
	double usv_speed = ins_msg.speed * 0.5144f; //0.5144
	if (usv_speed < 0.1f){
		usv_speed = 0.1f;
	}
	dt_s = (elapsed_ms - lasttime_ms) * 1.0e-6f;
	if (dt_s > 0.1)
	{
		dt_s = 0.1;
		_L1_xtrack_i = 0.0f;//积分清零
	}
	lasttime_ms = elapsed_ms;
	//printf("dt_s == %d\n", dt_s);
	damping = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].P); //阻尼
	period = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].D);	//周期
	_L1_xtrack_i_gain = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].I);//累积误差

	K_L1 = 4.0*damping*damping;
	dist_L1 = 0.3183099*damping*period*usv_speed; //0.3183099≈1/pi

	//当前点为C 上一个点为A 下一个点为B
	angle_AB = Get_heading(lastLat, lastLng, nextLat, nextLng);	//AB方位角
	angle_AC = Get_heading(lastLat, lastLng, curLat, curLng);	//AC方位角
	angle_CB = Get_heading(curLat, curLng, nextLat, nextLng);	//CB方位角

	dis_AB = Get_distance(lastLat, lastLng, nextLat, nextLng);	//AB距离
	dis_AC = Get_distance(lastLat, lastLng, curLat, curLng);	//CA距离
	dis_CB = Get_distance(curLat, curLng, nextLat, nextLng);	//CB距离

	along_track_dis = dis_CB*abs(cos(Radian(angle_CB - angle_AB)));//CB在AB上的投影距离
	cross_track_error = dis_AC*sin(Radian(angle_AB - angle_AC));//路径的横向偏差
	uint8_t turn_off = 1;
	//printf("cross_track_error == %f\n", cross_track_error);
	//autoNaviCfg.u16_arrival_distance1 = 0.7071*dist_L1;
	autoNaviSt.double_speed_exp = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;
	if (dis_AB < 1.0f) //单独目标点
	{
		nu = 0;
		nav_bearing = wrap_180_cd(angle_AC) + 180;
		autoNaviSt.double_heading_exp = nav_bearing;
		return;

	}
	//else if (abs(turn_angle) >= 45){  //大于等于45度转弯
	//	turn_direction = copysign(turn_direction,turn_angle);//置位转弯方向
	//	autoNaviCfg.u16_arrival_distance1 = dist_L1 / (2 * fabs(cos(turn_angle / 2.0)));
	//}
	else if (dis_AC > dist_L1 && along_track_dis > dis_AB){ //CB在AB上的投影距离大于AB的距离则以A点为跟踪目标跟踪
		double delta_radial = ins_msg.heading - angle_AC;//径向角差向外为负 向内为正
		double delta_tagential = ins_msg.heading - ((angle_AC + 90) >= 360 ? (angle_AC - 270) : (angle_AC + 90));//与顺时针切向角差
		delta_radial = wrap_180_cd(delta_radial); //±180
		delta_tagential = wrap_180_cd(delta_tagential);//±180
		xtrackVel = usv_speed*cos(Radian(delta_tagential));//切向速度
		ltrackVel = -usv_speed*cos(Radian(delta_radial));//径向速度 指向A
		nu = atan2f(xtrackVel, ltrackVel);
		nav_bearing = angle_AC + 180;
	}
	else{ //正常L1跟踪
		//turn_angle = angle_AB - ins_msg.heading; //需要转弯角度
		//turn_angle = wrap_180_cd(turn_angle);
		//if (fabs(turn_angle) > 45){
		//	turn_off = twistCircle(angle_AB);
		//	autoNaviSt.double_speed_exp = 0;
		//	autoNaviSt.double_heading_exp = angle_AB;
		//	return;
		//}
		//else
		//{
		//	twistCircleOff();
		//	turn_off = 0;
		//}

		double delta_l = ins_msg.heading - angle_AB; //平行AB径向角
		double delta_x = ins_msg.heading - ((angle_AB + 90) >= 360 ? (angle_AB - 270) : (angle_AB + 90)); //横向偏角
		delta_l = wrap_180_cd(delta_l); //±180
		delta_x = wrap_180_cd(delta_x);//±180
		xtrackVel = usv_speed*cos(Radian(delta_x)); //AB垂直方向速度
		ltrackVel = usv_speed*cos(Radian(delta_l)); //AB投影方向速度 指向B
		nu2 = atan2f(xtrackVel, ltrackVel);
		float sin_nu1 = cross_track_error / (dist_L1 > 0.1 ? dist_L1 : 0.1);
		nu1 = asinf(constrain_value(sin_nu1, -0.7071f, 0.7071f));

		//计算积分项，减少稳态误差
		if (_L1_xtrack_i_gain <= 0 || _L1_xtrack_i_gain != _L1_xtrack_i_gain_prev)
		{
			_L1_xtrack_i = 0;
			_L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
		}
		else if (fabsf(nu1) < Radian(5))
		{
			_L1_xtrack_i += nu1 * _L1_xtrack_i_gain * dt_s;
			// an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
			_L1_xtrack_i = constrain_value(_L1_xtrack_i, -0.1f, 0.1f); //1度是0.0174
		}
		nu1 += _L1_xtrack_i;
		//printf("_L1_xtrack_i == %f\n", _L1_xtrack_i);
		nu = nu1 + nu2;
		nav_bearing = angle_AB + 180 / Pi*nu1;
		//radius_loiter = abs(dist_L1 / (2 * sin(Radian(nu)))); //跟踪旋转半径
	}
	prevent_indecision(nu);
	last_nu = nu;
	nu = constrain_value(nu, -1.5708f, +1.5708f);
	//	copysign(angle_exp, angle_AB);

	a_cmd = K_L1*pow(usv_speed, 2) / dist_L1*sin(nu);
	//printf("angle_eta == %f \t  a_cmd == %f\n", nu, a_cmd);
	autoNaviSt.exp_rot = (180 / Pi) * (a_cmd / (xtrackVel > 0.1 ? xtrackVel : 0.1));
	autoNaviSt.double_heading_exp = nav_bearing;
	//return w_cmd;
}

float turn_distance(float turn_angle, float dist_l1)
{
	float radius;
	float usv_speed = ins_msg.speed * 0.5144f;
	//90度转弯半径为参考基准
	radius = dist_l1 <= (usv_speed*usv_speed) ? dist_l1 : (usv_speed*usv_speed);
	turn_angle = fabsf(turn_angle);
	if (turn_angle >= 90){
		return radius;
	}
	return radius * turn_angle / 90.0f;
}

void calRotSpd(void)
{
	float angleFix = 0.0;
	double exp_speed;
	static double start_lat, start_lng;
	static uint8_t starton_sign = 0;

	uint64 elapsed_ms;
	static uint64 lasttime_ms;
	float dt_s;
#ifndef WINNT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	elapsed_ms = tv.tv_usec / 1000 + tv.tv_sec * 1000;
#else
	struct timeb tv;
	ftime(&tv);
	elapsed_ms = tv.millitm + tv.time * 1000;
#endif
	double usv_speed = ins_msg.speed * 0.5144f; //0.5144
	if (usv_speed < 0.1f){
		usv_speed = 0.1f;
	}
	dt_s = (elapsed_ms - lasttime_ms) * 1.0e-6f;
	if (dt_s > 0.1)
	{
		dt_s = 0.1;
		starton_sign = 0;//状态复位 变量生存周期为0.1
	}
	lasttime_ms = elapsed_ms;
#ifdef L1_CONTROL_ON
	//计算航点距离单位 米
	autoNaviSt.double_dst = Get_distance(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);
	autoNaviSt.double_speed_exp = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;//

	if (sailTask.sailMsg.u8_pointSum <= 1){ //只有一个航点任务
		if (0 == starton_sign)
		{
			starton_sign = 1;
			start_lat = ins_msg.latitude;
			start_lng = ins_msg.longitude;
		}
		trackingPathL1On(start_lat, start_lng,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
			ins_msg.latitude, ins_msg.longitude);

	}
	else{
		if (sailTask.u8_PointNum < 1) //第一个航点
		{
			if (0 == starton_sign)
			{
				starton_sign = 1;
				start_lat = ins_msg.latitude;
				start_lng = ins_msg.longitude;
			}
			trackingPathL1On(start_lat, start_lng,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum + 1].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum + 1].f64_longitude,
				ins_msg.latitude, ins_msg.longitude);
		}
		else if (sailTask.u8_PointNum >= 1 && sailTask.u8_PointNum < sailTask.sailMsg.u8_pointSum - 1){ //中间点任务
			starton_sign = 0;
			trackingPathL1On(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum + 1].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum + 1].f64_longitude,
				ins_msg.latitude, ins_msg.longitude);
		}
		else{ //最后的航点
			starton_sign = 0;
			trackingPathL1On(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
				ins_msg.latitude, ins_msg.longitude);
		}
	}
#else
	//计算航点距离单位 米
	autoNaviSt.double_dst = Get_distance(ins_msg.latitude, \
		ins_msg.longitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude, \
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude	\
		);
	autoNaviSt.double_speed_exp = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;//
	//autoNaviSt.double_speed_exp = smootherSpeed(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed);  //速度平滑
	//航迹矫正2
	if (sailTask.u8_PointNum < 1) //第一个点
	{

		trackingPathL1(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
			ins_msg.latitude,
			ins_msg.longitude);
	}
	else{ //多个航点
		trackingPathL1(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_latitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum - 1].f64_longitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude,
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude,
			ins_msg.latitude,
			ins_msg.longitude);
	}
#endif
	

}

void trackingPathL1On(double lastLat, double lastLng, double nextLat, double nextLng, double thirdLat1, double thirdLng1, double curLat, double curLng)
{
	double a_cmd;//向心加速度
	double angle_eta, eta1, eta2, nu, nu1, nu2;//η角 单位度
	double radius_loiter;//旋转半径
	double angle_AB, angle_BD, angle_AC, angle_CB, turn_angle, nav_bearing;
	double dis_AB, dis_AC, dis_CB, dis_BD, cross_track_error, along_track_dis;
	double tracker_angle, l1_angle;
	double dist_L1, K_L1; // L1
	double w_cmd;
	float xtrackVel;
	float ltrackVel;
	float damping, period;

	float _L1_xtrack_i = 0.0f; //I
	float _L1_xtrack_i_gain = 0.0f; //P
	static float _L1_xtrack_i_gain_prev = 0;
	int8_t turn_direction = 1;
	uint64 elapsed_ms;
	static uint64 lasttime_ms;
	float dt_s;
#ifndef WINNT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	elapsed_ms = tv.tv_usec / 1000 + tv.tv_sec * 1000;
#else
	struct timeb tv;
	ftime(&tv);
	elapsed_ms = tv.millitm + tv.time * 1000;
#endif
	double usv_speed = ins_msg.speed * 0.5144f; //0.5144
	if (usv_speed < 0.1f){
		usv_speed = 0.1f;
	}
	dt_s = (elapsed_ms - lasttime_ms) * 1.0e-6f;
	if (dt_s > 0.1)
	{
		dt_s = 0.1;
		_L1_xtrack_i = 0.0f;//积分清零
	}
	lasttime_ms = elapsed_ms;

	damping = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].P); //阻尼
	period = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].D);	//周期
	_L1_xtrack_i_gain = *(&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[9].I);//累积误差

	K_L1 = 4.0*damping*damping;
	dist_L1 = 0.3183099*damping*period*usv_speed; //0.3183099≈1/pi

	//当前点为C 上一个点为A 下一个点为B 下下一个点为D
	angle_AB = Get_heading(lastLat, lastLng, nextLat, nextLng);		//AB方位角
	angle_AC = Get_heading(lastLat, lastLng, curLat, curLng);		//AC方位角
	angle_CB = Get_heading(curLat, curLng, nextLat, nextLng);		//CB方位角
	angle_BD = Get_heading(nextLat, nextLng, thirdLat1, thirdLng1);	//BD方位角
	dis_AB = Get_distance(lastLat, lastLng, nextLat, nextLng);		//AB距离
	dis_AC = Get_distance(lastLat, lastLng, curLat, curLng);		//CA距离
	dis_CB = Get_distance(curLat, curLng, nextLat, nextLng);		//CB距离
	dis_BD = Get_distance(nextLat, nextLng, thirdLat1, thirdLng1);  //BD距离

	along_track_dis = dis_CB*abs(cos(Radian(angle_CB - angle_AB)));//CB在AB上的投影距离
	cross_track_error = dis_AC*sin(Radian(angle_AB - angle_AC));//路径的横向偏差
	turn_angle = angle_AB - angle_BD; //需要转弯角度
	turn_angle = wrap_180_cd(turn_angle);
	printf("cross_track_error == %f\n", cross_track_error);

	if (fabsf(turn_angle) >= 0.1) //有转弯
	{
		autoNaviCfg.u16_arrival_distance1 = turn_distance(turn_angle, dist_L1);
	}
	else{//无转弯
		autoNaviCfg.u16_arrival_distance1 = usv_speed*usv_speed;
	}
	autoNaviCfg.u16_arrival_distance1 = 0.7071*dist_L1;
	if (dis_AB < 1.0f) //单独目标点
	{
		nu = 0;
		nav_bearing = wrap_180_cd(angle_AC) + 180;
		autoNaviSt.double_heading_exp = nav_bearing;
		return;

	}
	////else if (abs(turn_angle) >= 45){  //大于等于45度转弯
	////	turn_direction = copysign(turn_direction,turn_angle);//置位转弯方向
	////	autoNaviCfg.u16_arrival_distance1 = dist_L1 / (2 * fabs(cos(turn_angle / 2.0)));
	////}
	else if (dis_AC > dist_L1 && along_track_dis > dis_AB){ //CB在AB上的投影距离大于AB的距离则以A点为跟踪目标跟踪
		double delta_radial = ins_msg.heading - angle_AC;//径向角差向外为负 向内为正
		double delta_tagential = ins_msg.heading - ((angle_AC + 90) >= 360 ? (angle_AC - 270) : (angle_AC + 90));//与顺时针切向角差
		delta_radial = wrap_180_cd(delta_radial); //±180
		delta_tagential = wrap_180_cd(delta_tagential);//±180
		xtrackVel = usv_speed*cos(Radian(delta_tagential));//切向速度
		ltrackVel = -usv_speed*cos(Radian(delta_radial));//径向速度 指向A
		nu = atan2f(xtrackVel, ltrackVel);
		nav_bearing = angle_AC + 180;
	}
	else{ //正常L1跟踪
		double delta_l = ins_msg.heading - angle_AB; //平行AB径向角
		double delta_x = ins_msg.heading - ((angle_AB + 90) >= 360 ? (angle_AB - 270) : (angle_AB + 90)); //横向偏角
		delta_l = wrap_180_cd(delta_l); //±180
		delta_x = wrap_180_cd(delta_x);//±180
		xtrackVel = usv_speed*cos(Radian(delta_x)); //AB垂直方向速度
		ltrackVel = usv_speed*cos(Radian(delta_l)); //AB投影方向速度 指向B
		nu2 = atan2f(xtrackVel, ltrackVel);
		float sin_nu1 = cross_track_error / (dist_L1 > 0.1 ? dist_L1 : 0.1);
		nu1 = asinf(constrain_value(sin_nu1, -0.7071f, 0.7071f));

		//计算积分项，减少稳态误差
		if (_L1_xtrack_i_gain <= 0 || _L1_xtrack_i_gain != _L1_xtrack_i_gain_prev)
		{
			_L1_xtrack_i = 0;
			_L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
		}
		else if (fabsf(nu1) < Radian(5))
		{
			_L1_xtrack_i += nu1 * _L1_xtrack_i_gain * dt_s;

			// an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
			_L1_xtrack_i = constrain_value(_L1_xtrack_i, -0.1f, 0.1f);
		}
		nu1 += _L1_xtrack_i;
		printf("_L1_xtrack_i == %f\n", _L1_xtrack_i);
		nu = nu1 + nu2;
		nav_bearing = angle_AB + 180 / Pi*nu1;
		//radius_loiter = abs(dist_L1 / (2 * sin(Radian(nu)))); //跟踪旋转半径
	}
	prevent_indecision(nu);
	last_nu = nu;
	nu = constrain_value(nu, -1.5708f, +1.5708f);
	//	copysign(angle_exp, angle_AB);

	a_cmd = K_L1*pow(usv_speed, 2) / dist_L1*sin(nu);
	printf("angle_eta == %f \t  a_cmd == %f\n", nu, a_cmd);
	autoNaviSt.exp_rot = (180 / Pi) * (a_cmd / (xtrackVel > 0.1 ? xtrackVel : 0.1));
	autoNaviSt.double_heading_exp = nav_bearing;
	//return w_cmd;
}

void Emergency_brake_ctrl()
{
	#if 1
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 255 * 0.15;
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_DOWN;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;

	jet_system.jetR.u8_Cmd_MotorOpenDeg = 255 * 0.15;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_DOWN;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;
	#else
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;
	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;

	jet_system.jetR.u8_Cmd_MotorOpenDeg = 0;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;
	#endif
}

// static void set_throttle(float throttle)
// {
// 	jet_system.jetL.u8_Cmd_MotorOpenDeg = 255 * throttle;
// 	jet_system.jetR.u8_Cmd_MotorOpenDeg = 255 * throttle;
// 
// 	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_UP;
// 	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_UP;
// 
// }
// 
// static void set_steering(float steering)
// {
// 	jet_system.jetL.i16_Cmd_MotorRudderDeg = steering;
// 	jet_system.jetR.i16_Cmd_MotorRudderDeg = steering;
// }
