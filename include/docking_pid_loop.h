/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_pid_loop.h
-	Author	   : 
-	Date	   : 2019/06/18 20:12
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef __DOCKING_PID_LOOP_H_
#define __DOCKING_PID_LOOP_H_
#include "usv_include.h"

//PID 参数定义
/* 定义算法中可配置参数的结构体*/
typedef struct
{
	int8    ctlSign;            // 控制器符号，取决于实际的物理系统
	//double    pidCtlZone;        // Bang-Bang控制器与PID控制器的切换点

	float    *proportional;       // 比例增益
	float    *integral;           // 积分增益
	float    *derivative;         // 微分增益

	float    *integralZone;       // 误差积分作用区间 EI
	float    *antiWinup_Kb;       // 饱和溢出反馈增益 KB

	float    outUpperLimt;       // 输出上限
	float    outLowerLimit;      // 输出下限
	float    *outZeroBias;        // 静态偏置补偿  ZB
}pidCtlParams_t;//控制配置参数


/* 定义算法中输入输出的信号结构体*/
typedef struct
{
	double desired;              // 参考  输入信号
	double measured;             // 反馈  输入信号

	double cur_err;              // 当前跟随误差
	double prev_err;             // 上次时标跟随误差

	double propOut;              // 比例控制量输出
	double dervOut;              // 微分控制量输出
	double intOut;               // 积分控制量输出

	double pidOut;               // PID合成控制输出
	double antiWinupOut;         // 饱和溢出部分

	double prevSatCtlOut;        // 饱和器控制前输出
	double aftSatCtlOut;         // 饱和器控制后输出，即为控制器最终输出结果
}pidCtlSigs_t;//进坞算法输入输出量

typedef struct{
	/*
	* 模型： 线性微分跟踪器
	*       fh = -wn^2(v1-v)-2wn*yita*v2
	*        v1 = v1 + hv2
	*        v2 = v2 + hfh
	%        y= v1
	*/
	float dt;
	float *wn;                // 开环截止频率 rad/s
	float *yita;              // 阻尼因子     0-1
	float v[2];
	float in;
	float out;				//模型输出

}pidRefMode_t; //模型

/*
* 更新PID 控制器的信号，此控制器为绝对式的，要想使用增量式的，需自己根据情况局部修改。
*/
extern int16 pix_ex_out;

extern pidCtlParams_t imagePidCtlParams;//控制可配置参数
extern pidRefMode_t   imagePidRefMode;	//跟随模型配置

void PidControlUpdate(pidRefMode_t *pidMode, pidCtlParams_t *pxPidParm, pidCtlSigs_t *pxPidSig);
#endif//__DOCKING_PID_LOOP_H_