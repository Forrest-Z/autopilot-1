/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_pid_loop.cpp
-	Author	   : 
-	Date	   : 2019/05/28 15:13
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#include "stdafx.h"
#include "../include/docking_pid_loop.h"

int16 pix_ex_out;
pidCtlParams_t imagePidCtlParams;//
pidRefMode_t   imagePidRefMode;	//
void PidControlUpdate(pidRefMode_t *pidMode, pidCtlParams_t *pxPidParm, pidCtlSigs_t *pxPidSig)
{
	float cur_desired = 350;
	/*
	* 对控制器目标进行时间规划，获得每一时标下的当前参考目标
	*/
#if 0
	float fh;
	pidMode->out = pidMode->v[0];//

	pidMode->in = pxPidSig->desired;
	fh = -(*pidMode->wn)*(*pidMode->wn)*(pidMode->v[0] - pidMode->in) - 2 * (*pidMode->wn)*(*pidMode->yita)*pidMode->v[1];
	pidMode->v[0] += pidMode->dt*pidMode->v[1];
	pidMode->v[1] += pidMode->dt*fh;

	/* 获取当前跟随误差 */
	cur_desired = pidMode->out;//

	pix_ex_out = cur_desired;//获取像素期望值输出

#endif
	pix_ex_out = cur_desired;//获取像素期望值输出
	pxPidSig->cur_err = pxPidParm->ctlSign* (cur_desired - pxPidSig->measured);


	/* 获取比例控制输出 */
	pxPidSig->propOut = *pxPidParm->proportional * pxPidSig->cur_err;

	/* 获取积分控制输出*/
	if (pxPidSig->cur_err > *pxPidParm->integralZone || pxPidSig->cur_err < -(*pxPidParm->integralZone))
	{
		pxPidSig->intOut = 0;
	}
	else
	{
		pxPidSig->intOut += (pxPidSig->cur_err * (*pxPidParm->integral) +
			pxPidSig->antiWinupOut*(*pxPidParm->antiWinup_Kb));
	}

	/* 获取微分增益输出 */
	pxPidSig->dervOut = (pxPidSig->cur_err - pxPidSig->prev_err)*(*pxPidParm->derivative);

	/* 计算PID总输出 */
	pxPidSig->pidOut = (pxPidSig->propOut) + (pxPidSig->intOut) + (pxPidSig->dervOut);


	/* 饱和控制器调制，输出最终控制信号 */
	pxPidSig->prevSatCtlOut = (pxPidSig->pidOut);
	pxPidSig->aftSatCtlOut = pxPidSig->prevSatCtlOut;

	if (pxPidSig->aftSatCtlOut  > pxPidParm->outUpperLimt)
		pxPidSig->aftSatCtlOut = pxPidParm->outUpperLimt;
	if (pxPidSig->aftSatCtlOut  < pxPidParm->outLowerLimit)
		pxPidSig->aftSatCtlOut = pxPidParm->outLowerLimit;


	/* 内部状态记忆 */
	pxPidSig->prev_err = pxPidSig->cur_err;
	pxPidSig->antiWinupOut = pxPidSig->aftSatCtlOut - pxPidSig->prevSatCtlOut;
}