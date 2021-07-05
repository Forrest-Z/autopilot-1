/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
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
	* �Կ�����Ŀ�����ʱ��滮�����ÿһʱ���µĵ�ǰ�ο�Ŀ��
	*/
#if 0
	float fh;
	pidMode->out = pidMode->v[0];//

	pidMode->in = pxPidSig->desired;
	fh = -(*pidMode->wn)*(*pidMode->wn)*(pidMode->v[0] - pidMode->in) - 2 * (*pidMode->wn)*(*pidMode->yita)*pidMode->v[1];
	pidMode->v[0] += pidMode->dt*pidMode->v[1];
	pidMode->v[1] += pidMode->dt*fh;

	/* ��ȡ��ǰ������� */
	cur_desired = pidMode->out;//

	pix_ex_out = cur_desired;//��ȡ��������ֵ���

#endif
	pix_ex_out = cur_desired;//��ȡ��������ֵ���
	pxPidSig->cur_err = pxPidParm->ctlSign* (cur_desired - pxPidSig->measured);


	/* ��ȡ����������� */
	pxPidSig->propOut = *pxPidParm->proportional * pxPidSig->cur_err;

	/* ��ȡ���ֿ������*/
	if (pxPidSig->cur_err > *pxPidParm->integralZone || pxPidSig->cur_err < -(*pxPidParm->integralZone))
	{
		pxPidSig->intOut = 0;
	}
	else
	{
		pxPidSig->intOut += (pxPidSig->cur_err * (*pxPidParm->integral) +
			pxPidSig->antiWinupOut*(*pxPidParm->antiWinup_Kb));
	}

	/* ��ȡ΢��������� */
	pxPidSig->dervOut = (pxPidSig->cur_err - pxPidSig->prev_err)*(*pxPidParm->derivative);

	/* ����PID����� */
	pxPidSig->pidOut = (pxPidSig->propOut) + (pxPidSig->intOut) + (pxPidSig->dervOut);


	/* ���Ϳ��������ƣ�������տ����ź� */
	pxPidSig->prevSatCtlOut = (pxPidSig->pidOut);
	pxPidSig->aftSatCtlOut = pxPidSig->prevSatCtlOut;

	if (pxPidSig->aftSatCtlOut  > pxPidParm->outUpperLimt)
		pxPidSig->aftSatCtlOut = pxPidParm->outUpperLimt;
	if (pxPidSig->aftSatCtlOut  < pxPidParm->outLowerLimit)
		pxPidSig->aftSatCtlOut = pxPidParm->outLowerLimit;


	/* �ڲ�״̬���� */
	pxPidSig->prev_err = pxPidSig->cur_err;
	pxPidSig->antiWinupOut = pxPidSig->aftSatCtlOut - pxPidSig->prevSatCtlOut;
}