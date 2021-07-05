/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
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

//PID ��������
/* �����㷨�п����ò����Ľṹ��*/
typedef struct
{
	int8    ctlSign;            // ���������ţ�ȡ����ʵ�ʵ�����ϵͳ
	//double    pidCtlZone;        // Bang-Bang��������PID���������л���

	float    *proportional;       // ��������
	float    *integral;           // ��������
	float    *derivative;         // ΢������

	float    *integralZone;       // �������������� EI
	float    *antiWinup_Kb;       // ��������������� KB

	float    outUpperLimt;       // �������
	float    outLowerLimit;      // �������
	float    *outZeroBias;        // ��̬ƫ�ò���  ZB
}pidCtlParams_t;//�������ò���


/* �����㷨������������źŽṹ��*/
typedef struct
{
	double desired;              // �ο�  �����ź�
	double measured;             // ����  �����ź�

	double cur_err;              // ��ǰ�������
	double prev_err;             // �ϴ�ʱ��������

	double propOut;              // �������������
	double dervOut;              // ΢�ֿ��������
	double intOut;               // ���ֿ��������

	double pidOut;               // PID�ϳɿ������
	double antiWinupOut;         // �����������

	double prevSatCtlOut;        // ����������ǰ���
	double aftSatCtlOut;         // ���������ƺ��������Ϊ����������������
}pidCtlSigs_t;//�����㷨���������

typedef struct{
	/*
	* ģ�ͣ� ����΢�ָ�����
	*       fh = -wn^2(v1-v)-2wn*yita*v2
	*        v1 = v1 + hv2
	*        v2 = v2 + hfh
	%        y= v1
	*/
	float dt;
	float *wn;                // ������ֹƵ�� rad/s
	float *yita;              // ��������     0-1
	float v[2];
	float in;
	float out;				//ģ�����

}pidRefMode_t; //ģ��

/*
* ����PID ���������źţ��˿�����Ϊ����ʽ�ģ�Ҫ��ʹ������ʽ�ģ����Լ���������ֲ��޸ġ�
*/
extern int16 pix_ex_out;

extern pidCtlParams_t imagePidCtlParams;//���ƿ����ò���
extern pidRefMode_t   imagePidRefMode;	//����ģ������

void PidControlUpdate(pidRefMode_t *pidMode, pidCtlParams_t *pxPidParm, pidCtlSigs_t *pxPidSig);
#endif//__DOCKING_PID_LOOP_H_