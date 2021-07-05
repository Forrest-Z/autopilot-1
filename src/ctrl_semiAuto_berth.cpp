/*==========================================================*
 * ģ��˵��: ctrl_semiAuto_berth.cpp                        *
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
#include "../include/ctrl_semiAuto_berth.h"
#include <math.h>
/******************************  Local Variable  *****************************/
JOYSTICK_COMPELX joy1Complex;
BERTH_PARAM		 berthParam	;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
double getAngle(double fx,double fy);
void CalcJoyStickComplex(void);
void calcBerth(void);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void berthInit(void)
{
	
}


void GetBerth(void)
{
    if(semi_modeTrig.b1_trig_berth == 1)
		berthInit();
}



void SemiAutoBerth(void)
{
    GetBerth();
    if(command_signal.func_mode_cmd.b1_dock_cmd == COMMAND_ON && semi_modeTrig.b1_trig_berth == 0)
	{
		calcBerth();
	}
}

/**************************************************************************
//����ģʽ	
1��ǰ��		���ſ�������50%
2������		���ſ�������50%
3����ƽ��	�̶���ǳ�ʼֵ����λֵ���������ű���	���ҡ�����ҿɶ�
4����ƽ��	�̶���ǳ�ʼֵ����λֵ���������ű���	���ҡ�����ҿɶ�
********************************
*******************************************/
void berthParaCal(void)
{
	if(joy1Complex.d64_joystickAmp < 5.0)
	{
		berthParam.b3_berth_mode = BERTH_STOP	;
	}
	else
	{
		if(joy1Complex.d64_joystickAng < 45.0 || joy1Complex.d64_joystickAng > 315.0)
		{
			berthParam.b3_berth_mode = BERTH_MV_FORWARD;
		}
		else if(joy1Complex.d64_joystickAng >= 45.0 && joy1Complex.d64_joystickAng <135.0)
		{
			berthParam.b3_berth_mode = BERTH_MV_RIGHT;
		}
		else if(joy1Complex.d64_joystickAng >= 135.0 && joy1Complex.d64_joystickAng <225.0)
		{
			berthParam.b3_berth_mode = BERTH_MV_BACKWARD;
		}
		else if(joy1Complex.d64_joystickAng >= 225.0 && joy1Complex.d64_joystickAng <315.0)
		{
			berthParam.b3_berth_mode = BERTH_MV_LEFT;
		}
		else
		{
			berthParam.b3_berth_mode = BERTH_STOP;
		}
	}
	//debug start
	printf("BERTH MODE %d \n",berthParam.b3_berth_mode);
	//debug end

}


void calcBerth(void)
{
	//todo ���㲴��ģʽ�� ���ſ��ȡ���λ�����
	CalcJoyStickComplex();	//��ÿ���ָ��

	berthParaCal();
	


}

//�����ֱ�ֵ
void CalcJoyStickComplex(void)
{
	joy1Complex.d64_joystickAmp = sqrt((double)command_signal.joystick_cmd.i16_X1*(double)command_signal.joystick_cmd.i16_X1 + (double)command_signal.joystick_cmd.i16_Y1*(double)command_signal.joystick_cmd.i16_Y1);
	joy1Complex.d64_joystickAng = getAngle((double)command_signal.joystick_cmd.i16_X1,(double)command_signal.joystick_cmd.i16_Y1);			//���� ��λΪ��
	//debug start
	printf("Amp = %f \t Ang = %f\n",joy1Complex.d64_joystickAmp,joy1Complex.d64_joystickAng);
	//debug end
}


//���ؽǶ� ����Ϊ0��,˳ʱ������
double getAngle(double fx,double fy)
{
	uint8 b1_GoRight,b1_GoUp;

	if(fx >= 0.0)
		b1_GoRight = TRUE;	//����
	else
		b1_GoRight = FALSE;	//����

	if(fy >= 0.0)
		b1_GoUp = TRUE;		//����
	else
		b1_GoUp = FALSE;	//����

	if(fy == 0)	//
	{
		if(fx == 0)			//X�����
		{
			return 0;
		}
		return b1_GoRight?90:270;
	}
	double dirc = atan(fx/fy)*180/Pi;
	if(!b1_GoRight&&b1_GoUp)
		dirc = 360+dirc;
	if(!b1_GoRight&&!b1_GoUp)
		dirc = 180+dirc;
	if(b1_GoRight&&!b1_GoUp)
		dirc = 180+dirc;
	return dirc;
}

