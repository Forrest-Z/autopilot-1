/*==========================================================*
 * 模块说明: ctrl_semiAuto_berth.cpp                        *
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
//区分模式	
1、前进		油门开度限制50%
2、后退		油门开度限制50%
3、左平移	固定舵角初始值、档位值、左右油门比例	舵角摇杆左右可动
4、右平移	固定舵角初始值、档位值、左右油门比例	舵角摇杆左右可动
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
	//todo 计算泊岸模式下 油门开度、档位、舵角
	CalcJoyStickComplex();	//获得控制指令

	berthParaCal();
	


}

//计算手柄值
void CalcJoyStickComplex(void)
{
	joy1Complex.d64_joystickAmp = sqrt((double)command_signal.joystick_cmd.i16_X1*(double)command_signal.joystick_cmd.i16_X1 + (double)command_signal.joystick_cmd.i16_Y1*(double)command_signal.joystick_cmd.i16_Y1);
	joy1Complex.d64_joystickAng = getAngle((double)command_signal.joystick_cmd.i16_X1,(double)command_signal.joystick_cmd.i16_Y1);			//方向 单位为度
	//debug start
	printf("Amp = %f \t Ang = %f\n",joy1Complex.d64_joystickAmp,joy1Complex.d64_joystickAng);
	//debug end
}


//返回角度 正上为0度,顺时针增加
double getAngle(double fx,double fy)
{
	uint8 b1_GoRight,b1_GoUp;

	if(fx >= 0.0)
		b1_GoRight = TRUE;	//向右
	else
		b1_GoRight = FALSE;	//向左

	if(fy >= 0.0)
		b1_GoUp = TRUE;		//向上
	else
		b1_GoUp = FALSE;	//向下

	if(fy == 0)	//
	{
		if(fx == 0)			//X轴相等
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

