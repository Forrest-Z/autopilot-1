/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
-	File name  : docking_usv_control.h
-	Author	   : fushuai
-	Date	   : 2019/06/03 19:36
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef __DOCKING_USV_CONTROL_H_
#define __DOCKING_USV_CONTROL_H_
#include "usv_include.h"


//倒船出坞控制
extern int8 dockOutUSVControl(void);
//进坞船控制
extern int8 dockInUSVControl(void);
//DSP控制指令输入函数
extern void dspControlCmd(float _heading, float _vx, float _wr);
extern void dspControlOFF();

extern uint8 reDockIn();
extern void resetDockControlEvent();
extern void getLatLng(double lat, double lng, double dst, double heading, double *Lat, double *Lng);
void set_throttle(float throttle);
void set_steering(float steering);

#endif//__DOCKING_USV_CONTROL_H_