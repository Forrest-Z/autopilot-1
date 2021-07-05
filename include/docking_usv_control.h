/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
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


//�����������
extern int8 dockOutUSVControl(void);
//���봬����
extern int8 dockInUSVControl(void);
//DSP����ָ�����뺯��
extern void dspControlCmd(float _heading, float _vx, float _wr);
extern void dspControlOFF();

extern uint8 reDockIn();
extern void resetDockControlEvent();
extern void getLatLng(double lat, double lng, double dst, double heading, double *Lat, double *Lng);
void set_throttle(float throttle);
void set_steering(float steering);

#endif//__DOCKING_USV_CONTROL_H_