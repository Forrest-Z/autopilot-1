//ctrl_semiAuto_berth.h

#ifndef __CTRL_SEMIAUTO_BERTH__H_
#define __CTRL_SEMIAUTO_BERTH__H_

typedef struct{
	double d64_joystickAmp;	//幅值
	double d64_joystickAng;	//角度
//	uint8 u8_joystickArea;	//区域
}JOYSTICK_COMPELX;		//摇杆矢量


#define		BERTH_STOP			0
#define		BERTH_MV_FORWARD	1
#define		BERTH_MV_BACKWARD	2
#define		BERTH_MV_LEFT		3
#define		BERTH_MV_RIGHT		4


typedef struct {
	uint8	b3_berth_mode;		//	0:	怠速不动		1:前进	2:后退	3：左平移	4：右平移
}BERTH_PARAM;



extern void SemiAutoBerth(void);   //泊岸模式


#endif /*__CTRL_SEMIAUTO_BERTH__H_*/