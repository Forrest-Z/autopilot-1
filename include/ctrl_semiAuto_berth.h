//ctrl_semiAuto_berth.h

#ifndef __CTRL_SEMIAUTO_BERTH__H_
#define __CTRL_SEMIAUTO_BERTH__H_

typedef struct{
	double d64_joystickAmp;	//��ֵ
	double d64_joystickAng;	//�Ƕ�
//	uint8 u8_joystickArea;	//����
}JOYSTICK_COMPELX;		//ҡ��ʸ��


#define		BERTH_STOP			0
#define		BERTH_MV_FORWARD	1
#define		BERTH_MV_BACKWARD	2
#define		BERTH_MV_LEFT		3
#define		BERTH_MV_RIGHT		4


typedef struct {
	uint8	b3_berth_mode;		//	0:	���ٲ���		1:ǰ��	2:����	3����ƽ��	4����ƽ��
}BERTH_PARAM;



extern void SemiAutoBerth(void);   //����ģʽ


#endif /*__CTRL_SEMIAUTO_BERTH__H_*/