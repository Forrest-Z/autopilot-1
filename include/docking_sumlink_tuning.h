/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
-	File name  : docking_param_tuning.h
-	Author	   : fushuai
-	Date	   : 2019/06/02 9:43
-   Version    : 1.0
-	Description:������Ʋ����趨
-	Others:
*********************************************************************/
#ifndef __DOCKING_TUNING_H_
#define __DOCKING_TUNING_H_
#include "usv_include.h"

#ifndef WINNT 
const char USV_DOCKING_SETTING_FILE_NAME[] = "../cfg/usv_dock_control.inf";						//������������ļ�
#else
const char USV_DOCKING_SETTING_FILE_NAME[] = "../../cfg/usv_dock_control.inf";						//������������ļ�
#endif


//typedef struct
//{
//	int cmd_r;
//	//float pidctrSign;		/*���Ʒ���*/
//	float pid_P;			/*��������*/
//	float pid_I;			/*��������*/
//	float pid_D;			/*΢������*/
//	float pid_EI;			/*�����������*/
//	float pid_KB;			/*���������������*/
//	float pid_ZB;			/*���ƫ��*/
//}DockingParamsPID;
#pragma pack(1)
typedef struct  
{
	uint8 u8_reset;		 //��λ
	uint8 u8_return_set; //������
	uint8 u8_startp_set;//������ ���ڵ�������Ϊ���� ��󷢲��ü����ȡ
	uint8 u8_dockin;	 //����
	uint8 u8_dockout;	 //����
	uint8 u8_ready_in;	 //�ɽ���
	uint8 u8_already_in; //�ѽ���
	uint8 u8_ready_out;	 //�ɳ���
	uint8 u8_already_out;//�ѳ���
}SumlinkCmd;
#pragma pack()
#pragma pack(1)
typedef struct
{

	uint8	u8_st_oil1;				//����ֵ
	uint8	u8_st_oil2;				//����ֵ
	int16	i16_st_moter1Rudder;	// ���			33
	int16	i16_st_moter2Rudder;	// 36


	uint16	u16_st_moter1Rpm;		// ������ת�� 31
	uint16	u16_st_moter2Rpm;		// 34

	uint16	u16_st_speed;		// �ٶ�			
	uint16	u16_st_heading;		// ����				
	int16	i16_st_rot;			// ת����	

	uint16 i16_cur_cx;
	uint16 i16_cur_cy;
	uint8  u8_cur_lock;

	int16 u16_ex_cx;


	uint8	u8_st_lonSt;		// ���ȱ�־		
	uint8	u8_st_lonDeg;		// ���� ��		
	uint8	u8_st_lonMin;		// ���� ��		
	uint8	u8_st_lonSec;		// ���� ��		
	uint8	u8_st_lonSecDec;	// ���� ��С��	

	uint8	u8_st_latSt;		// γ�ȱ�־		
	uint8	u8_st_latDeg;		// γ�� ��		
	uint8	u8_st_latMin;		// γ�� ��		
	uint8	u8_st_latSec;		// γ�� ��		
	uint8	u8_st_latSecDec;	// γ�� ��С��

	//uint8 u8_return_set;
	//uint8 u8_dockin;
	//uint8 u8_dockout;
	//uint8 u8_ready_in;
	//uint8 u8_ready_out;
	//uint8 u8_already_in;

}SumLinkDocking;
#pragma pack()

typedef struct 
{
	uint8  u8_lidar_lock_on;	//����״̬
	float  f32_lidar_x;			//�յ��״�����x		��
	float  f32_lidar_y;			//�յ��״�����y		��
	double f64_dis;			    //Ŀ�����
	double f64_target_angle;	//���뷽��			��
	double f64_real_angle;		//����ǰ����		��
	double f64_track_angle;		//���ٵ����Ƕ�		��
	double f64_track_err;		//���ٺ���ƫ��		��
}SumlinkDockingLidar;

extern char docking_contrl_cfg[30];
extern uint32 SUMLINK_PORT;
extern uint32 SUMLINK_RECVPORT;
extern SumLinkDocking sumlink_dockin_state;
extern SumlinkDockingLidar sumlink_lidardockin_state;
extern SumlinkCmd sumlink_cmd;
void *tuningParsmsSumlinkTempThread(void *aa);
#endif