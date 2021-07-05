/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// 
// (C) Copyright 2017,Beijing
// All Rights Reserved
//
//
// FileName:        read_usv_state.cpp
// Programmer(s):  syp ,2017-6-22
// Description: ��ȡ�ļ�����
//                  [s1] s2=value ";"��Ϊע��
*******************************************************************************************/

#ifndef _READ_USV_STATE_H_
#define _READ_USV_STATE_H_


#ifndef WINNT 
const char USV_STATE_FILE_NAME[]="../cfg/usv_flash_state.inf";		//usv����״̬
#else
const char USV_STATE_FILE_NAME[]="../../cfg/usv_flash_state.inf";		//usv����״̬
#endif

#define WRITE_METER_CLOCK_TIME 10


//������̱���Ϣ
typedef struct {
	uint32	run_time_second;			//����ʱ�� 	��λ:	��
	uint32	run_total_dist;				//�����		��λ:	0.1����
	uint32	run_total_dist_this_time;	//�������		��λ:	0.1����
	uint32	run_time_second_this_time;	//��������ʱ�� ��λ:	��
	uint32	run_dist_10min;				//ʮ�������	��λ:	��*100 *s
	uint32	run_average_speed;			//ƽ������	��λ:	0.1kn/h
} USV_Meter_Clock;


//ȫ�ֱ���
extern USV_Meter_Clock USV_Meter_Clock_Data;



//��������
int8	read_usv_state(void);
int8	Update_Meter_Clock(void);
int8    write_usv_state(void);
int8	updateMeterClock(void);		//С�ͻ�װ����̱�
#endif
