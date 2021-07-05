/*
* can_api.h --CAN��ʼ��
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#ifndef	CAN_API_H
#define     CAN_API_H
/*
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <signal.h>

#ifndef WINNT
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>

#endif

#include "USV_const.h"
*/
#include "../include/usv_include.h"


#define	CAN							"can0"								//CAN
#define 	CAN0_bitrate   				250000							//����CAN������
#define	SP_ST1_PGN					65305								//�������״̬PGN1
#define	SP_ST2_PGN					65306								//�������״̬PGN2
#define	SP_ST3_PGN					65307								//�������״̬PGN3
#define	SP_ST4_PGN					65308								//�������״̬PGN4
#define	SP_ST5_PGN					65309								//�������״̬PGN5
#define	SP_ST6_PGN					65310								//�������״̬PGN6
#define	SP_ST7_PGN					65311								//�������״̬PGN7
#define	UAV_PGN					65320								//UAV״̬����PGN
#define	SR_Speed_PGN				65330								//���ܶ��ٶ�״̬����PGN
#define	SR_Rudder_PGN				65331								//���ܶ���״̬����PGN
#define	POW_CON_PGN				65340								//��Դ����״̬����PGN
#define	SP_Model_PGN				65353								//�������ģʽ����PGN
#define	SP_CON1_PGN				65350								//����������PGN1
#define	SP_CON2_PGN				65351								//����������PGN2
#define	SP_CON3_PGN				65352								//����������PGN3
#define	ST_PL_PGN					65360								//�ȶ�ƽ̨״̬����PGN
#define	BAT_ST_PGN					65370								//��Դ����״̬����PGN
#define	BAT_L_PGN					65371								//��Դ��������״̬����PGN
#define	BAT_R_PGN					65372								//��Դ�����ҵ��״̬����PGN



//���ܶ�״̬�ṹ��
typedef struct  
{
       uint16	Motor_RPM_ACT_L;						//�󷢶���ʵ��ת��
       uint16	Motor_RPM_ACT_R;						//�ҷ�����ʵ��ת��
       uint8	Motor_Gear_ACT_L;						//�󷢶���ʵ�ʵ�λ
       uint8	Motor_Gear_ACT_R;						//�ҷ�����ʵ�ʵ�λ
       uint16	Rudder_ACT_L;							//���ʵ�ʶ��
       uint16	Rudder_ACT_R;							//�Ҷ�ʵ�ʶ��
}Motor_ACT;
//���˻�ƽ̨״̬�ṹ��
typedef struct  
{
       uint8	Motor_RPM_MAX_L;						//�󷢶������ת��
       uint8 	Motor_RPM_MIN_L;						//�󷢶������ת��
       uint8	Motor_RPM_MAX_R;						//�ҷ��������ת��
       uint8	Motor_RPM_MIN_R;						//�ҷ��������ת��
       uint8	Motor_Gear_MAX_L;						//�󷢶�����ߵ�λ
       uint8	Motor_Gear_MIN_L;						//�󷢶�����͵�λ
       uint8	Motor_Gear_MAX_R;						//�ҷ�������ߵ�λ
       uint8	Motor_Gear_MIN_R;						//�ҷ�������͵�λ
}UAV_ACT;
//��������ṹ��
typedef struct  
{
       uint8	Rudder_Limit_L;							//�����ƶ��
       uint8	Rudder_Limit_R;							//�Ҷ���ƶ��
}Rudder_EXP_Con;

//���ܺ�����Ϣ�ṹ��
typedef struct  
{
       uint16	USV_Speed_EXP;							//��������
       uint16	USV_Heading_EXP;							//��������,���з���
}Sailing_EXP_Con;

typedef struct  
{
	uint8 CAN_add;
	uint8  CAN_Priority;
	uint16 CAN_PGN;
	uint8	CAN_Data[8];
}CAN_Struct;


extern CAN_Struct 	CAN_Msg;
extern uint8 FAULT_CODE_Last[2][10];
extern uint8 Sealight_Count,Sealight_Sign;

#ifdef WINNT
uint16 read_can(HANDLE hCom,struct can_frame *buff,uint16 len);
int8 write_can(HANDLE hCom,struct can_frame *lpOutBuffer,uint16 write_len);
#else
uint16 read_can(int hCom,struct can_frame *buff,uint16 len);
int8 write_can(int hCom,struct can_frame *lpOutBuffer,uint16 write_len);
#endif

void CAN_Analyzing(CAN_Struct *frame);
void Check_Communication_State(void);
void CAN_Init(void);
void Send_CAN_Control(void);
void Get_CAN_SR_Con();
void Get_CAN_SR_Smart_Con();
void Get_CAN_SR_Engine_Config();
void Get_CAN_SR_Gear_Config();
void Get_CAN_SR_Gear2_Config();
void Get_CAN_SR_Rudder_Config();
void Get_CAN_SR_Rudder2_Config();
void Get_CAN_SR_Rudder3_Config();
void Get_CAN_UAV_Con();
void Get_CAN_POW_Con();
void Get_CAN_ST_PL_Con();
void Get_CAN_Energy_Con();
void Send_CAN_State(void);
void Send_CAN_65305();
void Send_CAN_65306();
void Send_CAN_65307();
void Send_CAN_65308();
void Send_CAN_65309();
void Send_CAN_65310();
void Send_CAN_65311();
void Send_CAN_Cfg();
void Write_Config();

//void Get_Radar_DCPA_TCPA(uint32 DCPA,sint16 TCPA);//��ȡ�״��ϰ�Ŀ�����ײ����

#endif /* CAN_API_H */
