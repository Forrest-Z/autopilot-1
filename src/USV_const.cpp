#include "stdafx.h"
#include "../include/usv_include.h"

#ifdef WINNT
HANDLE UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//���崮�ڱ�־	
HANDLE CAN_0;//����CAN
#else
int UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//���崮�ڱ�־	
int CAN_0;//����CAN
#endif
Radar_OBS Radar_OBS_Msg;				//�״��ϰ������Ϣ
Path_Coordinate Path_Coordinate_Msg;	//���Ϲ滮������

IP_ADDRESS_struct  ip_address;
SR_Config SR_Config_Msg;
UAV_CAN	UAV_CAN_State;//���˻�ƽ̨״̬
SR_CAN	SR_CAN_State;//���ܶ�״̬
POW_CAN POW_CAN_State;//��Դ����ϵͳ״̬
SP_Model_CAN	SP_CAN_Model_Con;//������ģʽָ��
SP_Sail_CAN		SP_Sail_CAN_Con;//�����Ʒ�����ָ��
SP_Equipment_CAN 	SP_Equipment_CAN_Con;//�������ȶ�ƽָ̨��
ST_PL_CAN ST_PL_CAN_State;//�ȶ�ƽ̨״̬
BAT_CON_CAN BAT_CON_CAN_State;//��Դ����ϵͳ״̬
Motor_CAN Motor_CAN_State;//������״̬
RECEIVE_STATU_MACHINE_T Receive_Status[2];//J1939����״̬��
FAULT_CODE_T	Fault_Code[2];//������������

AIS_Msg AIS_Msg_St;//ͼ����̨����AIS��Ϣ

Motor_Detail_State	Motor_Detail_St;//ͼ����̨���ͷ�������ϸ��Ϣ
Rudder_Detail_State Rudder_Detail_St;//ͼ����̨�������ܶ���ϸ��Ϣ
Stable_Platform_Detail_State Stable_Platform_St;//ͼ����̨�����ȶ�ƽ̨��ϸ��Ϣ
UAV_Detail_State UAV_Detail_St;//ͼ����̨�������˻�ƽ̨��ϸ��Ϣ
MCU_State MCU_St;//ͼ����̨���Ϳ��������ϵͳ��ϸ��Ϣ
Hull_State Hull_St;//ͼ����̨���ʹ�������ϸ��Ϣ
Panel_Control_Msg Panel_Control_St;//ͼ����̨���ʹ������ܿ��������ϸ��Ϣ
Energy_Control_Msg Energy_Control_St;//ͼ����̨������Դ����ϵͳ��ϸ��Ϣ
Power_Control_Msg  Power_Control_St;//ͼ����̨���͵�Դ����ϵͳ��ϸ��Ϣ
Check_Collision		USV_Check_Collision[AIS_Collision_Num];//��ײ����
Control_Message	USV_Control;//���˴�����ָ��
Sailing_Message		USV_Sailing;				//���˴���������
Dradio_Config_Message Dradio_Config;			//���˳�������Ϣ���������ļ����
State_Message 		USV_State;					//���˴�״̬
LAN0_RM_Message	USV_RM_MSG;//MPC����RAM���ڱ���
Waypoint				Point_Return[2];			//һ�������� 0--��ʱ��λ��  1--��ǰλ��
DSP_State DSP_State_Msg;//DSP״̬
	
Spd_PID	vPID;						//������ת�ٵ���PID������������
Spd_PID	rPID;						//��ǵ���PID����
Spd_PID	aPID;                       //ע�͵� û��
Spd_S	vSpd,rSrd;							//�����ٶȺ���S����ƽṹ��

TASK_TIME_STRUCT	task_time;
Track_Control		track_control;		//������������
E_STOP_ALL	e_stop_inf;					//��ͣ��Ϣ
	
uint8	Dradio_Com1_Sign;//���ֵ�̨COM1����״̬
uint8	Dradio_Com2_Sign;//���ֵ�̨COM2����״̬
uint8	SpareDradio_Com1_Sign;//�������ֵ�̨COM1����״̬
uint8	SpareDradio_Com2_Sign;//�������ֵ�̨COM2����״̬
uint8	Bradio_Con_Sign;//�����̨����״̬
uint8	BDS_Con_Sign;//������̨����״̬
uint8	SP_CON_Sign;//������������ȡ����Ȩ�ޱ�־ 1--��������������  0--�����������
uint8	BD_Engine_Run_L;	//���յ��ָ��L
uint8	BD_Engine_Run_R;	//�������ָ��R


int LAN0_Fd,LAN1_Fd;//��������

uint8	Radio_Sign,Radio_Sign_old,Connect_Sign;
uint8	Sailing_Sign;
uint8	Sailing_Cnt_Old;			//��ǰ���еĺ������
uint8	Avoid_Point_Arrive=0;			//���ϵ㵽���־
double Speed_EXP,Heading_EXP;//�����㷨��������������٣�Speed_EXP--�ٶ�����ֵ��Heading_EXP--��������ֵ
double	Collision_Speed;//���Ϻ���
double	Collision_Heading;//���Ϻ���
double	Radar_Collision_Heading;	//�״���Ϻ���
sint16 GLB_TCPA;
int32	GLB_Safe_DCPA,GLB_DCPA;
uint8 Collision_Num;
uint8	Rudder_Con_Angle;//���ƶ�ǣ�ʵ�ʶ�ǲ��ܴ��ڿ��ƶ��
uint8	Rudder_Count;	//��ĳһ�����ĳ���ʱ��
uint8	Collision_Sign;	//��ײ��־
int Watch_fd;
double lau_old,lnu_old;//������̵ľ�γ�����
uint32  Mileage;//���
uint8 E_Stop,E_Stop_INS,E_Stop_DSP,E_Stop_Dradio,E_Stop_SmartRudder,Time_Sign;
int Rudder_Zero_Angle;//������,�������ƫ��
int Accelerator_L,Accelerator_R,Rudder_L,Rudder_R,Gear_L,Gear_R;			//Accelerator--���� Rudder--��� Gear--��λ
double Accelerator_Coefficient_L,Accelerator_Coefficient_R,Rudder_Coefficient_L,Rudder_Coefficient_R,Gear_Coefficient_L_F,Gear_Coefficient_L_B,Gear_Coefficient_R_F,Gear_Coefficient_R_B;
uint16 Heartbeat_Num;

float Dst_monitor=0;
int32 NoStopPoint_Sign=0;				//ͣ����־
int32 Obstacle_Sign = 0;				//���ϱ�־	PSO ���ϱ�־

int32 radar_obstacle_sign =0;			//��̬�ϰ�����ײ��־
int32 radar_avoid_type=0;				//0 �޶��� 1 ����  2 ׷�� 3 �󽻲�  4 �ҽ���

int sign,Get_Control_Sign,count_Estop,SP_CAN_Count;
uint8	USV_State_Current[State_Current_Num];//���˴���ǰ״̬����̨����
char Version[10][100];
int ARMVersion_sign,DSPVersion_sign,MPCVersion_sign,UAVVersion_sign,SRVersion_sign,POWVersion_sign,SPVersion_sign,STVersion_sign,BATVersion_sign;

//������sockid
int16	sockid_test = 0;
uint8	sockid_test_buf[20];


//����汾
int ship_version=0;

//����
uint16 usv_num = 0;