//uart_irtk.h
#ifndef __UART_IRTK_H_
#define __UART_IRTK_H_

#include "usv_include.h"

#ifndef WINNT
typedef int UART_TYPE;
#else
typedef HANDLE UART_TYPE;
#endif

#define IRTK_COMM_TIMEMAX 3
#ifdef WINNT
#define IRTK_COMMUNICATION_COM    "COM6"                      /*ͨѶ����*/
#define IRTK_SEND_COM			  "COM9"					  /*����ת������*/
#else
#define IRTK_COMMUNICATION_COM    "/dev/ttyO1"                /*ͨѶ���� ��ʱ��˿ӡ�ϵ�COM1����IRTKͨ��*/
#define IRTK_SEND_COM			  "/dev/ttyO6"				  /*����ת������*/
#endif


//�ߵ�״̬�ṹ��
typedef struct
{
	uint16	USV_Speed;						//�ܺ���  ���պ� *100 Ϊ��λ��0.01�ڣ�
	uint16	USV_Heading;					//����
	uint16	USV_Position_Heading;			//�����
	uint16	USV_Move_Heading;				//������
	uint16	USV_Roll;						//���(��ҡ)
	uint16	USV_Pitch;						//����(��ҡ)
	uint16	USV_Bow;						//��ҡN
	uint16	USV_Transverse;					//�ᵴN
	uint16	USV_Surge;						//�ݵ�N
	uint16	USV_Heave;						//����(����)   
	uint16	USV_Yaw;						//ƫ����N
	uint16	USV_ROT;						//ת����			
	uint32	USV_Height;						//����
	uint8	Latitude_Sign_St;				//���˴�λ�þ��ȱ�־
	uint8	Longitude_Sign_St;				//���˴�λ��ά�ȱ�־	1--������ ��2--�ϰ���
	uint8	USV_Latitude_Degree;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Minute;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Second;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Decimal_2;			//USVλ�þ���ֵ��С��λ1/2λ
	uint8	USV_Latitude_Decimal_4;			//USVλ�þ���ֵ��С��λ3/4λ
	uint8	USV_Longitude_Degree;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Minute;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Second;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Decimal_2;		//USVλ��ά��ֵ��С��λ1/2λ	
	uint8	USV_Longitude_Decimal_4;		//USVλ��ά��ֵ��С��λ3/4λ
	uint8	USV_Year;						//��
	uint8	USV_Month;						//��
	uint8	USV_Date;						//��
	uint8	USV_Hour;						//ʱ
	uint8	USV_Minute;						//��
	uint8	USV_Second;						//��
	uint8	USV_Second_2;					//���С�������λ
	uint8	Satellite_Num_1;				//����1���Ǹ���
	uint8	Satellite_Num_2;				//����2���Ǹ���
	uint8	Sys_State_1;					//ϵͳ״̬1
	uint8	Sys_State_2;					//ϵͳ״̬2 N
	uint8	Power_Light;					//��Դ��
	uint8	Serial_Light;					//���ڵ�
	uint8	Main_Antenna_Light;				//������ָʾ��
	uint8	Minor_Antenna_Light;			//������ָʾ��N
	uint8	Differerntial_Signal_Light;		//����ź�ָʾ��
	uint8	Differential_Position_Light;	//��ֶ�λָʾ��
	uint8	Smart_Navigation_Heatbeat;		//����
	double	USV_Lat;						//���ȣ�С����ʾ
	double	USV_Lng;						//ά�ȣ�С����ʾ
}iRTK_Smart_Navigation_Msg;


typedef struct{
	uint8	u8_sateliteNum1;			//��λ���Ǹ���
	uint8	u8_sateliteNum2;			//����2���Ǹ���
	uint8	u8_sysState1;				//GPS���� / ���ݶ���
	uint8	u8_sysState1_old;			//GPS���� / ���ݶ�����ʷֵ
	uint8	u8_sysState2;				//����
	uint8	b1_diffSignalValid_old;		//����ź���Ч��ʷֵ
	uint8	b1_diffSignalValid;			//����ź���Ч
	uint8	b1_diffPostionValid;		//��ֶ�λ��Ч
	uint8	b1_dateValid;				//������Ч
	uint8	b1_timeValid;				//ʱ����Ч
	char	c_rmcValid;					//GPRMCλ����Ч
	char	c_rmcValid_old;				//GPRMCλ����Чold
}iRTK_STATE;


typedef struct{
	double	speed;					// kn
	double	heading;				// ����
	double  motionDirection;		// ���溽��
	double	longitude;				// ����
	double	latitude;				// γ��
	float	rotRate;				// ת����

	float	pseudorRangeError;		// α���׼�������
	uint8	locationState;			// GPGGA��λ״̬ 0 �޶�λ 1 ���㶨λ 2 ��ֶ�λ 4 RTK�̶��� 5 RTK�����
	uint8	u8_year;	//1
	uint8	u8_month;	//1
	uint8	u8_date;	//1

	uint8	u8_hour;	//1
	uint8	u8_minute;	//1
	uint8	u8_second;	//1
	uint8	u8_second_2;	//��С��

	uint16	u16_speed;	//
	uint16	u16_heading;	//
	uint16  u16_volecityDir;

	int16	i16_rot;	//
	int16	i16_pitch;	//
	int16	i16_roll;	//
	int16	i16_heaving;	//

	uint8	u8_longiSt;	//
	uint8	u8_longiDeg;	//
	uint8	u8_longiMin;	//
	uint8	u8_longiSec;	//
	uint8	u8_longiSecDec;	//
	uint8	u8_longiSecDec2;	//

	uint8	u8_latiSt;	//
	uint8	u8_latiDeg;	//
	uint8	u8_latiMin;	//
	uint8	u8_latiSec;	//
	uint8	u8_latiSecDec;	//��С�� 1 2 λ
	uint8	u8_latiSecDec2;	//��С�� 3 4 λ

	iRTK_STATE rtk_state;
}iRTK_DETELL;	//������λ�ü��ٶȲ���

extern iRTK_Smart_Navigation_Msg irtk_sn_msg;	//iRTKϵͳ��ϸ��Ϣ
extern iRTK_DETELL	irtk_msg;					//��������ڵ���ϵͳ����
extern COMM_SIGN	irtk_sign;					//iRTKͨѶ״̬
extern int8 rtk_locating_fuc;

extern char irtk_udp_addr[30];
extern uint32 irtk_udp_port;
extern int16  irtk_sockid;
extern uint64_t last_rtk_time;
//extern char hdt_msg_buf[128];
void *uartRTKMsgProcess(void *aa);
void rtkCommCal(void *);						//ͳ��ͨѶ��ʱ
extern void irtkCommCalInit(void);
extern void irtkSendDataUDP(int16 sockid, int8* pBuf, int len);
#endif /*__UART_DEAL_INS__H_*/