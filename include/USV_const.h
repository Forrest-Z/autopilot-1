#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h> 


#include <sys/types.h>
#include <sys/stat.h>

#include <sys/types.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>

#include <time.h>

#ifndef	WINNT
#include <dirent.h>
#include <unistd.h>
#include <sys/socket.h>
#include <libsocketcan.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>
#include <sys/time.h>
#include <linux/watchdog.h>
#include <stdbool.h>
#endif

#define VARIABLE_TYPE extern
typedef char    int8;
typedef unsigned char   uint8;
typedef signed   char   sint8;
typedef unsigned short 	uint16;
typedef signed short 	sint16;
typedef signed short 	int16;
typedef unsigned int 	uint32;
typedef signed  int 	sint32;
typedef signed  int 	int32;
typedef unsigned long long uint64;
typedef signed long long int64;
typedef int                     STATUS;  

typedef unsigned long           UNSIGNED;
typedef long                    SIGNED;
typedef unsigned char           DATA_ELEMENT;
typedef DATA_ELEMENT            OPTION;
/* typedef int                     STATUS;  */
typedef unsigned char           UNSIGNED_CHAR;
//typedef char                    CHAR;
typedef int                     INT;
/* typedef void                    VOID; */
typedef unsigned long *         UNSIGNED_PTR;
typedef unsigned char *         BYTE_PTR;
typedef  unsigned char BYTE;
typedef  unsigned short MSG_Q_ID;

#include "monitor.h"

#define 	TRUE 							1
#define		FALSE							0
#define 	_TRUE 							1
#define		_FALSE							0
#define 	LINUX_ERROR 							-1
#define 	READLEN  						1024
#define	MPC_State_Lenght				32		//�������ĳ���
#define	Accelerator_Con_Num			32		//RAM����DSP���ſ���ָ��ĳ���
#define	State_Current_Num				65		//USV����״̬���ĳ���
#define	USV_NUM							0		//������ ���˴����
#define	Dradio_T						0.2		//���ֵ�̨���Ĵ���ʱ����
#define	Filter_a							0.6		//abc�˲�����
#define	Filter_b							0.05	//abc�˲�����
#define	Filter_c							0.005	//abc�˲�����
#define 	DSP_Heartbeat_Buffer			7		//DSP�������ĳ���
#define 	DSP_State_Buffer				16		//DSP״̬���ĳ���
#define 	AIS_Message_Buffer				128	//AIS������󳤶�
#define	INS_Buffer						256	//�ߵ�������󳤶�
#define 	Dradio_Con_Msg					32		//���ֵ�̨����ָ��ĳ���
#define	Dradio_Sal_Msg					3328	//������������󳤶�
#define	Dradio_Cfg_Msg					255	 //���ֵ�̨���ñ��ĳ���
#define	USV_Waypoint_Nummber			255	//������Ϣ������
#define 	Radio_Num						5		//����ͨ������
#define 	Dradio_Num						4		//��������ͨ������

#define SERVER_MAX_CONNECTIONS  		5        	// max clients connected at a time 
#define LAN0_PORT_NUM 					5001    //lan0ͨѶ�˿�
#define LAN1_PORT_NUM 					5002    //lan1ͨѶ�˿�
#define LAN1_PORT_SPARE_NUM				5003	//lan1ͨѶ�˿�	�������232ת��̫��������
#define LAN0_BUFF						255	//MPC������󳤶�
#define LAN1_BUFF						1024	//MPC������󳤶�
#define MPC_AIS_BUFF					128
#define OBS_NUM							10		//�����״��ϰ����ܸ���
#define USV_Safe_Distance				50	//���˴����ñ����㷨�ľ���

#define Rudder_Angle_Max				25		//������ֵ
#define Rudder_Con_Angle_Max			8		//�����ƶ��cxy
#define Rudder_Con_Angle_Min			5		//��С���ƶ��
#define Calculate_Rudder_ks				4		//������ƶ������
#define Correct_Rudder_Max				3		//����������
#define Delay_Time						2		//�涯���ͺ�ʱ��
#define Correct_Rudder_ks				0.9		//������ǵ�������
#define Algorithm_Samp_Time			200		//������ǵ�������
#define Heading_Con_Angle_Max			60		//�����������ֵ
#define TCPA_MIN						30		//��ײʱ������

#define Motor_Detail_Buff_Num			96			//ͼ����̨���������ĳ���
#define Rudder_Detail_Buff_Num			32			//ͼ����̨���ܶ汨�ĳ���
#define Stable_Platform_Detail_Buff_Num	32			//ͼ����̨�ȶ�ƽ̨���ĳ���
#define UAV_Detail_Buff_Num				32			//ͼ����̨���˻�ƽ̨���ĳ���
#define MCU_Buff_Num					32			//ͼ����̨���������ϵͳ���ĳ���
#define Hull_Buff_Num						64			//ͼ����̨���ؼ��ϵͳ���ĳ���
#define AIS_Buff_Num						128		//ͼ����̨AIS���ĳ���
#define Smart_Navigation_Buff_Num		80			//ͼ����̨�๦�ܹߵ����ĳ���
#define Panel_Control_Buff_Num			32			//ͼ����̨���ؿ�����屨�ĳ���
#define Energy_Control_Buff_Num			48			//ͼ����̨��Դ����ϵͳ���ĳ���
#define Power_Control_Buff_Num			32			//ͼ����̨��Դ

#define Motor_Detail_Sign					0			//ͼ����̨���������ı�־
#define Rudder_Detail_Sign				1			//ͼ����̨���ܶ汨�ı�־
#define Stable_Platform_Sign				2			//ͼ����̨�ȶ�ƽ̨���ı�־
#define UAV_Detail_Sign					3			//ͼ����̨���˻�ƽ̨���ı�־
#define MCU_Sign							4			//ͼ����̨���������ϵͳ���ı�־
#define Hull_Sign							5			//ͼ����̨���ؼ��ϵͳ���ı�־
#define AIS_Sign							6			//ͼ����̨AIS���ı�־
#define Smart_Navigation_Sign			7			//ͼ����̨�๦�ܹߵ����ı�־
#define Panel_Control_Sign				11			//ͼ����̨���ؿ�����屨�ı�־
#define Energy_Control_Sign				12			//ͼ����̨��Դ����ϵͳ���ı�־
#define Power_Control_Sign				13			//ͼ����̨��Դ����ϵͳ���ı�־

#define AIS_Collision_Num					40			//AIS��ײ����¼�봬ֻ�������
#define AIS_TCPA_1						600		//AIS����Ļ�����ȫʱ��1
#define AIS_TCPA_2						60			//AIS����Ļ�����ȫʱ��2
#define AIS_TCPA_3						15			//AIS����Ļ�����ȫʱ��3
#define AIS_ShipLength_A					300		//A�ബ����󳤶�300������
#define AIS_ShipLength_B					35			//B�ബ����󳤶�300�����°��������洬			

#define Nmile								1852		//�������׻��㱶��ֵ
#define Pi									3.1415926
#define Earth_Radius						6378137	//����뾶
#define Acc_Coefficient						0.95

#define CAN_ARM_ADD						128		//ARM ECU��CAN��ַ
#define CAN_DSP_ADD						129		//DSP  ECU��CAN��ַ
#define CAN_UAV_ADD						130		//���˻�ƽ̨ ECU��CAN��ַ
#define CAN_SR_ADD						131		//���ܶ� ECU��CAN��ַ
#define CAN_POW_CON_ADD				132		//��Դ���� ECU��CAN��ַ
#define CAN_SP_ADD						133		//������� ECU��CAN��ַ
#define CAN_ST_PL_ADD					134		//�ȶ�ƽ̨ ECU��CAN��ַ
#define CAN_BAT_CON_ADD				135		//��Դ���� ECU��CAN��ַ
#define CAN_Motor_ADD_L					1			//�󷢶���CAN��ַ
#define CAN_Motor_ADD_R					0			//�ҷ�����CAN��ַ
#define CAN_SR_Con_PGN					65280		//���ܶ����PGN
#define CAN_SR_Smart_PGN				65281		//���ܶ溽���ٿ���PGN
#define CAN_SR_Spd_Limit_PGN			65282		//���ܶ���������PGN
#define CAN_SR_Gear_Limit_PGN			65283		//���ܶ浵λ����PGN
#define CAN_SR_Gear_Rudder_PGN			65284		//���ܶ浵λ����PGN
#define CAN_SR_Rudder_PGN				65285		//���ܶ�������PGN
#define CAN_SR_Rudder2_PGN				65286		//���ܶ�������PGN
#define CAN_SR_Rudder3_PGN				65287		//���ܶ�������PGN
#define CAN_UAV_PGN						65290		//���˻�����PGN
#define CAN_POW_PGN					65291		//��Դ��������PGN
#define CAN_ST_PL_PGN					65292		//�ȶ�ƽ̨����PGN
#define CAN_Energy_PGN					65293		//��Դ����ϵͳPGN
#define TP_CM_RTS            					16			//��֡���������ֽ�����
#define TP_CM_BAM						32			//��֡�㲥�����ֽ�����
#define J1939_MAX_MESSAGE_LENGTH          1785		//J1939��֡������������ֽ���

#define Motor_MAX_Speed					3000		//���������ת��
#define Motor_Min_Speed					800		//��������Сת��
#define Gear_Max							180		//���λcxy
#define Gear_Min							0			//��С��λcxy
#define USV_Speed_Max					45			//USV������ƺ���45kn

#define INS_HEADING_COR				0	            /*-3.3�켫�Ž���*/			//�ߵ��������

#define TIANJI_VERSION 1		//�켫�ų���
#define WATER_QUALITY 2			//ˮ�ʴ�����

extern int ship_version;
extern uint16 usv_num;

typedef struct 
{
	uint32	uart0_task_50ms;
	uint32	uart0_task_2s;
	uint32	uart1_task_10s;
	uint32	uart2_task_1s;
	uint32	uart2_task_250ms;
	uint32	uart3_task_100s;
	uint32	uart4_task_1s;
	uint32	uart4_task_250ms;
	uint32	uart5_task_1s;
	uint32	lan0_task_50ms;
	uint32   lan0_task_2s;
	uint32	lan1_task_250ms;
	uint32	lan1_task_1s;

	uint32	CAN_DSP_50ms;
	uint32	CAN_UAV_50ms;
	uint32	CAN_SR_50ms;
	uint32	CAN_POW_50ms;
	uint32	CAN_SP_50ms;
	uint32	CAN_ST_PL_50ms;
	uint32	CAN_BAT_50ms;
	uint32	CAN_Motor_L_10ms;
	uint32	CAN_Motor_R_10ms;
}TASK_TIME_STRUCT;

typedef struct 
{
	uint32 User_ID;					//�û�ʶ����MMSI
	uint32 DCPA;						//����������뵥λ��
	uint16 TCPA;						//��������������ʱ�䵥λs
	float SOG;							//�Եغ���
	float COG;							//�Եغ���
	int Longitude;						//���ȵ�λ�֣�1/10000�֣���Ϊ����Ϊ��
	int Latitude;						//ά�ȵ�λ�֣�1/10000�֣���Ϊ����Ϊ��
	int Longitude_old;					//ǰһʱ�̾��ȵ�λ�֣�1/10000�֣���Ϊ����Ϊ��
	int Latitude_old;					//ǰһʱ��ά�ȵ�λ�֣�1/10000�֣���Ϊ����Ϊ��
	uint32	UTC_Time;					//������Ϣʱ�䵥λ��
	uint8	UTC_Time_2;				//���С�������λ
	uint8	UTC_Time_old_2;			//���С�������λ
	uint32	UTC_Time_old;				//��һ�θ�����Ϣʱ�䵥λ��
	uint16 Ship_Lenght;					//����
	uint32 Safe_DCPA;					//��ȫ��������
}Check_Collision;


//�����״��ϰ��ﱨ�Ľṹ��
typedef struct  
{
	uint32	OBS_Distance;				//�ϰ����뱾���ľ���
	uint16	OBS_Direction;				//�ϰ�����Ա����ķ�λ
	uint8		OBS_Speed;					//�ϰ���ĶԵ��ٶ�
	uint16	OBS_Heading;				//�ϰ���Եغ���
	uint16	OBS_Length;				//�ϰ��ﳤ��
}D_radar_Msg;
//����վ��Ϣ�ṹ��

//����Ϣ�ṹ��
typedef struct  
{
	uint16	Wind_Direction;				//����
	uint16	Wind_Speed;				//����
	uint8	Wind_Update;				//����Ϣ����ʱ��
}Wind;
//������ѹʪ�Ƚṹ��
typedef struct  
{
	uint16	Temperature;				//����
	uint16	Atmospheric;				//��ѹ
	uint16	Humidity;					//ʪ��
	uint8	Wheather_Updata;			//���¡���ѹ��ʪ�ȸ���ʱ��
}Wheather;
//��ˮ��Ϣ�ṹ��
typedef struct  
{
	uint32	Rainfall;					//������
	uint16	Rainfall_time;				//����ʱ��
	uint16	Rainfall_Intensity;			//����ǿ��
	uint16	Hail;						//����
	uint16	Hail_Time;					//����ʱ��
	uint16	Hail_Intensity;				//����ǿ��
	uint8	Precipitation_Updata;			//��ˮ����ʱ��
}Precipitation;
//MPC��Դ�ṹ��
typedef struct  
{
	uint8	MPC_Disk;						//MPCӲ��ʹ�ðٷֱ�	
	uint8	MPC_Memory;					//MPC�ڴ�ʹ�ðٷֱ�	
	uint8	MCP_CPU;						//MPC CPUʹ�ðٷֱ�	
}MPC_Msg;

typedef struct  
{
	Wind		Wind_Msg;				//����Ϣ
	Wheather	Wheather_Msg;			//���¡���ѹ��ʪ����Ϣ
	Precipitation Precipitation_Msg;	//��ˮ��Ϣ
	uint8		Sensor_Add;			//��������ַ
	uint16		Heating_TEMP;		//�����¶�
	uint16		Heating_VOL;		//���ȵ�ѹ
}Meteorological_Msg;
//MPC����RAM���ڱ��Ľṹ��

//����,����Ϊ��λ������100����ȡ���õ�32λ�з�������
typedef struct {
	int32	lng;			//����	
	int32	lat;			//γ��	
}COORDINATE_STRUCT;

typedef struct {	
	COORDINATE_STRUCT	obstacle_locate;	//����
	float				obstacle_speed;				//�ٶ�
	float				obstacle_direction;
	float				obstacle_radius;		//��ȫ�뾶
}OBSTACLE_LOCATE_INF_STRUCT;


////MPC�����ϰ��ﱨ�Ľṹ��
typedef struct  
{
    uint16	Msg_Num;										//���ı��
    uint8	Obstacles_Num;									//�ϰ������
	//D_radar_Msg	RM_radar_Msg[OBS_NUM];					//�����״��ϰ���
	OBSTACLE_LOCATE_INF_STRUCT	RM_radar_Msg[OBS_NUM];		//�״��ϰ�����Ϣ
	Meteorological_Msg RM_Meteorological_Msg;				//����վ��Ϣ
	MPC_Msg MPC_Msg_St;										//MPC��Դ��Ϣ
	uint8 MPC_Heatbeat;										//����
	uint8 MPC_radar_valid;									//MPC�״�ͼ����Ч
}LAN0_RM_Message;

//ͼ����̨������״̬�ṹ��
typedef struct  
{
	//״̬
	uint16 Motor_Spd_L;				//�󷢶���ת��
	uint16 Motor_Spd_R;				//�ҷ�����ת��
	uint8	Cool_Temp_L;				//�󷢶�����ȴҺ�¶�
	uint8	Cool_Temp_R;				//�ҷ�������ȴҺ�¶�
	uint8	Cool_Lev_L;					//�󷢶�����ȴҺҺλ
	uint8	Cool_Lev_R;					//�ҷ�������ȴҺҺλ
	uint16 Oil_Temp_L;				//�󷢶��������¶�
	uint16 Oil_Temp_R;				//�ҷ����������¶�
	uint8	Fuel_Temp_L;				//�󷢶���ȼ���¶�
	uint8	Fuel_Temp_R;				//�ҷ�����ȼ���¶�
	uint8	Inlet_Air_Temp_L;			//�󷢶�����������¶�
	uint8	Inlet_Air_Temp_R;			//�ҷ�������������¶�
	uint8	Oil_Pressure_L;				//�󷢶�������ѹ��
	uint8	Oil_Pressure_R;				//�ҷ���������ѹ��
	uint8	Supercharger_Pressure_L;	//�󷢶�����ѹ��ѹ��
	uint8	Supercharger_Pressure_R;	//�ҷ�������ѹ��ѹ��
	uint16 Fuel_Usage_L;				//�󷢶���ȼ��ʹ����
	uint16 Fuel_Usage_R;				//�ҷ�����ȼ��ʹ����
	uint16 Average_Fuel_Usage_L;		//�󷢶���ƽ��ȼ��Ч��
	uint16 Average_Fuel_Usage_R;		//�ҷ�����ƽ��ȼ��Ч��
	uint32 Working_Time_L;			//�󷢶����ܹ���ʱ��
	uint32 Working_Time_R;			//�ҷ������ܹ���ʱ��
	uint32 Total_turn_L;				//�󷢶�����ת��
	uint32 Total_turn_R;				//�ҷ�������ת��
	uint8	Air_preeure_L;				//����ѹ����ȡ�󷢶�����������
	uint16 Single_Mileage;				//�������N
	uint32 Total_Mileage;				//�����N
	uint16 Single_Fuel_Consume;		//����ȼ������
	uint32 Total_Fuel_Consume;		//��ȼ������
	uint16 Ldle_Spd_L;				//�󷢶�������ת��N
	uint16 Ldle_Spd_R;				//�ҷ���������ת��
	uint8	Ldle_Time_Left;				//�󷢶�������ʱ��
	uint8	Ldle_Time_Right;			//�ҷ���������ʱ��
	//�澯
	uint8	SuperLoad_Alarm_Left;				//�󷢶��������ؼ�ת���쳣
	uint8	SuperLoad_Alarm_Right;				//�ҷ����������ؼ�ת���쳣
	uint8	ECU_Alarm_L;						//�󷢶���ECU�쳣
	uint8	ECU_Alarm_R;						//�ҷ�����ECU�쳣
	uint8	Cooling_TEMP_Alarm_Left;			//�󷢶�����ȴҺ�¶��쳣
	uint8	Cooling_TEMP_Alarm_Right;			//�ҷ�������ȴҺ�¶��쳣
	uint8	Cooling_Level_Alarm_Left;			//�󷢶�����ȴҺҺλ�쳣
	uint8	Cooling_Level_Alarm_Right;			//�ҷ�������ȴҺҺλ�쳣
	uint8	OilPressure_Alarm_Left;				//�󷢶�������ѹ���쳣
	uint8	OilPressure_Alarm_Right;			//�ҷ���������ѹ���쳣	
	uint8	Oil_TEMP_Alarm_L;					//�󷢶��������¶ȹ���
	uint8	Oil_TEMP_Alarm_R;					//�ҷ����������¶ȹ���
	uint8	Supercharger_Pressure_Alarm_L;	//�󷢶����������ѹ���쳣
	uint8	Supercharger_Pressure_Alarm_R;	//�ҷ������������ѹ���쳣
	uint8	InletTEMP_Alarm_Left;				//�󷢶�����������¶��쳣
	uint8	InletTEMP_Alarm_Right;				//�ҷ�������������¶��쳣
	uint8	Fuel_Pressure_Alarm_L;				//�󷢶���ȼ��ѹ���쳣
	uint8	Fuel_Pressure_Alarm_R;				//�ҷ�����ȼ��ѹ���쳣
	uint8	Fuel_Moisture_Alarm_L;				//�󷢶���ȼ��ˮ�ֹ���
	uint8	Fuel_Moisture_Alarm_R;				//�ҷ�����ȼ��ˮ�ֹ���
	uint8	Nozzle_Pressure_L;					//�󷢶�������ѹ���쳣
	uint8	Nozzle_Pressure_R;					//�ҷ���������ѹ���쳣
	uint8	Electricity_Alarm_L;					//�󷢶�����ص�����
	uint8	Electricity_Alarm_R;					//�ҷ�������ص�����
	uint8	Whole_Alarm_L;						//�󷢶��������쳣
	uint8	Whole_Alarm_R;						//�ҷ����������쳣
	uint8	Heatbeat_L;							//�󷢶�������
	uint8	Heatbeat_R;							//�ҷ�����������RAM�ж������쳣����0
}Motor_Detail_State;

//���ܶ�״̬�ṹ��
typedef struct  
{
	uint16	Rudder_Angle_Left_St;			//����ʵ��ֵ(���������ֵ����125��Ӧ0��)	
	uint16	Rudder_Angle_Right_St;			//�Ҷ��ʵ��ֵ		
	uint16	Rudder_Angle_Left_Con;			//��������ֵ(���������ֵ����125��Ӧ0��)	
	uint16	Rudder_Angle_Right_Con;		//�Ҷ������ֵ		
	uint8		Speed_Limit;			    		//����ֵ
       uint16	Motor_Speed_L;					//������ת��L
       uint16	Motor_Speed_R;					//������ת��R
	uint8   	System_Power_5;				//��Դ5V״̬
	uint8   	System_Power_12;				//��Դ12V״̬
	uint8   	System_Power_3;				//��Դ3.3V״̬
	uint8   	System_Power_24;				//��Դ24V״̬	
	uint8   	System_Check;					//ϵͳ�Լ�
	uint8		System_TEMP;					//ϵͳ�¶�
	uint8		Heatbeat_Rudder;				//���ܶ�ϵͳ����
}Rudder_Detail_State;

//�ȶ�ƽ̨״̬�ṹ��
typedef struct  
{
       uint8	Stable_Platform_RL_St;			//�ȶ�ƽ̨����ʵ����б�Ƕ�
       uint8	Stable_Platform_AT_St;			//�ȶ�ƽ̨ǰ��ʵ����б�Ƕ�
	uint8	System_TEMP;					//ϵͳ�¶�
	uint8   System_Power_5;				//��Դ5V״̬
	uint8   System_Power_12;				//��Դ12V״̬
	uint8   System_Power_3;				//��Դ3.3V״̬
	uint8   System_Power_24;				//��Դ24V״̬	
	uint8   System_Check;					//ϵͳ�Լ�
	uint8	Stable_Platform_Heatbeat;		//�ȶ�ƽ̨ϵͳ����
}Stable_Platform_Detail_State;

//���˻�ƽ̨״̬�ṹ��
typedef struct  
{
       uint8	Platform_Hatch_St;				//���˻�ƽ̨����״̬
       uint8	Platform_Lift_St;				//���˻�ƽ̨����״̬
       uint8	Platform_Open_St;				//���˻�ƽ̨���̿���״̬
       uint8	UAV_Electricity;					//���˻���ص���
       uint8	UAV_Con_Mod;					//���˻�����ģʽ
       uint8	UAV_Run_Mod;					//���˻�����ģʽ
       uint8	UAV_Charging_St;				//���˻����״̬
       uint8	UAV_Heatbeat;					//���˻�ƽ̨����
	uint8	UAV_5V_St;   					//���˻�5v��Դ״̬
	uint8	UAV_3V_St;   					//���˻�3.3v��Դ״̬
	uint8	UAV_12V_St;   					//���˻�12v��Դ״̬
	uint8	UAV_24V_St;   					//���˻�24v��Դ״̬	
	uint8	UAV_Check_St;   				//���˻��Լ�״̬
	uint8	UAV_TEMP;   					//���˻�ϵͳ�¶�

}UAV_Detail_State;

//���������״̬
typedef struct  
{
       uint8	MCU_TEMP_1;					//�������¶ȼ���1�¶�ֵN
       uint8	MCU_TEMP_2;					//�������¶ȼ���2�¶�ֵN
	uint8	MCU_TEMP_3;					//�������¶ȼ���3�¶�ֵN
	uint8	RAM_Memory;					//RAM�ڴ�ʹ�ðٷֱ�N
	uint8	RAM_CPU;						//RAM CPUʹ�ðٷֱ�N
	uint8	MPC_Disk;						//MPCӲ��ʹ�ðٷֱ�
	uint8	MPC_Memory;					//MPC�ڴ�ʹ�ðٷֱ�
	uint8	MCP_CPU;						//MPC CPUʹ�ðٷֱ�
	uint8	Mesh_Bandwidth;				//������̨ʵʱ����N
}MCU_State;

//������״̬
typedef struct  
{
       uint8	Hull_TEMP_1;					//�����¶ȼ���1�¶�ֵ
       uint8	Hull_TEMP_2;					//�����¶ȼ���2�¶�ֵ
	uint8	Hull_TEMP_3;					//�����¶ȼ���3�¶�ֵ
	uint8	Outfire_St;						//�������ϵͳ״̬
	uint8	Ventilation_St;					//���崬��ͨ��ϵͳ״̬
	uint8	Water_Pump_St;					//���崬��ˮ��״̬
	uint8	Water_Level_St;					//���崬��ˮλ�澯״̬
	uint8	Motor_Vibration_1;				//�󷢶�������������ֵ
	uint8	Motor_Vibration_2;				//�󷢶�������������ֵ
	uint8	Motor_Vibration_3;				//�󷢶�������������ֵ
	uint8	Motor_Vibration_4;				//�ҷ���������������ֵ
	uint8	Motor_Vibration_5;				//�ҷ���������������ֵ
	uint8	Motor_Vibration_6;				//�ҷ���������������ֵ
}Hull_State;

//AIS״̬�ṹ��
typedef struct  
{
       char	AIS_Message[AIS_Message_Buffer];//AIS����
       uint8	AIS_Heatbeat;					//AIS�������
}AIS_Msg;

//ͼ����̨���͹ߵ�ϵͳ��ϸ��Ϣ


//�������ܿ������״̬�ṹ��
typedef struct  
{
       uint8	Panel_Control_St;				//��������ȡϵͳ����Ȩ��
       uint8	Panel_Control_light;				
}Panel_Control_Msg;
//��Դ����ϵͳ״̬�ṹ��
typedef struct  
{
	uint8	Battery_Left;					//USV����ʣ������ٷֱ�
	uint8	Battery_TEMP_Left;				//USV�����¶�
	uint8	Battery_Power_Left;				//USV����˲ʱ����
	uint8	Battery_Current_Left;			//USV����˲ʱ����
	uint8	Battery_Voltage_Left;			//USV����˲ʱ��ѹ
	uint8	Battery_Right;					//USV�ҵ��ʣ������ٷֱ�
	uint8	Battery_TEMP_Right;				//USV�ҵ���¶�
	uint8	Battery_Power_Right;			//USV�ҵ��˲ʱ����
	uint8	Battery_Current_Right;			//USV�ҵ��˲ʱ����
	uint8	Battery_Voltage_Right;			//USV�ҵ��˲ʱ��ѹ
	uint8	Battery_Charge_Left;			//USV���س��״̬
	uint8	Battery_Chager_Right;			//USV�ҵ�س��״̬
	uint8	Battery_TEMP_Alarm_Left;		//USV�����¶ȸ澯
	uint8	Battery_TEMP_ALarm_Right;		//USV�ҵ���¶ȸ澯
	uint8	Battery_Capacity_Left;			//USV���������澯
	uint8	Battery_Capacity_Right;			//USV�ҵ�������澯
	uint8	Power_Control_Heatbeat;		//��Դ����ϵͳ����
	uint8	SYS_DIN1_St;					//����1����״̬
	uint8	SYS_DIN2_St;					//����2����״̬
	uint8	SYS_DIN3_St;					//����3����״̬
	uint8	SYS_DIN4_St;					//����4����״̬
	uint8	SYS_DIN5_St;					//����5����״̬
	uint8	SYS_DIN6_St;					//����6����״̬
	uint8	SYS_DIN7_St;					//����7����״̬
	uint8	SYS_DIN8_St;					//����8����״̬
	uint8	SYS_5V_St;						//ϵͳ5v��Դ״̬
	uint8	SYS_12V_St;					//ϵͳ12v��Դ״̬
	uint8	SYS_3V_St;						//ϵͳ3.3v��Դ״̬
	uint8	SYS_24V_St;					//ϵͳ24v��Դ״̬
	uint8	SYS_Check_St;					//ϵͳ�Լ�״̬
	uint8	SYS_TEMP;						//ϵͳ�¶�

}Energy_Control_Msg;

//��Դ����ϵͳ״̬�ṹ��
typedef struct  
{
       uint8	Stable_Platform_Power;		//�ȶ�ƽ̨��Դ
       uint8	UAV_Power;					//���˻�ƽ̨��Դ
	uint8	Horn_Power;				//���ȵ�Դ
	uint8	Navigationlight_Power;		//���еƵ�Դ
	uint8	D_radar_Power;				//�����״��Դ
	uint8	Camera_main_Power;		//������ͷ��Դ
	uint8	Application_24V;			//24VӦ���豸��Դ
	uint8	Searchlight_Power;			//̽�յƵ�Դ

	uint8	Camera_ahead_Power;		//ǰ����ͷ��Դ
	uint8	Camera_lesser_Power;		//������ͷ��Դ
	uint8	Camera_tail_Power;			//β����ͷ��Դ
	uint8	Application_12V;			//12VӦ���豸��Դ

	uint8   System_Power_5;			//��Դ5V״̬
	uint8   System_Power_12;			//��Դ12V״̬
	uint8   System_Power_3;			//��Դ3.3V״̬
	uint8   System_Check;				//ϵͳ�Լ�
	uint8   System_Voltage;			//ϵͳ��ѹ
	uint8   System_Electricity;			//ϵͳ����
	uint8	DIN1_Connection_St;		//����1����״̬
	uint8	DIN2_Connection_St;		//����2����״̬
	uint8	DIN3_Connection_St;		//����3����״̬
	uint8	DIN4_Connection_St;		//����4����״̬
	uint8	System_Heatbeat;			//��Դϵͳ����
	uint8	System_TEMP;				//ϵͳ�¶�

}Power_Control_Msg;

//�������ַ���ת��
typedef  union
{
	float 	ftemp;
	uint8	ctemp[4];

}floatchange;	

//USV�豸��Դ�ṹ��
typedef struct 
{
       uint8	UAV_Power;				//���˻�ƽ̨��Դ
       uint8	Stable_Platform_Power;	//�ȶ�ƽ̨��Դ
	uint8	Camera_ahead_Power;	//ǰ����ͷ��Դ
	uint8	D_radar_Power;			//�����״��Դ
	uint8	Camera_main_Power;	//������ͷ��Դ
	uint8	Horn_Power;			//���ȵ�Դ
	uint8	Searchlight_Power;		//̽�յƵ�Դ
	uint8	Camera_lesser_Power;	//������ͷ��Դ
	uint8	Navigationlight_Power;	//���еƵ�Դ
	uint8	Camera_tail_Power;		//β����ͷ��Դ	   
}USV_Device_Power;

//USVģʽ�ṹ��
typedef struct 
{
       uint8	Sailing_Mod;				//����ģʽ
       uint8	Differential_Mod;			//����ģʽ
	uint8	Set_Return_Point_Mod;		//���÷�����
	uint8	Speed_Constant_Mod;		//����ģʽ
	uint8	Direction_Constant_Mod;		//����ģʽ
	uint8	Return_Mod;				//����ģʽ	 0--����Ŀ�ĵ�   1һ������cxy   2--
}USV_model;

//USV�������ƽṹ��
typedef struct 
{
       uint8	Gear_Left;					//�󷢶�����λ
       uint8	Gear_Right;					//�ҷ�������λ
	uint8	Accelerator_Left;			//�󷢶�������
	uint8	Accelerator_Right;			//�ҷ���������
	uint8	Rudder_Angle_Left;			//����(���������ֵ����125��Ӧ0��)
	uint8	Rudder_Angle_Right;			//�Ҷ��	   
}USV_Drive;

//�ȶ�ƽ̨���ƽṹ��
typedef struct
{
       uint8	Stable_Platform_RL;			//�ȶ�ƽ̨������б�Ƕ�
       uint8	Stable_Platform_AT;			//�ȶ�ƽ̨ǰ����б�Ƕ�
}USV_Stable_Platform;

//���˻����ƽṹ��
typedef struct  
{
       uint8	Platform_Hatch;				//���˻�ƽ̨���ſ���
       uint8	Platform_Lift;				//���˻�ƽ̨��������
       uint8	Platform_Open;				//���˻�ƽ̨���̿��Ͽ���
       uint8	UAV_Charging;				//���˻�������
}USV_UAV_Control;

//ƽ����תģʽָ��
typedef struct
{
	uint8   translation_mode;				//ƽ��ģʽ 0--�޶��� 1--ƽ��ģʽ
	uint8	translation_command;			//00--ֹͣ  01--��ƽ��  10--��ƽ��
	uint8   rotation_mode;					//��תģʽ 0--��     1--��תģʽ
	uint8	rotation_command;				//00--ֹͣ  01--��ʱ��  10--˳ʱ��
}TRANS_ROT_CMD;


//���ֵ�̨����ָ��ṹ�� (������̨��
typedef struct  
{
	uint8	USV_Num;											//���˴����
       uint16	Msg_Num;											//���ı��
	USV_Device_Power	Dradio_USV_Device_Power;			//USV�豸��Դ
	uint8	Navigation_Tsak_sign;								//��������������־
	uint8	Engine_Run_L;										//��������Դָ��
	uint8	Engine_Run_R;											//����������ָ��
	USV_model	Dradio_USV_Model;							//USVģʽ
	USV_Drive	Dradio_USV_Drive;							//USV����ϵͳ
	uint8	Speed_Limit;			    							//����ֵ
	USV_Stable_Platform	Dradio_USV_Stable_Platform;	//�ȶ�ƽ̨
	USV_UAV_Control		Dradio_USV_UAV_Control;		//���˻�ƽ̨
	uint8 Hull_Fan_Con;										//������
	uint8 Hull_Pump_Con;										//����ˮ��
	uint8 Oil_Con;												//��·����
//	uint8 Sealight_LR_Con;										//̽�յ�������ת�Ƕ�
//	uint8 Sealignt_HT_Con;										//̽�յ�������ת�Ƕ�
	uint8 Application_24;										//24VӦ���豸��Դָ��
	uint8 Application_12;										//12VӦ���豸��Դָ��
	uint8 Sealight_Speed;										//̽�յ���ת�ٶȣ�Ĭ�Ͽ�0
	uint8 Sealight_Direction;									//̽�յ���ת�������
	uint8 Emergency_Stop;										//��ͣ
	uint8 Outboard_Engine_L;									//�����������ָ��
	uint8 Outboard_Engine_R;									//�����������ָ��
	TRANS_ROT_CMD	trans_rot_cmd;								//ƽ����ת����
}Dradio_Control_Message;

//������Ϣ��Դ
typedef struct  
{
	uint8	Radio_Nummber;					//����ͨ����־��0--���ֵ�̨  1--����  2--����    3--���� 4--��������������
	Dradio_Control_Message	USV_Control_Message[Radio_Num];			//���˴��Ŀ�����Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ 4--��������������
}Control_Message;

//������Ϣ�ṹ��
typedef struct  
{
	uint8	Waypoint_Latitude_Sign;				//���㾭�ȱ�־��0����1����
	uint8	Waypoint_Longitude_Sign;			//����ά�ȱ�־��0��γ1��γ
	uint8	Waypoint_Latitude_Degree;			//������Ϣ����ֵ��
	uint8	Waypoint_Latitude_Minute;			//������Ϣ����ֵ��
	uint8	Waypoint_Latitude_Second;			//������Ϣ����ֵ��
	uint8	Waypoint_Latitude_Decimal;			//������Ϣ����ֵ��С��λ
	uint8	Waypoint_Longitude_Degree;			//������Ϣά��ֵ��
	uint8	Waypoint_Longitude_Minute;			//������Ϣά��ֵ��
	uint8	Waypoint_Longitude_Second;			//������Ϣά��ֵ��
	uint8	Waypoint_Longitude_Decimal;			//������Ϣά��ֵ��С��λ
	uint16	Waypoint_Stop_Time;					//����ͣ��ʱ��
	uint16	Waypoint_speed;						//��������֮��ĺ����ٶ�
	uint8	Waypoint_Type;						//��������	0:��������	1����ʱ����(������)
}Waypoint;
//�����������ݽṹ��
typedef struct  
{
	uint8	USV_Num;				//���˴����
	uint8	Waypoint_Nummber;		//��������
	Waypoint		Dradio_Waypoint[USV_Waypoint_Nummber];//������Ϣ ��������255������ 0--��ʼ����
	uint8	Sailing_Nummber;		//����������
}Dradio_Sailing_Message;
//��������ṹ��
typedef struct  
{
	uint8	Radio_Nummber;					//����ͨ����־��0���ֵ�̨    1����    2����  3����
	Dradio_Sailing_Message	USV_Sailing_Message[Dradio_Num];	//�ߵ�ϵͳ����Ϣ  Dradio_Num=3 ��������ͨ������
}Sailing_Message;
//����������Ϣ�ṹ��
typedef struct  
{
	uint8	Hull_Type;				//��������
	uint8	Drive_Type;				//��������
	uint8	Propeller_Type;			//�ƽ�������
	uint8	Motor_Type;				//����������
	uint8	Drive_Num;				//����ϵͳ��Ŀ
}Basic_Config;
//����Ϣ�ṹ��
typedef struct  
{
	uint32	MMSI;				//MMSI���
	uint16	Board_length;		//�ͳ�
	uint16	Board_Wide;		//�Ϳ�
	uint16	Board_Deep;		//����
	uint16	Max_Static_Draft;	//���̬��ˮ
	uint8		Max_Speed;			//�����
	uint16	Max_Voyage;		//��󺽳�
	uint16 	INS_Antenna_Distance;//�ߵ����߾���
}Board_Config;
//�������ýṹ��
typedef struct  
{
	uint8	Motor_Speed_Max_L;				//�󷢶������ת��
	uint8	Motor_Speed_Idle_L;				//�󷢶�������ת��
	uint8	Motor_Speed_Max_R;				//�ҷ��������ת��
	uint8	Motor_Speed_Idle_R;				//�ҷ���������ת��
	uint8	Motor_Trip_L;						//�󷢶����г�
	uint8	Motor_Trip_R;						//�ҷ������г�
	uint8	Motor_Address_L;					//�󷢶�����ַ
	uint8	Motor_Address_R;					//�ҷ�������ַ
	uint8	Motor_Direction_L;					//�����ŷ���
	uint8	Motor_Direction_R;					//�����ŷ���
	uint8	Motor_Sign;							//���Ų����仯��־
	uint8	Motor_Fire_Time;					
}Throttle_Config;
//��λ���ýṹ��
typedef struct  
{
	uint8 Gear_Forward_Travel_L;				//��λǰ�����г�
	uint8 Gear_Forward_Travel_R;				//�ҵ�λǰ�����г�
	uint8 Gear_Back_Travel_L;					//��λǰ�����г�
	uint8 Gear_Back_Travel_R;					//�ҵ�λǰ�����г�
	uint8 Gear_Forward_Angle_L;				//��λǰ�����Ƕ�����
	uint8 Gear_Forward_Angle_R;				//�ҵ�λǰ�����Ƕ�����
	uint8 Gear_Back_Angle_L;					//��λ���˵��Ƕ�����
	uint8 Gear_Back_Angle_R;					//�ҵ�λ���˵��Ƕ�����
	uint8 Gear_Neutral_Angle_L;				//��λ�յ��Ƕ�
	uint8 Gear_Neutral_Angle_R;				//�ҵ�λ�յ��Ƕ�
	uint8 Gear_Direction_L;					//��λ����
	uint8 Gear_Direction_R;					//�ҵ�λ����
	uint8 Gear_Sign;							//��λ�����仯��־
}Gear_Config;
//������ýṹ��
typedef struct  
{
	uint8 Rudder_Angle_Max_L;				//������Ƕ�
	uint8 Rudder_Angle_Max_R;				//�Ҷ����Ƕ�
	uint8 Rudder_Left_Limit_Resistance_L;		//������޵���ֵ
	uint8 Rudder_Left_Limit_Resistance_R;		//�Ҷ����޵���ֵ
	uint8 Rudder_Middle_Limit_Resistance_L;	//�����λ����ֵ
	uint8 Rudder_Middle_Limit_Resistance_R;	//�Ҷ���λ����ֵ
	uint8 Rudder_Right_Limit_Resistance_L;		//����Ҽ��޵���ֵ
	uint8 Rudder_Right_Limit_Resistance_R;		//�Ҷ��Ҽ��޵���ֵ
	uint8 Rudder_Control_Angle_Max_L;		//��������ƶ��
	uint8 Rudder_Control_Angle_Max_R;		//�Ҷ������ƶ��
	uint8 Rudder_Control_Angle_Min_L;			//�����С���ƶ��
	uint8 Rudder_Control_Angle_Min_R;			//�Ҷ���С���ƶ��
	uint8 Rudder_Direction_L;					//���Ƿ���
	uint8 Rudder_Direction_R;					//�Ҷ�Ƿ���
	uint8 Rudder_Sign;							//��ǲ����仯��־
	uint8 Rudder_Control_Accuracy_L;			//�����ƾ���
	uint8 Rudder_Control_Accuracy_R;			//�Ҷ���ƾ���
	uint8 Rudder_Left_Travel_L;				//�����ת�г�
	uint8 Rudder_Left_Travel_R;				//�����ת�г�
	uint8 Rudder_Right_Travel_L;				//�Ҷ���ת�г�
	uint8 Rudder_Right_Travel_R;				//�Ҷ���ת�г�
	
}Rudder_Config;
//�������ýṹ��
typedef struct  
{
	uint8 Control_Type;						//��������㷨���� 0--PID�㷨��1-- ����S���㷨 
	uint32 PID_P;								//P
	uint32 PID_I;								//I
	uint32 PID_D;								//D
	uint8 S_K1;								//k1
	uint8 S_K2;								//k2
	uint8 S_K3;								//k3
	uint8 Filter_A;								//a
	uint8 Filter_B;								//b
	uint8 Filter_C;								//c
	uint8 Algorithm_Sign;						//�㷨�����仯��־
}Algorithm_Config;
//�������ýṹ��
typedef struct  
{
	uint8 Wind_Speed;							//����
	uint8 Wave_Hight;							//�˸�
	uint8 Flow_Speed;							//����
	uint8 Environment_Sign;					//���������仯��־
}Environment_Config;
//������Ϣ�ṹ��
typedef struct  
{
	uint8	USV_Num;							//���˴����
	uint8	Config_Direction;					//���ö�д��־
	uint8	Config_Num;						//���ñ��
	Basic_Config Basic_Cfg;					//��������
	Board_Config Board_Cfg;					//����Ϣ
	Throttle_Config Throttle_Cfg;			//��������
	Gear_Config	Gear_Cfg;					//��λ����	
	Rudder_Config Rudder_Cfg;				//�������	
	Algorithm_Config Algorithm_Cfg[2];	//�㷨����
	Environment_Config Environment_Cfg;	//��������
}Dradio_Config_Message;

//DSP״̬�ṹ��
typedef struct  
{
	uint8	Engine_power_st;					//��������Դ״̬
	uint8	Engine_run_st;						//����������״̬
	uint8	USV_oil_st;							//��������·״̬
	uint8	Hull_Fan_st;						//���״̬
	uint8	Outfire_St;							//��״̬
	uint8	Hull_Pump_st;						//ˮ��״̬
	uint8	DSP_Heatbeat;						//����
}DSP_State;


//���˴�����״̬�ṹ��
typedef struct
{
	uint16	USV_Speed;				//�ܺ���
       uint16	USV_Heading;			//����
       uint16	USV_Pitch;				//����
       uint16 USV_Roll;				//���
       uint16 USV_Heave;				//����       
}USV_Sailing_State;

//USVģʽ״̬�ṹ��
typedef struct  
{
	uint8	Latitude_Sign_St;					//���˴�λ�þ��ȱ�־	1--����   2--����
	uint8	Longitude_Sign_St;					//���˴�λ��ά�ȱ�־	1--������ ��2--�ϰ���
       uint8	Sailing_Model_St;					//����ģʽ
       uint8	Differential_Model_St;				//����ģʽ
	uint8	Set_Return_Point_Model_St;			//���÷�����
	uint8	Speed_Constant_Model_St;			//����ģʽ
	uint8	Direction_Constant_Model_St;		//����ģʽ
	uint8	Panel_Control_Sign_ST;				//����������ģʽ
	uint8	Emergency_St;						//Ӧ��ģʽ
	uint8   translation_mode_St;					//ƽ��ģʽ 0--�޶��� 1--ƽ��ģʽ
	uint8	translation_command_St;				//00--ֹͣ  01--��ƽ��  10--��ƽ��
	uint8   rotation_mode_St;						//��תģʽ 0--��     1--��תģʽ
	uint8	rotation_command_St;					//00--ֹͣ  01--��ʱ��  10--˳ʱ��
}USV_Model_State;

//���˴�λ����Ϣ�ṹ��
typedef struct 
{
	uint8	USV_Latitude_Degree;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Minute;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Second;			//USVλ�þ���ֵ��
	uint8	USV_Latitude_Decimal;			//USVλ�þ���ֵ��С��λ
	uint8	USV_Longitude_Degree;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Minute;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Second;			//USVλ��ά��ֵ��
	uint8	USV_Longitude_Decimal;			//USVλ��ά��ֵ��С��λ
}USV_Location;

//USV��������״̬�ṹ��
typedef struct 
{
	uint16	Accelerator_Left_St;					//�󷢶���ת��
	uint16	Accelerator_Right_St;					//�ҷ�����ת��
       uint8	Gear_Left_St;					//�󷢶�����λ
       uint8	Gear_Right_St;					//�ҷ�������λ
	uint8	Rudder_Angle_Left_St;			//����(���������ֵ����125��Ӧ0��)
	uint8	Rudder_Angle_Right_St;			//�Ҷ��	   
}USV_Drive_State;

//USV���״̬�ṹ��
typedef struct 
{
	uint8	Battery_Left;					//USV����ʣ������ٷֱ�
	uint8	Battery_Right;					//USV�ҵ��ʣ������ٷֱ�
	uint8	Battery_TEMP_Left;				//USV�����¶�
	uint8	Battery_TEMP_Right;				//USV�ҵ���¶�
}USV_Battery;

//USV�豸��Դ״̬�ṹ��
typedef struct
{
       uint8	UAV_Power_St;				//���˻�ƽ̨��Դ
       uint8	Stable_Platform_Power_St;	//�ȶ�ƽ̨��Դ
	uint8	Camera_ahead_Power_St;	//ǰ����ͷ��Դ
	uint8	D_radar_Power_St;			//�����״��Դ
	uint8	Camera_main_Power_St;		//������ͷ��Դ
	uint8	Horn_Power_St;				//���ȵ�Դ
	uint8	Searchlight_Power_St;		//̽�յƵ�Դ
	uint8	Camera_lesser_Power_St;	//������ͷ��Դ
	uint8	Navigationlight_Power_St;	//���еƵ�Դ
	uint8	Camera_tail_Power_St;		//β����ͷ��Դ	   
}USV_Device_State;

//USV��ظ澯״̬�ṹ��
typedef struct  
{
	uint8	Battery_Left_Charging;			//USV���س��״̬
	uint8	Battery_Right_Charging;			//USV�ҵ�س��״̬
	uint8	Battery_TEMP_Alarm_L;			//USV�����¶ȸ澯
	uint8	Battery_TEMP_Alarm_R;			//USV�ҵ���¶ȸ澯
}USV_Battery_Alarm;

//USV���������ṹ��
typedef struct  
{
	uint8	Outfire_St;						//USV���ϵͳ״̬
	uint8	Water_Level_St;					//USV����ˮλ�澯״̬
	uint8	Ventilation_St;					//USV����ͨ��ϵͳ״̬
}USV_Fire;

//USV������������Ϣ�ṹ��
typedef struct  
{
	uint8	SuperLoad_Alarm_Left;				//USV�󷢶���������
	uint8	SuperLoad_Alarm_Right;				//USV�ҷ�����������
	uint8	InletPressure_Alarm_Left;			//USV�󷢶�������ѹ���쳣
	uint8	InletPressure_Alarm_Right;			//USV�ҷ���������ѹ���쳣
	uint8	InletTEMP_Alarm_Left;				//USV�󷢶��������¶��쳣
	uint8	InletTEMP_Alarm_Right;				//USV�ҷ����������¶��쳣
	uint8	Cooling_TEMP_Alarm_Left;			//USV�󷢶�����ȴҺ�¶��쳣
	uint8	Cooling_TEMP_Alarm_Right;			//USV�ҷ�������ȴҺ�¶��쳣
	uint8	Cooling_Level_Alarm_Left;			//USV�󷢶�����ȴҺҺλ�쳣
	uint8	Cooling_Level_Alarm_Right;			//USV�ҷ�������ȴҺҺλ�쳣
	uint8	OilPressure_Alarm_Left;				//USV�󷢶�������ѹ���쳣
	uint8	OilPressure_Alarm_Right;			//USV�ҷ���������ѹ���쳣
	uint8	Fuel_Moisture_Alarm;				//USVȼ�ͺ�ˮ�澯
}USV_Engine_Alarm;

//USV���������ṹ��
typedef struct
{
	uint32	Date_Inside;					//USV�ڲ�����
	uint32	Time_Inside;					//USV�ڲ�ʱ��
}USV_Time;

//USV������״̬��Ϣ�ṹ��
typedef struct  
{
	uint8	Ldle_Time_Left;					//USV�󷢶�������ʱ��
	uint8	Ldle_Time_Right;				//USV�ҷ���������ʱ��
	uint8	Fuel_ConRate;					//USV������������	
//	uint16	Fuel_ConRate_Left;				//USV�󷢶���������
//	uint16	Fuel_ConRate_Right;				//USV�ҷ�����������
}USV_Engine_State;

//USV�ȶ�ƽ̨��б�Ƕ���Ϣ�ṹ��
typedef struct  
{
       uint8	Stable_Platform_RL_St;			//�ȶ�ƽ̨����ʵ����б�Ƕ�
       uint8	Stable_Platform_AT_St;			//�ȶ�ƽ̨ǰ��ʵ����б�Ƕ�
}USV_Stable_Platform_St;

//USV���˻�ƽ̨״̬�ṹ��
typedef struct  
{
       uint8	Platform_Hatch_St;				//���˻�ƽ̨���ſ���
       uint8	Platform_Lift_St;				//���˻�ƽ̨��������
       uint8	Platform_Open_St;				//���˻�ƽ̨���̿��Ͽ���
       uint8	UAV_Charging_St;				//���˻�������
}USV_UAV_State;
//����ϵͳ
typedef struct 
{
	uint8 Intelligent_rudder;				//���ܶ�
	uint8 Stabilized_platform;				//�ȶ�ƽ̨
	uint8 Energy_management;			//��Դ����ϵͳ
	uint8 Power_management;;			//��Դ����ϵͳ
	uint8 UAV_platform;			   		//���˻�ƽ̨
	uint8 Fire_fighting_system;			//����ϵͳ
	uint8 Engine_L;						//�󷢶���
	uint8 Engine_R;						//�ҷ�����
}Conrtol_System;

//AIS��Ϣ1��2��3�ṹ��
typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 Navigation_status;			//����״̬
	uint8 ROT;							//ת����
	uint32 SOG;						//�Եغ���
	uint8 Position_accuracy;			//��λ��ȷ��
	int Longitude;						//����
	int Latitude;						//ά��
	uint32 COG;						//�Եغ���
	uint32 True_Head;					//�����溽��
	uint8 Timer_stamp;					//ʱ����
	uint8 Regional_APP;				//������
	uint8 Spare;						//����
	uint8 RAIM_Flag;					//RAIM��־
	uint8 UTC_direct;					//ͬ��״̬
	uint8 Time_lost;					//ʱ϶��ʱ
	uint32 Sub_information;			//����Ϣ
}Ship_A_position;
//AIS��Ϣ4��11�ṹ��
typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint32 UTC_year;					//UTC���
	uint32 UTC_month;					//UTC�·�
	uint32 UTC_day;					//UTC����
	uint32 UTC_hour;					//UTCСʱ
	uint32 UTC_minute;				//UTC����
	uint32 UTC_second;				//UTC��
	uint8 Position_accuracy;			//��λ��ȷ��
	int Longitude;						//����
	int Latitude;						//ά��
	uint8 Device_type;					//���Ӷ�λ�豸����
	uint8 Spare;						//����
	uint8 RAIM_Flag;					//RAIM��־
	uint8 UTC_direct;					//ͬ��״̬
	uint8 Time_lost;					//ʱ϶��ʱ
	uint32 Sub_information;			//����Ϣ
}Ship_UTC;

typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 AIS_version;					//AIS�汾
	uint32 IMO;						//IMO���
	uint8 Call_sign[7];					//����
	uint8 Ship_name[20];				//����				
	uint8 Cargo_type;;					//�����ͻ�������
	uint32 Dimension;					//�߶�/λ�òο�
	uint8 Device_type;					//���Ӷ�λ�豸����
	uint32 EAT;						//Ԥ�ƺ���ʱ��
	uint8 Darught_Max;					//����ˮ���
	uint8 Destination[20];				//Ŀ�ĵ�
	uint8 DET;							//�����ն��豸
	uint8 Spare;						//����
}Ship_static_information;
typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 Regional_APP_1;				//������
	uint32 SOG;						//�Եغ���
	uint8 Position_accuracy;			//��λ��ȷ��
	int Longitude;						//����
	int Latitude;						//ά��
	uint32 COG;						//�Եغ���
	uint32 True_Head;					//�����溽��
	uint8 Timer_stamp;					//ʱ����
	uint8 Regional_APP_2;				//������
	uint8 Spare;						//����
	uint8 RAIM_Flag;					//RAIM��־
	uint8 Communication_Sign; 			//ͨ��״̬��ʶ
	uint8 UTC_direct;					//ͬ��״̬
	uint8 Time_lost;					//ʱ϶��ʱ
	uint32 Sub_information;				//����Ϣ
}Ship_B_position;
typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 Regional_APP_1;				//������
	uint32 SOG;						//�Եغ���
	uint8 Position_accuracy;			//��λ��ȷ��
	int Longitude;						//����
	int Latitude;						//ά��
	uint32 COG;						//�Եغ���
	uint32 True_Head;					//�����溽��
	uint8 Timer_stamp;					//ʱ����
	uint8 Regional_APP_2;				//������
	uint8 Ship_name[20];				//����				
	uint8 Cargo_type;;					//�����ͻ�������
	uint32 Dimension;					//�߶�/λ�òο�
	uint8 Device_type;					//���Ӷ�λ�豸����
	uint8 RAIM_Flag;					//RAIM��־
	uint8	Data_terminal;				//�����ն�
	uint8 Spare;						//����
}Ship_B_position_Extended;

typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 Part_Number;			   	//��Ϣ����ű�ʶ��0
	uint8 Ship_name[20];				//����				
}Ship_Static_A;
typedef struct 
{
	uint8 Msg_ID;						//��Ϣʶ���
	uint8 Rep_ID;						//�ظ�����ָʾ��
	uint32 User_ID;					//�û�ʶ����
	uint8 Part_Number;			   	//��Ϣ����ű�ʶ��1
	uint8 Cargo_type;;					//�����ͻ�������
	uint8 Vendor_ID[7];			   	//��Ӧ��ID
	uint8 Call_Sign[7];			   	//����
	uint32 Dimension;					//�����߶ȼ���λ�豸λ�òο�
	uint8 Spare;						//����
}Ship_Static_B;

//���˴�״̬���Ľṹ��
typedef struct 
{
	uint8	USV_Num;		//���˴����
       uint16	Msg_Num;		//���ı��
       USV_Sailing_State		Dradio_USV_Sailing_State;	//USV����״̬��Ϣ
       USV_Model_State		Dradio_USV_Model_State;	//USVģʽ״̬��Ϣ
       USV_Location			Dradio_USV_Location;		//USVλ����Ϣ
	USV_Drive_State		Dradio_USV_Drive_State;	//USV����ϵͳ״̬��Ϣ
	uint8	USV_Oil;										//USVʣ�������ٷֱ�
	USV_Battery				Dradio_USV_Battery;		//USV���״̬��Ϣ
	uint8	USV_Sailing_Intel_Sign;							//USV��������ִ��״̬��Ϣ 1--�յ��������񣬴�������2--�Զ����У�3--�����������
	USV_Device_State		Dradio_USV_Device_State;	//USV�豸��Դ״̬
	USV_Battery_Alarm	Dradio_USV_Battery_Alarm;	//USV����¶ȸ澯
	USV_Fire					Dradio_USV_Fire;			//USV��������ϵͳ״̬
	USV_Engine_Alarm	Dradio_USV_Engine_Alarm;	//USV�������澯��Ϣ
//	sint16	USV_Height;										//USV����
	uint8 Hull_Fan_st;										//���״̬
	uint8 Hull_Pump_st;									//ˮ��״̬
	uint8 APP_12V;										//12VӦ���豸��Դ״̬
	uint8 APP_24V;										//24VӦ���豸��Դ״̬
	uint8 Outboard_Engine_st_L;							//�����������״̬
	uint8 Outboard_Engine_st_R;							//�����������״̬
//	uint8 Sealight_LR_st;									//̽�յ�������ת״̬
//	uint8 Sealignt_HT_st;									//̽�յ�ǰ����ת״̬
	USV_Time					Dradio_USV_Time;			//USVʱ����Ϣ
	USV_Engine_State		Dradio_USV_Engine_State;	//USV������
	USV_Stable_Platform_St	Dradio_USV_Stable_Platform_St;//USV�ȶ�ƽ̨�Ƕ�״̬
	USV_UAV_State			Dradio_USV_UAV_State;		//USV���˻�ƽ̨״̬
	uint16 USV_ROT;
	uint8	USV_Current_Sailing;							//USV״̬ 3--һ���������Զ����� 0-
	uint8	Sailing_Nummber;								//USV����������
	uint8	Config_Nummber;								//USV���ñ��ı��
	uint8	Engine_power;									//��������Դ״̬
	uint8	Engine_run;										//��������ͣ״̬
	uint8	USV_oil_sign;									//��·����״̬
	uint8	USV_Speed_limit;								//����ֵ
	Conrtol_System Conrtol_System_Msg;				//��ϵͳ״̬
	TRANS_ROT_CMD trans_rot_cmd;					//ƽ����ת״̬
}State_Message;
//CAN���˻�ƽ̨״̬
typedef struct  
{
       uint8	Platform_Hatch_spn520576;		//���˻�ƽ̨���ſ���
       uint8	Platform_Lift_spn520577;		//���˻�ƽ̨��������
       uint8	Platform_Open_spn520578;		//���˻�ƽ̨���̿��Ͽ���
       uint8	UAV_Electricity_spn520579;		//���˻���ص���
       uint8	UAV_Con_spn520580;			//���˻�����ģʽ
       uint8	UAV_Run_spn520581;			//���˻�����ģʽ	
       uint8	UAV_Charging_spn520582;		//���˻�������
 	uint8   System_Power_5_spn520583;	//��Դ5V״̬
	uint8   System_Power_12_spn520584;	//��Դ12V״̬
	uint8   System_Power_3_spn520585;	//��Դ3.3V״̬
	uint8   System_Power_24_spn520586;	//ϵͳ��ѹ	
	uint8   System_Check_spn520587;		//ϵͳ�Լ�
	uint8	System_TEMP_spn502588;		//ϵͳ�¶�
      
       
}UAV_CAN;
//CAN���ܶ�״̬
typedef struct  
{
       uint16	Rudder_Angle_L_spn520766;			//���L
       uint16	Rudder_Angle_R_spn520767;			//���R
       uint8		Motor_Gear_L_spn520768;			//��λL
       uint8		Motor_Gear_R_spn520769;			//��λR
       uint16	Motor_Speed_L_spn520770;			//������ת��L
       uint16	Motor_Speed_R_spn520771;			//������ת��R
	   uint8	Rudder_Major_Failure	;			//��DSP�������ع���
	   uint8	Rudder_Minor_Failure	;			//��DSP����һ�����

       //������״̬�ӷ����������ж�
       uint8		Damper_st_L_spn520774;			//�����״̬
       uint8		Damper_st_R_spn520775;			//�ҷ���״̬
       uint8		Outboard_Engine_st_L_spn520776;	//�����������״̬
       uint8		Outboard_Engine_st_R_spn520777;	//�����������״̬
	uint8		Config_Answer_spn520778;   		//����Ӧ��
	uint8   	System_Power_5_spn520779;		//��Դ5V״̬
	uint8   	System_Power_12_spn520780;		//��Դ12V״̬
	uint8   	System_Power_3_spn520781;		//��Դ3.3V״̬
	uint8   	System_Power_24_spn520782;		//��Դ24V״̬	
	uint8   	System_Check_spn520783;			//ϵͳ�Լ�
	uint8		System_TEMP_spn520784;			//ϵͳ�¶�
	
}SR_CAN;
//��Դ����ϵͳ״̬
typedef struct  
{
       uint8	Stable_Platform_Power_spn520958;	//�ȶ�ƽ̨��Դ
       uint8	UAV_Power_spn520959;				//���˻�ƽ̨��Դ
	uint8	Horn_Power_spn520960;				//���ȵ�Դ
	uint8	Navigationlight_Power_spn520961;	//���еƵ�Դ
	uint8	D_radar_Power_spn520962;			//�����״��Դ
	uint8	Camera_main_Power_spn520963;	//������ͷ��Դ
	uint8	Application_24V_spn520964;			//24VӦ���豸��Դ
	uint8	Searchlight_Power_spn520965;		//̽�յƵ�Դ

	uint8	Camera_ahead_Power_spn520966;	//ǰ����ͷ��Դ
	uint8	Camera_lesser_Power_spn520967;	//������ͷ��Դ
	uint8	Camera_tail_Power_spn520968;		//β����ͷ��Դ
	uint8	Application_12V_spn520969;			//12VӦ���豸��Դ

	uint8   System_Power_5_spn520970;		//��Դ5V״̬
	uint8   System_Power_12_spn520971;		//��Դ12V״̬
	uint8   System_Power_3_spn520972;		//��Դ3.3V״̬
	uint8   System_Check_spn520973;			//ϵͳ�Լ�
	uint8   System_Voltage_spn520974;			//ϵͳ��ѹ
	uint8   System_Electricity_spn520975;		//ϵͳ����
	uint8	DIN1_Connection_St_spn520976;		//����1����״̬
	uint8	DIN2_Connection_St_spn520977;		//����2����״̬
	uint8	DIN3_Connection_St_spn520978;		//����3����״̬
	uint8	DIN4_Connection_St_spn520979;		//����4����״̬
	uint8	System_TEMP_spn502980;			//ϵͳ�¶�
	uint8	System_Heatbeat;					//����
}POW_CAN;





//CAN�������״̬
typedef struct  
{
	uint8	Emergency_spn521192;				//Ӧ��ģʽ
       uint8	Get_USV_Con_spn521193;			//��ȡ���˴�����Ȩ�� 1--������Ȩ�� ��0--����
       uint8 	System_Heatbeat;
	   uint8    translation_mode;				//ƽ��ģʽ 0--�޶��� 1--ƽ��ģʽ
	   uint8	translation_command;			//00--ֹͣ  01--��ƽ��  10--��ƽ��
	   uint8    rotation_mode;					//��תģʽ 0--��     1--��תģʽ
	   uint8	rotation_command;				//00--ֹͣ  01--��ʱ��  10--˳ʱ��
}SP_Model_CAN;

typedef struct  
{
       uint8	Gear_Left_spn521178;					//�󷢶�����λ       
       uint8	Gear_Right_spn521179;					//�ҷ�������λ	
       uint8	Accelerator_Left_spn521180;				//�󷢶�������	
       uint8	Accelerator_Right_spn521181;			//�ҷ���������	
       uint8	Rudder_Angle_Left_spn521182;			//����
       uint8	Rudder_Angle_Right_spn521183;			//�Ҷ��	      
	uint8	Speed_Limit_spn521184;			    		//����ֵ
}SP_Sail_CAN;

typedef struct  
{
      uint8	Stable_Platform_RL_spn521189;			//�ȶ�ƽ̨������б�Ƕ�       
      uint8	Stable_Platform_AT_spn521190;			//�ȶ�ƽ̨ǰ����б�Ƕ�
}SP_Equipment_CAN;

//CAN�ȶ�ƽ̨״̬
typedef struct  
{
       uint8	Stable_Platform_RL_spn521342;			//�ȶ�ƽ̨����ʵ����б�Ƕ�
       uint8	Stable_Platform_AT_spn521343;			//�ȶ�ƽ̨ǰ��ʵ����б�Ƕ�
	uint8	System_TEMP_spn520344;				//ϵͳ�¶�
	uint8   System_Power_5_spn521345;			//��Դ5V״̬
	uint8   System_Power_12_spn521346;			//��Դ12V״̬
	uint8   System_Power_3_spn521347;			//��Դ3.3V״̬
	uint8   System_Power_24_spn521348;			//��Դ24V״̬	
	uint8   System_Check_spn521349;				//ϵͳ�Լ�
	   
}ST_PL_CAN;

//CAN��Դ����������״̬
typedef struct  
{
	uint8	USV_Pow_spn521534;					//�����Դ״̬
	uint8	Battery_Charge_L_spn521535;			//USV���س��״̬
	uint8	Battery_Chager_R_spn521536;			//USV�ҵ�س��״̬
	uint8	Battery_TEMP_Alarm_L_spn521537;		//USV�����¶ȸ澯
	uint8	Battery_TEMP_ALarm_R_spn521538;		//USV�ҵ���¶ȸ澯
	uint8	Battery_Capacity_L_spn521539;			//USV���������澯
	uint8	Battery_Capacity_R_spn521540;			//USV�ҵ�������澯	
	uint8	Plug_Connection_St_spn521541;			//����1����״̬
	uint8	Plug_Connection_St_spn521542;			//����2����״̬
	uint8	Plug_Connection_St_spn521543;			//����3����״̬
	uint8	Plug_Connection_St_spn521544;			//����4����״̬
	uint8	Plug_Connection_St_spn521545;			//����5����״̬
	uint8	Plug_Connection_St_spn521546;			//����6����״̬
	uint8	Plug_Connection_St_spn521547;			//����7����״̬
	uint8	Plug_Connection_St_spn521548;			//����8����״̬
	uint8   System_Power_5_spn521549;			//��Դ5V״̬
	uint8   System_Power_12_spn521550;			//��Դ12V״̬
	uint8   System_Power_3_spn521551;			//��Դ3.3V״̬
	uint8   System_Power_24_spn521552;			//��Դ24V״̬	
	uint8   System_Check_spn521553;				//ϵͳ�Լ�
	
	uint8	Battery_Left_spn521554;				//USV����ʣ������ٷֱ�
	uint8	Battery_TEMP_L_spn521555;				//USV�����¶�
	uint8	Battery_Power_L_spn521556;			//USV����˲ʱ����
	uint8	Battery_Current_L_spn521557;			//USV����˲ʱ����
	uint8	Battery_Voltage_L_spn521558;			//USV����˲ʱ��ѹ
	uint8	Oil_L_spn521559;						//����������
	
	uint8	Battery_Right_spn521560;				//USV�ҵ��ʣ������ٷֱ�
	uint8	Battery_TEMP_R_spn521561;				//USV�ҵ���¶�
	uint8	Battery_Power_R_spn521562;			//USV�ҵ��˲ʱ����
	uint8	Battery_Current_R_spn521563;			//USV�ҵ��˲ʱ����
	uint8	Battery_Voltage_R_spn521564;			//USV�ҵ��˲ʱ��ѹ
	uint8	Oil_R_spn521565;						//����������
	uint8	System_TEMP_spn520566;				//ϵͳ�¶�
	
}BAT_CON_CAN;
typedef struct  
{
	//״̬
	uint8	Torque_Mod_spn899;				//������ת��ģʽ
	uint8	Torque_Percent_EXP_spn512;		//����������ת�ذٷֱ�
	uint8	Torque_Percent_ACT_spn513;		//������ʵ��ת�ذٷֱ�
	uint16	Motor_Spd_spn190;					//������ת��
	uint8	Motor_CON_ADD_spn1483;			//����������װ��Դ��ַ
	uint8 	Mortor_Starter_Mod_spn1675;		//������������ģʽ
}Motor_PGN61444;
typedef struct  
{
	//״̬
	uint32	Ldle_Fuel_Consume_spn236;			//������ȼ������
	uint32	Ldle_Time_spn235;					//������ʱ��
}Motor_PGN65244;
typedef struct  
{
	//״̬
	uint32	Working_Time_spn247;				//�ܹ���ʱ��
	uint32	Total_turn_spn249;					//��ת��
}Motor_PGN65253;
typedef struct  
{
	//״̬
	uint32	Single_Fuel_Consume_spn182;		//����ȼ������
	uint32	Total_Fuel_Consume_spn250;		//��ȼ������
}Motor_PGN65257;
typedef struct  
{
	//״̬
	uint8	Cool_Temp_spn110;					//��������ȴҺ�¶�
	uint8	Fuel_Temp_spn174;					//������ȼ���¶�
	uint16	Oil_Temp_spn175;					//�����������¶�
	uint16	Tuibine_Oil_Temp_spn176;			//�����������¶�
	uint8	Motor_Cooler_Temp_spn52;			//�����������¶�
	uint8 	Mortor_Cooler_Opening_spn1134;	//��������������������
}Motor_PGN65262;
typedef struct  
{
	//״̬
	uint8	Fuel_Delivery_Pressure_spn94;				//ȼ�ϴ���ѹ��
	uint8	Extension_Crankcase_Leakage_spn22;		//��չ��������©ѹ��
	uint8	Oil_Level_spn98;							//��������λ
	uint8	Oil_Pressure_spn100;						//��������ѹ
	uint16	Crankcase_Pressure_spn101;				//������ѹ��
	uint8 	Cool_Pressure_spn109;						//��������ȴҺѹ��
	uint8 	Cool_Level_spn111;							//��������ȴҺҺλ
}Motor_PGN65263;
typedef struct  
{
	//״̬
	uint16	Fuel_Usage_spn183;							//ȼ��ʹ����
	uint16	Instant_Fuel_Economy_spn184;				//˲ʱȼ�;�����
	uint16	Average_Fuel_Economy_spn185;				//ƽ��ȼ�;�����
	uint16	Throttle_Position_spn51;						//����λ��
}Motor_PGN65266;
typedef struct  
{
	//״̬
	uint8	Air_Pressure_spn108;						//����ѹ��
	uint16	AIr_Temp_spn171;							//��Χ�����¶�
	uint8	Air_inlet_Temp_spn172;						//�������¶�
}Motor_PGN65269;
typedef struct  
{
	//״̬
	uint8	Supercharger_Pressure_spn102;				//��ѹ��ѹ��
	uint8	Inlet_Air_Temp_spn105;						//��������¶�
	uint8	Air_inlet_Pressure_spn106;					//����������ѹ��
	uint8	Air_fillter_Diff_Pressure_spn107;				//�������������ѹ��
	uint16	Exhaust_Temp_spn173;						//�����¶�
	uint8	Cool_Fillter_Diff_Pressure_spn112;			//��ȴ�����������ѹ��
}Motor_PGN65270;
typedef struct  
{
	//״̬
	uint8	Fuel_Moisture_spn97;						//ȼ�ͺ�ˮָʾ
	uint8	spare;
}Motor_PGN65279;
//������
typedef struct
{

	uint32 SPN;
	uint8 FMI;
	uint8 CM;
	uint8 OC;
}SERVICE_CODE_T;
typedef struct
{
	uint8 count;   
	SERVICE_CODE_T Fault_Code_T[(J1939_MAX_MESSAGE_LENGTH-2)/4];
}FAULT_CODE_T;
 // J1939����״̬��*/
typedef struct
{
	uint32 PGN;						//PGN
	uint8  status;						//״̬
	uint8  total_packet_number;		//�ܰ�����
	uint16  byte_count;				//���ֽ���
	uint32 Dest_PGN;					//Ŀ��PGN
}RECEIVE_STATU_MACHINE_T;  

//CAN������״̬�ṹ��
typedef struct  
{
	//״̬
	Motor_PGN61444 Motor_PGN61444_State[2];		//ת�١�Ť��
	Motor_PGN65244 Motor_PGN65244_State[2];		//����ʱ��
	Motor_PGN65253 Motor_PGN65253_State[2];		//��״̬
	Motor_PGN65257 Motor_PGN65257_State[2];		//�ͺ�
	Motor_PGN65262 Motor_PGN65262_State[2];		//�¶�
	Motor_PGN65263 Motor_PGN65263_State[2];		//ѹ��
	Motor_PGN65266 Motor_PGN65266_State[2];		//ȼ����	
	Motor_PGN65269 Motor_PGN65269_State[2];		//����	
	Motor_PGN65270 Motor_PGN65270_State[2];		//������	
	Motor_PGN65279 Motor_PGN65279_State[2];		//ȼ��ˮ��
}Motor_CAN;

 //PID�ṹ�嶨��
typedef struct 
{             
        float  SetPoint;            		//�趨ֵ
        float  Actual;						//ʵ��ֵ
        float  Proportion;        	 		// Proportion ����ϵ��
        float  Integral;            		// Integral   ����ϵ��
        float  Derivative;          		// Derivative  ΢��ϵ��
        float  Error;					//��ǰ���
        float  LastError;          		// Error[-1]  ǰһ�����
        float  PreError;           		// Error[-2]  ǰ�������
        float  Ec;						//���仯��
        float  Integral_error;            		// 
} Spd_PID;

 //S��ṹ�嶨��
typedef struct 
{             
        float	SetPoint;            		//�趨ֵ
        float	Actual;				//ʵ��ֵ
        float	k1;        	 			//���ϵ��
        float	k2;            			//���仯��ϵ��
        float	k3;					//ѧϰ��
        float	k4;					//�������ϵ��
        float	Error;				//��ǰ���
        float	LastError;          		//ǰһ�����
        float	dError;				//���仯��
        float	out;				//�������
        float	eout;				//�������
		float	Lastout;				//ǰһ�εĿ������
} Spd_S;
//ip\u7ed3\u6784\u4f53
typedef struct{
//ETH0
	short local_port0  ;              		//�˿�   
	short local_ip0[4];               	//IP0
	short local_netmask0[4];		//��������
	short local_gw0[4];			//����
	short local_nameserver0[4];	//DNS
//ETH1
	short local_port1  ;              		//�˿�
	short local_ip1[4];               	//IP1
	short local_netmask1[4];		//��������
	short local_gw1[4];			//����
	short local_nameserver1[4];	//DNS
}IP_ADDRESS_struct;

//���²����������ļ��õ�
typedef struct
{
	uint8 Motor_MAX_Speed_L_spn520220;					//�󷢶������ת�� 
	uint8 Motor_Idling_Speed_L_spn520221;				//�󷢶�������ת�٣����ٹ���ָ�������޸�����ת״̬ 
	uint8 Motor_MAX_Speed_R_spn520222;					//�ҷ��������ת��
	uint8 Motor_Idling_Speed_R_spn520223;				//�ҷ���������ת��
	uint8 Motor_Travel_L_spn520224;						//�󷢶����г�
	uint8 Motor_Travel_R_spn520225;						//�ҷ������г�
	uint8 Motor_Fire_Time_spn520226;						//���������ʱ��
	uint8 Motor_Direction_L_spn520227;					//�󷢶�������
	uint8 Motor_Direction_R_spn520228;					//�ҷ���������	
	
	uint8 Gear_Forward_Travel_L_spn520240;				//��λǰ�����г�
	uint8 Gear_Forward_Travel_R_spn520241;				//�ҵ�λǰ�����г�
	uint8 Gear_Back_Travel_L_spn520242;					//��λǰ�����г�
	uint8 Gear_Back_Travel_R_spn520243;					//�ҵ�λǰ�����г�
	uint8 Gear_Forward_Angle_L_spn520244;				//��λǰ�����Ƕ�����
	uint8 Gear_Forward_Angle_R_spn520245;				//�ҵ�λǰ�����Ƕ�����
	uint8 Gear_Back_Angle_L_spn520246;					//��λ���˵��Ƕ�����
	uint8 Gear_Back_Angle_R_spn520247;					//�ҵ�λ���˵��Ƕ�����
	uint8 Gear_Neutral_Angle_L_spn520248;				//��λ�յ��Ƕ�
	uint8 Gear_Neutral_Angle_R_spn520249;				//�ҵ�λ�յ��Ƕ�
	uint8 Gear_Direction_L_spn520250;						//��λ���� 0--ǰ�� 1--����
	uint8 Gear_Direction_R_spn520251;						//�ҵ�λ����	0--ǰ�� 1--����
	uint8 Drive_Nummber_spn520252;						//����ϵͳ����
	
	uint8 Rudder_Angle_Max_L_spn520260;					//������Ƕ�
	uint8 Rudder_Angle_Max_R_spn520261;					//�Ҷ����Ƕ�
	uint8 Rudder_Left_Limit_Resistance_L_spn520262;		//������޵���ֵ
	uint8 Rudder_Left_Limit_Resistance_R_spn520263;		//�Ҷ����޵���ֵ
	uint8 Rudder_Middle_Limit_Resistance_L_spn520264;	//�����λ����ֵ
	uint8 Rudder_Middle_Limit_Resistance_R_spn520265;	//�Ҷ���λ����ֵ
	uint8 Rudder_Right_Limit_Resistance_L_spn520266;		//����Ҽ��޵���ֵ
	uint8 Rudder_Right_Limit_Resistance_R_spn520267;		//�Ҷ��Ҽ��޵���ֵ
	uint8 Rudder_Control_Angle_Max_L_spn520268;			//��������ƶ��
	uint8 Rudder_Control_Angle_Max_R_spn520269;			//�Ҷ������ƶ��
	uint8 Rudder_Control_Angle_Min_L_spn520270;			//�����С���ƶ��
	uint8 Rudder_Control_Angle_Min_R_spn520271;			//�Ҷ���С���ƶ��
	uint8 Rudder_Left_Travel_L_spn520272;				//��������ƶ��
	uint8 Rudder_Left_Travel_R_spn520273;				//�Ҷ������ƶ��
	uint8 Rudder_Right_Travel_L_spn520274;				//�����С���ƶ��
	uint8 Rudder_Right_Travel_R_spn520275;				//�Ҷ���С���ƶ��	
	uint8 Rudder_Control_Accuracy_L_spn520276;			//�����ƾ���
	uint8 Rudder_Control_Accuracy_R_spn520277;			//�Ҷ���ƾ���
	uint8 Rudder_Direction_L_spn520278;					//���Ƿ���
	uint8 Rudder_Direction_R_spn520279;					//�Ҷ�Ƿ���
}SR_Config;

//�״��ϰ������Ϣ
typedef struct  
{
    uint8		Obs_num[OBS_NUM];			//�ϰ�����
	int 		TCPA[OBS_NUM];				//�ϰ���TCPAֵ�����ﴬ������������ʱ��TCPA
	uint32		DCPA[OBS_NUM];				//�ϰ���DCPAֵ�����������������DCPA
	double		lat[OBS_NUM];				//�ϰ�������ά��
	double		lng[OBS_NUM];				//�ϰ������꾭��
}Radar_OBS;

typedef struct  
{
	double lat[255];//ά��
	double lng[255];//����
}Path_Coordinate;

typedef struct 
{
	double		wp_next_lati;		//��һ����γ��
	double		wp_next_lngi;		//��һ���㾭��
	double		wp_last_lati;		//��һ����γ��
	double		wp_last_lngi;		//��һ���㾭��
	double		local_lati	;		//Ŀǰλ��γ��
	double		local_lngi	;		//Ŀǰλ�þ���
	double		track_error	;		//����ƫ��
	double		heading_change;		//��������
}Track_Control;


typedef struct{
	uint8	dradio_btn_stop;	//���ֵ�̨��ͣ
	uint8	sradio_btn_stop;	//�����̨��ͣ
	uint8	bradio_btn_stop;	//������̨��ͣ
	uint8	bd_btn_stop;		//������ͣ
	uint8	sp_btn_stop;		//������弱ͣ
}E_STOP_CMD_INF;


typedef struct{
	uint8			e_stop_cmd;
	uint8			e_stop_logic;
	uint8			e_stop_rudder_failure;
	uint8			e_stop_comm;
	E_STOP_CMD_INF	e_stop_cmd_inf;
}E_STOP_ALL;


typedef struct{
	uint16	 whole_frame_num;
	uint16	 right_frame_num;
	uint16	 wrong_frame_num;
}DRADIO_FRAME_STATIC;		//����ͳ��


typedef struct{
	double	lng;
	double	lat;
}POSITION;	


#ifdef WINNT
extern HANDLE UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//���崮�ڱ�־	
extern HANDLE CAN_0;//����CAN
#else
extern int UART0_Fd,UART1_Fd,UART2_Fd,UART3_Fd,UART4_Fd,UART5_Fd;	//���崮�ڱ�־	
extern int CAN_0;//����CAN
#endif

#ifndef WINNT
typedef int CAN_TYPE;
#else
typedef HANDLE CAN_TYPE;
#endif

extern Radar_OBS Radar_OBS_Msg;
extern Path_Coordinate Path_Coordinate_Msg;//���Ϲ滮������

extern IP_ADDRESS_struct  ip_address;
extern SR_Config SR_Config_Msg;
extern UAV_CAN	UAV_CAN_State;//���˻�ƽ̨״̬
extern SR_CAN	SR_CAN_State;//���ܶ�״̬
extern POW_CAN POW_CAN_State;//��Դ����ϵͳ״̬
extern SP_Model_CAN	SP_CAN_Model_Con;//������ģʽָ��
extern SP_Sail_CAN		SP_Sail_CAN_Con;//�����Ʒ�����ָ��
extern SP_Equipment_CAN 	SP_Equipment_CAN_Con;//�������ȶ�ƽָ̨��
extern ST_PL_CAN ST_PL_CAN_State;//�ȶ�ƽ̨״̬
extern BAT_CON_CAN BAT_CON_CAN_State;//��Դ����ϵͳ״̬
extern Motor_CAN Motor_CAN_State;//������״̬
extern RECEIVE_STATU_MACHINE_T Receive_Status[2];//J1939����״̬��
extern FAULT_CODE_T	Fault_Code[2];//������������

extern AIS_Msg AIS_Msg_St;//ͼ����̨����AIS��Ϣ

extern Motor_Detail_State	Motor_Detail_St;//ͼ����̨���ͷ�������ϸ��Ϣ
extern Rudder_Detail_State Rudder_Detail_St;//ͼ����̨�������ܶ���ϸ��Ϣ
extern Stable_Platform_Detail_State Stable_Platform_St;//ͼ����̨�����ȶ�ƽ̨��ϸ��Ϣ
extern UAV_Detail_State UAV_Detail_St;//ͼ����̨�������˻�ƽ̨��ϸ��Ϣ
extern MCU_State MCU_St;//ͼ����̨���Ϳ��������ϵͳ��ϸ��Ϣ
extern Hull_State Hull_St;//ͼ����̨���ʹ�������ϸ��Ϣ
extern Panel_Control_Msg Panel_Control_St;//ͼ����̨���ʹ������ܿ��������ϸ��Ϣ
extern Energy_Control_Msg Energy_Control_St;//ͼ����̨������Դ����ϵͳ��ϸ��Ϣ
extern Power_Control_Msg  Power_Control_St;//ͼ����̨���͵�Դ����ϵͳ��ϸ��Ϣ
extern Check_Collision		USV_Check_Collision[AIS_Collision_Num];//��ײ����
extern Control_Message	USV_Control;//���˴�����ָ��
extern Sailing_Message		USV_Sailing;				//���˴���������
extern Dradio_Config_Message Dradio_Config;			//���˳�������Ϣ���������ļ����
extern State_Message 		USV_State;					//���˴�״̬
extern LAN0_RM_Message	USV_RM_MSG;//MPC����RAM���ڱ���
extern Waypoint				Point_Return[2];//һ��������
extern DSP_State DSP_State_Msg;//DSP״̬
	
extern Spd_PID	vPID,rPID,aPID;                       //�����ٶȺ���PID���ƽṹ��
extern Spd_S	vSpd,rSrd;				//�����ٶȺ���S����ƽṹ��
extern TASK_TIME_STRUCT	task_time;
	
extern uint8	Dradio_Com1_Sign;//���ֵ�̨COM1����״̬
extern uint8	Dradio_Com2_Sign;//���ֵ�̨COM2����״̬
extern uint8	SpareDradio_Com1_Sign;//�������ֵ�̨COM1����״̬
extern uint8	SpareDradio_Com2_Sign;//�������ֵ�̨COM2����״̬
extern uint8	Bradio_Con_Sign;//������̨����״̬
extern uint8	BDS_Con_Sign;//������̨����״̬
extern uint8	SP_CON_Sign;//������������ȡ����Ȩ�ޱ�־
extern uint8	BD_Engine_Run_L;	//���յ��ָ��L
extern uint8	BD_Engine_Run_R;	//�������ָ��R


extern int LAN0_Fd,LAN1_Fd;//��������

extern uint8	Radio_Sign,Radio_Sign_old,Connect_Sign;
extern uint8	Sailing_Sign;
extern uint8	Sailing_Cnt_Old;
extern uint8	Avoid_Point_Arrive;			//���ϵ㵽���־
extern double Speed_EXP,Heading_EXP;//�����㷨��������������٣�Speed_EXP--�ٶ�����ֵ��Heading_EXP--��������ֵ
extern double	Collision_Speed;//���Ϻ���
extern double	Collision_Heading;//���Ϻ���
extern double Radar_Collision_Heading;//�״���Ϻ���
extern sint16	GLB_TCPA;
extern int32	GLB_Safe_DCPA,GLB_DCPA;


extern uint8 Collision_Num;
extern uint8	Rudder_Con_Angle;//���ƶ�ǣ�ʵ�ʶ�ǲ��ܴ��ڿ��ƶ��
extern uint8	Rudder_Count;	//��ĳһ�����ĳ���ʱ��
extern uint8	Collision_Sign;	//��ײ��־
extern int Watch_fd;
extern double lau_old,lnu_old;//������̵ľ�γ�����
extern uint32  Mileage;//���
extern uint8 E_Stop,E_Stop_INS,E_Stop_DSP,E_Stop_Dradio,E_Stop_SmartRudder,Time_Sign;
extern int Rudder_Zero_Angle;//������,�������ƫ��
extern int Accelerator_L,Accelerator_R,Rudder_L,Rudder_R,Gear_L,Gear_R;			//Accelerator--���� Rudder--��� Gear--��λ
extern double Accelerator_Coefficient_L,Accelerator_Coefficient_R,Rudder_Coefficient_L,Rudder_Coefficient_R,Gear_Coefficient_L_F,Gear_Coefficient_L_B,Gear_Coefficient_R_F,Gear_Coefficient_R_B;
extern uint16 Heartbeat_Num;
extern int sign,Get_Control_Sign,count_Estop,SP_CAN_Count;
extern uint8	USV_State_Current[State_Current_Num];//���˴���ǰ״̬����̨����
extern char Version[10][100];
extern int ARMVersion_sign,DSPVersion_sign,MPCVersion_sign,UAVVersion_sign,SRVersion_sign,POWVersion_sign,SPVersion_sign,STVersion_sign,BATVersion_sign;

extern float Dst_monitor;
extern int32 NoStopPoint_Sign;				//ͣ����־
extern int32 Obstacle_Sign;					//���ϱ�־
extern int32 radar_obstacle_sign;			//��̬���ϱ�־
extern int32 radar_avoid_type;				//0 �޶��� 1 ����  2 ׷�� 3 �󽻲�  4 �ҽ���
extern int Compensating_Dst[255];

//extern USV_Meter_Clock USV_Meter_Clock_Data;
extern Track_Control		track_control;		//������������
extern E_STOP_ALL e_stop_inf;	//��ͣ��Ϣ
extern int16 sockid_test;
extern uint8 sockid_test_buf[20];

//��ʱ���㣬PSO�㷨���
extern double temp_point_lat;
extern double temp_point_lng;
extern int	  point_sum;
extern int	  epv_type;

//֡��ͳ��
extern DRADIO_FRAME_STATIC dradio_frame_static;

extern char MoniforIpCfg[30];		//��̨������IP����

extern void Update_USV_State(void);
extern void Send_Dradio_State(uint16 State_Num);
extern void Send_Sradio_State();
extern uint8 Check_Radio(void);
extern void Emergency_Stop_Control(int Con_Type);
extern void Oil_Control();

void Send_Accelerator(uint16 Heartbeat_Num);

void *Uart0_Heartbeat(void *aa);
void *Uart1_AIS_Receive(void *aa);
void *Uart2_Dradio_Rec(void *aa);
void *Uart3_BDS(void *aa);
void *Uart4_SpareDradio_Rec(void *aa);
void *Uart5_Ins_Rec(void *aa);
void *Lan0_MPC_Rec(void *aa);
void *Lan1_Bradio_Rec(void *aa);
void *Lan1_Bradio_Client(void *aa);
void *Lan1_Bradio_UDP(void *aa);
void *CAN0_ARM_Rec(void *aa);
void *Intelligent_Navigation(void *aa);
void *GetSysinfo(void *aa);
void *calculate_path_point(void *aa);
void *COMM_Task(void *aa);
extern uint8 get_radar_moving_obs(void);
extern uint8 get_radar_collision(void);
void sleep_1(int delay_ms);

void Sailing_Intelligent_algorithm(double *dst,uint8 sailing_cnt);
void Filter_abc(int n,double *x,double a,double b,double c);
double Calculate_Speed_Heading();
double Calculate_Speed_Heading_tem();		//�����Ӧ��ʱ����ĺ��� syp
double Get_heading(double lat1,double lng1,double lat2,double lng2);
void Speed_PID(double speed);
void Speed_S(double speed);
void Heading_PID(double heading);
void Heading_S(double heading);
float fuzzy_kp(float e, float ec) ;
float fuzzy_ki(float e, float ec) ;
float fuzzy_kd(float e, float ec) ;
extern uint8 Check_Intelligent_Mod(void);
uint8 Check_Intelligent_Switch(void);
uint8 Check_Speed_Const(void);
uint8 Check_Heading_Const(void);
uint8 Calculate_Accelerator_Rudder();
uint8 Calculate_Accelerator_Rudder_temp();
void Con_Rudder(void);
uint8 Calculate_Rudder(void);
uint8 Calculate_Rudder_Zero(void);
uint8 Get_SeaState(void);
//uint8 Calculate_Collision(void);//������ײ����
uint32 Get_Safe_DCPA(uint8 Sea_State);
void Get_DCPA(uint32 DCPA,sint16 TCPA);
void Get_AIS_DCPA_TCPA(uint32 *DCPA,sint16 *TCPA);
double Get_AQ(double vx,double vy);
double Radian(double d);
double Get_distance(double lat1,double lng1,double lat2,double lng2);
void Get_lat_lng(double lat, double lng, double dst, double heading, double *Lat, double *Lng);
double Get_ATU(double lat1,double lng1,double lat2,double lng2);
double Get_AUT(double lat1,double lng1,double lat2,double lng2);
void Obstacle_acoidance(void);
void Avoidance_Strategy(double lat1,double lng1,double COGA,double COGU);
void Clear_EStop();
void Get_Return_Point(void);
void GetCheck(uint8 *CRC,uint8 *buff,uint32 length);
void Send_Bradio_State();
void Send_Bradio_Ais();
void Send_Bradio_SP_State();
void Send_Bradio_Detail_Msg(void);
void Send_CAN_CRC();
void Send_MPC_CRC();
int set_direction(char* index, char* direction);
int get_direction(char* index, char* direction);
int set_value(char* index, char* value);
int get_value(char* index, char* value);

extern void fireOndelay(void);
extern int8 test_udp_send( uint8 * lpBuf, int16 iLen,int16 sockid);


