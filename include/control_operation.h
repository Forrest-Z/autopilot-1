//control_operation.h
#ifndef __CONTROL_OPERATION__H_
#define __CONTROL_OPERATION__H_

#include "usv_include.h"

#define SAIL_MODE_MANUAL   0
#define SAIL_MODE_SEMIAUTO 1
#define SAIL_MODE_AUTO	   2
#define SAIL_MODE_STANBY   3

#define ENGINE_STOP_EMERGENCY 1
#define COMM_DISCONNECT_EMERGENCY 2

typedef struct{
	int16	i16_rudder1NaturlAngle;
	int16	i16_rudder2NaturlAngle;
}RUDDER_CONFIG;

typedef struct{
	RUDDER_CONFIG	rudderConfig;
}CTRL_CONFIG;


typedef struct{
	uint8   b2_Cmd_MotorOnOff          ;	//��ͣ
//	uint8   b1_Cmd_MotorEmergencyStop  ;	//��ͣ
	uint8   u8_Cmd_MotorOpenDeg        ;   //���ſ���
	int16   i16_Cmd_MotorGearDeg       ;   //��λ�Ƕ�
	int16   i16_Cmd_MotorRudderDeg     ;   //���
}JET_DRIVER;

typedef struct{
	uint8	   b1_cmd_emergencyStop	;	//��ͣ
	JET_DRIVER jetL					;	//��ද��ϵͳ
	JET_DRIVER jetR					;	//�Ҳද��ϵͳ
}JET_SYSTEM;

typedef struct{
	double		double_lat	;
	double		double_lng	;
}ESTOP_POSITION;

extern char disconnectCfg[20];	//ͨѶ�ж�����
extern JET_SYSTEM jet_system;
extern CTRL_CONFIG jet_config;

extern	uint8   log_b1_emergencyStop;	/*���漱ͣ�¼�״̬*/
extern	uint8	log_b2_motorFireOn;		/*���淢������ͣ״̬*/
extern	uint8	log_b2_sailMode;		/*���溽��ģʽ״̬*/


extern void initControlOperation(void);
void* runControlOperation(void*);
uint8 calEmergency_Stop(void);
void jetContrlModify(void);
void outBoardEngineModify(void);
void rudderModify(void);
void gearModify(void);
void MotorModify(void);

#endif /*__CONTROL_OPERATION__H_*/