/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人���������޹�˾
-	File name  : dock_communication.h
-	Author	   : fushuai
-	Date	   : 2019/05/13 16:50
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef  __DOCK_COMMUNICATION_H_
#define  __DOCK_COMMUNICATION_H_

#define CMAERA_TRACKON	1				//����ͷ���ٿ�ʼ
#define CMAERA_TRACKOFF 2				//����ͷ���ٹر�
#define USV_1
#define CAN_USV_ADD 0x80
#define CAN_DOCK_ADD 0x8a

#define CAN_DOCK_PS 0xe9
#define  CAN_DOCK_DISCONNECT_MAX 100 //50msһ������
#include "usv_include.h"

#ifndef WINNT
#include <sys/time.h>
#else
#include<winsock.h>
#endif
#define		RETURN_POINT_MAX_NUMBER			3			//���ķ�������
#ifndef WINNT 
const char USV_DOCKING_FILE_NAME[] = "../cfg/usv_flash_docking.inf";		//usv����״̬
#else
const char USV_DOCKING_FILE_NAME[] = "../../cfg/usv_flash_docking.inf";		//usv����״̬
#endif


#include <climits>
#include <stdint.h>

enum STATS
{
	STAT_RESET = 0,
	STATS_ON,
	STATS_OFF,
};
//�ⲿ���ÿ��ƽṹ��
//��Ӧcan���е�ps = 13
#pragma pack(1)
typedef struct
{
	uint8 cmd_state;
	float heading;
	float vx;
	float wr;

}DockingControlCmd;
#pragma pack()


//�ⲿ����״̬�ṹ��
#pragma pack(1)
typedef struct
{
	uint8 x_open;					/*�ɽ���*/
	uint8 x_in;						/*������*/
	uint8 x_outgoing;				/*�ɳ���*/
	uint8 x_in_entrance;			/*���ﴬ���*/
}DOCK_STATE;
#pragma pack()

//���水ť�ӿ�
#pragma pack(1)
typedef struct
{
	uint8 x_reset;					/*��λ*/
	uint8 x_open;					/*�ɽ���*/
	uint8 x_in;						/*������*/
	uint8 x_outgoing;				/*�ɳ���*/
	uint8 x_outsuccess;				/*�ѳ���*/
}DOCK_SUMLINK;
#pragma pack()

typedef struct 
{
	uint8 power_on;					/*�ϵ�ɹ�*/
	uint8 set_retpos_on;			/*���÷�����ɹ�״̬ 0:ȱʡ 1:�ɹ� 2:��λ��Ч 3:������д��ʧ��*/
	uint8 cmd_feedback;				/*�����������յ�����*/
	uint8 entry_docking;			/*������*/
	uint8 out_docking;				/*������*/
	uint8 succcessed_entry;			/*�ɹ�����*/
	uint8 succcessed_out;			/*�ɹ�����*/
	uint8 log_b1_dock_state;		/*���沴��״̬*/
	uint8 log_b1_return_state;		/*���淵��״̬*/
}USV_STATE;
#pragma pack(1)

typedef struct {
	
	uint8   dock_entrydock_readyon;		/*�����Ƿ�߱���������*/
	uint8   dock_entrydock_success;		/*�����Ƿ�ɹ�����*/
	uint8   dock_outdock_readyon;		/*�����Ƿ�߱�����*/
	uint8   dock_entrydock_entrance;	/*�����Ƿ񵽴�����*/
}DOCK_COMM_COIL_READ;
#pragma pack()

typedef struct 
{
	uint8 usv_entrydock_req;			/*������������*/
	uint8 usv_prower_off_successed;		/*��Ϩ��ɹ�*/ 
	uint8 usv_prower_on_successed;		/*���ϵ�������ʼ���ɹ�*/
	uint8 usv_outdock_successed;		/*��������ɹ�*/

}DOCK_COMM_COIL_SET;

typedef enum
{
	EVENT_POWERON_USV = 0,		  /*���ϵ��ʼ�����*/
	EVENT_OUTDOCK_READY,		  /*������׼��*/
	EVENT_OUTING,				  /*��������*/
	EVENT_ENTRY_DOCK_READY,		  /*������׼��*/
	EVENT_ENTRYING,				  /*��������*/
	EVENT_ENTRYED_USV,		  /*��Ϩ��*/
	EVENT_OUTED_USV,		  /*���ѳ��������*/
}USVDockEventState;

//������
typedef struct  
{
	double	heading;	/*����*/
	double	longitude;	/*����*/
	double	latitude;	/*γ��*/
}RETURN_POINT;

//����Ŀ��λ��
typedef struct
{
	uint16 cols;		//ͼ��Ŀ�
	uint16 rows;		//ͼ��ĸ�
	uint16 target_w;	//Ŀ��Ŀ�
	uint16 target_h;	//Ŀ��ĸ�
	uint16 t_x;		//Ŀ�������λ��x
	uint16 t_y;		//Ŀ�������λ��y
	uint8 lock_on;
}TargetPos;

typedef struct
{
	float delta_x;
	float delta_y;
	uint8 lock_on;
}TargetPosLidar;

//usv�ڲ���֪��������
extern char docking_tracker_cfg[30];
extern TargetPos t_pos;
extern char lidar_tracker_cfg[30];
extern TargetPosLidar target_pos_lidar;

//�����ƽ����ṹ��
extern DOCK_SUMLINK dock_sumlink;
extern DOCK_STATE dock_sign;
extern USV_STATE usv_sign;
extern uint8 dockin_fun_enable;
extern RETURN_POINT return_point[RETURN_POINT_MAX_NUMBER];
extern int8 return_num;
extern float docking_speed;

//dock can 
extern COMM_SIGN DOCK_comm_sign;

extern DockingControlCmd docking_control_cmd;//�����������
//���������빦�ܺ���

extern void *dockCommunicationUSVRun(void *);//�� - �뽻������
extern void *cameraTrackPosFlush(void *);//����ͷ��������
extern void *lidarTrackerPosFlush(void *);//�����״��������

//can
extern void initDocksendTask(void);
extern void DOCK_Init();
extern void DOCK_recv(uint8 ps_id, uint8* data);
extern void DOCK_reInit();
extern void DOCK_sendMsg();
extern void DOCK_CommCal(void);
void runDocksendTask(void *);
//los ����
extern uint16 n_ship;//��������
extern float l_ship;//����
extern float temp_dockin_distance;//��ʱ�����������
extern float image_servo_start_distance;//�Ӿ��������
extern float image_servo_end_distance;//�Ӿ��������
extern float image_servo_pix_multi;//���ؾ����ϵ��
extern uint8 motorEnable_sign;//������ʹ�ܱ���λ
extern int8 dockingIn();
extern uint32 dock_number ;
#endif//DOCK_COMMUNICATION_H_