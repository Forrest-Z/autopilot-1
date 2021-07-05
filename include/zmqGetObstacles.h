#ifndef __ZMQ_GET_OBSTACLES__H_
#define __ZMQ_GET_OBSTACLES__H_

#include "usv_include.h"
#include "zhelper.h"

#define OBS_NUM_MAX 256
#define OBS_COMM_DISCONNECT_MAX 100
typedef struct{
	double lat;			//γ��		8   // ��Ծ���
	double lng;			//����		16  // ��Է�λ
	float  radius;		//�뾶		20
	float  velocityMag;	//�ٶȴ�С	24
	float  velocityDir;	//�ٶȷ���	28
	uint8  type;        // 29
}OBS_ATTR;	//�ϰ�������

typedef struct{
	uint16		obsNum;					//�ϰ������	���256
	int8		obsSenserValid;			//��������Ч -1�쳣 0����
	int8		obsInsValid;			//imu��Ч -1�쳣 0����
	OBS_ATTR	obsAttr[OBS_NUM_MAX];	//
}OBS_STRU;

extern OBS_STRU	obs_var;
extern char obsCommCfg[30];
extern uint8 log_b1_obsComm_sign;

void obsCommCal(void *);
void obsClear(void);
extern inline uint8 obsPerceptionComm(void);
extern void obsCommCalInit(void);
extern void *zmq_getObstacles(void *aa);
extern void *zmq_pub_ins(void *aa);



#endif /*__ZMQ_GET_OBSTACLES__H_*/