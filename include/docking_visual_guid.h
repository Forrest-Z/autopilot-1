/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人��������޹�˾
-	File name  : docking_visual_guid.h
-	Author	   : fushuai
-	Date	   : 2019/05/28 11:34
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#ifndef  __DOCK_VISUAL_GUID_H_
#define  __DOCK_VISUAL_GUID_H_
#include "usv_include.h"

/* ����ͷ���� �ṹ�� */
typedef struct
{
	uint16 cols;		//ͼ��Ŀ�
	uint16 rows;		//ͼ��ĸ�
	uint16 target_w;	//Ŀ��Ŀ�
	uint16 target_h;	//Ŀ��ĸ�
	uint16 t_x;		//Ŀ�������λ��x
	uint16 t_y;		//Ŀ�������λ��y
	uint8 b_locking;	//Ŀ���Ƿ�����
}ImageInfo_t;


typedef struct
{
	unsigned short	request_id;
	unsigned short	dockin_cmd; //ȱʡ 0  ����1 ����2
}DockCmd;

extern char docking_zmq_cfg[30];
extern DockCmd dock_zmq_cmd;
extern void *cameraComm(void *aa);
extern void initImageControlParms();
int8 imageServo_yawControl(int16 *rudder_deg);
float imageServo(float target_dis);
#endif //DOCK_VISUAL_GUID_H_