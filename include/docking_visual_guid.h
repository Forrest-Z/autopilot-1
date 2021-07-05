/********************************************************************
-	Copyright (c),2017-	,四方继保（武汉）软件有限公司
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

/* 摄像头像素 结构体 */
typedef struct
{
	uint16 cols;		//图像的宽
	uint16 rows;		//图像的高
	uint16 target_w;	//目标的宽
	uint16 target_h;	//目标的高
	uint16 t_x;		//目标的像素位置x
	uint16 t_y;		//目标的像素位置y
	uint8 b_locking;	//目标是否锁定
}ImageInfo_t;


typedef struct
{
	unsigned short	request_id;
	unsigned short	dockin_cmd; //缺省 0  跟踪1 结束2
}DockCmd;

extern char docking_zmq_cfg[30];
extern DockCmd dock_zmq_cmd;
extern void *cameraComm(void *aa);
extern void initImageControlParms();
int8 imageServo_yawControl(int16 *rudder_deg);
float imageServo(float target_dis);
#endif //DOCK_VISUAL_GUID_H_