
#ifndef _MSGQUE_H
#define _MSGQUE_H

#include "usv_include.h"

typedef  uint16 WORD;

typedef struct  
{
	WORD	msgNo;
	WORD	year;
	WORD	month;
	WORD	day;
	WORD	hour;
	WORD	minute;
	WORD	second;
	WORD	m_second;
}StruTime;


typedef struct
{
	StruTime SysTime;
	char msgbuf[128];
}StruOpMsg;



typedef struct
{
    int pToBuf;
    int pFromBuf;
    int bufSize;
    StruOpMsg opMsg[128];    //操作信息
}MsgQue;

typedef MsgQue *MsgQue_ID;

MsgQue_ID MsgQueCreate(int nMsgs);
void    MsgQueDelete(MsgQue_ID msgId);
void    MsgQueFlush(MsgQue_ID msgId);
int     MsgQueMsgGet(MsgQue_ID msgId,StruOpMsg *pdata);
int     MsgQueMsgPut(MsgQue_ID msgId,StruOpMsg *pdata);
uint8   MsgQueIsEmpty(MsgQue_ID msgId);
uint8   MsgQueIsFull(MsgQue_ID msgId);
int   MsgQueNMsgs(MsgQue_ID msgId);   //返回队列消息个数


#endif //_MSGQUE_H