//2017/9/19 9:29:36
//shaoyuping
//AisMsgQue.h


#ifndef _AISMSGQUE_H
#define _AISMSGQUE_H

#include "usv_include.h"


typedef struct
{
    uint8   frame_length;
    uint8   frame_buff[119];
}StruAisMsg;


typedef struct{
    int pToBuf;
    int pFromBuf;
    int bufSize;
    StruAisMsg opAisMsg[128];   //������Ϣ
}AisMsgQue;

typedef AisMsgQue *AisMsgQue_ID;

AisMsgQue_ID AisMsgQueCreate(int nMsgs);
void         AisMsgQueDelete(AisMsgQue_ID aisMsgId);
void         AisMsgQueFlush(AisMsgQue_ID aisMsgId);
int          AisMsgQueGet(AisMsgQue_ID aisMsgId,StruAisMsg *pdata);
int          AisMsgQuePut(AisMsgQue_ID aisMsgId,StruAisMsg *pdata);
uint8        AisMsgQueIsEmpty(AisMsgQue_ID aisMsgId);
uint8        AisMsgQueIsFull(AisMsgQue_ID aisMsgId);
int          AisMsgQueNMsgs(AisMsgQue_ID aisMsgId);     //���ض��и���



//application part
extern void  AisMsgInit(void);
extern uint8 AisMsgPost(uint8 *pAis_msg);		//��������
extern uint8 AisMsgCost(StruAisMsg *ptoAis_msg);	//��������


#endif  //_AISMSGQUE_H



