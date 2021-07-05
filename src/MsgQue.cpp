//MSgQue.cpp

#include "stdafx.h"
#include "../include/MsgQue.h"



/*********************************************************************************
	** implementation of functions **
**********************************************************************************/


MsgQue_ID MsgQueCreate( int nMsgs )
{
	StruOpMsg *msgbuf;
	MsgQue_ID msgId = (MsgQue_ID)malloc(sizeof(MsgQue));

	if(msgId == NULL)
		return (NULL);


	//if(msgbuf == NULL)
	//{
	//	free((char *)msgId);
	//	return (NULL);
	//}

	msgId->bufSize = nMsgs;

	MsgQueFlush(msgId);

	return (msgId);
}
/*******************************************************************************/
/*******************************************************************************/

void MsgQueDelete(MsgQue_ID msgId)
{
	free(msgId->opMsg);
	free((char *)msgId);
}
/*******************************************************************************/
/*******************************************************************************/

void MsgQueFlush(MsgQue_ID msgId)
{
	msgId->pToBuf = 0;
	msgId->pFromBuf = 0;
}

/*******************************************************************************/
/*******************************************************************************/

int MsgQueMsgGet(MsgQue_ID msgId,StruOpMsg *pdata)
{
	int pToBuf = msgId->pToBuf;
	
	int pMsgTmp = 0;

	//isEmpty
	if(MsgQueIsEmpty(msgId))
	{
		return (0);
	}

	memcpy((char *)pdata,(char *)&(msgId->opMsg[msgId->pFromBuf]),sizeof(StruOpMsg));

	pMsgTmp = msgId->pFromBuf+1;

	if(pMsgTmp == msgId->bufSize)
	{
		pMsgTmp = 0;
	}

	msgId->pFromBuf = pMsgTmp;
	
	return(1);
}

/*******************************************************************************/
/*******************************************************************************/

int MsgQueMsgPut(MsgQue_ID msgId,StruOpMsg *pdata)
{
	int pFromBuf = msgId->pFromBuf;
	int pMsgTmp = 0;

	pMsgTmp = msgId->pToBuf + 1;
	if(pMsgTmp == msgId->bufSize)
	{
		pMsgTmp = 0;
	}
	
	//ring full
	if(pMsgTmp == msgId->pFromBuf)
	{
		MsgQueFlush(msgId);
		return 0;
	}

	memcpy((char *)&msgId->opMsg[msgId->pToBuf],(char *)pdata,sizeof(StruOpMsg));
	msgId->pToBuf = pMsgTmp;
	return 1;
}

/*******************************************************************************/
/*******************************************************************************/
uint8 MsgQueIsEmpty(MsgQue_ID msgId )
{
	return (msgId->pToBuf == msgId->pFromBuf);
}

/*******************************************************************************/
/*******************************************************************************/
uint8 MsgQueIsFull( MsgQue_ID msgId )
{
	int n = msgId->pToBuf - msgId->pFromBuf + 1;

	return((n==0)||(n== msgId->bufSize));
}

/*******************************************************************************/
/*******************************************************************************/
int MsgQueNMsgs(MsgQue_ID msgId)
{
	int n = msgId->pToBuf - msgId->pFromBuf;

	if(n < 0)
		n += msgId->bufSize;

	return(n);
}