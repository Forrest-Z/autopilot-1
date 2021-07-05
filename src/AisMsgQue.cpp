//2017/9/19 9:29:36
//shaoyuping
//AisMsgQue.cpp

#include "stdafx.h"
#include "../include/AisMsgQue.h"


/*********************************************************************************
	** implementation of functions **
**********************************************************************************/
AisMsgQue_ID AisMsgQueCreate( int nMsgs )
{
	StruAisMsg *aisMsgBuf;
	AisMsgQue_ID aisMsgId = (AisMsgQue_ID)malloc(sizeof(AisMsgQue));

	if(aisMsgId == NULL)
		return (NULL);

	aisMsgId->bufSize = nMsgs;
	AisMsgQueFlush(aisMsgId);
	return (aisMsgId);
}

/*******************************************************************************/
/*******************************************************************************/

void AisMsgQueDelete( AisMsgQue_ID aisMsgId )
{
	free(aisMsgId->opAisMsg);
	free((char *)aisMsgId);
}

/*******************************************************************************/
/*******************************************************************************/

void AisMsgQueFlush( AisMsgQue_ID aisMsgId )
{
	aisMsgId->pFromBuf = 0;
	aisMsgId->pToBuf = 0;
}

/*******************************************************************************/
/*******************************************************************************/

int AisMsgQueGet( AisMsgQue_ID aisMsgId,StruAisMsg *pdata )
{
	int pToBuf = aisMsgId->pToBuf;

	int pMsgTmp = 0;

	//isEmpty
	if(AisMsgQueIsEmpty(aisMsgId))
	{
		return (0);
	}

	memcpy((char *)pdata,(char *)&aisMsgId->opAisMsg[aisMsgId->pFromBuf],sizeof(StruAisMsg));
	pMsgTmp = aisMsgId->pFromBuf+1;

	if(pMsgTmp == aisMsgId->bufSize)
	{
		pMsgTmp = 0;
	}

	aisMsgId->pFromBuf = pMsgTmp;

	return(1);
}

/*******************************************************************************/
/*******************************************************************************/
int AisMsgQuePut( AisMsgQue_ID aisMsgId,StruAisMsg *pdata )
{
	int pFromBuf = aisMsgId->pFromBuf;
	int pMsgTmp = 0;

	pMsgTmp = aisMsgId->pToBuf + 1;
	if(pMsgTmp == aisMsgId->bufSize)
	{
		pMsgTmp = 0;
	}

	//ring full
	if(pMsgTmp == aisMsgId->pFromBuf)
	{
		AisMsgQueFlush(aisMsgId);
		return 0;
	}

	memcpy((char *)&aisMsgId->opAisMsg[aisMsgId->pToBuf],(char *)pdata,sizeof(StruAisMsg));
	aisMsgId->pToBuf = pMsgTmp;
	return 1;

}


/*******************************************************************************/
/*******************************************************************************/
uint8 AisMsgQueIsEmpty( AisMsgQue_ID aisMsgId )
{
	return(aisMsgId->pToBuf == aisMsgId->pFromBuf);
}

/*******************************************************************************/
/*******************************************************************************/
uint8 AisMsgQueIsFull( AisMsgQue_ID aisMsgId )	
{
	int n = aisMsgId->pToBuf - aisMsgId->pFromBuf + 1;
	
	return((n==0)||(n== aisMsgId->bufSize));
}



/*******************************************************************************/
/*******************************************************************************/
int AisMsgQueNMsgs(AisMsgQue_ID aisMsgId)
{
	int n = aisMsgId->pToBuf - aisMsgId->pFromBuf;

	if(n<0)
		n+=aisMsgId->bufSize;

	return(n);
}




AisMsgQue_ID aisMsgQ;

void AisMsgInit( void )
{
	aisMsgQ = AisMsgQueCreate(128);
}



uint8 AisMsgPost( uint8 *pAis_msg )
{
	int length = pAis_msg[0];
	
	StruAisMsg aisMsgTmp;
	memset((char *)&aisMsgTmp.frame_length,0,sizeof(StruAisMsg));

	memcpy((char *)&aisMsgTmp,(char *)pAis_msg,length+1);

	AisMsgQuePut(aisMsgQ,&aisMsgTmp);

	return 1;
}

uint8 AisMsgCost( StruAisMsg *ptoAis_msg )
{
	StruAisMsg aisMsgTmp;
	int length;

	if(AisMsgQueIsEmpty(aisMsgQ))
	{
		return 0;
	}

	AisMsgQueGet(aisMsgQ,ptoAis_msg);
	

	return 1;
}









