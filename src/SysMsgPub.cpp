/**********************************  Include  ********************************/
#include "stdafx.h"
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/ws.h>
#include "../include/SysMsgPub.h"
#include <iconv.h>

/******************************  Local Variable  *****************************/
MsgQue_ID	sysPubMsgQ;	
int	SysPubMsgSaveTime=0;
void *context;		
void *publisher;	
void *zsync;

//nanomsg variable
int nanoSock;
char nanoSysMsgAddr[30];

/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
int GetTimeDiff(int time1,int time2);	//��ȡʱ���
int GetTime(void);						//��ȡʱ���
void GetSysTime(StruTime *systime);
uint8 MsgNeedPub(void);
int	GetPubMsgNum(void);
uint8 SysPubMsgRead(char* szBuf);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

int gb2312ToUtf8(char *sOut, int iMaxOutLen, const char *sIn, int iInLen)
{
	char *pIn  = (char *)sIn;
	char *pOut = sOut;
	size_t ret;
	size_t iLeftLen = iMaxOutLen;
	iconv_t cd;

	cd = iconv_open("utf-8","gb2312");
	if(cd == (iconv_t) - 1)
	{
		printf("iconv open error\n");
		return -1;	
	}
	size_t iSrcLen = iInLen;
	ret = iconv(cd, &pIn, &iSrcLen, &pOut, &iLeftLen);
	*pOut = '\0';	
	if(ret == (size_t) -1 )
	{
		printf("iconv error  ret = %d\n",ret);
		iconv_close(cd);
		return -1;		
	}
	iconv_close(cd);
	return (iMaxOutLen - iLeftLen);
}


void SysPubMsgInit( void )
{
	sysPubMsgQ = MsgQueCreate(MAX_POST_QUE_DEPTH_LOG);

	SysPubMsgSaveTime = GetTime();

	//zmqInit
	context = zmq_ctx_new();

	zsync = zmq_socket(context,ZMQ_PULL);
	zmq_bind(zsync,"tcp://*:5564");

	publisher = zmq_socket(context,ZMQ_PUB);

	zmq_bind(publisher,"tcp://*:5563");

	//nanomsgInit
	nanoSock = nn_socket(AF_SP,NN_PUB);
	if(nanoSock<0){
		printf("init nano sock error!\n");
	}

	int opt = NN_WS_MSG_TYPE_TEXT;
	nn_setsockopt(nanoSock,NN_WS,NN_WS_MSG_TYPE,&opt,sizeof(opt));
	if(nn_bind(nanoSock,nanoSysMsgAddr)<0){
		printf("nano bind error");
	}

}

uint8 SysPubMsgPost( const char *fmt,... )
{
	StruOpMsg curRcd;
	va_list ap;
	char szLog[200];

	va_start(ap,fmt);
	vsprintf(szLog,fmt,ap);
	va_end(ap);

	GetSysTime(&curRcd.SysTime);
	strcpy(curRcd.msgbuf,szLog);

	MsgQueMsgPut(sysPubMsgQ,&curRcd);

	return 1;
}

void SysMsgPub( void )
{
	char lzBuf[200];
	const char lzEnvelope[] = "usvPostMsg";
	
	////�ȴ�ͬ����Ϣ
	//char buffer[256];
	//int size = zmq_recv(zsync,buffer,255,3);
	//if(size = -1)
	//	return;
	//free(buffer);

	memset(lzBuf, 0, sizeof(lzBuf));

	if(!MsgNeedPub())
		return;

	if(!SysPubMsgRead(lzBuf))
	return;



	s_sendmore(publisher,(char *)lzEnvelope);
	s_send(publisher,lzBuf);



	char utf8buf[200];
	int  utf8len;
	utf8len = gb2312ToUtf8(utf8buf,200,lzBuf,strlen(lzBuf));
	if(utf8len > 0){
		int nnbytes = nn_send(nanoSock,utf8buf,strlen(utf8buf),0);	//����utf8
		printf("%s\n",lzBuf);
		if(nnbytes < 0){
			printf("nn_send failed: %s\n",nn_strerror(errno));
		}
	}

}


int GetTimeDiff( int time1,int time2 )
{
	int timediff;
	timediff = time2-time1;
	return timediff;
}

int GetTime( void )
{
	int timeTick;
#ifdef WINNT
	timeTick = GetTickCount();
#else
	struct timeval time_stru;
	gettimeofday( &time_stru, NULL );
	timeTick = (1000 *  ( time_stru.tv_sec) + (time_stru.tv_usec)*0.001); 
#endif
	return timeTick;
}

void GetSysTime( StruTime *systime )
{
	static uint16 msgNo = 0;
	systime->msgNo = msgNo++;
#ifdef WINNT
	SYSTEMTIME sys; 
	GetLocalTime( &sys ); 

	systime->year     = sys.wYear	;
	systime->month    = sys.wMonth	;
	systime->day	   = sys.wDay	;
	systime->hour	   = sys.wHour	;
	systime->minute   = sys.wMonth	;
	systime->second   = sys.wSecond	;
	systime->m_second = sys.wMilliseconds	;
#else
	struct timeval tv;
	struct timezone tz;
	struct tm* t_tm; 

	gettimeofday(&tv,&tz);
	t_tm = localtime(&tv.tv_sec);



	//time_t timer;   
	//time(&timer);   
	//t_tm = localtime(&timer); 

	systime->year     = t_tm->tm_year+1900	;
	systime->month    = t_tm->tm_mon+1	;
	systime->day	   = t_tm->tm_mday	;
	systime->hour	   = t_tm->tm_hour	;
	systime->minute   = t_tm->tm_min	;
	systime->second   = t_tm->tm_sec	;
	systime->m_second = tv.tv_usec/1000;



	//systime->year   = 2000+Smart_Navigation_St.USV_Year		;
	//systime->month  = Smart_Navigation_St.USV_Month			;
	//systime->day	 = Smart_Navigation_St.USV_Date			;
	//systime->hour	 = 8+Smart_Navigation_St.USV_Hour		;
	//systime->minute = Smart_Navigation_St.USV_Minute		;
	//systime->second = Smart_Navigation_St.USV_Second		;
	//systime->m_second = Smart_Navigation_St.USV_Second_2	;

	//if(systime->hour >=24)
	//{
	//	systime->hour-=24;
	//	systime->day+=1;
	//}
	//printf("Local time is %s/n",asctime(t_tm)); 
#endif
}

uint8 MsgNeedPub( void )
{
	if(GetPubMsgNum()==0)
		return 0;
	if(GetPubMsgNum()>MAX_POST_QUE_DEPTH_LOG/16)
		return 1;
	if(GetTimeDiff(GetTime(),SysPubMsgSaveTime)>=50);
		return 1;

	return 0;

}


int GetPubMsgNum( void )
{
	return MsgQueNMsgs(sysPubMsgQ);
}

uint8 SysPubMsgRead( char* szBuf )
{
	StruOpMsg curRcd;

	if(MsgQueMsgGet(sysPubMsgQ,&curRcd)==0)
	{
		return 0;
	}

	sprintf(szBuf,"%04d-%02d-%02d %02d:%02d:%02d.%03d,[%05d] %s",
		curRcd.SysTime.year,
		curRcd.SysTime.month,
		curRcd.SysTime.day,
		curRcd.SysTime.hour,
		curRcd.SysTime.minute,
		curRcd.SysTime.second,
		curRcd.SysTime.m_second,
		curRcd.SysTime.msgNo,
		curRcd.msgbuf);

	return 1;
}

extern void PubMsgTest( void )
{
	static uint16 u16_cntr=0;
	u16_cntr++;
//	SysPubMsgPost("������Ϣ���� %d",u16_cntr);
}



