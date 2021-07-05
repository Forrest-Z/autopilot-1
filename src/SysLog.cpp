//SysLog.cpp

#include "stdafx.h"
#include "../include/SysLog.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#ifdef WINNT
#include <direct.h>  
#include <io.h>  
#endif




#ifdef WINNT
	#include <windows.h>
#else
	#include<time.h> //C���Ե�ͷ�ļ�   
#endif


#ifndef WINNT 
const char g_szDatDir[]="../cfg";		//usv������־
#else
const char g_szDatDir[]="../../cfg";		//usv������־
#endif

#ifndef WINNT 
const char g_szDatName[]="../cfg/syslog.txt";		
#else
const char g_szDatName[]="../../cfg/syslog.txt";	
#endif


#ifndef WINNT 
const char g_szDatNameBak[]="../cfg/syslogbak.txt";		
#else
const char g_szDatNameBak[]="../../cfg/syslogbak.txt";	
#endif

MsgQue_ID sysLogMsgQ;
int	SysLogSaveTime=0;

//extern StruTime SysClock;

uint8 SysLogMsgRead(char* szBuf);
uint8 LogNeedSave(void);
void SysLogSave(void);

int  GetLogMsgNum(void);


//DWORD FileGetLength(FILE *pFile);
//void FileSeekToEnd(FILE *pFile);
int GetPassedTime(int time1,int time2);
int GetTick(void);

/*********************************************************************************
	** implementation of functions **
**********************************************************************************/

void SysLogInit(void)
{
	if(MKDIR(g_szDatDir)==0)	//Ŀ¼������
	{
		printf("mk log dir dat error\n");
		return;
	}

	sysLogMsgQ = MsgQueCreate(MAX_QUE_DEPTH_LOG);
	
	//ʱ��� todo
	SysLogSaveTime = GetTick();

//	SysLogMsgPost("��־����");
//	SysPubMsgPost("������־����");
}


/*********************************************************************************
**********************************************************************************/
void SysLogSave(void)
{
	FILE *pFile;
	char lzBuf[200];
	unsigned long dwLen;
	uint8 bEof = 0x0a;

	if(!LogNeedSave())
		return;

	if((pFile = fopen(g_szDatName,"ab+"))==NULL)
	{
		printf("open syslog file failed\n");
		return;
	}

	do 
	{
		//dwLen = FileGetLength(pFile);
		dwLen = ftell(pFile);

		fseek(pFile,0,2);   //ָ�룺�ƶ����ļ�β��
		dwLen = ftell(pFile);   //����ָ��ƫ���ļ�ͷ��λ��(���ļ����ַ�����)


		if(dwLen > MAX_LOGFILE_LEN)
		{
			fclose(pFile);

			remove(g_szDatNameBak);

			rename(g_szDatName,g_szDatNameBak);

			remove(g_szDatName);

		//	SysLogMsgPost("syslog.txtת��syslogbak.txt");
			return;
		}

		if(!SysLogMsgRead(lzBuf))
		{
			fclose(pFile);
			return;
		}


		
		fwrite(lzBuf, 1, strlen(lzBuf), pFile);
		fwrite(&bEof, 1, sizeof(bEof),pFile);

	} while (GetLogMsgNum());

	fclose(pFile);

	//todo
	SysLogSaveTime = GetTick();
}

/*********************************************************************************
**********************************************************************************/
uint8 SysLogMsgRead(char* szBuf)
{
	StruOpMsg curRcd;

	if(MsgQueMsgGet(sysLogMsgQ,&curRcd)==0)
	{
		return 0;
	}

	sprintf(szBuf,"[%04d-%02d-%02d %02d:%02d:%02d.%05d]  %s",
		curRcd.SysTime.year,
		curRcd.SysTime.month,
		curRcd.SysTime.day,
		curRcd.SysTime.hour,
		curRcd.SysTime.minute,
		curRcd.SysTime.second,
		curRcd.SysTime.m_second,
		curRcd.msgbuf);

	return 1;
}

/*********************************************************************************
**********************************************************************************/
int GetLogMsgNum(void)
{
	return MsgQueNMsgs(sysLogMsgQ);
}

/*********************************************************************************
**********************************************************************************/
uint8 LogNeedSave(void)
{
	if(GetLogMsgNum() == 0)
		return 0;

	if(GetLogMsgNum() > MAX_QUE_DEPTH_LOG/2)
		return 1;

	if(GetPassedTime(GetTick(),SysLogSaveTime) >= 50)
		return 1;

	return 1;
}

/*********************************************************************************
**********************************************************************************/
void ReadSysTime(StruTime *systime)
{	
	//StruTime volatile *p = &SysClock;


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
	
	//ϵͳʱ��
	struct timeval tv;
	struct timezone tz;
	struct tm* t_tm; 

	gettimeofday(&tv,&tz);
	t_tm = localtime(&tv.tv_sec);

	systime->year     = t_tm->tm_year+1900;
	systime->month    = t_tm->tm_mon	;
	systime->day	   = t_tm->tm_mday	;
	systime->hour	   = t_tm->tm_hour	;
	systime->minute   = t_tm->tm_min	;
	systime->second   = t_tm->tm_sec	;
	systime->m_second = tv.tv_usec				;

	//�ߵ�ʱ��
	//systime->year   = 2000+Smart_Navigation_St.USV_Year	;
	//systime->month  = Smart_Navigation_St.USV_Month		;
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
#endif


}

/*********************************************************************************
**********************************************************************************/
uint8 SysLogMsgPost(const char *fmt,...)
{
	StruOpMsg curRcd;
	va_list ap;
	char szLog[200];

	va_start(ap,fmt);
	vsprintf(szLog,fmt,ap);
	va_end(ap);

	ReadSysTime(&curRcd.SysTime);
	strcpy(curRcd.msgbuf,szLog);

	MsgQueMsgPut(sysLogMsgQ,&curRcd);

	return 1;
}

/*********************************************************************************
**********************************************************************************/
void logTest(void)
{
//	SysLogMsgPost("��־���� %d %d %d %d",1,2,3,4);
}

/*********************************************************************************
**********************************************************************************/
int GetTick( void )
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


/*********************************************************************************
**********************************************************************************/
int GetPassedTime( int time1,int time2 )
{
	int timediff;
	timediff = time2-time1;
	return timediff;
}
