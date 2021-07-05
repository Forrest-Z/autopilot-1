//ins_recorder.cpp

#include "stdafx.h"
#include "../include/MsgQue.h"

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
#include<time.h> //C语言的头文件   
#endif

#ifndef WINNT 
const char g_szInsName[]="../cfg/ins_recorder.txt";		
#else
const char g_szInsName[]="../../cfg/ins_recorder.txt";	
#endif


//#ifndef WINNT 
//const char g_szInsNameBak[]="../cfg/ins_recorderBak.txt";		
//#else
//const char g_szInsNameBak[]="../../cfg/ins_recorderBak.txt";	
//#endif

FILE *pFile_ins;

uint8 insRecordInit( void )
{
	if((pFile_ins = fopen(g_szInsName,"ab+"))==NULL)
	{
		printf("open ins_recoder failed\n");
		return FALSE;
	}
	else
	{
		fprintf(pFile_ins,"时间,经度,纬度,航向角,航速,航迹角,转向率,升沉,横滚,俯仰\n");
		return TRUE;
	}
}


extern void insRecordSave( void )
{
	StruTime systime;

#ifdef WINNT
	SYSTEMTIME sys; 
	GetLocalTime( &sys ); 

	systime.year     = sys.wYear	;
	systime.month    = sys.wMonth	;
	systime.day	   = sys.wDay	;
	systime.hour	   = sys.wHour	;
	systime.minute   = sys.wMonth	;
	systime.second   = sys.wSecond	;
	systime.m_second = sys.wMilliseconds	;
#else

	//系统时间
	struct timeval tv;
	struct timezone tz;
	struct tm* t_tm; 

	gettimeofday(&tv,&tz);
	t_tm = localtime(&tv.tv_sec);

	systime.year     = t_tm->tm_year+1900;
	systime.month    = t_tm->tm_mon	;
	systime.day		 = t_tm->tm_mday	;
	systime.hour	 = t_tm->tm_hour	;
	systime.minute   = t_tm->tm_min	;
	systime.second   = t_tm->tm_sec	;
	systime.m_second = tv.tv_usec				;

	//systime.year   = 2000+Smart_Navigation_St.USV_Year		;
	//systime.month  = Smart_Navigation_St.USV_Month			;
	//systime.day	 = Smart_Navigation_St.USV_Date			;
	//systime.hour	 = 8+Smart_Navigation_St.USV_Hour		;
	//systime.minute = Smart_Navigation_St.USV_Minute		;
	//systime.second = Smart_Navigation_St.USV_Second		;
	//systime.m_second = Smart_Navigation_St.USV_Second_2	;

	//if(systime.hour >=24)
	//{
	//	systime.hour-=24;
	//	systime.day+=1;
	//}
#endif

	fprintf(pFile_ins,"[%04d-%02d-%02d %02d:%02d:%02d.%05d],%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
		systime.year,
		systime.month,
		systime.day,
		systime.hour,
		systime.minute,
		systime.second,
		systime.m_second,
		/*Smart_Navigation_St.USV_Lat,				*/	ins_msg.longitude	,
		/*Smart_Navigation_St.USV_Lng,				*/  ins_msg.latitude	,
		/*Smart_Navigation_St.USV_Heading,			*/	ins_msg.heading		,
		/*Smart_Navigation_St.USV_Speed,			*/	ins_msg.speed		,
		/*Smart_Navigation_St.USV_Position_Heading,	*/	ins_msg.motionDirection	,				//航迹角
		/*Smart_Navigation_St.USV_ROT,				*/	ins_msg.i16_roll		,				//转向率
		/*Smart_Navigation_St.USV_Heave,			*/	ins_msg.i16_heaving		,				//升沉
		/*Smart_Navigation_St.USV_Roll,				*/	ins_msg.i16_roll		,				//横摇
		/*Smart_Navigation_St.USV_Pitch			    */	ins_msg.i16_pitch						//俯仰

		);	
}



void insRecordClose( void )
{
	fclose(pFile_ins);
}




