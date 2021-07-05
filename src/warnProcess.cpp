
/*==========================================================*
* 模块说明: warnProcess.cpp                                             *
* 文件版本: v1.00 (说明本文件的版本信息)                   *
* 开发人员:                                                *
* 创建时间: 				                                *
* Copyright(c) sf-auto.ltd									*
*==========================================================*
* 程序修改记录(最新的放在最前面):                          *
*  <修改日期>, <修改人员>: <修改功能概述>                  *
*==========================================================*
*=========================================================*/

/**********************************  Include  ********************************/
#include "stdafx.h"
#include <stdarg.h>
#include <string>
#include "../include/usv_include.h"
#include "../include/commMsgQueue/commMsgQueue.h"
#include <iostream>
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/ws.h>
#include <time.h>
#ifdef WINNT
#include <windows.h>
#else
#include <sys/time.h>
#endif

using namespace std;
/******************************  Local Variable  *****************************/
int arm_warn_number;
vector <string> vec_arm_warn_string;
int ihc_warn_number;
vector <string> vec_ihc_warn_string;
CMsgQueue <WARN_MSG> *m_pWarnQueueNanoMsg;
CMsgQueue <WARN_MSG> *m_pWarnQueueUdp;

int warnNanoSock;
char warnNanoAddr[30];

/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

int read_warn_setting(void)
{
	FILE *pFile;
	int8 *p_file_memory;				//缓存区
	int32 *p_buffer;
	uint32 lSize;
	int32 result;
	uint32 len;
	int8 s1[32];
	int8 s2[50];
	uint16	loop_i;
	int8	ret_val = TRUE;
	char tm_warn_name[WARN_MSG_LEN_MAX];
	string tm_string;

	pFile = fopen(ARM_WARN_CFG_FILE_NAME, "r+");
	
	if (pFile == NULL){
		printf("read warn setting error\n");
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);				//16K
	if (NULL == p_file_memory)
	{
		printf("warn setting file memory not enough\n");
		fclose(pFile);
		return FALSE;
	}

	p_buffer = (int32 *)malloc(0x10000);				//64K
	if (NULL == p_buffer)
	{
		printf("warn setting explain memory\n");
		free(p_file_memory);
		fclose(pFile);
		return FALSE;
	}

	// 获取文件大小 
	fseek(pFile, 0, SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile);					//将指针指向文件开头

	if (lSize >= 0xffff){
		printf("warn setting read file too large\n");
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}


	result = fread(p_file_memory, 1, lSize, pFile);			 // 将文件拷贝到buffer中   

	if (32768 < lSize) len = 2 * lSize;
	else len = 1280 + 2 * lSize;
	len = len * 2;
	result = ini_Initialize((char *)p_file_memory, p_buffer, len);

	if (result != 0){										//文件初始化出错
		printf("warn setting  memory explain error\n");
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

	//开始解析配置文件
	//解析arm告警配置文件
	sprintf_usv(s1,"ARM_WARN_SETTING");
	sprintf_usv(s2, "Arm_Warn_Setting_Number"); 
	if (read_sub_setting(s1, s2, 0, (uint32 *)&arm_warn_number, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}
	for (loop_i = 0; loop_i < arm_warn_number; loop_i++){
		sprintf_usv(s2, "Arm_Warn_Name_%d", loop_i);
		memset(tm_warn_name, 0, sizeof(tm_warn_name));
		read_sub_setting_string(s1, s2, 0, (char*)tm_warn_name);
		vec_arm_warn_string.push_back(tm_warn_name);
	}

	//for (int i = 0; i < arm_warn_number; i++){
	//	printf("sn:%d	content:%s \n", i, vec_arm_warn_string[i].c_str());
	//}

	//解析IHC告警配置文件
	sprintf_usv(s1, "IHC_WARN_SETTING");
	sprintf_usv(s2, "IHC_Warn_Setting_Number");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&ihc_warn_number, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}
	for (loop_i = 0; loop_i < ihc_warn_number; loop_i++){
		sprintf_usv(s2, "IHC_Warn_Name_%d", loop_i);
		memset(tm_warn_name, 0, sizeof(tm_warn_name));
		read_sub_setting_string(s1, s2, 0, (char*)tm_warn_name);
		vec_ihc_warn_string.push_back(tm_warn_name);
	}

	//for (int i = 0; i < ihc_warn_number; i++){
	//	printf("sn:%d	content:%s \n", i, vec_ihc_warn_string[i].c_str());
	//}

	//解析告警通讯端口
	memset(warnNanoAddr, 0, sizeof(warnNanoAddr));
	sprintf_usv(s1, "WARN_NANO_SOCK");
	sprintf_usv(s2, "Warn_Nano_Addr");
	read_sub_setting_string(s1, s2, 0, (char*)warnNanoAddr);

	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	printf("read warn setting ok\n");
	return ret_val;

}

void initWarnMsgQueue(void)
{
	m_pWarnQueueNanoMsg = new CMsgQueue<WARN_MSG>(100);
	m_pWarnQueueNanoMsg->QueueSetEmpty();

	m_pWarnQueueUdp = new CMsgQueue<WARN_MSG>(100);
	m_pWarnQueueUdp->QueueSetEmpty();
}

void WarnMsgQueuePut(int warnSource, int warnSquence, int8 warn_stat)
{
	WARN_MSG tm_warnMsg;
	tm_warnMsg.warn_source = warnSource;
	tm_warnMsg.warn_squence = warnSquence;
	tm_warnMsg.warn_stat = warn_stat;
	ReadWarnTime(&tm_warnMsg.warn_time);
	m_pWarnQueueNanoMsg->QueueAddElement(&tm_warnMsg);
	m_pWarnQueueUdp->QueueAddElement(&tm_warnMsg);
}

int initNanoPubSender(char *nanoAddr)
{
	warnNanoSock = nn_socket(AF_SP, NN_PUB);
	if (warnNanoSock<0)
	{
		return -1;
	}

	int opt = NN_WS_MSG_TYPE_TEXT;
	nn_setsockopt(warnNanoSock, NN_WS, NN_WS_MSG_TYPE, &opt, sizeof(opt));

	if (nn_bind(warnNanoSock, warnNanoAddr)<0)
	{
		return -1;
	}
	return 1;
}

void warnNanoPub(char *warnMsg)
{
	char utf8buf[200];
	int  utf8len;

	memset(utf8buf, 0, sizeof(utf8buf));

	utf8len = gb2312ToUtf8(utf8buf, 200, warnMsg, strlen(warnMsg));
	
	if (utf8len > 0)
	{
		int nnbytes = nn_send(warnNanoSock, utf8buf, strlen(utf8buf), 0);
		if (nnbytes < 0)
		{
			printf("warn nn_send failed: %s\n", nn_strerror(errno));
		}
	}

	
	//nn_send(warnNanoSock, warnMsg, strlen(warnMsg), 0);
}


void ReadWarnTime(WARN_TIME *systime)
{
#ifdef WINNT
	SYSTEMTIME sys;
	GetLocalTime(&sys);

	systime->year = sys.wYear;
	systime->month = sys.wMonth;
	systime->day = sys.wDay;
	systime->hour = sys.wHour;
	systime->minute = sys.wMinute;
	systime->second = sys.wSecond;
	systime->m_second = sys.wMilliseconds;
#else
	//系统时间
	struct timeval tv;
	struct timezone tz;
	struct tm* t_tm;

	gettimeofday(&tv, &tz);
	t_tm = localtime(&tv.tv_sec);

	systime->year = t_tm->tm_year + 1900;
	systime->month = t_tm->tm_mon + 1;
	systime->day = t_tm->tm_mday;
	systime->hour = t_tm->tm_hour;
	systime->minute = t_tm->tm_min;
	systime->second = t_tm->tm_sec;
	systime->m_second = tv.tv_usec;
#endif
}

void WarnMsgQueuGetPub(void)
{
	WARN_MSG tm_warnMsg;
	string warnNameString;
	string warnStatString;
	uint16 warnSeqNet = 0;
	char warnBuf[200];
	
	memset(warnBuf, 0, sizeof(warnBuf));

	if (m_pWarnQueueNanoMsg->QueueIsEmpty() == 0){
	//	printf("Warn Queue is Empty\n");
		return;
	}
	
	m_pWarnQueueNanoMsg->QueueDelElement(&tm_warnMsg);

	switch (tm_warnMsg.warn_source)
	{
		case WARN_SRC_ARM:
			if (tm_warnMsg.warn_squence < vec_arm_warn_string.size()){
				warnNameString = vec_arm_warn_string[tm_warnMsg.warn_squence];
				warnSeqNet = tm_warnMsg.warn_squence + ARM_WARN_SQ_BASE;
				break;
			}
			else{
				warnNameString = "warn from arm: sequence out of limit";
				break;
			}
		case WARN_SRC_IHC:
			if (tm_warnMsg.warn_squence < vec_ihc_warn_string.size()){
				warnNameString = vec_ihc_warn_string[tm_warnMsg.warn_squence];
				warnSeqNet = tm_warnMsg.warn_squence + IHC_WARN_SQ_BASE;
				break;
			}
			else{
				warnNameString = "warn from ihc: sequence out of limit";
				break;
			}
		default:
			warnNameString = "warn source error";
			break;
	}

	if (tm_warnMsg.warn_stat == 1){
		warnStatString = "告警";
	}
	else if (tm_warnMsg.warn_stat == 0)
	{
		warnStatString = "恢复";
	}
	else
	{
		warnStatString = "状态未定义";
	}
	
	sprintf(warnBuf, "%04d-%02d-%02d %02d:%02d:%02d.%05d,<%04d> %s %s",
		tm_warnMsg.warn_time.year,
		tm_warnMsg.warn_time.month,
		tm_warnMsg.warn_time.day,
		tm_warnMsg.warn_time.hour,
		tm_warnMsg.warn_time.minute,
		tm_warnMsg.warn_time.second,
		tm_warnMsg.warn_time.m_second,
		warnSeqNet,
		warnNameString.c_str(),
		warnStatString.c_str());

	printf("%s\n", warnBuf);
	warnNanoPub(warnBuf);
}

int iniWarnModel(void)
{
	int iret = FALSE;
	iret = read_warn_setting();
	if (iret == TRUE){
		initWarnMsgQueue();
		initNanoPubSender(warnNanoAddr);
	}

	return iret;

}

//队列为空，返回true
int8 isWarnMsgUdpEmpty(void)
{
	return (m_pWarnQueueUdp->QueueIsEmpty()==0);
}

int8 getWarnMsgFromUdpQueue(WARN_MSG *pWarn_msg)
{
	return (m_pWarnQueueUdp->QueueGetFront(pWarn_msg) == 0);
}

int8 delWarnMsgFromUdpQueue(void)
{
	WARN_MSG tm_warnMsg;
	return (m_pWarnQueueUdp->QueueDelElement(&tm_warnMsg)==0);
}




