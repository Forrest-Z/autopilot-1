/*==========================================================*
 * ģ��˵��: watch_dog.cpp                                  *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա:                                                *
 * ����ʱ��: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/watch_dog.h"
/******************************  Local Variable  *****************************/
int fd;
int timeout = 15;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void initWatchDog(void);
void feedWatchDog(void);
void disableWatchDog(void);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
#ifndef WINNT
void initWatchDog(void)
{
    fd=open("/dev/watchdog",O_WRONLY);
    /* set timeout to 10 seconds */
    ioctl(fd,WDIOC_SETTIMEOUT,&timeout);
}

void feedWatchDog(void)
{
    ioctl(fd,WDIOC_KEEPALIVE,NULL);
}

void disableWatchDog(void)
{
	int i_dis = WDIOS_DISABLECARD;
	ioctl(fd,WDIOC_SETOPTIONS, &i_dis);
}

#else

void initWatchDog(void)
{

}

void feedWatchDog(void)
{

}

void disableWatchDog(void)
{

}
#endif