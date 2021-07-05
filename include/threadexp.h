/*
* Threadexp.c --���崴���̵߳Ļ����ṹ��Ͳ�������
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#ifndef THREADEXP_H
#define THREADEXP_H
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <netdb.h>
#include <sched.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/sockios.h>
#include<sys/ioctl.h>
#include <semaphore.h>  	 //�ź��������ļ�
#include <termios.h>
#include <pthread.h>     	//Linux���̶߳����ļ�
//#include <i386-linux-gnu/bits/pthreadtypes.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <net/if.h>



//�߳̽ڵ�ṹ
typedef struct  {
	pthread_t        m_pthread;		//�߳�
        void             *m_pPrevPtr;	//��һ����ָ��
        void             *m_pNextPtr;	//��һ��ָ��
}threadnode;

//�߳����ṹ
typedef struct{
	unsigned int		(*Add)(void *pThis,threadnode *pNew);
	threadnode*	(*GetAt)(void *pThis,int i);
	unsigned int        (*RemoveAt)(void *pThis,int i);
	void                   	(*ResetArray)(void *pThis);
	int                      	m_Size;
	threadnode 	*m_pHeadPtr;	
	threadnode 	*m_pTailPtr;	
}threadnode_ids;


void threadnode_destroy(threadnode *ptn);					 									//ɾ��һ���߳�
void threadnode_ids_init(threadnode_ids *ptids);                     									//��ʼ���̶߳���
void threadnode_ids_destroy(threadnode_ids *ptids);                   		 						//�̶߳������
unsigned int CreateNewThread(threadnode *pNEW,void *New_thread(void *),char diffprio);		//�������߳�
void threadnode_init(threadnode *ptn);															//�̳߳�ʼ��


#endif
