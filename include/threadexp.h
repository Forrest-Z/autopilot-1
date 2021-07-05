/*
* Threadexp.c --定义创建线程的基本结构体和操作函数
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
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
#include <semaphore.h>  	 //信号量定义文件
#include <termios.h>
#include <pthread.h>     	//Linux下线程定义文件
//#include <i386-linux-gnu/bits/pthreadtypes.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <net/if.h>



//线程节点结构
typedef struct  {
	pthread_t        m_pthread;		//线程
        void             *m_pPrevPtr;	//上一个的指针
        void             *m_pNextPtr;	//下一个指针
}threadnode;

//线程链结构
typedef struct{
	unsigned int		(*Add)(void *pThis,threadnode *pNew);
	threadnode*	(*GetAt)(void *pThis,int i);
	unsigned int        (*RemoveAt)(void *pThis,int i);
	void                   	(*ResetArray)(void *pThis);
	int                      	m_Size;
	threadnode 	*m_pHeadPtr;	
	threadnode 	*m_pTailPtr;	
}threadnode_ids;


void threadnode_destroy(threadnode *ptn);					 									//删除一个线程
void threadnode_ids_init(threadnode_ids *ptids);                     									//初始化线程队列
void threadnode_ids_destroy(threadnode_ids *ptids);                   		 						//线程队列清空
unsigned int CreateNewThread(threadnode *pNEW,void *New_thread(void *),char diffprio);		//创建新线程
void threadnode_init(threadnode *ptn);															//线程初始化


#endif
