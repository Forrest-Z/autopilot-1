/*
* Threadexp.c --�����߳�
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"
#include <sched.h>
#include "../include/usv_include.h"

unsigned int CreateThread_low(pthread_t *ptid,void *pfun(void *),void *pp,char diffprio);	                    
extern threadnode_ids thread_ids;
unsigned int CreateNewThread(threadnode *pNEW,void *New_thread(void *),char diffprio);



unsigned int CreateNewThread(threadnode *pNEW,void *New_thread(void *),char diffprio)
{
	unsigned int Res=0;

	pNEW=(threadnode *)malloc(sizeof(threadnode));
	threadnode_init(pNEW);				 	 				//��ʼ��һ���µ��߳�
	Res=CreateThread_low(&(pNEW->m_pthread),	 		//�̺߳�						
		New_thread,											//��ʱ�̳߳�����						
		NULL,														//�޴��ݵĲ���ָ��
		diffprio);													//���ȼ�������
	if(!Res)
		thread_ids.Add(&thread_ids,pNEW);				 //����һ���̣߳�����һ���̶߳���
	else 											 				//�����߳�ʧ��
		free(pNEW);												//�ͷ��߳�	
	return Res;
}


unsigned int CreateThread_low(pthread_t *ptid,void *pfun(void *),void *pp,char diffprio)
{
	unsigned int Res=0;
	pthread_attr_t attr;														//�߳�����
	struct sched_param sch;
	
	if(ptid)
	{	
		pthread_attr_init(&attr);     											//��ʼ���߳�
		pthread_attr_getschedparam(&attr,&sch);								//��õ�ǰ���߳�����
		if(diffprio==0)
		{
		    pthread_attr_setschedpolicy( &attr, SCHED_RR );//max 0,min 0      	//�޸�ΪRR ģʽ
		}
		else
		{

		    pthread_attr_setschedpolicy( &attr, SCHED_RR );  //max 99,min 1     
			sch.sched_priority=diffprio;
			pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
			pthread_attr_setschedparam(&attr,&sch);							//��ǰ���߳�����
		}
		pthread_attr_setscope(&attr,PTHREAD_SCOPE_SYSTEM);					//�����̰߳����
		pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);			//�����̷߳�������				   
	   	Res=pthread_create (ptid, &attr,pfun, pp);  				                    
	} 
	return Res;
}

//��ʼ���߳�
void threadnode_init(threadnode *ptn)
{
	if(!ptn)return;
	ptn->m_pthread=0;         			//�̺߳�=0
	ptn->m_pPrevPtr=0;        			//ǰָ��=0
	ptn->m_pNextPtr=0;        			//��ָ��=0
	return  ;
}

void threadnode_destroy(threadnode *ptn)
{
	if(!ptn)return;
	close(ptn->m_pthread);                 	//�ر��߳�

	ptn->m_pthread=0;                       	//�̺߳�=0
	ptn->m_pPrevPtr=0;                       	//ǰָ��=0
	ptn->m_pNextPtr=0;                      	//��ָ��=0
	return  ;
}

//��һ���̶߳���������һ���µ��߳�
//pThis--�̶߳��У�pNew--�µ��߳�ָ��
unsigned int threadnode_ids_Add(void *pThis,threadnode *pNew)
{
	threadnode_ids *This;
	
	if((!pThis)||(!pNew))return 0;
	This=(threadnode_ids *)pThis;           										//�߳̽ṹ
	
	pNew->m_pPrevPtr=NULL;                											//���̵߳�ǰָ��=��
	pNew->m_pNextPtr=NULL;                											//���̵߳ĺ�ָ��=��
	if(!This)return 0;
	
	if(!This->m_pHeadPtr)	                        											 //�����ӵ�Ϊ��һ���߳�
		This->m_pHeadPtr=This->m_pTailPtr=pNew;								 //ͷβָ���ָ����߳�
	else                                                    												 //�������߳�
	{
		pNew->m_pPrevPtr=This->m_pTailPtr;       									 //�¶��е���һ��ָ�� = �̶߳��е�βָ��
		This->m_pTailPtr->m_pNextPtr=pNew;        								 //��ǰ���һ���̵߳���һ��ָ�� = �µ��߳�ָ��
		This->m_pTailPtr=(threadnode *)(This->m_pTailPtr->m_pNextPtr);	   	 //�̶߳��е�βָ��= �µ��߳� 
	}	
	This->m_Size++;	                                          									//�̶߳��е��̸߳���+1
	return 1;
}

//���ݵڼ����̣߳������̵߳�ָ��
//i--�ڼ����̣߳�pThis--�̶߳���
threadnode*        threadnode_ids_GetAt(void *pThis,int i)
{
	int Index;
	threadnode *pTemp;
	
	threadnode_ids *This=(threadnode_ids *)pThis;
	if((!This)||(i>=This->m_Size))return NULL;    					 //������Ϊ�ջ�Ҫ���ҵ�ָ�볬�����е��̸߳�����������������
    
	Index=0;                           											 //�����Ŵ�0��ʼ
	pTemp=This->m_pHeadPtr;   										  //�ӵ�һ��ָ�뿪ʼ
	while((Index<i)&&pTemp)               								  //ָ��Ϊ��ָ���˳���������>=�ҵ��̺߳��˳�
	{
		pTemp=(threadnode *)(pTemp->m_pNextPtr);       			 //ȡ�߳�ָ��
		Index++;               												 //������+1
	}
	return pTemp;                                 								//�����߳�ָ��
}

//�����Ƴ�һ���߳�
//i--�ڼ����̣߳�pThis--�̶߳���
unsigned int     threadnode_ids_RemoveAt(void *pThis,int i)
{
	int Index;
	unsigned char bRet;
	threadnode_ids *This;
	threadnode *pTemp;
	
	This=(threadnode_ids *)pThis;
	if((!This)||(i>=This->m_Size))return 0;
	bRet=0;
	Index=0;
	
    	pTemp=This->m_pHeadPtr;
	while((Index<i)&&pTemp)                       										//�����߳�ָ��
	{
		pTemp=(threadnode *)(pTemp->m_pNextPtr);
		Index++;
	}
	if(pTemp&&(Index==i))															//�и��߳�     

	{
		if((pTemp==This->m_pHeadPtr)                    									//Ϊͷָ��
			&&(pTemp!=This->m_pTailPtr))                     								//��֤�ö��в�ֹһ���߳�
		{                 
			This->m_pHeadPtr=(threadnode *)(This->m_pHeadPtr->m_pNextPtr);    	 //�̶߳��е���ָ�� = ���̵߳���һ��ָ��
			This->m_pHeadPtr->m_pPrevPtr=NULL;       									 //��һ���߳���һ��ָ�� = ��
		}
		else if((pTemp!=This->m_pHeadPtr)         									//��֤�ö��в�ֹһ���߳�
			&&(pTemp==This->m_pTailPtr))        										//Ϊβָ�� 
		{
			This->m_pTailPtr=(threadnode *)(pTemp->m_pPrevPtr);     				//�̶߳��е�βָ�� = ���̵߳���һ��ָ��
			This->m_pTailPtr->m_pNextPtr=NULL;                   								//��һ���߳���һ��ָ�� = ��
		}
		else if(This->m_pTailPtr==This->m_pHeadPtr)         							//�̶߳���ֻ�и��߳�
		{
			This->m_pTailPtr=This->m_pHeadPtr=NULL;       								//ͷָ��=βָ��=��
		}
		else           																		//�߳��ڶ����м�
		{
			((threadnode *)(pTemp->m_pPrevPtr))->m_pNextPtr=pTemp->m_pNextPtr;//���߳���һ���̵߳���һ��ָ�� = ���̵߳���һ��ָ��		

			
			((threadnode *)(pTemp->m_pNextPtr))->m_pPrevPtr=pTemp->m_pPrevPtr;//���߳���һ���̵߳���һ��ָ�� = ���̵߳���һ��ָ��		;
		}
		This->m_Size--;                   													   //�̶߳��е��߳���-1
		threadnode_destroy(pTemp);        										  //�ͷŸ��߳�
		free(pTemp);                      														  //�ͷŸ��̵߳��ڴ�
		pTemp=NULL;
		bRet=1;			
	}
	return bRet;
}

//���̶߳����е��߳����
//�����̺߳�
void  threadnode_ids_ResetArray(void *pThis)
{
	threadnode_ids *This=(threadnode_ids *)pThis;
	while(This&&(This->m_Size>0))                     	//���̶߳����е��߳����
		threadnode_ids_RemoveAt(pThis,0);
	
	This->m_Size=0;                       						//����=0��
	This->m_pHeadPtr=NULL;                					//ǰָ��=��
	This->m_pTailPtr=NULL;               					 //��ָ��=��
	return  ;
}

//�̶߳��г�ʼ��
void threadnode_ids_init(threadnode_ids *ptids)
{
	if(!ptids)return;
	ptids->Add=&threadnode_ids_Add;          				 //����һ���߳�
	ptids->GetAt=&threadnode_ids_GetAt;       				 //�����̺߳Ż���̵߳�ָ��
	ptids->RemoveAt=&threadnode_ids_RemoveAt;  		 //�Ƴ�һ���߳�
	ptids->ResetArray=&threadnode_ids_ResetArray;   	 //���̶߳����е��߳����
	ptids->m_Size=0;                                							//�߳���=0
	ptids->m_pHeadPtr=NULL;	               							//ͷָ��=��
	ptids->m_pTailPtr=NULL;	                							//βָ��=��
	return  ;
}

//�̶߳������
//���룺�̶߳���ָ��
void threadnode_ids_destroy(threadnode_ids *ptids)
{
	threadnode_ids_ResetArray(ptids);         				//����̻߳�����
	return  ;
}

