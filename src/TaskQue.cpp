/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/TaskQue.h"
#include "../include/usv_include.h"
/******************************  Local Variable  *****************************/
/*define task queues */
TASKQUE *level1,*level2,*level3,*level4;

uint32	task1_run_cnt=0;
uint32	task2_run_cnt=0;
uint32	task3_run_cnt=0;
uint32	task4_run_cnt=0;

/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

/******************************************************************/
static TASKQUE *initTaskQue(void)
{
	TASKQUE *taskque=(TASKQUE *)calloc(1,sizeof(TASKQUE));
	if(!taskque) return 0;
	taskque->head = taskque->tail = NULL;
	return taskque;
}

static int detroyTaskQue(TASKQUE *taskque)
{
	TASK *p=taskque->head;
	TASK *q=p;
	while (p)
	{
		q=q->next;
		free(p);
		p=q;
	}
	return 0;
}

/******************************************************************/
int addTask(int level,void (*newtask)(void*),void *arg)
{
	TASKQUE *taskque;
	TASK *p=(TASK *)calloc(1,sizeof(TASK));
	if(!p)return -1;
	p->func = newtask;
	p->arg  =(Component *)arg;

	if(level == 1) taskque = level1;
	else if(level == 2) taskque = level2;
	else if(level == 3) taskque = level3;
	else if(level == 4) taskque = level4;
	else
	{
		free(p);
		return -1;
	}
	if(taskque->tail == NULL)
	{
		taskque->head = p;
		taskque->tail = p;
	}
	else
	{
		taskque->tail->next=p;
		taskque->tail = p;
	}
	return 0;
}
/******************************************************************/
int addTaskQue(TASKQUE *taskque,void(* newtask)(void *),void *arg)
{
	TASK *p = (TASK *)calloc(1,sizeof(TASK));
	if(!p) return 0;
	p->func = newtask;
	p->arg = (Component *)arg;
	if(taskque->tail==NULL)
	{
		taskque->head = p;
		taskque->tail = p;
	}
	else
	{
		taskque->tail->next=p;
		taskque->tail=p;
	}
}
/******************************************************************/
void runTaskQue(TASKQUE *taskque)
{
	TASK *p;
	p = taskque->head;
	while(p)
	{
		p->func(p->arg);
		p=p->next;
	}
}
/******************************************************************/
int	initlevel1TaskQue(void)
{
	level1 = initTaskQue();
	if(!level1) return -1;
	return 0;	
}
/******************************************************************/
int initlevel2TaskQue(void)
{
	level2 = initTaskQue();
	if(!level2) return -1;
	return 0;
}
/******************************************************************/
int initlevel3TaskQue(void)
{
	level3 = initTaskQue();
	if(!level3) return -1;
	return 0;
}
/******************************************************************/
int initlevel4TaskQue(void)
{
	level4 = initTaskQue();
	if(!level4) return -1;
	return 0;
}
/******************************************************************/
int initAllTaskQue(void)
{
	level1 = initTaskQue();
	level2 = initTaskQue();
	level3 = initTaskQue();
	level4 = initTaskQue();
	if (!level1 || !level2 || !level3 || !level4) return -1;
	return 0;
}
/******************************************************************/
int destoryAllTaskQue()
{
	detroyTaskQue(level1);
	detroyTaskQue(level2);
	detroyTaskQue(level3);
	detroyTaskQue(level4);
	return 0;
}
/******************************************************************/
void level1TaskSchedue(void)
{
	runTaskQue(level1);
	task1_run_cnt++;
}
/******************************************************************/
void level2TaskSchedue(void)
{
	runTaskQue(level2);
	task2_run_cnt++;
}
/******************************************************************/
void level3TaskSchedue(void)
{
	runTaskQue(level3);
	task3_run_cnt++;
}
/******************************************************************/
void level4TaskSchedue(void)
{
	runTaskQue(level4);
	task4_run_cnt++;
}
/******************************************************************/