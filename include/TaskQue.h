/*==========================================================*              
 * ģ��˵��: TaskQue.h                                      *              
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *              
 * ������Ա: shaoyuping                                     *              
 * ����ʱ��: 				                                *              
 * Copyright(c) sf-auto.ltd									*              
 *==========================================================*              
 * �����޸ļ�¼(���µķ�����ǰ��):                          *              
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *              
 *==========================================================*              
 *=========================================================*/            
 
 
#ifndef __TASKQUE_H_
#define __TASKQUE_H_

#include "Component.h"
#include "usv_include.h"

typedef struct TASK
{
    void	(*	func)(void *);
	Component	*arg	 ;
    struct TASK *next	 ;
}TASK;

typedef struct TASKQUE
{
    TASK *head;
    TASK *tail;
}TASKQUE;

extern 	uint32 task1_run_cnt;
extern 	uint32 task2_run_cnt;
extern 	uint32 task3_run_cnt;
extern 	uint32 task4_run_cnt;
extern  TASKQUE	*level1,*level2,*level3,*level4;	



//extern uint32	getTaskCycleTime(int task_lvl);
//extern void		initTaskTimeRelation(void);
//extern void		reSetTaskCycleTime(int task_lvl,int task_execute_cycle);
extern  int addTask(int level,void (*newtask)(void*),void *arg);
extern int		addTaskQue(TASKQUE *,void(*)(),void*);
extern void		taskSchedule(void);
extern int		initAllTaskQue(void);

extern int		initlevel1TaskQue(void);
extern int		initlevel2TaskQue(void);
extern int		initlevel3TaskQue(void);
extern int		initlevel4TaskQue(void);

extern int		destoryAllTaskQue(void);
extern void		level1TaskSchedue(void);
extern void		level2TaskSchedue(void);
extern void		level3TaskSchedue(void);
extern void		level4TaskSchedue(void);


#endif /*__TASKQUE_H_*/  