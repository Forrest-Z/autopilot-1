
///**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/control_main.h"
#include "../include/usv_include.h"
///******************************  Local Variable  *****************************/
///******************************  Extern Variable  ****************************/
///******************************  Local Function   ****************************/
void testA(void *);
void testB(void *);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/


void * control_main_thread( void *aa )
{
	if(initlevel1TaskQue()<0)
	{
		return ((void*)0);
	}
	else
	{
		control_main_init();
	}
	for(;;)
	{
		level1TaskSchedue();
		sleep_1(50);	
	}



}

void control_main_init( void )
{
	initStateSignal();
	initControlCmd();
	initControlOperation();
	pid_contorl_init();		

	initImageControlParms(); 
}

void testA(void *)
{
	printf("runing A! \n");
	printf("task 1 running :%d times\n",task1_run_cnt);
}

void testB(void *)
{
	printf("running B! \n");
}

