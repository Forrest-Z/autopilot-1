//uart_deal_BD.h

#ifndef __UART_DEAL_BD__H_
#define __UART_DEAL_BD__H_

#include "usv_include.h"



//北斗航行任务
typedef struct{
	uint8		u8_msgSrc	;	//命令来源 3:北斗
	uint8		u8_sailNum	;	//任务编号
	WAYPOINT	wayPoint	;	//任务航点
}BD_SAIL_MESG;

typedef struct{
	uint8			u8_St_sailMsgRev	;	//航行任务执行状态 0:无任务 1:有任务 2:执行完毕
	uint8			u8_St_BDSailState	;	//北斗航行状态	 0:未执行 1:执行中 2:执行完毕
	uint8			u8_cmd_sailOnOff	;	//航行任务执行命令 1:执行   2:停止
	uint8			u8_St_TaskRun		;	//北斗航行状态	 0:未执行 1:正在执行 2:执行完毕
	BD_SAIL_MESG	sailMsg				;	//航行信息
}BD_SAIL_TASK;

extern BD_SAIL_TASK	bd_sailTask;


void getBDSailMsg(uint8* recBuf);
extern void removeBDTask(void);
extern void *uart_deal_BD(void *aa);
extern void uart_BD_send(void);
#endif /*__UART_DEAL_BD__H_*/