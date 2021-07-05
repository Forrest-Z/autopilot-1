//uart_deal_BD.h

#ifndef __UART_DEAL_BD__H_
#define __UART_DEAL_BD__H_

#include "usv_include.h"



//������������
typedef struct{
	uint8		u8_msgSrc	;	//������Դ 3:����
	uint8		u8_sailNum	;	//������
	WAYPOINT	wayPoint	;	//���񺽵�
}BD_SAIL_MESG;

typedef struct{
	uint8			u8_St_sailMsgRev	;	//��������ִ��״̬ 0:������ 1:������ 2:ִ�����
	uint8			u8_St_BDSailState	;	//��������״̬	 0:δִ�� 1:ִ���� 2:ִ�����
	uint8			u8_cmd_sailOnOff	;	//��������ִ������ 1:ִ��   2:ֹͣ
	uint8			u8_St_TaskRun		;	//��������״̬	 0:δִ�� 1:����ִ�� 2:ִ�����
	BD_SAIL_MESG	sailMsg				;	//������Ϣ
}BD_SAIL_TASK;

extern BD_SAIL_TASK	bd_sailTask;


void getBDSailMsg(uint8* recBuf);
extern void removeBDTask(void);
extern void *uart_deal_BD(void *aa);
extern void uart_BD_send(void);
#endif /*__UART_DEAL_BD__H_*/