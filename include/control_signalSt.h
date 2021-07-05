//control_signalSt.h
#ifndef __CONTROL_SIGNALST__H_
#define __CONTROL_SIGNALST__H_

#include "usv_include.h"


typedef struct{
	uint8	u8_year			;	//��
	uint8	u8_month		;	//��
	uint8	u8_date			;	//��
	uint8	u8_hour			;	//ʱ
	uint8	u8_minute		;	//��
	uint8	u8_second		;	//��    
}TIME; 

typedef struct{
	uint8   b1_IOP;
	uint8   b1_IHC;
	uint8   b1_IDU;
	uint8   b1_Dradio;
	uint8   b1_Bradio;
	uint8   b1_INS;
	uint8   b1_MPC;
}CONNECT_ST;

typedef struct{
	uint8    b1_Wn_IOPCommOutage	;   //IOPͨѶ�ж�
	uint8    b1_Wn_IOPWarn		    ;   //IOP�ܸ澯  
	uint8    b1_Wn_IHCCommOutage	;   //IHCͨѶ�ж�
	uint8    b1_Wn_IHCWarn		    ;   //IHC�ܸ澯  
	uint8    b1_Wn_IDUCommOutage	;   //IDUͨѶ�ж�
	uint8    b1_Wn_IDUWarn		    ;   //IDU�ܸ澯  
	uint8    b1_Wn_MCUCommOutage	;   //MCUͨѶ�ж�
	uint8    b1_Wn_MCUWarn		    ;   //MCU�ܸ澯  
}WARNNING;


typedef struct{
	TIME                    time                ;               //ʱ��
}STATE_SIGNAL; 

extern STATE_SIGNAL state_signal;

void runStateSignal(void *);
void getINS(void);
extern void initStateSignal(void);

#endif
