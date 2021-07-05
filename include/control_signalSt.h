//control_signalSt.h
#ifndef __CONTROL_SIGNALST__H_
#define __CONTROL_SIGNALST__H_

#include "usv_include.h"


typedef struct{
	uint8	u8_year			;	//年
	uint8	u8_month		;	//月
	uint8	u8_date			;	//日
	uint8	u8_hour			;	//时
	uint8	u8_minute		;	//分
	uint8	u8_second		;	//秒    
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
	uint8    b1_Wn_IOPCommOutage	;   //IOP通讯中断
	uint8    b1_Wn_IOPWarn		    ;   //IOP总告警  
	uint8    b1_Wn_IHCCommOutage	;   //IHC通讯中断
	uint8    b1_Wn_IHCWarn		    ;   //IHC总告警  
	uint8    b1_Wn_IDUCommOutage	;   //IDU通讯中断
	uint8    b1_Wn_IDUWarn		    ;   //IDU总告警  
	uint8    b1_Wn_MCUCommOutage	;   //MCU通讯中断
	uint8    b1_Wn_MCUWarn		    ;   //MCU总告警  
}WARNNING;


typedef struct{
	TIME                    time                ;               //时间
}STATE_SIGNAL; 

extern STATE_SIGNAL state_signal;

void runStateSignal(void *);
void getINS(void);
extern void initStateSignal(void);

#endif
