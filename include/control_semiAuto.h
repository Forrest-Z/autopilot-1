//control_semiAuto.h

#ifndef __CONTROL_SEMIAUTO__H_
#define __CONTROL_SEMIAUTO__H_

#define COMMAND_OFF 0
#define COMMAND_ON  1

typedef struct{
	uint8   b1_trig_speedConstant   ;   //  定速模式
	uint8   b1_trig_headingConstant ;   //  定向模式
	uint8   b1_trig_setReturn       ;   //  设置返航点
	uint8   b1_trig_autoReturn      ;   //  自动返航
	uint8   b1_trig_berth           ;   //  泊岸模式
}SEMI_MODE_TRIG;

extern SEMI_MODE_TRIG semi_modeTrig;

extern void semiAuto_operation(void);

#endif /*__CONTROL_SEMIAUTO__H_*/