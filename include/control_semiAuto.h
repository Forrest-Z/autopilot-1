//control_semiAuto.h

#ifndef __CONTROL_SEMIAUTO__H_
#define __CONTROL_SEMIAUTO__H_

#define COMMAND_OFF 0
#define COMMAND_ON  1

typedef struct{
	uint8   b1_trig_speedConstant   ;   //  ����ģʽ
	uint8   b1_trig_headingConstant ;   //  ����ģʽ
	uint8   b1_trig_setReturn       ;   //  ���÷�����
	uint8   b1_trig_autoReturn      ;   //  �Զ�����
	uint8   b1_trig_berth           ;   //  ����ģʽ
}SEMI_MODE_TRIG;

extern SEMI_MODE_TRIG semi_modeTrig;

extern void semiAuto_operation(void);

#endif /*__CONTROL_SEMIAUTO__H_*/