//control_BD.h
#ifndef __CONTROL_BD__H_
#define __CONTROL_BD__H_



typedef struct{
	double double_heading_exp	;
	double double_speed_exp		;
	double double_dst			;
}BD_NAVI_ST;


void BDNavigation(void);
extern void BD_operation(void);





#endif /*__CONTROL_BD__H_*/