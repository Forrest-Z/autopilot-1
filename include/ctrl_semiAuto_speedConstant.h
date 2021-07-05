//ctrl_semiAuto_speedConstant.h

#ifndef __CTRL_SEMIAUTO_SPEEDCONSTANT__H_
#define __CTRL_SEMIAUTO_SPEEDCONSTANT__H_



typedef struct{
	double  double_speed_exp;	
	float	f32_motorL_deg;
	float	f32_motorR_deg;
}SPEED_CONSTANT_ST;

void GetSpeedConstant(void);
void SemiAutoSpeedConstant(void);

#endif /*__CTRL_SEMIAUTO_SPEEDCONSTANT__H_*/