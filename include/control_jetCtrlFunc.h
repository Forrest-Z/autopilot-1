//control_jetCtrlFunc.h
#ifndef __CONTROL_JETCTRL_FUNC__H_
#define __CONTROL_JETCTRL_FUNC__H_





void idlingSpeedCtrl(void);
void idlingSpeedRoundCtrl(void);	//原地转圈
void jetGearFoward(void);
void jetGearBackward(void);

//航点保持原子动作
void idlingRollRight(void);		//原地右转
void idlingRollLeft(void);		//原地左转
void idlingForward(void);		//低速前进

#endif /*__CONTROL_JETCTRL_FUNC__H_*/