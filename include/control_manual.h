//control_manual.h
#ifndef __CONTROL_MANUAL__H_
#define __CONTROL_MANUAL__H_

#define JOYSTICK_GEAR_COFF 24	//1/3处到极限
#define JOYSTICK_RUDDER_COFF 2.56
#define JOYSTICK_MOTOR_OPENDEG_COFF 2.55
#define GEAR_UP 255
#define GEAR_DOWN -255
#define RUDDER_UP 255
#define RUDDER_DOWN -255
#define OPEN_DEG_MAX 255
//#define OPEN_DEG_MAX 100

extern void manual_operation(void);

#endif /*__CONTROL_MANUAL__H_*/