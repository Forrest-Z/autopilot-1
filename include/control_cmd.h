//control_cmd.h

#ifndef __CONTROL_CMD__H_
#define __CONTROL_CMD__H_

#define IOP_AUTHORITY		0
#define DIOP_AUTHORITY		1
#define BIOP_AUTHORITY		2
#define BD_AUTHORITY		3
#define BRCC_AUTHORITY		4
#define	NO_AUTHORITY		255

typedef struct{
	uint8   b1_emergencyMode   ;   //  emergence mode
	uint8   b1_emergencyStop   ;   //  emergency
	uint8   u8_authority       ;   //  255:NULL  0:IOP 1:DIOP 2:BIOP 3:BD 
	uint8   b2_sailMode        ;   //  
	uint8   b2_sailTask        ;   //  
}SAIL_MODE;

typedef struct{
	uint8   b1_speedConstant   ;   //  
	uint8   b1_headingConstant ;   //  
	uint8   b1_setReturn       ;   //  
	uint8   b1_autoReturn      ;   //  
	uint8   b1_dock_cmd        ;   //  
}FUNC_MODE;

typedef struct{
	int16    i16_X1;
	int16    i16_Y1;
	int16    i16_Z1;
	int16    i16_X2;
	int16    i16_Y2;
	int16    i16_Z2;
}JOYSTICK_VAL;

typedef struct{
	uint8	b2_motorOnOff;
}FIRE_BUTTON;

#define SEMI_MANUAL				0
#define SEMI_SPEEDCONSTANT		1
#define SEMI_HEADCONSTANT		2
#define SEMI_SETRETURN			3
#define SEMI_AUTORETURN			4
#define SEMI_BERTH				5

typedef struct{
	uint8	semi_mode;	//
	float	speed_const_value;	//
	float	heading_const_value;	//
}SEMI_MODE;


typedef struct{
	uint8	b1_elecWindlass1OnOff	;
	uint8	b1_elecWindlass1UpDown	;
	uint8	b1_elecWindlass2OnOff	;
	uint8	b1_elecWindlass2UpDown	;
	uint8	b1_sprayStrip1OnOff		;
	uint8	b1_sprayStrip1UpDown	;
	uint8	b1_sprayStrip2OnOff		;
	uint8	b1_sprayStrip2UpDown	;

	uint8	b1_SystemRestart;		//PS21:7

	uint8	b1_periPowK1;		//PS21:1:0
	uint8	b1_periPowK2;
	uint8	b1_periPowK3;
	uint8	b1_periPowK4;
	uint8	b1_periPowK5;
	uint8	b1_periPowK6;
	uint8	b1_periPowK7;
	uint8	b1_periPowK8;

	uint8	b1_periPowK9;
	uint8	b1_periPowK10;
	uint8	b1_periPowK11;
	uint8	b1_periPowK12;
	uint8	b1_periPowK13;
	uint8	b1_periPowK14;
	uint8	b1_periPowK15;
	uint8	b1_periPowK16;

	uint8	b1_periPowK17;
	uint8	b1_periPowK18;
	uint8	b1_periPowK19;
	uint8	b1_periPowK20;

	uint8	b1_SailLightPowK1;
	uint8	b1_SailLightPowK2;
	uint8	b1_SailLightPowK3;
	uint8	b1_SailLightPowK4;
	uint8	b1_SailLightPowK5;
	uint8	b1_SailLightPowK6;
	uint8	b1_SailLightPowK7;
	uint8	b1_SailLightPowK8;
}EQUPMENT_CMD;


typedef struct{
	uint8	b2_sailTask;
}SAIL_FEEDBACK;



typedef struct{
	uint8			b1_authority		;	//	权限位置 0:远方	1：船端
	SAIL_MODE       sail_mode_cmd		;	//  接收到的航行指令
	FUNC_MODE       func_mode_cmd		;	//  接收到的功能至指令
	SEMI_MODE		semi_mode_cmd		;
	JOYSTICK_VAL    joystick_cmd		;	//	接收到的摇杆指令
	FIRE_BUTTON		start_button_cmd	;	//	接收到的点火指令
	EQUPMENT_CMD    equpment_cmd		;	//	接收到的设备开关指令
	SAIL_FEEDBACK	sail_feedBack		;	//	航行反馈量
}COMMAND_SIGNAL;


extern COMMAND_SIGNAL command_signal;

void getCmdFromIOP(void);	//船端控制器
void getCmdFromDR(void);	//便携基站数字电台
void getCmdFromBR(void);	//便携基站宽带电台
void getCmdFromCC(void);	//集控中心4G电台
void getCmdFromBD(void);	//北斗
void runControlCmd(void *);

extern void initControlCmd(void);


#endif /*__CONTROL_CMD__H_*/