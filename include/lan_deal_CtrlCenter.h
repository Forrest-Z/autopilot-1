//lan_deal_CtrlCenter.h
#ifndef __LAN_DEAL_CTRL_CENTER__H_
#define __LAN_DEAL_CTRL_CENTER__H_

#include "usv_include.h"

#ifndef TASK_POINT_MAX
#define TASK_POINT_MAX 20	
#endif

typedef enum{
	USV_NO_AUTHROTY = 0,
	USV_CC_AUTHROTY,		
	USV_PB_AUTHROTY,		
	USV_RM_AUTHROTY,		
	USV_AUTHROTY_COUNT
}USV_AUTHROTY;

typedef enum{
	FRAME_TYPE_NULL = 0,	// null
	FRAME_TYPE_CONTROL,		// control
	FRAME_TYPE_SAILTASK,	// sail task
	FRAME_TYPE_WARNCONFIRM,	// Warn
	FRAME_TYPE_COUNT        // Must be last
}FRAME_TYPE;

#pragma pack(1)
typedef struct __BRADIO_CC_CMD__	
{
	uint8	u8_cmd_moterOnOff;    // 1: START 2:STOP
	uint8	u8_cmd_emergencyStop;
	uint8	u8_cmd_getAuthority;  // 0: give 1:get
	
	uint8	u8_cmd_sailMode;	 // 0: mual 1:adas 2:intelligent	
	uint8	u8_cmd_sailSwitch;	 // 1: delete  2: start 3: halt	

	uint8	u8_cmd_speedConstant;	//
	uint8	u8_cmd_headingConstant;	//
	uint8	u8_cmd_return_dock;		// 

	sint8	i8_cmd_joy1X;
	sint8	i8_cmd_joy1Y;
	sint8	i8_cmd_joy1Z;

	sint8	i8_Cmd_joy2X;
	sint8	i8_Cmd_joy2Y;
	sint8	i8_Cmd_joy2Z;

	//add backup
	uint16  u16_cmd_setSpeed;		// 0.1kn/bit
	uint16	u16_cmd_setHeading;		// 0.1deg/bit	0~3599

	uint8	u8_cmd_reserve5;		
	uint8	u8_cmd_reserve6;
	uint8	u8_cmd_reserve7;
	uint8	u8_cmd_reserve8;
	uint8	u8_cmd_reserve9;
	uint8	u8_cmd_reserve10;
	uint8	u8_cmd_reserve11;
	uint8	u8_cmd_reserve12;
	uint8	u8_cmd_reserve13;
	uint8	u8_cmd_reserve14;
	uint8	u8_cmd_reserve15;
	uint8	u8_cmd_reserve16;
	uint8	u8_cmd_reserve17;
	uint8	u8_cmd_reserve18;
	uint8	u8_cmd_reserve19;
	uint8	u8_cmd_reserve20;
}BRADIO_CC_CMD;
#pragma pack()

#pragma pack(1)
typedef struct __BRADIO_USV_STATE__	
{
	uint8	u8_st_year;			
	uint8	u8_st_month;		
	uint8	u8_st_date;			

	uint8	u8_st_hour;			
	uint8	u8_st_minute;		
	uint8	u8_st_second;		

	uint8	u8_st_latSt;		
	uint8	u8_st_latDeg;		
	uint8	u8_st_latMin;		
	uint8	u8_st_latSec;		
	uint8	u8_st_latSecDec;	

	uint8	u8_st_lonSt;		
	uint8	u8_st_lonDeg;		
	uint8	u8_st_lonMin;		
	uint8	u8_st_lonSec;		
	uint8	u8_st_lonSecDec;	

	uint16	u16_st_speed;		
	uint16	u16_st_heading;		
	uint16	u16_st_velocityDir;	

	int16	i16_st_pitch;		
	int16	i16_st_roll;		
	int16	i16_st_heaving;		
	int16	i16_st_rot;			

	uint8	u8_st_emergencyStop;	
	uint8	u8_st_authority;	
	uint8	u8_st_sailMode;		
	uint8	u8_st_sailTaskState;

	uint8	u8_st_speedConstant;	
	uint8	u8_st_headingConstant;	
	uint8	u8_st_berthMode;		

	uint8	u8_st_motorState;		

	uint16	u16_st_moter1Rpm;		
	int16	i16_st_moter1Gear;		
	int16	i16_st_moter1Rudder;	

	uint16	u16_st_moter2Rpm;		// 34
	int16	i16_st_moter2Gear;		// 35
	int16	i16_st_moter2Rudder;	// 36

	uint16	u16_st_moter3Rpm;		// 37
	int16	i16_st_moter3Gear;		// 38
	int16	i16_st_moter3Rudder;	// 39

	uint16	u16_st_moter4Rpm;		// 40
	int16	i16_st_moter4Gear;		// 41
	int16	i16_st_moter4Rudder;	// 42

	uint16	u16_st_voltage;			// 43
	uint16	u16_st_current;			// 44

	uint8	u8_remainTime_h;		// 45
	uint8	u8_remainTime_m;		// 46
	uint8	u8_remainTime_s;		// 47
	uint8	u8_powerPercent;		

	//add by syp
	uint16	u16_pseudoRangeError;	
	uint8	u8_locationState;		
	uint8	u8_readyGetOutDock;		
	uint8	u8_sailPointSq;			
	uint8	u8_avoidControlState;	

	uint16  u16_expSpeed;
	uint16  u16_expHeading;

	uint8	u8_return_state;		
	uint8	u8_dockcmd_feedback;	
	uint8	u8_returnPointSetOn;	
	uint8	u8_sateliteNum_ins; 
	uint8	u8_sateliteNum_irtk; 
	uint8	u8_c_rmcValid_ins; 
	uint8	u8_c_rmcValid_irtk; 

	uint8	u8_reserve18;
	uint8	u8_reserve19;
	uint8	u8_reserve20;
}BRADIO_USV_STATE;
#pragma pack()



#pragma pack(1)
typedef struct{
	uint8	u8_lat_st;
	uint8	u8_lat_deg;
	uint8	u8_lat_min;
	uint8	u8_lat_sec;
	uint8	u8_lat_secDec;
	uint8	u8_lon_st;
	uint8	u8_lon_deg;
	uint8	u8_lon_min;
	uint8	u8_lon_sec;
	uint8	u8_lon_secDec;
	uint16	u16_speed;
	uint16	u16_stopTime;
	uint64  u64_pointId;
	uint8	u8_pointType;
	uint16	u16_sampleVolume;
}TASK_POINT;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint8	socket_ID_h;		
	uint8	socket_ID_l;		
	uint16	frame_len;			
	uint8	point_sum;			
	uint8	task_squence;		
	uint8	point_num;			
	TASK_POINT arrayTaskPoint[TASK_POINT_MAX];	
}UDP_SAILTASK_REPORT_STRUCT;
#pragma pack()

#pragma pack(1)
typedef struct{
	SAIL_TASK sailTask;
	uint8	  savedPointNum;	
	uint8	  updateOk;			
}SAIL_TASK_BUFF;
#pragma pack()

extern COMM_SIGN brCc_sign;
extern BRADIO_CC_CMD br_usv_cmd;
extern char CtrlCenterIP[30];
extern uint32 BRCC_UDP_RECV_PORT;
extern uint32 BRCC_UDP_SEND_PORT;


extern void *lan_deal_CtrlCenter(void *aa);
extern void BrCC_CommCalInit(void);
void updateBrCC_usvState(void);
void runBrCC_SendMsgFast(void*);
extern void initBrCCSendTask(void);




#endif // !__LAN_DEAL_CTRL_CENTER__H_

