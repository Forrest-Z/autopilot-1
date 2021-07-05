//can_deal_main.h
#ifndef __CAN_DEAL_MAIN__H_
#define __CAN_DEAL_MAIN__H_
#include "usv_include.h"

extern CAN_TYPE can_0;

#define IDU_SWITCH_NUM_MAX 20

#ifndef WINNT 
const char PERI_MCU_CFG_FILE_NAME[]="../cfg/PeriMcuConfig.cfg";						//配置文件
#else
const char PERI_MCU_CFG_FILE_NAME[]="../../cfg/PeriMcuConfig.cfg";						//配置文件
#endif

//开关映射配置
typedef struct{
	int32	switchNum;
	int32	switch_k[IDU_SWITCH_NUM_MAX];
}SWITCH_CONNECT;	

//IDU配置
typedef struct{
	SWITCH_CONNECT	switch_connect;
}IDU_CFG;			


//IOP开关映射配置
typedef struct{
	SWITCH_CONNECT switch_connect;
}IOP_CFG;


//外设配置
typedef struct{
	IDU_CFG	IDG_cfg;
	IOP_CFG IOP_cfg;
}PERI_MCU_CFG;

// Battery configuration
typedef struct{
	uint16 battery_number;
	uint16 battery_data_channel;
}Battery_cfg_t;




typedef struct 
{
	/* data */
	uint16 voltage;
	int16  current;
	uint16 batt_remain;
	uint16 batt_total;
}BSM_msg_t;

extern BSM_msg_t bms1_msg;
extern BSM_msg_t bms2_msg;
extern PERI_MCU_CFG peri_mcu_cfg;	//外设配置
extern Battery_cfg_t batt_cfg;


extern void *can0_rec_thread(void *aa);

void callVersion(void);
void callSelfCheck(void);
void callAllStart(void);

extern void canCommCalInit(void);
void canModelCommCal(void *);

int8 read_PeriMcuCfg_file(void);
void write_PeriMcuCfg_file(void);
void set_PeriMcuCfg_default(void);

#endif /*__CAN_DEAL_MAIN__H_*/