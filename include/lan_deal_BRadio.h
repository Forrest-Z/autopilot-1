//lan_deal_BRadio.h 宽带电台
#ifndef	__LAN_DEAL_BRADIO__H_
#define __LAN_DEAL_BRADIO__H_

#include "usv_include.h"

typedef struct{
	uint8	b1_Wn_warn				;		//
	uint8	b1_Wn_mainBoardPower	;
	uint8	b1_Wn_mainBoardTempe	;
	uint8	b1_Wn_int				;
	uint8	b1_Wn_CANComm			;
	uint8	b1_Wn_SCIA				;
	uint8	b1_Wn_SCIB				;		//

	uint8	b1_Wn_externSP1			;		//0		扩展串口1通讯断
	uint8	b1_Wn_externSP2			;
	uint8	b1_Wn_externSP3			;
	uint8	b1_Wn_externSP4			;
	uint8	b1_Wn_externSP5			;
	uint8	b1_Wn_externSP6			;
	uint8	b1_Wn_externSP7			;
	uint8	b1_Wn_externSP8			;

	uint8	b2_Cmd_motor			;	//
	uint8	b1_Cmd_SystemRestart	;

	uint8	b1_Cmd_emergencyMode	;		//应急模式
	uint8	b1_Cmd_emergencyStop	;		//急停
	uint8	b1_Cmd_getAuthority		;		//获取权限
	uint8	b2_Cmd_sailMode			;		//航行模式
	uint8	b2_Cmd_sailTask			;		//航行任务

	uint8	b1_Cmd_speedConstant	;	//定速
	uint8	b1_Cmd_headingConstant	;	//定向
	uint8	b1_Cmd_berthMode		;	//停泊模式

	int16	i16_Cmd_joy1X			;	//
	int16	i16_Cmd_joy1Y			;	//
	int16	i16_Cmd_joy1Z			;

	int16	i16_Cmd_joy2X			;
	int16	i16_Cmd_joy2Y			;
	int16	i16_Cmd_joy2Z			;

	uint8	b1_Cmd_periPowerS1 		;
	uint8	b1_Cmd_periPowerS2 		;
	uint8	b1_Cmd_periPowerS3 		;
	uint8	b1_Cmd_periPowerS4 		;
	uint8	b1_Cmd_periPowerS5 		;
	uint8	b1_Cmd_periPowerS6 		;
	uint8	b1_Cmd_periPowerS7 		;
	uint8	b1_Cmd_periPowerS8 		;

	uint8	b1_Cmd_periPowerS9 		;
	uint8	b1_Cmd_periPowerS10		;
	uint8	b1_Cmd_periPowerS11		;
	uint8	b1_Cmd_periPowerS12		;
	uint8	b1_Cmd_periPowerS13		;
	uint8	b1_Cmd_periPowerS14		;
	uint8	b1_Cmd_periPowerS15		;
	uint8	b1_Cmd_periPowerS16		;

	uint8	b1_Cmd_periPowerS17		;
	uint8	b1_Cmd_periPowerS18		;
	uint8	b1_Cmd_periPowerS19		;
	uint8	b1_Cmd_periPowerS20		;

	uint8	b1_Cmd_elecWindlass1OnOff	;
	uint8	b1_Cmd_elecWindlass1UpDown	;
	uint8	b1_Cmd_elecWindlass2OnOff	;
	uint8	b1_Cmd_elecWindlass2UpDown	;
	uint8	b1_Cmd_sprayStrip1OnOff		;
	uint8	b1_Cmd_sprayStrip1UpDown	;
	uint8	b1_Cmd_sprayStrip2OnOff		;
	uint8	b1_Cmd_sprayStrip2UpDown	;

	uint16	u16_version		;	//版本号
	uint16	u16_svn			;	
	uint16	u16_crc			;
}BRIOP_REV_MSG;


typedef struct{
	//0
	uint8	*b1_Wn_IOPCommOutage	;	//IOP通讯中断
	uint8	*b1_Wn_IOPWarn			;	//IOP总告警
	uint8	*b1_Wn_IHCCommOutage	;	//IHC通讯中断
	uint8	*b1_Wn_IHCWarn			;	//IHC总告警
	uint8	*b1_Wn_IDUCommOutage	;	//IDU通讯中断
	uint8	*b1_Wn_IDUWarn			;	//IDU总告警
	uint8	*b1_Wn_MCUCommOutage	;	//MCU通讯中断
	uint8	*b1_Wn_MCUWarn			;	//MCU总告警

	uint8	*u8_St_year			;	//年		1
	uint8	*u8_St_month		;	//月		2
	uint8	*u8_St_date			;	//日		3

	uint8	*u8_St_hour			;	//时		4
	uint8	*u8_St_minute		;	//分		5
	uint8	*u8_St_second		;	//秒		6

	uint16	*u16_St_speed		;	//航速	7
	uint16	*u16_St_heading		;	//航向	9

	int16	*i16_St_rot			;	//转向率	11
	int16	*i16_St_pitch		;	//俯仰	13
	int16	*i16_St_roll		;	//横滚	15
	int16	*i16_St_heaving		;	//升沉	17

	uint8	*u8_St_longiSt		;	//经度区域	19
	uint8	*u8_St_longiDeg		;	//经度 度	20
	uint8	*u8_St_longiMin		;	//经度 分	21
	uint8	*u8_St_longiSec		;	//经度 秒	22
	uint8	*u8_St_longiSecDec	;	//经度 秒小数	23

	uint8	*u8_St_latiSt		;	//纬度区域	24
	uint8	*u8_St_latiDeg		;	//纬度 度	25
	uint8	*u8_St_latiMin		;	//纬度 分	26
	uint8	*u8_St_latiSec		;	//纬度 秒	27
	uint8	*u8_St_latiSecDec	;	//纬度 秒小数28

	uint8	*b1_St_emergencyMode	;	//应急模式	29
	uint8	*b1_St_emergencyStop	;	//急停
	uint8	*b1_St_localAuthority	;	//船端权限
	uint8	*b2_St_sailMode			;	//航行模式
	uint8	*b2_St_sailTask			;	//航行任务

	uint8	*b1_St_speedConstant		;	//定速	30
	uint8	*b1_St_headingConstant		;	//定向
	uint8	*b1_St_berthMode			;	//泊岸模式
	uint8	*b1_St_setReturnOk			;	//设置返航点成功  
	uint8	*b1_autoReturn				;	//自动返航

	uint8	*b1_St_motorSt		;	//发动机启停	31
	uint8	*b1_Wn_motor1Wn		;	//发动机1故障
	uint8	*b1_Wn_motor2Wn		;	//发动机2故障

	uint16	*u16_St_motor1Rpm		;	//发动机1转速		32
	int16	*i16_St_motor1Gear		;	//发动机1档位		34
	int16	*i16_St_motor1Rudder	;	//发动机1舵向		36

	uint16	*u16_St_motor2Rpm		;	//发动机2转速		38
	int16	*i16_St_motor2Gear		;	//发动机2档位		40
	int16	*i16_St_motor2Rudder	;	//发动机2舵向		42

	uint16	*u16_St_motor3Rpm		;	//发动机3转速		44
	int16	*i16_St_motor3Gear		;	//发动机3档位		46
	int16	*i16_St_motor3Rudder	;	//发动机3舵向		48

	uint16	*u16_St_motor4Rpm		;	//发动机4转速		50
	int16	*i16_St_motor4Gear		;	//发动机4档位		52
	int16	*i16_St_motor4Rudder	;	//发动机4舵向		54

	uint8	*b1_St_remoteKey		;	//遥控钥匙				56
	uint8	*b1_St_PORTMotorCharge	;	//PORT电池发动机充电
	uint8	*b1_St_STBDMotorCharge	;	//STBD电池发动机充电
	uint8	*b1_St_shorePower		;	//岸电
	uint8	*b1_St_PORTShoreCharge	;	//PORT电池岸电充电
	uint8	*b1_St_STBDShoreCharge	;	//STBD电池岸电充电
	uint8	*b1_St_ShoreChargeEnd	;	//岸电充电完成
	uint8	*b1_St_SystemPowerOn	;	//系统上电

	uint8	*b3_St_supplySource		;	//供电回路号				57
	uint8	*b1_Wn_batLow			;	//电量低告警
	uint8	*b1_Wn_oilLow			;	//油量低告警
	uint8	*b1_Wn_volOver			;	//过压告警
	uint8	*b1_Wn_volBelow			;	//欠压告警
	uint8	*b1_Wn_curOver			;	//过流告警

	uint8	*u8_St_PORTBatLvl		;	//左电量		58
	uint8	*u8_St_STBDBatLvl		;	//右电量		59
	uint8	*u8_St_PORTOilLvl		;	//左油量		60
	uint8	*u8_St_STBDOilLvl		;	//有油量		61

	uint16	*u16_St_BatteryVoltage		;	//供电电压
	uint16	*u16_St_BatteryCurrent		;	//供电电流
	uint8	*u8_St_BatteryRemainPercent;	//剩余电量
	uint8	*u8_St_BatteryRemainTime_H	;	//剩余时间 小时
	uint8	*u8_St_BatteryRemainTime_M	;	//剩余时间 分钟
	uint8	*u8_St_BatteryRemainTime_S	;	//剩余时间 秒
	

	uint8	*b1_St_periPowK1					;	//外设电源K1		62
	uint8	*b1_St_periPowK2					;
	uint8	*b1_St_periPowK3					;
	uint8	*b1_St_periPowK4					;
	uint8	*b1_St_periPowK5					;
	uint8	*b1_St_periPowK6					;
	uint8	*b1_St_periPowK7					;
	uint8	*b1_St_periPowK8					;

	uint8	*b1_St_periPowk9				;	//63
	uint8	*b1_St_periPowk10				;
	uint8	*b1_St_periPowk11				;
	uint8	*b1_St_periPowk12				;
	uint8	*b1_St_periPowk13				;
	uint8	*b1_St_periPowk14				;
	uint8	*b1_St_periPowk15				;
	uint8	*b1_St_periPowk16				;

	uint8	*b1_St_periPowk17				;	//64
	uint8	*b1_St_periPowk18				;
	uint8	*b1_St_periPowk19				;
	uint8	*b1_St_periPowk20				;

	uint8	*b1_St_elecWindlass1St			;	//电动锚机1启停		65
	uint8	*b1_St_elecWindlass2St			;	//电动锚机2启停
	uint8	*b1_St_sprayStrip1St			;	//压浪板1启停
	uint8	*b1_St_sprayStrip2St			;	//压浪板2启停

}BRIOP_SEND_MSG;


extern	BRIOP_REV_MSG	BrIOP_rev_msg;
extern	BRIOP_SEND_MSG	BrIOP_send_msg;
extern	COMM_SIGN		bradio_sign	;

extern void	*lan_deal_dradio(void *aa);
void	BRIOP_sdMsg_Init(void);
extern	void BRIOPCommCalInit(void);
extern	void initBRIOPsendTask();
void	runBRIOPSendMsg(void*);







#endif