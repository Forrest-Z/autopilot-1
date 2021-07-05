//can_deal_IOP.h
#ifndef __CAN_DEAL_IOP__H_
#define __CAN_DEAL_IOP__H_

#include "usv_include.h"



typedef struct{
	//PS=0
	uint8	b1_Wn_warn				;	//0		总告警
	uint8	b1_Wn_mainBoardPower	;	//1		主板电源
	uint8	b1_Wn_mainBoardTempe	;	//2		主板温度
	uint8	b1_Wn_int				;	//3		中断异常
	uint8	b1_Wn_CANComm			;	//4		CAN2通讯断
	uint8	b1_Wn_SCIA				;	//5		SCIA通讯断
	uint8	b1_Wn_SCIB				;	//6		SCIB通讯断
	
	uint8	b1_Wn_externSP1			;	//0		扩展串口1通讯断
	uint8	b1_Wn_externSP2			;
	uint8	b1_Wn_externSP3			;
	uint8	b1_Wn_externSP4			;
	uint8	b1_Wn_externSP5			;
	uint8	b1_Wn_externSP6			;
	uint8	b1_Wn_externSP7			;
	uint8	b1_Wn_externSP8			;

	uint8	b2_Cmd_motor			;		//发动机控制
	uint8	b1_Cmd_systemReset		;		//系统重启
	
	uint8	b1_Cmd_emergencyMode	;		//应急模式
	uint8	b1_Cmd_emergencyStop	;		//急停
	uint8	b1_Cmd_getAuthority		;		//获取权限
	uint8	b2_Cmd_sailMode			;		//航行模式
	uint8	b2_Cmd_sailTask			;		//航行任务

	uint8	b1_Cmd_speedConstant		;	//定速
	uint8	b1_Cmd_headingConstant		;	//定向
	uint8	b1_Cmd_setReturnPoint		;	//设置返航点
	uint8	b1_Cmd_autoReturn			;	//自动返航
	uint8	b1_Cmd_berthMode			;	//停泊模式
	//PS = 1
	int16	i16_Cmd_joy1X			;	//
	int16	i16_Cmd_joy1Y			;	//
	int16	i16_Cmd_joy1Z			;
	//PS = 2
	int16	i16_Cmd_joy2X			;
	int16	i16_Cmd_joy2Y			;
	int16	i16_Cmd_joy2Z			;
	//PS = 3
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

}IOP_REV_MSG;

typedef struct{
	uint16	u16_version		;	//版本号
	uint16	u16_svn			;	
	uint16	u16_crc			;
	uint8	u8_confState	;	//配置读取状态
	uint8	u8_initState	;	//初始化状态
}IOP_REV_CONFIG;


typedef struct{
	//PS31
	uint8	*b1_Wn_IOPCommOutage		;	//IOP通讯中断
	uint8	*b1_Wn_IOPWarn			;	//IOP总告警
	uint8	*b1_Wn_IHCCommOutage		;	//IHC通讯中断
	uint8	*b1_Wn_IHCWarn			;	//IHC总告警
	uint8	*b1_Wn_IDUCommOutage		;	//IDU通讯中断
	uint8	*b1_Wn_IDUWarn			;	//IDU总告警
	uint8	*b1_Wn_MCUCommOutage		;	//MCU通讯中断
	uint8	*b1_Wn_MCUWarn			;	//MCU总告警

	uint8	*b1_Wn_radio1			;	//电台1
	uint8	*b1_Wn_radio2			;	//电台2
	uint8	*b1_Wn_BD				;	//北斗

	//PS32
	uint8	*u8_St_year				;	//年
	uint8	*u8_St_month			;	//月
	uint8	*u8_St_date				;	//日
	uint8	*u8_St_hour				;	//时
	uint8	*u8_St_minute			;	//分
	uint8	*u8_St_second			;	//秒

	//PS33
	uint16	*u16_St_speed			;	//航速
	uint16	*u16_St_heading			;	//航向
	

	//PS34
	int16	*i16_St_rot				;	//转向率
	int16	*i16_St_pitch			;	//俯仰
	int16	*i16_St_roll			;	//横滚
	int16	*i16_St_heaving			;	//升沉

	//PS35
	uint8	*u8_St_longiSt			;	//经度区域
	uint8	*u8_St_longiDeg			;	//经度 度
	uint8	*u8_St_longiMin			;	//经度 分
	uint8	*u8_St_longiSec			;	//经度 秒
	uint8	*u8_St_longiSecDec		;	//经度 秒小数

	//PS36
	uint8	*u8_St_latiSt			;	//纬度区域
	uint8	*u8_St_latiDeg			;	//纬度 度
	uint8	*u8_St_latiMin			;	//纬度 分
	uint8	*u8_St_latiSec			;	//纬度 秒
	uint8	*u8_St_latiSecDec		;	//纬度 秒小数

	//PS37
	uint8	*b1_St_emergencyMode		;	//应急模式
	uint8	*b1_St_emergencyStop		;	//急停
	uint8	*b1_St_localAuthority	;	//船端权限
	uint8	*b2_St_sailMode			;	//航行模式
	uint8	*b2_St_sailTask			;	//航行任务

	uint8	*b1_St_speedConstant		;	//定速
	uint8	*b1_St_headingConstant	;	//定向
	uint8	*b1_St_setReturn			;	//返航点设置成功
	uint8	*b1_St_autoReturn		;	//自动返航
	uint8	*b1_St_berthMode			;	//泊岸模式

	uint8	*b1_St_motorSt			;	//发动机启停
	uint8	*b1_St_motor1Wn			;	//发动机1告警
	uint8	*b1_St_motor2Wn			;	//发动机2告警

	//PS38
	uint16	*u16_St_motor1Rpm		;	//发动机1转速
	int16	*i16_St_motor1Gear		;	//发动机1档位
	int16	*i16_St_motor1Rudder		;	//发动机1舵向

	//PS39
	uint16	*u16_St_motor2Rpm		;	//发动机2转速
	int16	*i16_St_motor2Gear		;	//发动机2档位
	int16	*i16_St_motor2Rudder		;	//发动机2舵向

	//PS40
	uint16	*u16_St_motor3Rpm		;	//发动机3转速
	int16	*i16_St_motor3Gear		;	//发动机3档位
	int16	*i16_St_motor3Rudder		;	//发动机3舵向

	//PS41
	uint16	*u16_St_motor4Rpm		;	//发动机4转速
	int16	*i16_St_motor4Gear		;	//发动机4档位
	int16	*i16_St_motor4Rudder		;	//发动机4舵向

	//PS42
	uint8	*b1_St_remoteKey			;	//遥控钥匙
	uint8	*b1_St_PORTMotorCharge	;	//PORT电池发动机充电
	uint8	*b1_St_STBDMotorCharge	;	//STBD电池发动机充电
	uint8	*b1_St_shorePower		;	//岸电
	uint8	*b1_St_PORTShoreCharge	;	//PORT电池岸电充电
	uint8	*b1_St_STBDShoreCharge	;	//STBD电池岸电充电
	uint8	*b1_St_ShoreChargeEnd	;	//岸电充电完成
	uint8	*b1_St_systemPowerOn		;	//系统上电

	uint8	*b3_St_supplySource		;	//供电回路号	
	uint8	*b1_Wn_batLow			;	//电量低告警
	uint8	*b1_Wn_oilLow			;	//油量低告警
	uint8	*b1_Wn_volOver			;	//过压告警
	uint8	*b1_Wn_volBelow			;	//欠压告警
	uint8	*b1_Wn_curOver			;	//过流告警

	uint8	*u8_St_PORTBatLvl		;	//左电量
	uint8	*u8_St_STBDBatLvl		;	//右电量
	uint8	*u8_St_PORTOilLvl		;	//左油量
	uint8	*u8_St_STBDOilLvl		;	//有油量

	//PS43
	uint8	*b1_St_periPowK1			;	//外设电源K1
	uint8	*b1_St_periPowK2			;
	uint8	*b1_St_periPowK3			;
	uint8	*b1_St_periPowK4			;
	uint8	*b1_St_periPowK5			;
	uint8	*b1_St_periPowK6			;
	uint8	*b1_St_periPowK7			;
	uint8	*b1_St_periPowK8			;

	uint8	*b1_St_periPowk9			;
	uint8	*b1_St_periPowk10		;
	uint8	*b1_St_periPowk11		;
	uint8	*b1_St_periPowk12		;
	uint8	*b1_St_periPowk13		;
	uint8	*b1_St_periPowk14		;
	uint8	*b1_St_periPowk15		;
	uint8	*b1_St_periPowk16		;

	uint8	*b1_St_periPowk17		;
	uint8	*b1_St_periPowk18		;
	uint8	*b1_St_periPowk19		;
	uint8	*b1_St_periPowk20		;

	uint8	*b1_St_elecWindlass1St	;	//电动锚机1启停
	uint8	*b1_St_elecWindlass2St	;	//电动锚机2启停
	uint8	*b1_St_sprayStrip1St		;	//压浪板1启停
	uint8	*b1_St_sprayStrip2St		;	//压浪板2启停

}IOP_SEND_MSG;

#define CAN_IOP_ADD 134
#define CAN_IOP_DISCONNECT_MAX 10
extern IOP_REV_MSG		IOP_rev_msg		;
extern COMM_SIGN		IOP_comm_sign	;
void IOP_rvMsg_Init(void);
void IOP_sdMsg_Init(void);
void IOP_config_Init(void);

extern void IOP_Init(void);
extern void IOP_CommCal(void);
extern void IOP_recv(uint8 psID,uint8* data);
extern void IOP_sendMsg(void);

extern void initIOPsendTask(void);
void runIOPSendMsg(void *);

#endif /*__CAN_DEAL_IOP__H_*/