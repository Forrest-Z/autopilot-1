//can_deal_IDU.h
#ifndef __CAN_DEAL_IDU__H_
#define __CAN_DEAL_IDU__H_

#include "usv_include.h"

typedef struct{
	uint8	b1_Wn_volOver	;	//��ѹ�澯
	uint8	b1_Wn_volBelow	;	//Ƿѹ�澯
	uint8	b1_Wn_curOver	;	//�����澯
}IDUmidMsg;


typedef struct{
	//PS = 0
	uint8	b1_St_remoteKey			;	//0	 Byte0
	uint8	b1_St_PORTMotorCharge	;	//1
	uint8	b1_St_STBDMotorCharge		;	//2
	uint8	b1_St_shorePower		;	//3
	uint8	b1_St_PORTShoreCharge	;	//4
	uint8	b1_St_STBDShoreCharge	;	//5
	uint8	b1_St_ShoreChargeEnd	;	//6
	uint8	b1_St_SystemPowerOn		;	//7

	uint8	b3_St_supplySource		;	//0-2
	uint8	b1_Wn_batLow			;	//3
	uint8	b1_Wn_oilLow			;	//4
	uint8	b1_Wn_autoOff			;	//5
	uint8	b1_Wn_oilLevelGauge		;	//6
	uint8	b1_Wn_ShoreVolOver		;	//7

	uint8	b1_Wn_ShoreVolBelow		;	//0
	uint8	b1_Wn_ShoreCurOver		;	//1
	uint8	b1_Wn_PORTVolOver		;	//2
	uint8	b1_Wn_PORTVolBelow		;	//3
	uint8	b1_Wn_PORTCurOver		;	//4
	uint8	b1_Wn_STBDVolOver		;	//5
	uint8	b1_Wn_STBDVolBelow		;	//6
	uint8	b1_Wn_STBDCurOver		;	//7

	uint8	b1_Wn_warn				;	//0		�ܸ澯
	uint8	b1_Wn_mainBoardPower	;	//1		�����Դ
	uint8	b1_Wn_mainBoardTempe	;	//2		�����¶�
	uint8	b1_Wn_int				;	//3		�ж��쳣
	uint8	b1_Wn_CANComm			;	//4		CAN2ͨѶ��
	uint8	b1_Wn_SCIA				;	//5		SCIAͨѶ��
	uint8	b1_Wn_SCIB				;	//6		SCIBͨѶ��

	uint8	u8_St_PORTBatLvl			;	//Byte4	PORT��ص���
	uint8	u8_St_STBDBatLvl			;	//Byte5	STBD��ص���
	uint16	u16_St_remainBatTime		;	//Byte6-7	ʣ�๩��ʱ��
	//PS=1
	uint16	u16_St_PORTInstanVol			;	//Byte0-1	PORT˲ʱ��ѹ
	uint16	u16_St_PORTInstanCur			;	//Byte2-3	PORT˲ʱ����
	uint16	u16_St_PORTInstanPow			;	//Byte4-5	PORT˲ʱ����
	//PS=2
	uint16	u16_St_STBDInstanVol			;	//Byte0-1	STBD˲ʱ��ѹ
	uint16	u16_St_STBDInstanCur			;	//Byte2-3	STBD˲ʱ����
	uint16	u16_St_STBDInstanPow			;	//Byte4-5	STBD˲ʱ����
	//PS=3
	uint16	u16_St_ShoreInstanVol		;	//Byte0-1	����˲ʱ��ѹ
	uint16	u16_St_ShoreInstanCur		;	//Byte1-2	����˲ʱ��ѹ
	uint16	u16_St_ShoreInstanPow		;	//Byte3-4	����˲ʱ����
	//PS=4
	uint8	u8_St_PortOilLvl				;	//Byte0		PORT����
	float	f32_St_PORTIntergFlow		;	//Byte4-7	PORT�ۼ�����
	//PS=5
	float	f32_St_PORTInstanFlow		;	//Byte0-3	PORT˲ʱ����
	float	f32_St_PORTIntergTime		;	//Byte4-7	PORT�ۼ�ʱ��
	//PS=6
	uint8	u8_St_STBDOilLvl			;	//Byte0		STBD����
	float	f32_St_STBDIntergFlow		;	//Byte4-7	STBD�ۼ�����
	//PS=7
	float	f32_St_STBDInstanFlow		;	//Byte0-3	STBD˲ʱ����
	float	f32_St_STBDIntergTime		;	//Byte4-7	STBD�ۼ�����
	//PS=8
	uint8	b1_St_periPowK1					;
	uint8	b1_St_periPowK2					;
	uint8	b1_St_periPowK3					;
	uint8	b1_St_periPowK4					;
	uint8	b1_St_periPowK5					;
	uint8	b1_St_periPowK6					;
	uint8	b1_St_periPowK7					;
	uint8	b1_St_periPowK8					;

	uint8	b1_St_periPowk9					;
	uint8	b1_St_periPowk10				;
	uint8	b1_St_periPowk11				;
	uint8	b1_St_periPowk12				;
	uint8	b1_St_periPowk13				;
	uint8	b1_St_periPowk14				;
	uint8	b1_St_periPowk15				;
	uint8	b1_St_periPowk16				;
	
	uint8	b1_St_periPowk17				;
	uint8	b1_St_periPowk18				;
	uint8	b1_St_periPowk19				;
	uint8	b1_St_periPowk20				;

	uint8	b1_St_sailLightK1 				;
	uint8	b1_St_sailLightK2 				;
	uint8	b1_St_sailLightK3 				;
	uint8	b1_St_sailLightK4 				;
	uint8	b1_St_sailLightK5 				;
	uint8	b1_St_sailLightK6 				;
	uint8	b1_St_sailLightK7 				;
	uint8	b1_St_sailLightK8 				;


	uint8	b1_St_plugConK1 					;
	uint8	b1_St_plugConK2 					;
	uint8	b1_St_plugConK3 					;
	uint8	b1_St_plugConK4 					;
	uint8	b1_St_plugConK5 					;
	uint8	b1_St_plugConK6 					;
	uint8	b1_St_plugConK7 					;
	uint8	b1_St_plugConK8 					;

	uint8	b1_St_plugConK9 					;
	uint8	b1_St_plugConK10					;
	uint8	b1_St_plugConK11					;
	uint8	b1_St_plugConK12					;
	uint8	b1_St_plugConK13					;
	uint8	b1_St_plugConK14					;
	uint8	b1_St_plugConK15					;
	uint8	b1_St_plugConK16					;

	uint8	b1_St_plugConK17					;
	uint8	b1_St_plugConK18					;
	uint8	b1_St_plugConK19					;
	uint8	b1_St_plugConK20					;


	IDUmidMsg	midMsg							;
}IDU_REV_MSG;

typedef struct{
	uint16	u16_version		;
	uint16	u16_svn			;
	uint16	u16_crc			;
	uint8	u8_confState	;	//���ö�ȡ״̬
	uint8	u8_initState	;	//��ʼ��״̬
}IDU_REV_CONFIG;

typedef struct{
	uint8		*b1_St_MotorOnOff;			//PS21:0
	uint8		*b1_Cmd_SystemRestart;		//PS21:7
	
	uint8		*b1_Cmd_periPowK1;		//PS21:1:0
	uint8		*b1_Cmd_periPowK2;
	uint8		*b1_Cmd_periPowK3;
	uint8		*b1_Cmd_periPowK4;
	uint8		*b1_Cmd_periPowK5;
	uint8		*b1_Cmd_periPowK6;
	uint8		*b1_Cmd_periPowK7;
	uint8		*b1_Cmd_periPowK8;

	uint8		*b1_Cmd_periPowK9;
	uint8		*b1_Cmd_periPowK10;
	uint8		*b1_Cmd_periPowK11;
	uint8		*b1_Cmd_periPowK12;
	uint8		*b1_Cmd_periPowK13;
	uint8		*b1_Cmd_periPowK14;
	uint8		*b1_Cmd_periPowK15;
	uint8		*b1_Cmd_periPowK16;

	uint8		*b1_Cmd_periPowK17;
	uint8		*b1_Cmd_periPowK18;
	uint8		*b1_Cmd_periPowK19;
	uint8		*b1_Cmd_periPowK20;

	uint8		*b1_Cmd_SailLightPowK1;
	uint8		*b1_Cmd_SailLightPowK2;
	uint8		*b1_Cmd_SailLightPowK3;
	uint8		*b1_Cmd_SailLightPowK4;
	uint8		*b1_Cmd_SailLightPowK5;
	uint8		*b1_Cmd_SailLightPowK6;
	uint8		*b1_Cmd_SailLightPowK7;
	uint8		*b1_Cmd_SailLightPowK8;

}IDU_SEND_MSG;


#define CAN_IDU_ADD 133
#define CAN_IDU_DISCONNECT_MAX 10


extern COMM_SIGN		IDU_comm_sign	;
extern IDU_REV_MSG		IDU_rev_msg		;
extern IDU_SEND_MSG		IDU_send_msg	;
void IDU_rvMsg_Init(void);
void IDU_sdMsg_Init(void);
void IDU_config_Init(void);
extern void IDU_Init(void);
extern void IDU_CommCal(void);
extern void IDU_recv(uint8 psID,uint8* data);
extern void IDU_sendMsg(void);

extern void initIDUsendTask();
void runIDUsendTask(void *);

//
uint8* IDU_Switch_connect(int32 connectNum);

#endif /*__CAN_DEAL_IDU__H_*/