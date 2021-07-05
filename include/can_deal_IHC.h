//can_deal_IHC.h
#ifndef __CAN_DEAL_IHC__H_
#define __CAN_DEAL_IHC__H_

#include "usv_include.h"

typedef struct{
	uint8	b1_St_MotorOn		;	
	uint8	b1_Wn_CommonWarn	;	
	uint8	b1_St_Authority		;	
}MID_ST;


typedef struct{
    //PS=0
    uint8   b1_St_Motor1OnOff           ;
    uint8   b1_St_Motor1EmergencyStop   ;
    uint16  u16_St_Motor1Rpm            ;
    int16   i16_St_Motor1Gear           ;
    int16   i16_St_Motor1Rudder         ;
	uint8   b1_water_level;
    
    //PS=1
    uint8   b1_St_Motor2OnOff           ;
    uint8   b1_St_Motor2EmergencyStop   ;
    uint16  u16_St_Motor2Rpm            ;
    int16   i16_St_Motor2Gear           ;
    int16   i16_St_Motor2Rudder         ;
    
    //PS=2
    uint8	b1_St_elecWindlass1OnOff	;
	uint8	b1_St_elecWindlass2OnOff	;
	uint8	b1_St_sprayStrip1OnOff		;
	uint8	b1_St_sprayStrip2OnOff		;
	
	uint8	b1_Wn_ClassIWarn			;
	uint8	b1_Wn_mainBoardPower		;
	uint8	b1_Wn_mainBoardTempe		;
	uint8	b1_Wn_SPI					;
	uint8	b1_Wn_TL16C554_1Comm		;
	uint8	b1_Wn_TL16C554_2Comm		;
	uint8	b1_Wn_TL16C554_3Comm		;
	uint8	b1_Wn_TL16C554_4Comm		;

	uint8	b1_Wn_ClassIIWarn			;
	uint8	b1_Wn_PORTMotorWarn			;
	uint8	b1_Wn_STBDMotorWarn			;
	uint8	b1_Wn_PORTGearWarn			;
	uint8	b1_Wn_PORTRudderWarn		;
	uint8	b1_Wn_STBDGearWarn			;
	uint8	b1_Wn_STBDRudderWarn		;

	uint8	u8_St_PORTBatLvl					;
	uint8	u8_St_STBDBatLvl					;

	uint16	u16_St_BatteryVoltage		;	//�����ѹ
	uint16	u16_St_BatteryCurrent		;	//�������
	uint8	u8_St_BatteryLevel			;	//ʣ�����
	uint8	u8_St_BatteryRemainTime_H	;	//ʣ��ʱ�� Сʱ
	uint8	u8_St_BatteryRemainTime_M	;	//ʣ��ʱ�� ����
	uint8	u8_St_BatteryRemainTime_S	;	//ʣ��ʱ�� ��

	//��ʱ�汾
	uint8	tmp_u8_St_BatteryVoltage;	//�����ѹ��ʱ�汾
	uint8	tmp_u8_St_BatteryCurrent;	//���������ʱ�汾
	
	MID_ST mid_st;
}IHC_REV_MSG;

typedef struct{
	uint16	u16_version		;
	uint16	u16_svn			;
	uint16	u16_crc			;
	uint8	u8_confState	;	//���ö�ȡ״̬
	uint8	u8_initState	;	//��ʼ��״̬    
}IHC_REV_CONFIG;

typedef struct{
    //PS=10
    uint8   *b2_Cmd_Motor1OnOff          ;
    uint8   *b1_Cmd_Motor1EmergencyStop  ;
    uint8   *u8_Cmd_Motor1OpenDeg        ;   //���ſ���
    int16   *i16_Cmd_Motor1GearDeg       ;   //��λ�Ƕ�
    int16   *i16_Cmd_Motor1RudderDeg     ;   //���
    //PS=11
    uint8   *b2_Cmd_Motor2OnOff          ;
    uint8   *b1_Cmd_Motor2EmergencyStop  ;
    uint8   *u8_Cmd_Motor2OpenDeg        ;   //���ſ���
    int16   *i16_Cmd_Motor2GearDeg       ;   //��λ�Ƕ�
    int16   *i16_Cmd_Motor2RudderDeg     ;   //���
    //PS=12
	uint8	*b1_Cmd_elecWindless1OnOff	 ;	//�綯ê����ͣ
	uint8	*b1_Cmd_elecWindless1UpDown	 ;	//�綯ê������
	uint8	*b1_Cmd_elecWindless2OnOff	 ;
	uint8	*b1_Cmd_elecWindless2UpDown	 ;
	uint8	*b1_Cmd_sprayStrip1OnOff	 ;	//ѹ�˰���ͣ
	uint8	*b1_Cmd_sprayStrip1UpDown	 ;	//ѹ�˰巽��
	uint8	*b1_Cmd_sprayStrip2OnOff	 ;	
	uint8	*b1_Cmd_sprayStrip2UpDown	 ;    
}IHC_SEND_MSG;

typedef struct{
	uint32	u32_Wn_driver1ErrCode		 ;
	uint32	u32_Wn_driver2ErrCode		 ;
	uint32	u32_Wn_driver3ErrCode		 ;
	uint32	u32_Wn_driver4ErrCode		 ;

	uint32	u32_Wn_Motor1ErrCode		 ;
	uint32	u32_Wn_Motor2ErrCode		 ;

}IHC_ERR_CODE;

// ������ ����ת������̨ʹ��
typedef struct{
	uint16	u16_errorSq;
	uint8	u8_errorStat;
}FAULT_CODE;

// ������ظ�
typedef struct{
	uint8	e_data[8];		//���ϱ���
	uint8	e_flag;			//�����跢�ͱ�־
}ERROR_MSG;

#define CAN_IHC_ADD 129
#define CAN_IHC_DISCONNECT_MAX 20

extern COMM_SIGN	IHC_comm_sign;
extern IHC_REV_MSG			IHC_rev_msg		;
void IHC_rvMsg_Init(void);
void IHC_sdMsg_Init(void);
void IHC_config_Init(void);
void IHC_Err_Init(void);

extern void IHC_Init(void);
extern void IHC_CommCal(void);
extern void IHC_recv(uint8 psID,uint8* data);
extern void IHC_sendMsg(void);
extern void IHC_call_driverErrCode(uint8 driverNum);
extern void IHC_call_motorErrCode(uint8	motorNum);

extern void initIHCsendTask(void);
void runIHCsendTask(void *);

//
void IHC_faultCodeTrans(uint8* data);
#endif /*__CAN_DEAL_IHC__H_*/