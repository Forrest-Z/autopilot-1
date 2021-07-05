//can_deal_IOP.h
#ifndef __CAN_DEAL_IOP__H_
#define __CAN_DEAL_IOP__H_

#include "usv_include.h"



typedef struct{
	//PS=0
	uint8	b1_Wn_warn				;	//0		�ܸ澯
	uint8	b1_Wn_mainBoardPower	;	//1		�����Դ
	uint8	b1_Wn_mainBoardTempe	;	//2		�����¶�
	uint8	b1_Wn_int				;	//3		�ж��쳣
	uint8	b1_Wn_CANComm			;	//4		CAN2ͨѶ��
	uint8	b1_Wn_SCIA				;	//5		SCIAͨѶ��
	uint8	b1_Wn_SCIB				;	//6		SCIBͨѶ��
	
	uint8	b1_Wn_externSP1			;	//0		��չ����1ͨѶ��
	uint8	b1_Wn_externSP2			;
	uint8	b1_Wn_externSP3			;
	uint8	b1_Wn_externSP4			;
	uint8	b1_Wn_externSP5			;
	uint8	b1_Wn_externSP6			;
	uint8	b1_Wn_externSP7			;
	uint8	b1_Wn_externSP8			;

	uint8	b2_Cmd_motor			;		//����������
	uint8	b1_Cmd_systemReset		;		//ϵͳ����
	
	uint8	b1_Cmd_emergencyMode	;		//Ӧ��ģʽ
	uint8	b1_Cmd_emergencyStop	;		//��ͣ
	uint8	b1_Cmd_getAuthority		;		//��ȡȨ��
	uint8	b2_Cmd_sailMode			;		//����ģʽ
	uint8	b2_Cmd_sailTask			;		//��������

	uint8	b1_Cmd_speedConstant		;	//����
	uint8	b1_Cmd_headingConstant		;	//����
	uint8	b1_Cmd_setReturnPoint		;	//���÷�����
	uint8	b1_Cmd_autoReturn			;	//�Զ�����
	uint8	b1_Cmd_berthMode			;	//ͣ��ģʽ
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
	uint16	u16_version		;	//�汾��
	uint16	u16_svn			;	
	uint16	u16_crc			;
	uint8	u8_confState	;	//���ö�ȡ״̬
	uint8	u8_initState	;	//��ʼ��״̬
}IOP_REV_CONFIG;


typedef struct{
	//PS31
	uint8	*b1_Wn_IOPCommOutage		;	//IOPͨѶ�ж�
	uint8	*b1_Wn_IOPWarn			;	//IOP�ܸ澯
	uint8	*b1_Wn_IHCCommOutage		;	//IHCͨѶ�ж�
	uint8	*b1_Wn_IHCWarn			;	//IHC�ܸ澯
	uint8	*b1_Wn_IDUCommOutage		;	//IDUͨѶ�ж�
	uint8	*b1_Wn_IDUWarn			;	//IDU�ܸ澯
	uint8	*b1_Wn_MCUCommOutage		;	//MCUͨѶ�ж�
	uint8	*b1_Wn_MCUWarn			;	//MCU�ܸ澯

	uint8	*b1_Wn_radio1			;	//��̨1
	uint8	*b1_Wn_radio2			;	//��̨2
	uint8	*b1_Wn_BD				;	//����

	//PS32
	uint8	*u8_St_year				;	//��
	uint8	*u8_St_month			;	//��
	uint8	*u8_St_date				;	//��
	uint8	*u8_St_hour				;	//ʱ
	uint8	*u8_St_minute			;	//��
	uint8	*u8_St_second			;	//��

	//PS33
	uint16	*u16_St_speed			;	//����
	uint16	*u16_St_heading			;	//����
	

	//PS34
	int16	*i16_St_rot				;	//ת����
	int16	*i16_St_pitch			;	//����
	int16	*i16_St_roll			;	//���
	int16	*i16_St_heaving			;	//����

	//PS35
	uint8	*u8_St_longiSt			;	//��������
	uint8	*u8_St_longiDeg			;	//���� ��
	uint8	*u8_St_longiMin			;	//���� ��
	uint8	*u8_St_longiSec			;	//���� ��
	uint8	*u8_St_longiSecDec		;	//���� ��С��

	//PS36
	uint8	*u8_St_latiSt			;	//γ������
	uint8	*u8_St_latiDeg			;	//γ�� ��
	uint8	*u8_St_latiMin			;	//γ�� ��
	uint8	*u8_St_latiSec			;	//γ�� ��
	uint8	*u8_St_latiSecDec		;	//γ�� ��С��

	//PS37
	uint8	*b1_St_emergencyMode		;	//Ӧ��ģʽ
	uint8	*b1_St_emergencyStop		;	//��ͣ
	uint8	*b1_St_localAuthority	;	//����Ȩ��
	uint8	*b2_St_sailMode			;	//����ģʽ
	uint8	*b2_St_sailTask			;	//��������

	uint8	*b1_St_speedConstant		;	//����
	uint8	*b1_St_headingConstant	;	//����
	uint8	*b1_St_setReturn			;	//���������óɹ�
	uint8	*b1_St_autoReturn		;	//�Զ�����
	uint8	*b1_St_berthMode			;	//����ģʽ

	uint8	*b1_St_motorSt			;	//��������ͣ
	uint8	*b1_St_motor1Wn			;	//������1�澯
	uint8	*b1_St_motor2Wn			;	//������2�澯

	//PS38
	uint16	*u16_St_motor1Rpm		;	//������1ת��
	int16	*i16_St_motor1Gear		;	//������1��λ
	int16	*i16_St_motor1Rudder		;	//������1����

	//PS39
	uint16	*u16_St_motor2Rpm		;	//������2ת��
	int16	*i16_St_motor2Gear		;	//������2��λ
	int16	*i16_St_motor2Rudder		;	//������2����

	//PS40
	uint16	*u16_St_motor3Rpm		;	//������3ת��
	int16	*i16_St_motor3Gear		;	//������3��λ
	int16	*i16_St_motor3Rudder		;	//������3����

	//PS41
	uint16	*u16_St_motor4Rpm		;	//������4ת��
	int16	*i16_St_motor4Gear		;	//������4��λ
	int16	*i16_St_motor4Rudder		;	//������4����

	//PS42
	uint8	*b1_St_remoteKey			;	//ң��Կ��
	uint8	*b1_St_PORTMotorCharge	;	//PORT��ط��������
	uint8	*b1_St_STBDMotorCharge	;	//STBD��ط��������
	uint8	*b1_St_shorePower		;	//����
	uint8	*b1_St_PORTShoreCharge	;	//PORT��ذ�����
	uint8	*b1_St_STBDShoreCharge	;	//STBD��ذ�����
	uint8	*b1_St_ShoreChargeEnd	;	//���������
	uint8	*b1_St_systemPowerOn		;	//ϵͳ�ϵ�

	uint8	*b3_St_supplySource		;	//�����·��	
	uint8	*b1_Wn_batLow			;	//�����͸澯
	uint8	*b1_Wn_oilLow			;	//�����͸澯
	uint8	*b1_Wn_volOver			;	//��ѹ�澯
	uint8	*b1_Wn_volBelow			;	//Ƿѹ�澯
	uint8	*b1_Wn_curOver			;	//�����澯

	uint8	*u8_St_PORTBatLvl		;	//�����
	uint8	*u8_St_STBDBatLvl		;	//�ҵ���
	uint8	*u8_St_PORTOilLvl		;	//������
	uint8	*u8_St_STBDOilLvl		;	//������

	//PS43
	uint8	*b1_St_periPowK1			;	//�����ԴK1
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

	uint8	*b1_St_elecWindlass1St	;	//�綯ê��1��ͣ
	uint8	*b1_St_elecWindlass2St	;	//�綯ê��2��ͣ
	uint8	*b1_St_sprayStrip1St		;	//ѹ�˰�1��ͣ
	uint8	*b1_St_sprayStrip2St		;	//ѹ�˰�2��ͣ

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