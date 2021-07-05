//lan_deal_BRadio.h �����̨
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

	uint8	b1_Wn_externSP1			;		//0		��չ����1ͨѶ��
	uint8	b1_Wn_externSP2			;
	uint8	b1_Wn_externSP3			;
	uint8	b1_Wn_externSP4			;
	uint8	b1_Wn_externSP5			;
	uint8	b1_Wn_externSP6			;
	uint8	b1_Wn_externSP7			;
	uint8	b1_Wn_externSP8			;

	uint8	b2_Cmd_motor			;	//
	uint8	b1_Cmd_SystemRestart	;

	uint8	b1_Cmd_emergencyMode	;		//Ӧ��ģʽ
	uint8	b1_Cmd_emergencyStop	;		//��ͣ
	uint8	b1_Cmd_getAuthority		;		//��ȡȨ��
	uint8	b2_Cmd_sailMode			;		//����ģʽ
	uint8	b2_Cmd_sailTask			;		//��������

	uint8	b1_Cmd_speedConstant	;	//����
	uint8	b1_Cmd_headingConstant	;	//����
	uint8	b1_Cmd_berthMode		;	//ͣ��ģʽ

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

	uint16	u16_version		;	//�汾��
	uint16	u16_svn			;	
	uint16	u16_crc			;
}BRIOP_REV_MSG;


typedef struct{
	//0
	uint8	*b1_Wn_IOPCommOutage	;	//IOPͨѶ�ж�
	uint8	*b1_Wn_IOPWarn			;	//IOP�ܸ澯
	uint8	*b1_Wn_IHCCommOutage	;	//IHCͨѶ�ж�
	uint8	*b1_Wn_IHCWarn			;	//IHC�ܸ澯
	uint8	*b1_Wn_IDUCommOutage	;	//IDUͨѶ�ж�
	uint8	*b1_Wn_IDUWarn			;	//IDU�ܸ澯
	uint8	*b1_Wn_MCUCommOutage	;	//MCUͨѶ�ж�
	uint8	*b1_Wn_MCUWarn			;	//MCU�ܸ澯

	uint8	*u8_St_year			;	//��		1
	uint8	*u8_St_month		;	//��		2
	uint8	*u8_St_date			;	//��		3

	uint8	*u8_St_hour			;	//ʱ		4
	uint8	*u8_St_minute		;	//��		5
	uint8	*u8_St_second		;	//��		6

	uint16	*u16_St_speed		;	//����	7
	uint16	*u16_St_heading		;	//����	9

	int16	*i16_St_rot			;	//ת����	11
	int16	*i16_St_pitch		;	//����	13
	int16	*i16_St_roll		;	//���	15
	int16	*i16_St_heaving		;	//����	17

	uint8	*u8_St_longiSt		;	//��������	19
	uint8	*u8_St_longiDeg		;	//���� ��	20
	uint8	*u8_St_longiMin		;	//���� ��	21
	uint8	*u8_St_longiSec		;	//���� ��	22
	uint8	*u8_St_longiSecDec	;	//���� ��С��	23

	uint8	*u8_St_latiSt		;	//γ������	24
	uint8	*u8_St_latiDeg		;	//γ�� ��	25
	uint8	*u8_St_latiMin		;	//γ�� ��	26
	uint8	*u8_St_latiSec		;	//γ�� ��	27
	uint8	*u8_St_latiSecDec	;	//γ�� ��С��28

	uint8	*b1_St_emergencyMode	;	//Ӧ��ģʽ	29
	uint8	*b1_St_emergencyStop	;	//��ͣ
	uint8	*b1_St_localAuthority	;	//����Ȩ��
	uint8	*b2_St_sailMode			;	//����ģʽ
	uint8	*b2_St_sailTask			;	//��������

	uint8	*b1_St_speedConstant		;	//����	30
	uint8	*b1_St_headingConstant		;	//����
	uint8	*b1_St_berthMode			;	//����ģʽ
	uint8	*b1_St_setReturnOk			;	//���÷�����ɹ�  
	uint8	*b1_autoReturn				;	//�Զ�����

	uint8	*b1_St_motorSt		;	//��������ͣ	31
	uint8	*b1_Wn_motor1Wn		;	//������1����
	uint8	*b1_Wn_motor2Wn		;	//������2����

	uint16	*u16_St_motor1Rpm		;	//������1ת��		32
	int16	*i16_St_motor1Gear		;	//������1��λ		34
	int16	*i16_St_motor1Rudder	;	//������1����		36

	uint16	*u16_St_motor2Rpm		;	//������2ת��		38
	int16	*i16_St_motor2Gear		;	//������2��λ		40
	int16	*i16_St_motor2Rudder	;	//������2����		42

	uint16	*u16_St_motor3Rpm		;	//������3ת��		44
	int16	*i16_St_motor3Gear		;	//������3��λ		46
	int16	*i16_St_motor3Rudder	;	//������3����		48

	uint16	*u16_St_motor4Rpm		;	//������4ת��		50
	int16	*i16_St_motor4Gear		;	//������4��λ		52
	int16	*i16_St_motor4Rudder	;	//������4����		54

	uint8	*b1_St_remoteKey		;	//ң��Կ��				56
	uint8	*b1_St_PORTMotorCharge	;	//PORT��ط��������
	uint8	*b1_St_STBDMotorCharge	;	//STBD��ط��������
	uint8	*b1_St_shorePower		;	//����
	uint8	*b1_St_PORTShoreCharge	;	//PORT��ذ�����
	uint8	*b1_St_STBDShoreCharge	;	//STBD��ذ�����
	uint8	*b1_St_ShoreChargeEnd	;	//���������
	uint8	*b1_St_SystemPowerOn	;	//ϵͳ�ϵ�

	uint8	*b3_St_supplySource		;	//�����·��				57
	uint8	*b1_Wn_batLow			;	//�����͸澯
	uint8	*b1_Wn_oilLow			;	//�����͸澯
	uint8	*b1_Wn_volOver			;	//��ѹ�澯
	uint8	*b1_Wn_volBelow			;	//Ƿѹ�澯
	uint8	*b1_Wn_curOver			;	//�����澯

	uint8	*u8_St_PORTBatLvl		;	//�����		58
	uint8	*u8_St_STBDBatLvl		;	//�ҵ���		59
	uint8	*u8_St_PORTOilLvl		;	//������		60
	uint8	*u8_St_STBDOilLvl		;	//������		61

	uint16	*u16_St_BatteryVoltage		;	//�����ѹ
	uint16	*u16_St_BatteryCurrent		;	//�������
	uint8	*u8_St_BatteryRemainPercent;	//ʣ�����
	uint8	*u8_St_BatteryRemainTime_H	;	//ʣ��ʱ�� Сʱ
	uint8	*u8_St_BatteryRemainTime_M	;	//ʣ��ʱ�� ����
	uint8	*u8_St_BatteryRemainTime_S	;	//ʣ��ʱ�� ��
	

	uint8	*b1_St_periPowK1					;	//�����ԴK1		62
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

	uint8	*b1_St_elecWindlass1St			;	//�綯ê��1��ͣ		65
	uint8	*b1_St_elecWindlass2St			;	//�綯ê��2��ͣ
	uint8	*b1_St_sprayStrip1St			;	//ѹ�˰�1��ͣ
	uint8	*b1_St_sprayStrip2St			;	//ѹ�˰�2��ͣ

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