/*
* can_api.h --CAN��ʼ��
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#ifndef		MONITOR_H
#define     MONITOR_H

#define  htons_86(x) (uint16) ( ((((uint16)(x)) & 0x00ff) << 8 ) |((((uint16)(x)) & 0xff00) >> 8 ))
#define  htonl_86(x) (uint32) ( (( ((uint32)(x)) & 0xff000000 ) >> 24) | (( ((uint32)(x)) & 0x00ff0000 ) >> 8 ) | \
						(( ((uint32)(x)) & 0x0000ff00 ) << 8 ) | (( ((uint32)(x)) & 0x000000ff ) << 24))

#ifdef WINNT
#define  htons_86_win(x) (uint16) ( ((((uint16)(x)) & 0x00ff) << 8 ) |((((uint16)(x)) & 0xff00) >> 8 ))
#define  htonl_86_win(x) (uint32) ( (( ((uint32)(x)) & 0xff000000 ) >> 24) | (( ((uint32)(x)) & 0x00ff0000 ) >> 8 ) | \
						(( ((uint32)(x)) & 0x0000ff00 ) << 8 ) | (( ((uint32)(x)) & 0x000000ff ) << 24))
#else
#define  htons_86_win(x) (uint16) ( x)
#define  htonl_86_win(x) (uint32) ( x)
#endif

#ifdef WINNT
#define  htons_86_Linux(x) (uint16) ( x)
#define  htonl_86_Linux(x) (uint32) ( x)
#else
#define  htons_86_Linux(x) (uint16) ( ((((uint16)(x)) & 0x00ff) << 8 ) |((((uint16)(x)) & 0xff00) >> 8 ))
#define  htonl_86_Linux(x) (uint32) ( (( ((uint32)(x)) & 0xff000000 ) >> 24) | (( ((uint32)(x)) & 0x00ff0000 ) >> 8 ) | \
						(( ((uint32)(x)) & 0x0000ff00 ) << 8 ) | (( ((uint32)(x)) & 0x000000ff ) << 24))
#endif

#define		SWITCH_ON						0x55
#define		PERIOD_TIME_10MS				 10					//10������
#define		MAX_UDP_RETORT_NUMBER			128					//UDP�ı��ĳ���

//UDP�Ķ˿ں�
#define	MONITOR_UDP_REC_PORT		8801				//���ն˿�
#define	MONITOR_UDP_SEND_PORT		8802				//���Ͷ˿�


//������UDP�˿ں�
#define TEST_UDP_SEND_PORT			8080				//���Ͷ˿�

//Ŀ�ĵ�ַ��Դ��ַ
#define	MONITOR_ADDR				0xfe				//PC��ַ
#define	USV_ADDR					0x01				//����ͧ��ַ

#define	USV_ARM_MODEL_ADDR			0x00				//ARM��ģ���ַ

#define	SHORT_LIFE_TIME				100					//���ķ�������ʱ��

//����
//��������
#define	DOWN_TPYE_ASK_PC_2_ARM_REPORT			0x31				//PC�ٻ�ARM����
#define	UP_TYPE_ARM_2_PC_REPORT					0xe3				//ARM����PC����

//����������
#define	DOWN_SUB_TEST					0x11					//���Խ���
#define	DOWN_SUB_CURVE					0x21					//����
#define	DOWN_SUB_MODEL_REPORT			0x31					//����ģ�鱨�ļ���
#define	DOWN_SUB_ASK_SETTING_PARAM		0x41					//�ٻ�����
#define	DOWN_SUB_SETTING_PARAM			0x42					//���������趨
#define	DOWN_SUB_SHIP_MODEL				0x51					//����ģ���趨
#define	DOWN_SUB_SIMULATE_ASK			0x61					//���˴�����(�ٻ���
#define	DOWN_SUB_SIMULATE_SET			0x62					//���˴�����(���أ�
#define	DOWN_SUB_VER					0x71					//�汾��ȡ

//������
//���Խ���
#define	DOWN_FUNC_COMM_MONITOR			0x10					//ͨѶ����
#define	DOWN_FUNC_ALARM					0x11					//�澯״̬
#define	DOWN_FUNC_TEST					0x12					//������Ϣ
#define	DOWN_FUNC_CFG_ERR_INF			0x13					//�����ļ�������Ϣ
#define	DOWN_FUNC_MAIN_PARAM			0x14					//��Ҫ���в���
#define	DOWN_FUNC_OTHER_PARAM			0x15					//�������в���
#define	DOWN_FUNC_RUN_STATE				0x16					//�豸����״̬
//����
#define	DOWN_FUNC_CURVE_REAL_DATA		0x20					//����ʵʱ����

//��ģ�鱨��
#define	DOWN_FUNC_STABLE_PLAT			0x30					//�ȶ�ƽ̨
#define	DOWN_FUNC_SMART_RUDDER			0x32					//���ܶ�
#define	DOWN_FUNC_MPC					0x33					//MPC
#define	DOWN_FUNC_ENERGY_MANAGE			0x34					//��Դ����
#define	DOWN_FUNC_AUXILIARY_POWER		0x35					//�����豸��Դ
#define	DOWN_FUNC_LEFT_ENGINE			0x36					//�󷢶���
#define	DOWN_FUNC_RIGHT_ENGINE			0x3E					//�ҷ�����
#define DOWN_FUNC_BRA                   0x3F                    //�����̨

#define MODEL_MAX_NUM					22						//��ģ��¼��ͨ����

//�����趨
#define	DOWN_FUNC_PID_PARAM				0x40					//PID�����趨
#define	DOWN_FUNC_S_FACE_PARAM			0x41					//S������趨
#define	DOWN_FUNC_SHIP_ROUTE			0x42					//�����趨
#define	DOWN_FUNC_OBSTACLE_LOCATE		0x43					//�ϰ����趨
#define	DOWN_FUNC_LAKE_LARGE_INF		0x44					//�����С��Ϣ
#define	DOWN_FUNC_TRANS_PARAM			0x45					//ƽ�Ʋ����趨
#define	DOWN_FUNC_ROTATE_PARAM			0x46					//��ת�����趨
#define	DOWN_FUNC_OPERATE_OK			0xf0					//������ȷ����
#define	DOWN_FUNC_OPERATE_ERR			0xf1					//���س�����

//����ģ�ͼ��� 
#define	DOWN_FUNC_SIM_SHIP_START_STATE	0x50					//���ĳ�ʼ״̬
#define	DOWN_FUNC_SIM_ROUTE				0x51					//��������
#define	DOWN_FUNC_SIM_OBSTACLE			0x52					//�ϰ�������
#define	DOWN_FUNC_SIM_INPUT_REAL_DATA	0x53					//ʵʱ������

//����ͧ����
#define	DOWN_FUNC_MMI					0x60					//����������


typedef struct{
	uint8	locate_usv_addr;							//�������˴��ĵ�ַ
	uint8	monitor_udp_rec_flag;
	sockaddr_in from_udp_ip;
}MONITOR_UDP_INF_STRUCT;

//UDP���߱��Ľṹ
typedef struct{
	uint8	apdu_len;			//APDU����
	uint8	dest_addr;			//Ŀ�ĵ�ַ
	uint8	src_addr;			//Դ��ַ
	uint8	report_type;		//��������
	uint8	report_len;			//���ĳ���
	uint8	sub_type;			//������
	uint8	socket_ID_h;		//֡��Ÿ�λ
	uint8	socket_ID_l;		//֡��ŵ�λ
	uint8	func;				//������
	uint8	model_addr;			//ģ���ַ
	uint8	buff[96];			//����������
}UDP_MONITOR_REPORT_STRUCT;

//��鷢�ͱ���
typedef struct {
	uint32	send_report_flag;		//���ͱ��ı�־��0xaaaa--���ڷ��ͣ�	
	uint32	sendlife_time;			//���ͱ�������ʱ��
	uint32	send_report_type;		//���ͱ��ı�������
	uint32	send_data_type;			//��������
	uint32	send_func;				//������
	uint32	param1;					//����1
	uint32	param2;					//����2
	uint32	param3;					//����3
	uint32	param4;					//����4
	uint32	send_report_seq;		//���ͱ������,��0��ʼ
}MONITOR_UDP_CHECK_SEND_REPORT_STRUCT;

//������Ӧ����
typedef struct{
	uint16			send_flag;				//���ͱ�־
	uint8			error_flag;				//�����־ SWITCH_ON--��ȷ��0--����
	uint8			report_type;			//��������
	uint8			sub_type;				//������
	uint8			func;					//������
	uint8			buff_len;				//������ʵ�ʳ���	
	uint8			buff[50];				//������Ϣ
}MONITOR_ASK_OPERATION_STRUCT;


extern MONITOR_UDP_CHECK_SEND_REPORT_STRUCT monitor_udp_check_send_report;
extern MONITOR_ASK_OPERATION_STRUCT	monitor_ask_operation;
extern uint8 msg_record_cmd[MODEL_MAX_NUM];

uint8	sys_CalcCheckSum(const uint8 *dest, uint16 u16len);
void *usv_monitor_thread(void *aa);
void clear_monitor_udp_life_time(void);
void	input_monitor_udp_life_time(uint16 life_time,uint8 report_type,uint8 data_type,uint8 func_type);
short int sprintf_usv(char *buffer ,const char *format, ...);
short int sscanf_usv(const char * str, const char * format, ...);
void memcpy_32(uint32* dest_buff,uint32* src_buff,uint8 report_len);
void memcpy_16(uint16* dest_buff, uint16* src_buff,uint8 report_len);
uint16	input_apdu_report(UDP_MONITOR_REPORT_STRUCT *p_udp_monitor_report,uint8 report_len);

extern int16 CreateUDPSocket(int16 iPort);
extern int8 monitor_udp_send_report( uint8 * lpBuf, int16 iLen,int16 sockid);
#endif /* CAN_API_H */
