/*
* can_api.h --CAN��ʼ��
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#ifndef		INPUT_MONITOR_DATA_H
#define     INPUT_MONITOR_DATA_H

#define		MONITOR_UART_COMM_NUMBER				6			//����ͨѶ����
#define		MONITOR_CAN_COMM_NUMBER					7			//CANͨѶ����
#define		MONITOR_NET_COMM_NUMBER					2			//��̫ͨѶ����
#define		MONITOR_BAK_COMM_NUMBER					5			//����ͨѶ����
#define		ALL_MONITOR_COMM_NUMBER		MONITOR_UART_COMM_NUMBER+MONITOR_CAN_COMM_NUMBER+MONITOR_NET_COMM_NUMBER+MONITOR_BAK_COMM_NUMBER

//#define		MONITOR_COMM_MAIN_DSP_SN		0	//����DSP
//#define		MONITOR_COMM_DRADIO_SN			1	//���ֵ�̨
//#define		MONITOR_COMM_BD_SN				2	//����
//#define		MONITOR_COMM_SRADIO_SN			3	//�����̨
//#define		MONITOR_COMM_INS_SN				4	//�ߵ�
//
//#define		MONITOR_COMM_ST_PL_SN			5	//�ȶ�ƽ̨
//#define		MONITOR_COMM_SR_SN				6	//��DSP
//#define		MONITOR_COMM_SP_SN				7	//�������
//#define		MONITOR_COMM_BAT_CON_SN			8	//��Դ����
//#define		MONITOR_COMM_POW_SN			9	//��Դ����
//#define		MONITOR_COMM_MOTOR_L_SN		10	//�󷢶���
//#define		MONITOR_COMM_MOTOR_R_SN		11	//�ҷ�����
//#define		MONITOR_COMM_UAV_SN			12
//
//#define		MONITOR_COMM_BRADIO_SN			13	//�����̨
//#define		MONITOR_COMM_MPC_SN				14	//MPC


#define		MONITOR_COMM_DRIOP_SN		0	//ң����崮��ͨ��
#define		MONITOR_COMM_BRIOP_SN		1	//ң���������ͨ��
#define		MONITOR_COMM_BTASK_SN		2	//��Я��̨��������ͨ��
#define		MONITOR_COMM_CCCMD_SN		3	//���غ�̨����ͨ��
#define		MONITOR_COMM_CCTASK_SN		4	//���غ�̨����ͨ��
#define		MONITOR_COMM_CCWARN_SN		5	//���غ�̨�澯ͨ��
#define		MONITOR_COMM_IHC_SN			6	//���ܶ�
#define		MONITOR_COMM_IOP_SN			7	//�������
#define		MONITOR_COMM_IDU_SN			8	//��Դ��
#define		MONITOR_COMM_INS_SN			9	//�ߵ�
#define		MONITOR_COMM_BD_SN			10	//����ͨѶ
#define		MONITOR_COMM_MTRADAR_SN		11	//�����״�ͨѶ
#define		MONITOR_COMM_MWRADAR_SN		12	//���ײ��״�ͨѶ

#define		MONITOR_COMM_DOCK_SN		13	//��Դ��

#define		MONITOR_REAL_DATA_NUMBER				12			//��Ҫ��������
#define		MONITOR_SUB_REAL_DATA_NUMBER			50			//��Ҫ��������
#define		MONITOR_USV_EQU_STATE_NUMBER			200			//�豸����״̬����
#define		MONITOR_USV_ALM_NUMBER				8			//�澯״̬
#define		MONITOR_TMP_DATA_NUMBER				50			//������Ϣ
#define		MONITOR_MAX_CFG_ERROR_ITEM			0x0f		//�����ļ�������Ϣ
#define		MONITOR_STOP_MAX_NUMBER				15			//ͣ�������
#define		MONITOR_OBSTACLE_MAX_NUMBER			10			//�ϰ������

#define		MONITOR_DSP_MSG				0		//DSP��������	���ڱ���
#define		MONITOR_AIS_MSG				1		//AIS��			���ڱ���
#define		MONITOR_DRADIO_MSG		2		//���ֵ�̨		���ڱ���
#define		MONITOR_BD_MSG				3		//��������		���ڱ���
#define		MONITOR_SPARE_DRADIO_MSG	4		//�������ֵ�̨	���ڱ���
#define		MONITOR_INS_MSG				5		//�ߵ�����		���ڱ���

#define		MONITOR_UAV_MSG				6		//���˻�ƽ̨����
#define		MONITOR_SR_MSG				7		//���ܶ�DSP����		CAN������
#define		MONITOR_POW_MSG				8		//��Դ����ϵͳ����	CAN������
#define		MONITOR_SP_MSG				9		//������屨��			CAN������
#define		MONITOR_ST_PL_MSG			10		//�ȶ�ƽ̨����			CAN������
#define		MONITOR_BAT_MSG				11		//��Դ����ϵͳ����	CAN������
#define		MONITOR_MOTOR_L_MSG			12		//�󷢶�������			CAN������
#define		MONITOR_MOTOR_R_MSG			13		//�ҷ���������			CAN������

#define		MONITOR_MPC_MSG				14		//MPC���ͱ���			LAN0��̫������
#define		MONITOR_BRADIO_MSG			15		//�����̨����			LAN1��̫������

#define		MAX_LAKE_POINT_NUMBER				12			//����������
#define		MAX_PID_GROUP_NUMBER						16			//PID�������
#define		MAX_TRANSLATION_MOTION_NUMBER	4			//ƽ�Ʋ����������
#define		MAX_ROTATE_MOTION_NUMBER		4			//��ת�����������

#define		CAN_PART0						0
#define		CAN_PART1						8
#define		CAN_PART2						16
#define		CAN_PART3						24
#define		CAN_PART4						32
#define		CAN_PART5						40
#define		CAN_PART6						48
#define		CAN_PART7						56
#define		CAN_PART8						64
#define		CAN_PART9						72
#define		CAN_PART10						80
#define		CAN_PART11						88
#define		CAN_PART12						96
#define		CAN_PART13						104

#define	 FLOAT_TYPE				1
#define	 INT_TYPE				2

//ͨѶͳ����Ϣ
typedef struct {
	uint32	send_ok_number;						//������ȷ֡��
	uint32	send_error_number;					//���ͳ���֡��
	uint32	rec_ok_number;						//������ȷ֡��
	uint32	rec_error_number;					//���ճ���֡��
	uint16	send_err_ID;						//���ͳ������
	uint16	rec_err_ID;							//���ճ������							
}MONITOR_COMM_INF_STRUCT;

////����,����Ϊ��λ������100����ȡ���õ�32λ�з�������
//typedef struct {
//	int32	lng;			//����	
//	int32	lat;			//γ��	
//}COORDINATE_STRUCT;

//����ֵ�͵�ǰֵ
typedef struct {
	float	expect_data;					//����ֵ
	float	output_data;					//�������
}MONITOR_MAIN_TWO_REAL_DATA_STRUCT;

//����ֵ ����͵�ǰֵ
typedef struct {
	//float	expect_data;					//����ֵ
	float	output_data;					//�������
	float	real_data;						//ʵ���������
}MONITOR_MAIN_THREE_REAL_DATA_STRUCT;

//��Ҫ���Ӳ���
typedef struct {
	float	expect_data;					//����ֵ
	float	output_data;					//�������
	float	real_data;						//ʵ���������
	float	delta_data;						//��ֵ
}MONITOR_MAIN_REAL_DATA_STRUCT;

//������ײ��Ϣ
typedef struct{
	uint8	sail_mode;			//����״̬ 0���������� 1��apf���Ϻ���
	uint8	collison_num;		//�ϰ�����
	uint8	collision_bak1;		//����1
	uint8	collision_bak2;		//����2
	float	dcpa;				//�ϰ���DCPAֵ
	float	tcpa;				//�ϰ���TCPA
}COLLISION_DATA_STRUCT;

//����ʵʱ���ݡ�һ��25��float
typedef struct {
	COORDINATE_STRUCT						current_locate;				//��ǰ����
	MONITOR_MAIN_TWO_REAL_DATA_STRUCT		ship_speed;					//����
	MONITOR_MAIN_TWO_REAL_DATA_STRUCT		ship_direction;				//����
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_Accelerator_speed;			//�󷢶���
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_Accelerator_speed;			//�ҷ�����
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_rudder_speed;			//���
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_rudder_speed;			//�Ҷ�
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_gear;					//��λ
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_gear;					//�ҵ�λ
	COORDINATE_STRUCT						exp_destination;			//��������
	float									track_error;				//����ƫ�� 
	COLLISION_DATA_STRUCT					collision_data	;			//������ײ��Ϣ
	int32									bak				;			//����
}MONITOR_CURVE_REAL_DATA_STRUCT;

//�����ļ�������Ϣ
//CFG��ini�ļ��������ϸ��Ϣ
typedef struct  
{
	uint8	string_title[32];			//ģ������
	uint8	string_line[50];			//��������
	uint8	param1;						//�к�
	uint8	bak;
}EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT;


typedef struct  
{	
	uint8									 cfg_alm_item_number;					//���������
	EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT			 monitor_cfg_alm_describe[MONITOR_MAX_CFG_ERROR_ITEM+1];		//����
}MONITOR_CFG_ALM_DESCRIBE_STRUCT;	

//�����趨
//PID�趨
typedef struct {
	float		P;
	float		I;
	float		D;
}EACH_MONITOR_SET_PID_PARAM_STRUCT;


//ƽ�Ʋ���
typedef struct{
	int32	rudder_L		;
	int32	rudder_R		;
	int32	gear_L			;
	int32	gear_R			;
	int32	accelerator_L	;
	int32	accelerator_R	;
}EACH_MONITOR_SET_TRANS_PARAM_STRUCT;

//��ת����
typedef struct{
	int32	rudder_L		;
	int32	rudder_R		;
	int32	gear_L			;
	int32	gear_R			;
	int32	accelerator_L	;
	int32	accelerator_R	;
}EACH_MONITOR_SET_ROTATE_PARAM_STRUCT;

//S�����
typedef struct {
	float		k1;
	float		k2;
	float		k3;
}EACH_MONITOR_SET_S_FACE_PARAM_STRUCT;


//����
typedef struct {	
	COORDINATE_STRUCT	stop_locate;	//����
	uint32				bak_32_1;
	uint32				bak_32_2;
	uint16				stop_time;		//ͣ��ʱ��	
	uint16				bak_16_1;		//����	�����ٶ�*100
}SHIP_STOP_INF_STRUCT;

////���ϰ������Ϣ
//typedef struct {	
//	COORDINATE_STRUCT	obstacle_locate;	//����
//	float				obstacle_speed;				//�ٶ�
//	float				obstacle_direction;
//	float				obstacle_radius;		//��ȫ�뾶
//}OBSTACLE_LOCATE_INF_STRUCT;


typedef struct {
	uint32									lake_point_number;					//�����ĸ���
	uint32									ship_stop_number;					//ʵ�ʺ������
	uint32									obstacle_number;					//ʵ���ϰ������
	uint32									PID_number;						//PID����
	uint32									s_face_number;						//S�����
	uint32									Translation_Motion_Setting_Number;	//ƽ�Ʋ�������
	uint32									Rotation_Motion_Setting_Number;			//��ת��������
	uint32									bak1;
	uint32									bak2;
	uint32									bak3;
	COORDINATE_STRUCT						lake_point[MAX_LAKE_POINT_NUMBER];				//����ĵ���Ϣ
	EACH_MONITOR_SET_PID_PARAM_STRUCT		monitor_set_pid_param[MAX_PID_GROUP_NUMBER];			//PID�����趨
	EACH_MONITOR_SET_S_FACE_PARAM_STRUCT	monitor_set_s_face_param[MAX_PID_GROUP_NUMBER];		//S�����
	EACH_MONITOR_SET_TRANS_PARAM_STRUCT		monitor_set_trans_param[MAX_TRANSLATION_MOTION_NUMBER]; //ƽ�Ʋ���
	EACH_MONITOR_SET_ROTATE_PARAM_STRUCT	monitor_set_rotate_param[MAX_ROTATE_MOTION_NUMBER];		//��ת����
	SHIP_STOP_INF_STRUCT					ship_stop_inf[MONITOR_STOP_MAX_NUMBER];			//��������Ϣ
	OBSTACLE_LOCATE_INF_STRUCT				obstacle_locate_inf[MONITOR_OBSTACLE_MAX_NUMBER];		//�ϰ�����Ϣ
}REC_MONITOR_ALL_SET_PARAM_STRUCT;


//��ģ������״̬����
typedef struct {
	uint8		report_detail[400];				//�������ݣ�ÿ֡����100��BYTE��3֡
}MONITOR_MODULE_REPORT_INF_STRUCT;

typedef struct {
	uint8	rec_mmi_buff[8];
}EACH_MMI_CAN_STRUCT;

//PC�������
typedef struct {
	uint8				 mmi_state;								//�˻��ӿ� 0xaa--���PC��11H���������22H---����
	EACH_MMI_CAN_STRUCT	 mmi_can_report[4];					//ARM==>PC��巢�ͱ��ģ��������USV��Լ P25��CAN��4֡����
}REC_PC_SIMULATE_MMI_STRUCT;

//��������Ϣ
typedef struct {
	MONITOR_COMM_INF_STRUCT				monitor_comm_inf[ALL_MONITOR_COMM_NUMBER];				//ͨѶ��Ϣ
	MONITOR_MAIN_REAL_DATA_STRUCT		monitor_main_real_data[MONITOR_REAL_DATA_NUMBER];		//��Ҫ���Ӳ���
	MONITOR_CURVE_REAL_DATA_STRUCT		monitor_curve_real_data;								//����ʵʱ����
	float								monitor_sub_real_data[MONITOR_SUB_REAL_DATA_NUMBER];	//��Ҫ���Ӳ���
	uint8								usv_equ_state[MONITOR_USV_EQU_STATE_NUMBER];			//���˴�����״̬
	uint32								alm_state[MONITOR_USV_ALM_NUMBER];						//�澯״̬
	uint32								alm_state_old[MONITOR_USV_ALM_NUMBER];					//�澯��״̬
	uint16								alm_state_number[32*MONITOR_USV_ALM_NUMBER];			//�澯����
	uint32								tmp_data[MONITOR_TMP_DATA_NUMBER];						//������Ϣ
	MONITOR_CFG_ALM_DESCRIBE_STRUCT			monitor_cfg_alm_describe;									//���ó�����Ϣ	
	MONITOR_MODULE_REPORT_INF_STRUCT		module_report_inf[ALL_MONITOR_COMM_NUMBER];								//��ģ�鱨��
	REC_MONITOR_ALL_SET_PARAM_STRUCT	rec_monitor_all_set_param;								//PID��������
	REC_PC_SIMULATE_MMI_STRUCT			rec_pc_simulate_mmi;										//PC�������
}MONITOR_ALL_INF_STRUCT;

// #############################################################################################
//				���洬�Ľṹ
// #############################################################################################
//���ĳ�ʼ����Ϣ
typedef struct {
	uint8			Intellingent_Switch;	//�����𺽱�־ 0�����٣�1���𺽣�2--����������ͣ
	uint8			Intellingent_Mod;	   //����ģʽ��0--�ֶ�ģʽ��--���Զ�2--ȫ�Զ���--����ģʽ�����٣�
	uint8			auto_return_Mod;			//1������
	uint8			bak;						//����״̬
	COORDINATE_STRUCT	ship_locate;		//���ĵ�ǰλ��
	float			speed;				//��ǰ�ٶ�
	float			direction;			//��ǰ����
	float			bak1;				//����
	float			gear_l;						//��λ
	float			gear_r;						//�ҵ�λ				
	float			Accelerator_L;				//�󷢶���ת��
	float			Accelerator_R;				//�ҷ�����ת��
	float			Rudder_L;					//����
	float			Rudder_R;					//�Ҷ��
	float			bak2;
	float			bak3;
	float			bak4;
	float			bak5;
} SHIP_INIT_STATE_STRUCT;


//������Ϣ
typedef struct {
	uint8		stop_number;						//�������
	uint8		obstacle_number;					//�ϰ���				
	uint8		bak1;								//����״̬
	uint8		bak2;
	float		bak_float[4];						//���ò���
	SHIP_INIT_STATE_STRUCT						ship_init_state;				//���ĳ�ʼ״̬									
	SHIP_STOP_INF_STRUCT						ship_stop_inf[MONITOR_STOP_MAX_NUMBER];					//��������Ϣ
	OBSTACLE_LOCATE_INF_STRUCT					obstacle_locate_inf[MONITOR_OBSTACLE_MAX_NUMBER];		//�ϰ������Ϣ
}MONITOR_SHIP_RUN_INF_STRUCT;

//�������������
typedef struct {
	uint8		Intellingent_Switch;	//�����𺽱�־ 0�����٣�1���𺽣�2--����������ͣ
	uint8		Intellingent_Mod;	   //����ģʽ��0--�ֶ�ģʽ��1--���Զ�2--ȫ�Զ���3--����ģʽ�����٣�
	uint8		auto_return_Mod;			//1������
	uint8		bak;						//����״̬

	float		Accelerator_L;				//�����ſ���
	float		Accelerator_R;				//�����ſ���
	float		Rudder_L;					//����
	float		adj_Rudder_L;				//���ǵ���ֵ
	float		Rudder_R;					//�Ҷ��
	float		adj_Rudder_R;				//�Ҷ�ǵ���ֵ
	float		Gear_L;						//��λ
	float		Gear_R;						//�ҵ�λ
	float		Speed_EXP;					//�ٶ�����ֵ
	float		current_speed;				//��ǰ����
	float		Heading_EXP;				//��������ֵ
	float		current_Head;				//��ǰ����
	COORDINATE_STRUCT	ship_locate;		//���ĵ�ǰλ��
	float		bak_float[5];						//����
}SHIP_ADJ_OUTPUT_PARAM_STRUCT;







//�������������
typedef struct {
	MONITOR_SHIP_RUN_INF_STRUCT			monitor_ship_run_inf;						//���ķ�����Ҫ������Ϣ
	SHIP_ADJ_OUTPUT_PARAM_STRUCT		ship_adj_output_param;						//���洬������������������洬�������ź�
}SHIP_SIMULATION_PARAM_STRUCT;


extern MONITOR_ALL_INF_STRUCT monitor_all_inf;				//����ͨѶ��ȫ����Ϣ
extern SHIP_SIMULATION_PARAM_STRUCT	ship_simulate_param;

uint16 send_monitor_ask(uint8 *report_buff);
uint16	monitor_send_input_comm(uint8 *report_buff);
uint16	monitor_send_input_alarm(uint8 *report_buff);
uint16	monitor_send_input_test_inf(uint8 *report_buff);
uint16	monitor_send_input_cfg_error_inf(uint8 *report_buff);
uint16	monitor_send_input_main_run_real_data(uint8 *report_buff);
uint16	monitor_send_input_other_run_real_data(uint8 *report_buff);
uint16	monitor_send_input_run_state(uint8 *report_buff);
uint16	monitor_send_input_curve_real_data(uint8 *report_buff);
uint16	monitor_send_input_sub_equ_report(uint8 *report_buff,uint8 sub_equ_ID);
void	monitor_rec_deal_PID_param(uint8 *report_buff);
void	monitor_rec_deal_S_face_param(uint8 *report_buff);
void	monitor_rec_deal_trans_param(uint8 *report_buff);
void	monitor_rec_deal_rotate_param(uint8 *report_buff);
void	monitor_rec_deal_ship_route(uint8 *report_buff);
void	monitor_rec_deal_obstacle(uint8 *report_buff);
void	monitor_rec_deal_mmi_code(uint8 *report_buff);
void input_ask_report(uint8 *content,uint8 error_flag,uint8 report_type,uint8 sub_type,uint8 func);
int8 write_setting_file(void);
int8 read_setting_file(void);
uint16	monitor_send_input_PID_param(uint8 *report_buff);
uint16	monitor_send_input_S_face_param(uint8 *report_buff);
uint16	monitor_send_input_trans_param(uint8 *report_buff);
uint16	monitor_send_input_rotate_param(uint8 *report_buff);
uint16	monitor_send_input_ship_route(uint8 *report_buff);
uint16	monitor_send_input_obstacle(uint8 *report_buff);
uint16	monitor_send_input_lake_large_inf(uint8 *report_buff);
void init_flash_setting_default(void);
uint16	monitor_send_input_MMI_state(uint8 *report_buff);
uint16	monitor_send_input_ship_start_state(uint8 *report_buff);
uint16	monitor_send_input_simulate_route(uint8 *report_buff);
uint16	monitor_send_input_simulate_obstacle(uint8 *report_buff);
uint16	monitor_send_input_real_data(uint8 *report_buff);

void	monitor_rec_deal_record_cmd(uint8 func);

void test_monitor(void);
void monitor_data_fresh(void);
int8 read_sub_setting(int8* main_item,int8* sub_item,int8 id, uint32 *out_data,uint8 flag);
int8 read_sub_setting_df(int8* main_item, int8* sub_item, int8 id, uint64 *out_data, uint8 flag);
int8 read_sub_setting_string(int8* main_item,int8*sub_item,int8 id, char *out_data);
#endif 
