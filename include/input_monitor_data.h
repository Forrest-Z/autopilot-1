/*
* can_api.h --CAN初始化
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#ifndef		INPUT_MONITOR_DATA_H
#define     INPUT_MONITOR_DATA_H

#define		MONITOR_UART_COMM_NUMBER				6			//串口通讯个数
#define		MONITOR_CAN_COMM_NUMBER					7			//CAN通讯个数
#define		MONITOR_NET_COMM_NUMBER					2			//以太通讯个数
#define		MONITOR_BAK_COMM_NUMBER					5			//备用通讯个数
#define		ALL_MONITOR_COMM_NUMBER		MONITOR_UART_COMM_NUMBER+MONITOR_CAN_COMM_NUMBER+MONITOR_NET_COMM_NUMBER+MONITOR_BAK_COMM_NUMBER

//#define		MONITOR_COMM_MAIN_DSP_SN		0	//主控DSP
//#define		MONITOR_COMM_DRADIO_SN			1	//数字电台
//#define		MONITOR_COMM_BD_SN				2	//北斗
//#define		MONITOR_COMM_SRADIO_SN			3	//冗余电台
//#define		MONITOR_COMM_INS_SN				4	//惯导
//
//#define		MONITOR_COMM_ST_PL_SN			5	//稳定平台
//#define		MONITOR_COMM_SR_SN				6	//舵DSP
//#define		MONITOR_COMM_SP_SN				7	//智能面板
//#define		MONITOR_COMM_BAT_CON_SN			8	//能源管理
//#define		MONITOR_COMM_POW_SN			9	//电源管理
//#define		MONITOR_COMM_MOTOR_L_SN		10	//左发动机
//#define		MONITOR_COMM_MOTOR_R_SN		11	//右发动机
//#define		MONITOR_COMM_UAV_SN			12
//
//#define		MONITOR_COMM_BRADIO_SN			13	//宽带电台
//#define		MONITOR_COMM_MPC_SN				14	//MPC


#define		MONITOR_COMM_DRIOP_SN		0	//遥控面板串口通道
#define		MONITOR_COMM_BRIOP_SN		1	//遥控面板网络通道
#define		MONITOR_COMM_BTASK_SN		2	//便携后台任务网络通道
#define		MONITOR_COMM_CCCMD_SN		3	//集控后台控制通道
#define		MONITOR_COMM_CCTASK_SN		4	//集控后台任务通道
#define		MONITOR_COMM_CCWARN_SN		5	//集控后台告警通道
#define		MONITOR_COMM_IHC_SN			6	//智能舵
#define		MONITOR_COMM_IOP_SN			7	//智能面板
#define		MONITOR_COMM_IDU_SN			8	//能源箱
#define		MONITOR_COMM_INS_SN			9	//惯导
#define		MONITOR_COMM_BD_SN			10	//北斗通讯
#define		MONITOR_COMM_MTRADAR_SN		11	//海事雷达通讯
#define		MONITOR_COMM_MWRADAR_SN		12	//毫米波雷达通讯

#define		MONITOR_COMM_DOCK_SN		13	//能源箱

#define		MONITOR_REAL_DATA_NUMBER				12			//主要参数个数
#define		MONITOR_SUB_REAL_DATA_NUMBER			50			//次要参数个数
#define		MONITOR_USV_EQU_STATE_NUMBER			200			//设备运行状态个数
#define		MONITOR_USV_ALM_NUMBER				8			//告警状态
#define		MONITOR_TMP_DATA_NUMBER				50			//调试信息
#define		MONITOR_MAX_CFG_ERROR_ITEM			0x0f		//配置文件出错信息
#define		MONITOR_STOP_MAX_NUMBER				15			//停靠点个数
#define		MONITOR_OBSTACLE_MAX_NUMBER			10			//障碍点个数

#define		MONITOR_DSP_MSG				0		//DSP心跳报文	串口报文
#define		MONITOR_AIS_MSG				1		//AIS　			串口报文
#define		MONITOR_DRADIO_MSG		2		//数字电台		串口报文
#define		MONITOR_BD_MSG				3		//北斗报文		串口报文
#define		MONITOR_SPARE_DRADIO_MSG	4		//冗余数字电台	串口报文
#define		MONITOR_INS_MSG				5		//惯导报文		串口报文

#define		MONITOR_UAV_MSG				6		//无人机平台报文
#define		MONITOR_SR_MSG				7		//智能舵DSP报文		CAN网报文
#define		MONITOR_POW_MSG				8		//电源管理系统报文	CAN网报文
#define		MONITOR_SP_MSG				9		//智能面板报文			CAN网报文
#define		MONITOR_ST_PL_MSG			10		//稳定平台报文			CAN网报文
#define		MONITOR_BAT_MSG				11		//能源管理系统报文	CAN网报文
#define		MONITOR_MOTOR_L_MSG			12		//左发动机报文			CAN网报文
#define		MONITOR_MOTOR_R_MSG			13		//右发动机报文			CAN网报文

#define		MONITOR_MPC_MSG				14		//MPC发送报文			LAN0以太网报文
#define		MONITOR_BRADIO_MSG			15		//宽带电台报文			LAN1以太网报文

#define		MAX_LAKE_POINT_NUMBER				12			//湖面最大点数
#define		MAX_PID_GROUP_NUMBER						16			//PID最大组数
#define		MAX_TRANSLATION_MOTION_NUMBER	4			//平移参数最大组数
#define		MAX_ROTATE_MOTION_NUMBER		4			//旋转参数最大组数

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

//通讯统计信息
typedef struct {
	uint32	send_ok_number;						//发送正确帧数
	uint32	send_error_number;					//发送出错帧数
	uint32	rec_ok_number;						//接收正确帧数
	uint32	rec_error_number;					//接收出错帧数
	uint16	send_err_ID;						//发送出错代号
	uint16	rec_err_ID;							//接收出错代号							
}MONITOR_COMM_INF_STRUCT;

////坐标,以秒为单位，扩大100倍后，取整得到32位有符号整数
//typedef struct {
//	int32	lng;			//经度	
//	int32	lat;			//纬度	
//}COORDINATE_STRUCT;

//期望值和当前值
typedef struct {
	float	expect_data;					//期望值
	float	output_data;					//输出数据
}MONITOR_MAIN_TWO_REAL_DATA_STRUCT;

//期望值 输出和当前值
typedef struct {
	//float	expect_data;					//期望值
	float	output_data;					//输出数据
	float	real_data;						//实际输出数据
}MONITOR_MAIN_THREE_REAL_DATA_STRUCT;

//主要监视参数
typedef struct {
	float	expect_data;					//期望值
	float	output_data;					//输出数据
	float	real_data;						//实际输出数据
	float	delta_data;						//差值
}MONITOR_MAIN_REAL_DATA_STRUCT;

//船体碰撞信息
typedef struct{
	uint8	sail_mode;			//航行状态 0：正常航行 1：apf避障航行
	uint8	collison_num;		//障碍物编号
	uint8	collision_bak1;		//备用1
	uint8	collision_bak2;		//备用2
	float	dcpa;				//障碍物DCPA值
	float	tcpa;				//障碍物TCPA
}COLLISION_DATA_STRUCT;

//曲线实时数据　一共25个float
typedef struct {
	COORDINATE_STRUCT						current_locate;				//当前航点
	MONITOR_MAIN_TWO_REAL_DATA_STRUCT		ship_speed;					//航速
	MONITOR_MAIN_TWO_REAL_DATA_STRUCT		ship_direction;				//航向
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_Accelerator_speed;			//左发动机
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_Accelerator_speed;			//右发动机
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_rudder_speed;			//左舵
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_rudder_speed;			//右舵
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		left_gear;					//左档位
	MONITOR_MAIN_THREE_REAL_DATA_STRUCT		right_gear;					//右档位
	COORDINATE_STRUCT						exp_destination;			//期望航点
	float									track_error;				//航迹偏差 
	COLLISION_DATA_STRUCT					collision_data	;			//航行碰撞信息
	int32									bak				;			//备用
}MONITOR_CURVE_REAL_DATA_STRUCT;

//配置文件出错信息
//CFG和ini文件出错的详细信息
typedef struct  
{
	uint8	string_title[32];			//模块名称
	uint8	string_line[50];			//配置名称
	uint8	param1;						//列号
	uint8	bak;
}EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT;


typedef struct  
{	
	uint8									 cfg_alm_item_number;					//填入的项数
	EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT			 monitor_cfg_alm_describe[MONITOR_MAX_CFG_ERROR_ITEM+1];		//内容
}MONITOR_CFG_ALM_DESCRIBE_STRUCT;	

//参数设定
//PID设定
typedef struct {
	float		P;
	float		I;
	float		D;
}EACH_MONITOR_SET_PID_PARAM_STRUCT;


//平移参数
typedef struct{
	int32	rudder_L		;
	int32	rudder_R		;
	int32	gear_L			;
	int32	gear_R			;
	int32	accelerator_L	;
	int32	accelerator_R	;
}EACH_MONITOR_SET_TRANS_PARAM_STRUCT;

//旋转参数
typedef struct{
	int32	rudder_L		;
	int32	rudder_R		;
	int32	gear_L			;
	int32	gear_R			;
	int32	accelerator_L	;
	int32	accelerator_R	;
}EACH_MONITOR_SET_ROTATE_PARAM_STRUCT;

//S面参数
typedef struct {
	float		k1;
	float		k2;
	float		k3;
}EACH_MONITOR_SET_S_FACE_PARAM_STRUCT;


//航点
typedef struct {	
	COORDINATE_STRUCT	stop_locate;	//航点
	uint32				bak_32_1;
	uint32				bak_32_2;
	uint16				stop_time;		//停靠时间	
	uint16				bak_16_1;		//备用	航行速度*100
}SHIP_STOP_INF_STRUCT;

////各障碍点的信息
//typedef struct {	
//	COORDINATE_STRUCT	obstacle_locate;	//航点
//	float				obstacle_speed;				//速度
//	float				obstacle_direction;
//	float				obstacle_radius;		//安全半径
//}OBSTACLE_LOCATE_INF_STRUCT;


typedef struct {
	uint32									lake_point_number;					//湖面点的个数
	uint32									ship_stop_number;					//实际航点个数
	uint32									obstacle_number;					//实际障碍点个数
	uint32									PID_number;						//PID个数
	uint32									s_face_number;						//S面个数
	uint32									Translation_Motion_Setting_Number;	//平移参数个数
	uint32									Rotation_Motion_Setting_Number;			//旋转参数个数
	uint32									bak1;
	uint32									bak2;
	uint32									bak3;
	COORDINATE_STRUCT						lake_point[MAX_LAKE_POINT_NUMBER];				//湖面的点信息
	EACH_MONITOR_SET_PID_PARAM_STRUCT		monitor_set_pid_param[MAX_PID_GROUP_NUMBER];			//PID参数设定
	EACH_MONITOR_SET_S_FACE_PARAM_STRUCT	monitor_set_s_face_param[MAX_PID_GROUP_NUMBER];		//S面参数
	EACH_MONITOR_SET_TRANS_PARAM_STRUCT		monitor_set_trans_param[MAX_TRANSLATION_MOTION_NUMBER]; //平移参数
	EACH_MONITOR_SET_ROTATE_PARAM_STRUCT	monitor_set_rotate_param[MAX_ROTATE_MOTION_NUMBER];		//旋转参数
	SHIP_STOP_INF_STRUCT					ship_stop_inf[MONITOR_STOP_MAX_NUMBER];			//各航点信息
	OBSTACLE_LOCATE_INF_STRUCT				obstacle_locate_inf[MONITOR_OBSTACLE_MAX_NUMBER];		//障碍点信息
}REC_MONITOR_ALL_SET_PARAM_STRUCT;


//各模块运行状态报文
typedef struct {
	uint8		report_detail[400];				//报文内容，每帧考虑100个BYTE，3帧
}MONITOR_MODULE_REPORT_INF_STRUCT;

typedef struct {
	uint8	rec_mmi_buff[8];
}EACH_MMI_CAN_STRUCT;

//PC仿真面板
typedef struct {
	uint8				 mmi_state;								//人机接口 0xaa--面板PC，11H—智能面板22H---车载
	EACH_MMI_CAN_STRUCT	 mmi_can_report[4];					//ARM==>PC面板发送报文，定义参照USV规约 P25，CAN分4帧报文
}REC_PC_SIMULATE_MMI_STRUCT;

//监视总信息
typedef struct {
	MONITOR_COMM_INF_STRUCT				monitor_comm_inf[ALL_MONITOR_COMM_NUMBER];				//通讯信息
	MONITOR_MAIN_REAL_DATA_STRUCT		monitor_main_real_data[MONITOR_REAL_DATA_NUMBER];		//主要监视参数
	MONITOR_CURVE_REAL_DATA_STRUCT		monitor_curve_real_data;								//曲线实时数据
	float								monitor_sub_real_data[MONITOR_SUB_REAL_DATA_NUMBER];	//次要监视参数
	uint8								usv_equ_state[MONITOR_USV_EQU_STATE_NUMBER];			//无人船运行状态
	uint32								alm_state[MONITOR_USV_ALM_NUMBER];						//告警状态
	uint32								alm_state_old[MONITOR_USV_ALM_NUMBER];					//告警旧状态
	uint16								alm_state_number[32*MONITOR_USV_ALM_NUMBER];			//告警计数
	uint32								tmp_data[MONITOR_TMP_DATA_NUMBER];						//调试信息
	MONITOR_CFG_ALM_DESCRIBE_STRUCT			monitor_cfg_alm_describe;									//配置出错信息	
	MONITOR_MODULE_REPORT_INF_STRUCT		module_report_inf[ALL_MONITOR_COMM_NUMBER];								//各模块报文
	REC_MONITOR_ALL_SET_PARAM_STRUCT	rec_monitor_all_set_param;								//PID参数设置
	REC_PC_SIMULATE_MMI_STRUCT			rec_pc_simulate_mmi;										//PC仿真面板
}MONITOR_ALL_INF_STRUCT;

// #############################################################################################
//				仿真船的结构
// #############################################################################################
//船的初始化信息
typedef struct {
	uint8			Intellingent_Switch;	//船的起航标志 0—怠速，1—起航，2--航行任务暂停
	uint8			Intellingent_Mod;	   //智能模式，0--手动模式，--半自动2--全自动，--整定模式（怠速）
	uint8			auto_return_Mod;			//1—返航
	uint8			bak;						//备用状态
	COORDINATE_STRUCT	ship_locate;		//船的当前位置
	float			speed;				//当前速度
	float			direction;			//当前航向
	float			bak1;				//备用
	float			gear_l;						//左档位
	float			gear_r;						//右档位				
	float			Accelerator_L;				//左发动机转速
	float			Accelerator_R;				//右发动机转速
	float			Rudder_L;					//左舵角
	float			Rudder_R;					//右舵角
	float			bak2;
	float			bak3;
	float			bak4;
	float			bak5;
} SHIP_INIT_STATE_STRUCT;


//航行信息
typedef struct {
	uint8		stop_number;						//航点个数
	uint8		obstacle_number;					//障碍点				
	uint8		bak1;								//备用状态
	uint8		bak2;
	float		bak_float[4];						//备用参数
	SHIP_INIT_STATE_STRUCT						ship_init_state;				//船的初始状态									
	SHIP_STOP_INF_STRUCT						ship_stop_inf[MONITOR_STOP_MAX_NUMBER];					//各航点信息
	OBSTACLE_LOCATE_INF_STRUCT					obstacle_locate_inf[MONITOR_OBSTACLE_MAX_NUMBER];		//障碍点的信息
}MONITOR_SHIP_RUN_INF_STRUCT;

//船调节输出参数
typedef struct {
	uint8		Intellingent_Switch;	//船的起航标志 0—怠速，1—起航，2--航行任务暂停
	uint8		Intellingent_Mod;	   //智能模式，0--手动模式，1--半自动2--全自动，3--整定模式（怠速）
	uint8		auto_return_Mod;			//1—返航
	uint8		bak;						//备用状态

	float		Accelerator_L;				//左油门开度
	float		Accelerator_R;				//右油门开度
	float		Rudder_L;					//左舵角
	float		adj_Rudder_L;				//左舵角调整值
	float		Rudder_R;					//右舵角
	float		adj_Rudder_R;				//右舵角调整值
	float		Gear_L;						//左档位
	float		Gear_R;						//右档位
	float		Speed_EXP;					//速度期望值
	float		current_speed;				//当前船速
	float		Heading_EXP;				//航向期望值
	float		current_Head;				//当前航向
	COORDINATE_STRUCT	ship_locate;		//船的当前位置
	float		bak_float[5];						//备用
}SHIP_ADJ_OUTPUT_PARAM_STRUCT;







//船仿真相关数据
typedef struct {
	MONITOR_SHIP_RUN_INF_STRUCT			monitor_ship_run_inf;						//船的仿真需要运行信息
	SHIP_ADJ_OUTPUT_PARAM_STRUCT		ship_adj_output_param;						//仿真船调节输出参数，给仿真船的输入信号
}SHIP_SIMULATION_PARAM_STRUCT;


extern MONITOR_ALL_INF_STRUCT monitor_all_inf;				//与监控通讯的全部信息
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
