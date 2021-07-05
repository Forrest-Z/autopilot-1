/*
* can_api.h --CAN初始化
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
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
#define		PERIOD_TIME_10MS				 10					//10毫秒宽度
#define		MAX_UDP_RETORT_NUMBER			128					//UDP的报文长度

//UDP的端口号
#define	MONITOR_UDP_REC_PORT		8801				//接收端口
#define	MONITOR_UDP_SEND_PORT		8802				//发送端口


//调试用UDP端口号
#define TEST_UDP_SEND_PORT			8080				//发送端口

//目的地址和源地址
#define	MONITOR_ADDR				0xfe				//PC地址
#define	USV_ADDR					0x01				//无人艇地址

#define	USV_ARM_MODEL_ADDR			0x00				//ARM的模块地址

#define	SHORT_LIFE_TIME				100					//报文发送生存时间

//下行
//报文类型
#define	DOWN_TPYE_ASK_PC_2_ARM_REPORT			0x31				//PC召唤ARM报文
#define	UP_TYPE_ARM_2_PC_REPORT					0xe3				//ARM返回PC报文

//报文子类型
#define	DOWN_SUB_TEST					0x11					//调试界面
#define	DOWN_SUB_CURVE					0x21					//曲线
#define	DOWN_SUB_MODEL_REPORT			0x31					//各子模块报文监视
#define	DOWN_SUB_ASK_SETTING_PARAM		0x41					//召唤参数
#define	DOWN_SUB_SETTING_PARAM			0x42					//参数下行设定
#define	DOWN_SUB_SHIP_MODEL				0x51					//船的模型设定
#define	DOWN_SUB_SIMULATE_ASK			0x61					//无人船仿真(召唤）
#define	DOWN_SUB_SIMULATE_SET			0x62					//无人船仿真(下载）
#define	DOWN_SUB_VER					0x71					//版本调取

//功能码
//调试界面
#define	DOWN_FUNC_COMM_MONITOR			0x10					//通讯监视
#define	DOWN_FUNC_ALARM					0x11					//告警状态
#define	DOWN_FUNC_TEST					0x12					//调试信息
#define	DOWN_FUNC_CFG_ERR_INF			0x13					//配置文件出错信息
#define	DOWN_FUNC_MAIN_PARAM			0x14					//主要运行参数
#define	DOWN_FUNC_OTHER_PARAM			0x15					//其它运行参数
#define	DOWN_FUNC_RUN_STATE				0x16					//设备运行状态
//曲线
#define	DOWN_FUNC_CURVE_REAL_DATA		0x20					//曲线实时数据

//各模块报文
#define	DOWN_FUNC_STABLE_PLAT			0x30					//稳定平台
#define	DOWN_FUNC_SMART_RUDDER			0x32					//智能舵
#define	DOWN_FUNC_MPC					0x33					//MPC
#define	DOWN_FUNC_ENERGY_MANAGE			0x34					//能源管理
#define	DOWN_FUNC_AUXILIARY_POWER		0x35					//辅助设备电源
#define	DOWN_FUNC_LEFT_ENGINE			0x36					//左发动机
#define	DOWN_FUNC_RIGHT_ENGINE			0x3E					//右发动机
#define DOWN_FUNC_BRA                   0x3F                    //宽带电台

#define MODEL_MAX_NUM					22						//子模块录波通道数

//参数设定
#define	DOWN_FUNC_PID_PARAM				0x40					//PID参数设定
#define	DOWN_FUNC_S_FACE_PARAM			0x41					//S面参数设定
#define	DOWN_FUNC_SHIP_ROUTE			0x42					//航线设定
#define	DOWN_FUNC_OBSTACLE_LOCATE		0x43					//障碍点设定
#define	DOWN_FUNC_LAKE_LARGE_INF		0x44					//湖面大小信息
#define	DOWN_FUNC_TRANS_PARAM			0x45					//平移参数设定
#define	DOWN_FUNC_ROTATE_PARAM			0x46					//旋转参数设定
#define	DOWN_FUNC_OPERATE_OK			0xf0					//返回正确报文
#define	DOWN_FUNC_OPERATE_ERR			0xf1					//返回出错报文

//船的模型监视 
#define	DOWN_FUNC_SIM_SHIP_START_STATE	0x50					//船的初始状态
#define	DOWN_FUNC_SIM_ROUTE				0x51					//航点坐标
#define	DOWN_FUNC_SIM_OBSTACLE			0x52					//障碍点坐标
#define	DOWN_FUNC_SIM_INPUT_REAL_DATA	0x53					//实时输入量

//无人艇仿真
#define	DOWN_FUNC_MMI					0x60					//控制面板仿真


typedef struct{
	uint8	locate_usv_addr;							//本地无人船的地址
	uint8	monitor_udp_rec_flag;
	sockaddr_in from_udp_ip;
}MONITOR_UDP_INF_STRUCT;

//UDP工具报文结构
typedef struct{
	uint8	apdu_len;			//APDU长度
	uint8	dest_addr;			//目的地址
	uint8	src_addr;			//源地址
	uint8	report_type;		//报文类型
	uint8	report_len;			//报文长度
	uint8	sub_type;			//子类型
	uint8	socket_ID_h;		//帧序号高位
	uint8	socket_ID_l;		//帧序号低位
	uint8	func;				//功能码
	uint8	model_addr;			//模块地址
	uint8	buff[96];			//报文内容区
}UDP_MONITOR_REPORT_STRUCT;

//检查发送报文
typedef struct {
	uint32	send_report_flag;		//发送报文标志，0xaaaa--正在发送，	
	uint32	sendlife_time;			//发送报文生存时间
	uint32	send_report_type;		//发送报文报文类型
	uint32	send_data_type;			//数据类型
	uint32	send_func;				//功能码
	uint32	param1;					//参数1
	uint32	param2;					//参数2
	uint32	param3;					//参数3
	uint32	param4;					//参数4
	uint32	send_report_seq;		//发送报文序号,从0开始
}MONITOR_UDP_CHECK_SEND_REPORT_STRUCT;

//操作的应答报文
typedef struct{
	uint16			send_flag;				//发送标志
	uint8			error_flag;				//出错标志 SWITCH_ON--正确，0--出错
	uint8			report_type;			//报文类型
	uint8			sub_type;				//子类型
	uint8			func;					//功能码
	uint8			buff_len;				//缓存区实际长度	
	uint8			buff[50];				//发送信息
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
