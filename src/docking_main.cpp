/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人���������޹�˾
-	File name  : dock_communication.cpp
-	Author	   : fushuai
-	Date	   : 2019/05/13 19:35
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include "../include/modbus/modbus.h"
#include "../include/modbus/modbus-rtu.h"
#include "../include/usv_include.h"
#include "../include/docking_main.h"


#ifdef WINNT
#pragma comment(lib,"modbus.lib")
#endif

#ifdef WINNT
#define DOCK_COMMUNICATION_COM    "COM6"                      /*ͨѶ����*/
#else
#define DOCK_COMMUNICATION_COM    "/dev/ttyO1"                /*ͨѶ���� ��ʱ��˿ӡ�ϵ�COM1���ڴ���ͨ��*/
#endif

//���빦�� ���غ궨��

#define DOCK_WRITE_COMMAND 0
#define DOCK_READ_COMMAND 1


#define DOCK_CONTROLLER_ADD        166                      /*�ӻ���ַ */

//������ 0x01��
#define READ_COIL_ADDR				0


//������ 0x05д
#define SET_REQ_ENTRYDOCK_COIL		0					/*usv���������־λ��Ȧ*/
#define SET_USV_POWEROFF_COIL		1					/*usv�ϵ�Ϩ���־λ��Ȧ*/
#define SET_USV_POWERON_COIL		2					/*usv�ϵ��ʼ����ɱ�־λ��Ȧ*/
#define SET_USV_OUTSUCCESS_COIL		3					/*usv������ɱ�־λ��Ȧ*/



#define DOCK_COIL_MAX			3		//��Լ����������Ȧλ

#define TIMEOUT_WRITE			20		//���볬ʱ
#define TIMEOUT_READ			20		//����ʱ
#define TIMEOUT_REQ				200		//����ʱ
//���غ���

static void dockCommunicationUSVInit(void);
static void dockCoilReadTask();//������
static void resetCoilStatus();
int8 dockCoilWriteTask(uint8 coil_addr, uint8 status);//д����
static void initSubZmq(void);
static void initSubZmqLidar(void);
static void flushZmqDockPostion(void);
static void flushZmqDockPostionLidar(void);
static void closeSubZmq(void);
static void closeSubZmqLidar(void);
static int8 read_usv_docking_inf(void);
static int8 write_usv_docking_inf(void);
static void setDockReturnPoint(void);

//���빦�� ���ر���
USVDockEventState event_state = EVENT_POWERON_USV;        /*!< Init event     */

DOCK_SUMLINK dock_sumlink;
DOCK_STATE dock_sign; //���� ����
USV_STATE usv_sign;
DOCK_COMM_COIL_READ dock_comm_coil_read;
DOCK_COMM_COIL_SET	dock_comm_coil_set;

uint8 motorEnable_sign;//������ʹ�ܱ���λ
uint8 dockin_fun_enable;
RETURN_POINT return_point[RETURN_POINT_MAX_NUMBER];

int8 return_num = RETURN_POINT_MAX_NUMBER;

//dock can
COMM_SIGN DOCK_comm_sign;

DockingControlCmd docking_control_cmd;//�����������?
modbus_t *mb = NULL;


uint16 n_ship;//��������
float l_ship;//����
float temp_dockin_distance;//������ʱ��������
float image_servo_start_distance;//�Ӿ��������?
float image_servo_end_distance;//�Ӿ��������?
float image_servo_pix_multi;//���ؾ����ϵ��?

//camera zmq
char docking_tracker_cfg[30];
void *ctx_pos = NULL;
void *subscriber = NULL;
//lidar zmq
char lidar_tracker_cfg[30];
void *ctx_lidar = NULL;
void *subscriber_lidar = NULL;
TargetPos t_pos;
TargetPosLidar target_pos_lidar;

float docking_speed = 0.0;
//libmodbus modbus_read_bits����Ȧ�Ǵ�addr��ʼn��bit �ŵ�dest�����У�ÿ����Ա��ʾ��Ӧλ��true false
//libmodbus modbus_write_bitд����Ȧ��ʱ����ֱ�Ӷ���Ȧ��ַ��λ����

static void dockCommunicationUSVInit(void)
{
	//��ʼ��״̬
	memset(&dock_sumlink, 0, sizeof(dock_sumlink));
	memset(&dock_sign, 0, sizeof(dock_sign));
	memset(&usv_sign, 0, sizeof(usv_sign));
	memset(&dock_comm_coil_read, 0, sizeof(dock_comm_coil_read));
	memset(&dock_comm_coil_set, 0, sizeof(dock_comm_coil_set));
	memset(&docking_control_cmd,0,sizeof(docking_control_cmd));
}

void powerON(void) //���?
{
	uint8 ret = 0;
	if (IHC_rev_msg.b1_St_Motor1OnOff && IHC_rev_msg.b1_St_Motor2OnOff){ //先判断发动机是否为启动状�?
		return ;
	}
	jet_system.jetL.b2_Cmd_MotorOnOff = 1;		//����
	jet_system.jetL.i16_Cmd_MotorGearDeg = 0;	//����
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;	//����
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;	//��������

	jet_system.jetR.b2_Cmd_MotorOnOff = 1;		//����
	jet_system.jetR.i16_Cmd_MotorGearDeg = 0;	//����
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;	//����
	jet_system.jetR.u8_Cmd_MotorOpenDeg = 0;	//��������

	return ;
}
void powerOff(void)//熄火
{

	if (IHC_rev_msg.b1_St_Motor1OnOff == 0 && IHC_rev_msg.b1_St_Motor2OnOff == 0){ //先判断发动机是否为启动状�?
		return ;
	}
	jet_system.jetL.b2_Cmd_MotorOnOff = 2;		//ֹͣ
	jet_system.jetL.i16_Cmd_MotorGearDeg = 0;	//����
	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;	//����
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;	//��������

	jet_system.jetR.b2_Cmd_MotorOnOff = 2;		//ֹͣ
	jet_system.jetR.i16_Cmd_MotorGearDeg = 0;	//����
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;	//����
	jet_system.jetR.u8_Cmd_MotorOpenDeg = 0;	//��������
	return;
}

//��Ȧ������
void dockCoilReadTask()
{
	dock_sign.x_open = dock_comm_coil_read.dock_entrydock_readyon;
	dock_sign.x_in = dock_comm_coil_read.dock_entrydock_success;
	dock_sign.x_outgoing = dock_comm_coil_read.dock_outdock_readyon;
	dock_sign.x_in_entrance = dock_comm_coil_read.dock_entrydock_entrance;
}
//��λд��־λ
void resetCoilStatus()
{
	memset(&dock_comm_coil_set, 0, sizeof(dock_comm_coil_set));
}
//д��Ȧ����
int8 dockCoilWriteTask(uint8 coil_addr, uint8 status)
{
	int8 ret = 1;
	switch (coil_addr)
	{
	case SET_REQ_ENTRYDOCK_COIL:  //请求进坞
		dock_comm_coil_set.usv_entrydock_req = status;
		dock_comm_coil_set.usv_prower_off_successed = 0;
		dock_comm_coil_set.usv_prower_on_successed = 0;
		dock_comm_coil_set.usv_outdock_successed = 0;
		break;	
	case SET_USV_POWEROFF_COIL:	  //熄火完成
		dock_comm_coil_set.usv_prower_off_successed = status;
		dock_comm_coil_set.usv_entrydock_req = 0;
		dock_comm_coil_set.usv_prower_on_successed = 0;
		dock_comm_coil_set.usv_outdock_successed = 0;
		break;
	case SET_USV_POWERON_COIL:	  //上电成功
		dock_comm_coil_set.usv_prower_on_successed = status;
		dock_comm_coil_set.usv_entrydock_req = 0;
		dock_comm_coil_set.usv_prower_off_successed = 0;
		dock_comm_coil_set.usv_outdock_successed = 0;
		break;
	case SET_USV_OUTSUCCESS_COIL:	//出坞成功
		dock_comm_coil_set.usv_outdock_successed = status;
		dock_comm_coil_set.usv_entrydock_req = 0;
		dock_comm_coil_set.usv_prower_off_successed = 0;
		dock_comm_coil_set.usv_prower_on_successed = 0;
		break;
	default:
		break;
	}
	return ret;
}
//����������״̬��ȡ
int8 dockingIn()
{
	int8 ret = 0;
	static uint8 req_flag = 0;

	dockCoilWriteTask(SET_REQ_ENTRYDOCK_COIL, 1); //��λ��������

	if ((1 == dock_sign.x_open) || (dock_sumlink.x_open)){ //�ɽ���
		ret = 1;
	}
	else{
		ret = 0;
	}
//	resetCoilStatus();
	return ret;
}

int8 dockingInFinish()
{
	int8 ret = 0;
	if ((!IHC_rev_msg.b1_St_Motor1OnOff && !IHC_rev_msg.b1_St_Motor2OnOff && dock_sign.x_in) || dock_sumlink.x_in){ //����Ϩ��ɹ�?
		dockCoilWriteTask(SET_USV_POWEROFF_COIL, 1); //��λ�ѽ������Ϩ��ɹ�״̬
		br_usv_cmd.u8_cmd_getAuthority = 1; //��ȡȨ�� ����Ϩ��
		ret = 1;

	}else{
		ret = 0;
	}
//	resetCoilStatus();
	return ret;
}
int8 powerOnUSV()
{
	int8 ret = 0;
	dockCoilWriteTask(SET_USV_POWERON_COIL, 1); //��λ�ϵ��ʼ���ɹ�?����������
	if ((1 == dock_sign.x_outgoing) || (1 == dock_sumlink.x_outgoing)){ //�ɳ���״̬
		ret = 1;//
	}
	else{
		ret = 0;
	}
	//	resetCoilStatus();
	return ret;
}
int8 dockingOut()
{
	int8 ret = 0;
	dockCoilWriteTask(SET_USV_POWERON_COIL, 1); //��λ�ϵ��ʼ���ɹ�?����������
	if ((1 == dock_sign.x_outgoing) || (1 == dock_sumlink.x_outgoing)){ //�ɳ���״̬
		ret = 1;//
	}
	else{
		ret = 0;
	}
//	resetCoilStatus();
	return ret;
}
int8 dockingOutFinish()
{
	int8 ret = 0;
	dockCoilWriteTask(SET_USV_OUTSUCCESS_COIL, 1); //��λ�ϵ��ʼ���ɹ�?����������
	if (1 == usv_sign.succcessed_out){ //������ɹ�����������־�?
		ret = 1;
	}else{
		ret = 0;
	}
	return ret;
}
void sumlinkBtnEvent()
{
	uint8 write_inf_ret = 0;
	static uint8 reset_btn = 0;			//���Թ��߰�ť ��λ״̬
	static uint8 return_pos_btn = 0;	//���Թ��߰�ť ����������
	static uint8 temp_pos_btn = 0;		//���Թ��߰�ť ����������
	static uint8 allow_entry_btn = 0;	//���Թ��߰�ť �ɽ�������
	static uint8 allow_out_btn = 0;		//���Թ��߰�ť �ɳ�������
	static uint8 already_out_btn = 0;	//���Թ��߰�ť �ѳ�������
	static uint8 already_entry_btn = 0;	//���Թ��߰�ť �ѽ�������

	if (ins_msg.insState.c_rmcValid == 'A') //�жϹߵ��Ƿ��������� A:��λ��Ч V:��Ч��λ
	{
		if (return_pos_btn != sumlink_cmd.u8_return_set){  //TODO ��̨�����ȡ�����λ�ð�ť
			return_pos_btn = sumlink_cmd.u8_return_set;
			if (return_pos_btn == 1) //��λ��ȡ���뷵����
			{
				return_point[2].longitude = ins_msg.longitude;//��¼���һ��������?�����?
				return_point[2].latitude = ins_msg.latitude;
				return_point[2].heading = ins_msg.heading;


				write_inf_ret = write_usv_docking_inf();
				if (write_inf_ret == FALSE){
			//		SysPubMsgPost("������������洢ʧ��\n");

				}
				else{
			//		SysPubMsgPost("������������洢�ɹ�\n");
				}
			}
		}
		if (temp_pos_btn != sumlink_cmd.u8_startp_set){
			temp_pos_btn = sumlink_cmd.u8_startp_set;
			if (temp_pos_btn == 1) //��λ��ȡ���뷵����
			{
				return_point[1].longitude = ins_msg.longitude;
				return_point[1].latitude = ins_msg.latitude;
				return_point[1].heading = ins_msg.heading;


				write_inf_ret = write_usv_docking_inf();
				if (write_inf_ret == FALSE){
				//	SysPubMsgPost("��ʼ��������洢ʧ��\n");

				}
				else{
				//	SysPubMsgPost("��ʼ��������洢�ɹ�\n");
				}
			}
			
			if (temp_pos_btn == 2) //�͵���������
			{
				return_point[0].longitude = ins_msg.longitude;
				return_point[0].latitude = ins_msg.latitude;
				return_point[0].heading = ins_msg.heading;


				write_inf_ret = write_usv_docking_inf();
				if (write_inf_ret == FALSE){
		//			SysPubMsgPost("��ʼ��������洢ʧ��\n");

				}
				else{
		//			SysPubMsgPost("��ʼ��������洢�ɹ�\n");
				}
			}
		}
	}
	//
#if 0
	if (reset_btn != sumlink_cmd.u8_reset){ //��λ
		reset_btn = sumlink_cmd.u8_reset;
		if (1 == reset_btn){
			dock_sumlink.x_reset = 1;
			//memset(&dock_sumlink, 0, sizeof(dock_sumlink)); //���湤�߰�ť״̬��λ
			printf("reset ON\n");
		}
		else
		{
			//dock_sumlink.x_reset = 0;
			printf("reset OFF\n");
		}
	}

	if (allow_entry_btn != sumlink_cmd.u8_ready_in){ //�ɽ���
		allow_entry_btn = sumlink_cmd.u8_ready_in;
		if (1 == allow_entry_btn){
			dock_sumlink.x_open = 1;
			dock_sumlink.x_in = 0;
			dock_sumlink.x_outgoing = 0;
			dock_sumlink.x_outsuccess = 0;
			printf("ready_in ON\n");
		}
		else
		{
			//dock_sumlink.x_open = 0;
			printf("ready_in OFF\n");
		}
	}
	if (already_entry_btn != sumlink_cmd.u8_already_in){ //�ѽ���
		already_entry_btn = sumlink_cmd.u8_already_in;
		if (1 == already_entry_btn){
			dock_sumlink.x_in = 1;
			dock_sumlink.x_open = 0;
			dock_sumlink.x_outgoing = 0;
			dock_sumlink.x_outsuccess = 0;
			printf("already_in ON\n");
		}
		else{
			//dock_sumlink.x_in = 0;
			printf("already_in OFF\n");
		}
	}
	if (allow_out_btn != sumlink_cmd.u8_ready_out){ //�ɳ���
		allow_out_btn = sumlink_cmd.u8_ready_out;
		if (1 == allow_out_btn){
			dock_sumlink.x_outgoing = 1;
			dock_sumlink.x_open = 0;
			dock_sumlink.x_in = 0;
			dock_sumlink.x_outsuccess = 0;
			printf("ready_out ON\n");
		}
		else
		{
			//dock_sumlink.x_outgoing = 0;
			printf("ready_out OFF\n");
		}
	}
	if (already_out_btn != sumlink_cmd.u8_already_out){ //�ѳ���
		already_out_btn = sumlink_cmd.u8_already_out;
		if (1 == already_out_btn){
			dock_sumlink.x_outsuccess = 1;
			dock_sumlink.x_open = 0;
			dock_sumlink.x_in = 0;
			dock_sumlink.x_outgoing = 0;
			printf("already_out ON\n");
		}
		else
		{
			//dock_sumlink.x_outsuccess = 0;
			printf("already_out OFF\n");
		}
	}
#endif
}
void *dockCommunicationUSVRun(void*)
{

	uint8 flag_return = 0;
	uint8 flag_out = 0;

	uint8 read_inf_ret = 0;
	uint8 return_pos_set = 0;
	uint8 write_inf_ret = 0;

		//读返航点配置
	read_inf_ret = read_usv_docking_inf();
	if (read_inf_ret == FALSE)
	{
		printf("read read_usv_docking_inf wrong!\n");

	}
	else
	{
		printf("read read_usv_docking_inf success!\n");
	}
	dockCommunicationUSVInit();
	for (;;)//����<-->USV�¼��߳�
	{
		
// 		if (COMM_CONNECT_FAIL == DOCK_comm_sign.comm_sign){ //通信中断则打�?
// 			sleep_1(100);
// 			continue;
// 		}
		sumlinkBtnEvent();  //刷仿真工具状�?
		setDockReturnPoint();//刷设置返航点操作
		if (1 == command_signal.func_mode_cmd.b1_dock_cmd || sumlink_cmd.u8_dockin)
		{
			pAutoReturnInst->TurnOnAutoReturn();
		}
		if ((0 == flag_return) && pAutoReturnInst->isAutoReturnRunning()){ //进坞命令 暂时用泊岸对应字节调�?进坞
			flag_return = 1;
			flag_out = 1;
			event_state = EVENT_ENTRY_DOCK_READY; //切入进坞模式
			usv_sign.cmd_feedback = 1; //进坞命令反馈
		//	SysPubMsgPost("�յ�����ָ�� \n");
		//	SysLogMsgPost("�յ��������Ľ���ָ��");

		}
		if ((0 == flag_out) /*&& 1 == usv_sign.power_on */&& (2 == command_signal.func_mode_cmd.b1_dock_cmd)){ //�������� ��ʱ�ò�����Ӧ�ֽڵ��� ����
			flag_out = 1;
			flag_return = 1;
			//powerON();//���?Ŀǰ��Ϊ��������ͨ������ᱻ����?todo
			event_state = EVENT_POWERON_USV;//切入出坞模式
			usv_sign.cmd_feedback = 2;//出坞命令反馈
			pAutoReturnInst->reset_AutoReturn();//复位返航状态机和相关标志位
		//	SysPubMsgPost("�յ�����ָ�� \n");
		//	SysLogMsgPost("�յ��������ĳ���ָ��");

		}
		if (1 == sumlink_cmd.u8_reset)//模拟工具复位 或者重新返�?
		{
			flag_return = 0;
			flag_out = 0;
			memset(&usv_sign, 0, sizeof(usv_sign));
			//event_state = EVENT_POWERON_USV;
			dock_zmq_cmd.dockin_cmd = CMAERA_TRACKOFF;//关闭摄像头目标跟�?
			pAutoReturnInst->reset_AutoReturn();
		//	SysPubMsgPost("Docking Reset...\n");
		}
		if (reDockIn()) //���½���
		{
			memset(&usv_sign, 0, sizeof(usv_sign));//清楚usv状态标�?
			event_state = EVENT_ENTRY_DOCK_READY;//重新请求进坞
			resetDockControlEvent();
		}
		if (dockin_fun_enable != 1) //进坞功能没被使能
		{
			event_state = EVENT_POWERON_USV;
		}
		dockCoilReadTask();//����״̬
		switch (event_state)
		{
		case EVENT_POWERON_USV:
			if (1 == powerOnUSV()){//船上电成�?
				memset(&usv_sign, 0, sizeof(usv_sign));//清楚usv状态标�?
				usv_sign.power_on = 1;
				event_state = EVENT_OUTDOCK_READY;
			}
			break;
		case EVENT_OUTDOCK_READY://准备出坞
			if (!usv_sign.out_docking){//
				usv_sign.out_docking = 1;
				usv_sign.succcessed_out = 0;
				usv_sign.entry_docking = 0;
			//	SysLogMsgPost("��������");
			//	SysPubMsgPost("step out dock......\n");
				event_state = EVENT_OUTING;
			}
			break;
		case EVENT_OUTING://出坞�?
			if (usv_sign.succcessed_out){
				if (1 == dockingOutFinish()){
					usv_sign.succcessed_out = 0;
					flag_return = 0;//出坞完成后复位进坞命令标志位
					usv_sign.cmd_feedback = 0;//命令反馈置零
					event_state = EVENT_OUTED_USV;
				}	
			}
			break;
		case EVENT_ENTRY_DOCK_READY://准备进坞
			if (!usv_sign.entry_docking && (1 == dockingIn())){ //请求进坞
				usv_sign.entry_docking = 1;
				usv_sign.succcessed_entry = 0;
				usv_sign.out_docking = 0;
			//	SysPubMsgPost("step in dock......\n");
				event_state = EVENT_ENTRYING;
			}
			break;
		case EVENT_ENTRYING://进坞�?
			if (usv_sign.succcessed_entry && (1 == dockingInFinish())){
				usv_sign.succcessed_entry = 0;
				usv_sign.entry_docking = 0;
				flag_out = 0;//进坞完成后复位出坞命令标志位
				return_pos_set = 0;//复位自动获取返航点的标志�?
				usv_sign.cmd_feedback = 0;//命令反馈置零
				usv_sign.power_on = 0;
				event_state = EVENT_ENTRYED_USV;
			}
			break;
		case EVENT_ENTRYED_USV:
			dock_zmq_cmd.dockin_cmd = CMAERA_TRACKOFF;//关闭摄像头目标跟�?
			//event_state = EVENT_POWERON_USV; //Ϊ��ѭ�������� �����ѽ�״̬ѭ��
			break;
		case EVENT_OUTED_USV:
			dock_zmq_cmd.dockin_cmd = CMAERA_TRACKOFF;//关闭摄像头目标跟�?
			break;
		default:
			break;
		}
		sleep_1(20);
	}

}
static void initSubZmq(void)
{
	ctx_pos = zmq_ctx_new();
	subscriber = zmq_socket(ctx_pos, ZMQ_SUB);
	zmq_connect(subscriber, docking_tracker_cfg);
	char* filter = "";
	zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, filter, 0);
}

static void flushZmqDockPostion(void)
{
	char *str_rev = NULL;
	zmq_msg_t msg;
	int len = zmq_msg_init(&msg);
	if (len < 0){
		return;
	}
	int recv_len = zmq_msg_recv(&msg, subscriber, ZMQ_DONTWAIT);
	if (recv_len > 0){//�յ�����
		char *str_rev = (char*)malloc(recv_len);
		memcpy(&t_pos, zmq_msg_data(&msg),recv_len);
		
		//memcpy(str_rev, zmq_msg_data(&msg), recv_len);
		//char *str_rev = s_recv(subscriber);
		//printf("sub == %s\n", str_rev);
		/*memcpy(&t_pos.cols, str_rev, sizeof(t_pos.cols));
		memcpy(&t_pos.rows,	str_rev+4,sizeof(t_pos.rows));
		memcpy(&t_pos.target_w, str_rev + 8, sizeof(t_pos.target_w));
		memcpy(&t_pos.target_h, str_rev + 12, sizeof(t_pos.target_h));
		memcpy(&t_pos.t_x, str_rev + 16, sizeof(t_pos.t_x));
		memcpy(&t_pos.t_y, str_rev + 20, sizeof(t_pos.t_y));*/
		//sscanf(str_rev, "%d %d %d %d %d %d ", &t_pos.cols, &t_pos.rows, &t_pos.target_w, &t_pos.target_h, &t_pos.t_x, &t_pos.t_y);
		free(str_rev);
	}
	else
	{
		//no data
	}
	//printf("f_w = %d, f_h = %d\n", t_pos.cols, t_pos.rows);
	//printf("x == %d\tt_y == %d\n", t_pos.t_x, t_pos.t_y);
	//printf("t_w = %d, t_h = %d\n", t_pos.target_w, t_pos.target_h);
	//printf("locking on = %d\n", t_pos.lock_on);
}

static void closeSubZmq(void)
{
	zmq_close(subscriber);
	zmq_ctx_destroy(ctx_pos);
}

void * cameraTrackPosFlush(void *)
{
	for (;;) //ˢ����ͷ��������
	{
		if (subscriber == NULL)
		{
			initSubZmq();//��ʼ��zmq����
			continue;
		}
		else
		{
			flushZmqDockPostion();//ʵʱ��������ͷĿ���������?
			//printf("flush camera data\n");
		}
		s_sleep(20);
	}
	closeSubZmq();
}


//������������
int8  read_usv_docking_inf(void)
{
	int8 		*p_file_memory;		//������
	int32 	*p_buffer;
	FILE		*pFile;
	uint32	lSize;
	int32	result;
	uint32	len;
	int8		s1[32];
	int8		s2[50];
	int8		ret_val;

	ret_val = TRUE;
	if ((pFile = fopen(USV_DOCKING_FILE_NAME, "r+")) == NULL)	//��ȡ�ļ�����
	{
		sprintf_usv(s1, "usv_flash_docking.inf");
		sprintf_usv(s2, "not found");
		input_cfg_ini_err_sub(s1, s2, 0);
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);		//16K
	if (NULL == p_file_memory)
	{
		sprintf_usv(s1, "usv_flash_docking file memory");
		sprintf_usv(s2, "not enough");
		input_cfg_ini_err_sub(s1, s2, 0);
		fclose(pFile);
		return FALSE;
	}

	p_buffer = (int32 *)malloc(0x10000);			//64K
	if (NULL == p_buffer)
	{
		sprintf_usv(s1, "usv_flash_docking explain memory");
		sprintf_usv(s2, "not enough");
		input_cfg_ini_err_sub(s1, s2, 0);
		free(p_file_memory);
		fclose(pFile);
		return FALSE;
	}

	//��ȡ�ļ���С
	fseek(pFile, 0, SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile);			//��ָ��ָ��ͷ

	if (lSize >= 0xffff)
	{
		sprintf_usv(s1, "usv_flash_docking read file");
		sprintf_usv(s2, "too large");
		input_cfg_ini_err_sub(s1, s2, 0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

	result = fread(p_file_memory, 1, lSize, pFile); 	//���ļ�������buff ��
	//if(result != lSize)
	//{
	//	sprintf_usv(s1,"USV state read file");
	//	sprintf_usv(s2,"not same");
	//	input_cfg_ini_err_sub(s1,s2,0);
	//	free(p_file_memory);
	//	free(p_buffer);
	//	fclose(pFile);
	//	return FALSE;
	//}

	if (32768 < lSize) len = 2 * lSize;
	else len = 1280 + 2 * lSize;
	len = len * 2;
	result = ini_Initialize((char *)p_file_memory, p_buffer, len);

	if (result != 0){
		sprintf_usv(s1, "usv_flash_docking  memory");
		sprintf_usv(s2, "explain error");
		input_cfg_ini_err_sub(s1, s2, 0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

//�����ļ�
	sprintf_usv(s1, "Return_Function");
	sprintf_usv(s2, "Dockin_Enable");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&dockin_fun_enable, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	sprintf_usv(s1, "Return_Point");
	sprintf_usv(s2, "return_point_number");
	uint32 number = 0;
	if (read_sub_setting(s1, s2, 0, &number, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}
	return_num = number;//���뷵�������?
	if (number > RETURN_POINT_MAX_NUMBER){ //������󷵺�����?
		input_cfg_ini_err_sub(s1, s2, 0);
		ret_val = FALSE;
		number = 0;
	}
	for (int i = 0; i < number; i++)
	{
		sprintf_usv(s2, "Point_%d", i);
		if (read_sub_setting_df(s1, s2, 0, (uint64 *)&return_point[i].longitude, FLOAT_TYPE) == FALSE){		//����
			ret_val = FALSE;
		}
		if (read_sub_setting_df(s1, s2, 1, (uint64 *)&return_point[i].latitude, FLOAT_TYPE) == FALSE){		//γ��
			ret_val = FALSE;
		}
		if (read_sub_setting_df(s1, s2, 2, (uint64 *)&return_point[i].heading, FLOAT_TYPE) == FALSE){		//����
			ret_val = FALSE;
		}
	}
	sprintf_usv(s1, "Return_Control");
	sprintf_usv(s2, "tempDockInPointDis");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&temp_dockin_distance, FLOAT_TYPE) == FALSE){	
		ret_val = FALSE;
	}

	sprintf_usv(s1, "Return_Control");
	sprintf_usv(s2, "imageServoStartDis");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&image_servo_start_distance, FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}
	sprintf_usv(s1, "Return_Control");
	sprintf_usv(s2, "imageServoEndDis");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&image_servo_end_distance, FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	sprintf_usv(s1, "Return_Control");
	sprintf_usv(s2, "imageServoPixMulti");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&image_servo_pix_multi, FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	return ret_val;
}

int8 write_usv_docking_inf(void)	//����ʧ�ܷ���0���ɹ�����1
{
	FILE *pFile;
	int8 buff[200];
	uint16 ret_len;

	pFile = fopen(USV_DOCKING_FILE_NAME, "w+");

	if (pFile == NULL)
		return FALSE;

	//ret_len = sprintf_usv((int8 *)&buff[0],";\n");
	//fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len = sprintf_usv((int8 *)&buff[0], "[Return_Function]\n");
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	ret_len = sprintf_usv((int8*)&buff, "Dockin_Enable=%d;\n", dockin_fun_enable);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);

	ret_len = sprintf_usv((int8 *)&buff[0], "[Return_Point]\n");
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);

	ret_len = sprintf_usv((int8 *)&buff[0], "return_point_number=%d;\n", return_num);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	
	for (int i = 0; i < return_num; i++)
	{

		if (i == return_num - 1){
			ret_len = sprintf_usv((int8*)&buff, "Point_%d=%8.8f,%8.8f,%8.8f	;���ȣ�γ�ȣ�����\n", i, return_point[i].longitude, return_point[i].latitude, return_point[i].heading);
		}
		else{
			ret_len = sprintf_usv((int8*)&buff, "Point_%d=%8.8f,%8.8f,%8.8f	;���ȣ�γ�ȣ�����\n", i, return_point[i].longitude, return_point[i].latitude, return_point[i].heading);
		}
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	}

	ret_len = sprintf_usv((int8 *)&buff[0], "[Return_Control]\n");
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	ret_len = sprintf_usv((int8 *)&buff[0], "tempDockInPointDis=%f;\n", temp_dockin_distance);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	ret_len = sprintf_usv((int8 *)&buff[0], "imageServoStartDis=%f;\n", image_servo_start_distance);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	ret_len = sprintf_usv((int8 *)&buff[0], "imageServoEndDis=%f;\n", image_servo_end_distance);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);
	ret_len = sprintf_usv((int8 *)&buff[0], "imageServoPixMulti=%f;\n", image_servo_pix_multi);
	fwrite((int8 *)&buff[0], sizeof(char), ret_len, pFile);

	fclose(pFile);
	printf("write usv_flash_docking ok\n");
	return TRUE;
}

void setDockReturnPoint(void)
{
	uint8 write_inf_ret = 0;
	static uint8 log_retpos_set = 0;
	if (ins_msg.insState.c_rmcValid == 'A') //判断惯导是否锁星锁上 A:定位有效 V:无效定位
	{
		if (log_retpos_set != command_signal.func_mode_cmd.b1_setReturn){   //TODO 后台加入获取船坞的位置按�?
			log_retpos_set = command_signal.func_mode_cmd.b1_setReturn;
			if (log_retpos_set == 1) //置位获取近坞返航�?
			{
				return_point[2].longitude = ins_msg.longitude;//记录最后一个返航点 近坞�?
				return_point[2].latitude = ins_msg.latitude;
				return_point[2].heading = ins_msg.heading;


				write_inf_ret = write_usv_docking_inf();
				if (write_inf_ret == FALSE){
				//	SysPubMsgPost("�����������÷�������洢ʧ��\n");
					usv_sign.set_retpos_on = 3; //д��ʧ��

				}
				else{
				//	SysPubMsgPost("�����������÷�������洢�ɹ�\n");
				//	SysLogMsgPost("�����������÷�������洢�ɹ�\n");
					usv_sign.set_retpos_on = 1; //���óɹ�
				}
			}
			else if (log_retpos_set == 2){ //���õ͵���������
				return_point[0].longitude = ins_msg.longitude;//船坞位置
				return_point[0].latitude = ins_msg.latitude;
				return_point[0].heading = ins_msg.heading;


				write_inf_ret = write_usv_docking_inf();
				if (write_inf_ret == FALSE){
				//	SysPubMsgPost("�����������õ͵�����������洢ʧ��\n");
					usv_sign.set_retpos_on = 3; //д��ʧ��

				}
				else{
				//	SysPubMsgPost("�����������õ͵�����������洢�ɹ�\n");
				//	SysLogMsgPost("�����������õ͵�����������洢�ɹ�\n");
					usv_sign.set_retpos_on = 1; //���óɹ�
				}
			}
			else
			{
				usv_sign.set_retpos_on = 0;//��λ���ð�ť
			}
		}
	}
	else
	{
		usv_sign.set_retpos_on = 2;//��λ��Ч
	}
}

void DOCK_Init()
{
	comm_time_return(&(DOCK_comm_sign.timer), &(DOCK_comm_sign.comm_sign));
}

void DOCK_reInit()
{
	dock_comm_coil_read.dock_entrydock_readyon	= 0;
	dock_comm_coil_read.dock_entrydock_success	= 0;
	dock_comm_coil_read.dock_outdock_readyon	= 0;
	dock_comm_coil_read.dock_entrydock_entrance = 0;

}
void DOCK_recv(uint8 ps_id, uint8* data)
{

	if (CAN_DOCK_PS == ps_id && data[0] == 0x83){
		dock_comm_coil_read.dock_entrydock_readyon	= (data[1]&0x01) >> 0;
		dock_comm_coil_read.dock_entrydock_success  = (data[1]&0x02) >> 1;
		dock_comm_coil_read.dock_outdock_readyon	= (data[1]&0x04) >> 2;
		dock_comm_coil_read.dock_entrydock_entrance = (data[1]&0x08) >> 3;
	}
	//printf("�ɽ��� == %d\n",dock_comm_coil_read.dock_entrydock_readyon);
	//printf("����ɹ�?== %d\n", dock_comm_coil_read.dock_entrydock_success);
	//printf("�ɳ��� == %d\n", dock_comm_coil_read.dock_outdock_readyon);
	//printf("�������� == %d\n", dock_comm_coil_read.dock_entrydock_entrance);

	comm_time_return(&(DOCK_comm_sign.timer), &(DOCK_comm_sign.comm_sign));
}


void DOCK_sendMsg()
{
	uint8 data[8];
	memset(data, 0, 8);
	data[0] = 0x83;
	data[1] |= ((0x01 & dock_comm_coil_set.usv_entrydock_req) << 0);
	data[1] |= ((0x01 & dock_comm_coil_set.usv_prower_off_successed) << 1);
	data[1] |= ((0x01 & dock_comm_coil_set.usv_prower_on_successed) << 2);
	data[1] |= ((0x01 & dock_comm_coil_set.usv_outdock_successed) << 3);
	sendCanMsg(can_0, 255, CAN_DOCK_PS, CAN_USV_ADD,data);
}

void DOCK_CommCal(void)
{
	comm_time_cal(CAN_DOCK_DISCONNECT_MAX, &(DOCK_comm_sign.timer), &(DOCK_comm_sign.comm_sign));
}

void initDocksendTask(void)
{
	addTask(3, runDocksendTask, (void *)0);
}

void runDocksendTask(void *)
{
	DOCK_sendMsg();
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_DOCK_SN].send_ok_number++;	//���ͼ���
}

void * lidarTrackerPosFlush(void *)
{
	for (;;) //ˢ����ͷ��������
	{
		if (subscriber_lidar == NULL)
		{
			initSubZmqLidar();//��ʼ��zmq����
			continue;
		}
		else
		{
			flushZmqDockPostionLidar();//ʵʱ�����״�Ŀ���������?
			//printf("flush Lidar data\n");
		}
		s_sleep(20);
	}
	closeSubZmqLidar();
}
void initSubZmqLidar(void)
{
	//lidar
	ctx_lidar = zmq_ctx_new();
	subscriber_lidar = zmq_socket(ctx_lidar, ZMQ_SUB);
	zmq_connect(subscriber_lidar, lidar_tracker_cfg);
	char* filter = "test";
	zmq_setsockopt(subscriber_lidar, ZMQ_SUBSCRIBE, filter, 0);
}
void closeSubZmqLidar(void)
{
	zmq_close(subscriber_lidar);
	zmq_ctx_destroy(ctx_lidar);
}
void flushZmqDockPostionLidar(void)
{
	char *str_rev = NULL;
	char msg[128];
	//int len = zmq_msg_init(msg);
	/*if (len < 0){
	return;
	}*/
	int recv_len = zmq_recv(subscriber_lidar, msg, sizeof(msg), ZMQ_DONTWAIT);
	if (recv_len < 0)
	{
		return;
	}
	zmq_msg_t msg_data;
	int _len = zmq_msg_init(&msg_data);
	if (_len < 0){
		return;
	}

	int data_len = zmq_msg_recv(&msg_data, subscriber_lidar, ZMQ_DONTWAIT);
	if (data_len > 0){//�յ�����
		str_rev = (char*)malloc(data_len);
		memcpy(&target_pos_lidar, zmq_msg_data(&msg_data), data_len);//&target_pos_lidar
		free(str_rev);
	}
	else
	{
		//no data
	}
	//printf("d_x == %f \t d_y == %f\n", target_pos_lidar.delta_x, target_pos_lidar.delta_y);
	//printf("locking on = %d\n", target_pos_lidar.lock_on);
}
