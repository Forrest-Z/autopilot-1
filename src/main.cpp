#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/nanoObstacleSender.h"
#include <stdint.h>
#include <boat/boat.h>

int init();
void exit_deal(void);
void creat_thread(void);
void Msg_Init(void);

extern void Send_CAN_Control(void);
extern void Send_CAN_State(void);
extern void Send_CAN_Cfg();

//extern void Emergency_Stop_Control(int Con_Type);


void Wrte_Error(char *ErrorMsg);

void Send_AIS_Msg(void);
void Connect_count();
void DSP_Connect_St();
void AIS_Connect_St();
void Dradio_Connect_St();
void BDS_Connect_St();
void Sradio_Connect_St();
void INS_Connect_St();
void MPC_Connect_St();
void Bradio_Connect_St();
void CAN_Connect_St();
void SendCommand();
//void Get_ARM_Version();
void vershow();
int GetName(void);

extern void Get_CAN_SR_Engine_Config();
extern void Get_CAN_SR_Gear_Config();
extern void Get_CAN_SR_Gear2_Config();
extern void Get_CAN_SR_Rudder_Config();
extern void Get_CAN_SR_Rudder2_Config();
extern void Get_CAN_SR_Rudder3_Config();
void set_ip();
int GPIO_Init();
void help(void);

extern double Get_distance(double lat1,double lng1,double lat2,double lng2);//cxy

#ifndef WINNT
extern int errno;
#endif
  
threadnode_ids thread_ids;	

//uint8	DSP_Heartbeat[HeartbeatLenght]={0x55,0x55,0x13,0x00,0x00,0xaa,0xaa};//RAM����DSP������0x13
//uint8	MPC_Heartbeat[HeartbeatLenght]={0xaa,0xaa,0x12,0x00,0x00,0x55,0x55};//RAM����MPC������0x12

uint8	AIS_Send_Sign;//AIS���ķ��ͱ�־
struct timeval t_start,t_end;
long cost_time = 0,signe=0;
int cancount=0,CANsin=0;

extern nanoObstacleSender senderTestV1;

int poweron_init = 0;
int main(int argc,char **argv)
{
	uint8 task_200ms,task_50ms;
	uint16	task_1s, task_20s, task_5s, task_10s_lowenergy, State_Num;

	task_200ms = 0;
	task_50ms = 0;
	task_1s = 0;
	task_5s = 0;
	task_10s_lowenergy = 0;
	task_20s = 0;
	State_Num = 0;

	printf("################ usv start! ########################\n");	

	while(1){
		int c = getopt(argc,argv,"hv::");
		if(c==-1) break;

		switch(c){
			case 'h': help();exit(0);
			case 'v': ship_version = atol(optarg);break;
		}
	}

	switch(ship_version){
	case 1: printf("tianji no.1 programme \n");break;
		
	case 2: printf("water quality programme 2019/10/10 08:08:26\n");break;
		
	default:printf("version error\n");exit(0);break;
	}


	if(init()==1)
	{
//		exit_deal();
//		return 0;
	}

	//nanoObstacleSender senderTest init;
	usv_mutex_init(&apf_mutex);

	printf("begine create threads\n");
	creat_thread();

	while(1)
	{  
#ifndef WINNT
//		gettimeofday(&t_start,NULL);
#endif
		task_50ms++;
		task_200ms++;
		task_1s++;
		task_5s++;
		task_10s_lowenergy++;
		task_20s++;
		if(task_50ms>5){
		//	ins_zmq_publish();
			MPC_Connect_St();
		}

		if(task_200ms>20)      //200ms����
		{
			SysLogSave();
			SysMsgPub();
			
			WarnMsgQueuGetPub();
			task_200ms=0;
		}
		if(task_1s>100)		//1s����
		{
			updateMeterClock();
			feedWatchDog();
			
			AP_OADatabase *oaDb = AP::oadatabase();
			oaDb->send_adsb_vehicle();
			
			senderTestV1.obstacleGetHeadOfQueue();
			senderTestV1.obstacleSend();

			
			getHardwareState();
			task_1s=0;
		}
		if(task_5s>500)
		{
			//����
			//WarnMsgQueuePut(WARN_SRC_ARM, 0, 1);
			//WarnMsgQueuePut(WARN_SRC_ARM, 1, 0);
			//WarnMsgQueuePut(WARN_SRC_ARM, 2, 1);
			//WarnMsgQueuePut(WARN_SRC_IHC, 0, 1);
			//WarnMsgQueuePut(WARN_SRC_IHC, 4, 0);
			//WarnMsgQueuePut(WARN_SRC_IHC, 6, 1);

			writeSysTime();

			task_5s = 0;
		}

		if (task_10s_lowenergy > 1000)
		{
			pAutoReturnInst->JudgeNeedPowerOffAutoReturn(IHC_rev_msg.u8_St_BatteryLevel);

			task_10s_lowenergy = 0;
		}
		if (task_20s > 2000)
		{
			poweron_init = 1;
			task_20s = 0;
		}

#ifndef WINNT		
		//gettimeofday(&t_end,NULL);
#endif
		cost_time=t_end.tv_usec-t_start.tv_usec;

		sleep_1(10);            //10ms//cxy�Ƿ���Ҫר�Ŷ�ʱ��
	}
	exit_deal(); 
	return 0;
}

void SendCommand()
{
	if(MPCVersion_sign==0)
		Send_MPC_CRC();
	if((UAVVersion_sign==0)||(SRVersion_sign==0)||(POWVersion_sign==0)||(SPVersion_sign==0)||(STVersion_sign==0)||(BATVersion_sign==0))	
//	if((SRVersion_sign==0)||(POWVersion_sign==0)||(SPVersion_sign==0)||(BATVersion_sign==0))
		Send_CAN_CRC();
}
void Oil_Control()
{
		char value0[]="0",value1[]="1";	
		char index[]="204";
		
		if(1==USV_Control.USV_Control_Message[Radio_Sign].Oil_Con)
		{
			if(set_value(index, value1)<0)
			{
				printf("set value error!\n");
				return ;
			}
		}
		else 
		{
			if(set_value(index, value0)<0)
			{
				printf("set value error!\n");
				return ;
			}
		}
		return ;		
}


void e_stop_init(void)
{
	e_stop_inf.e_stop_cmd	=0;
	e_stop_inf.e_stop_logic	=0;
	e_stop_inf.e_stop_rudder_failure = 0;
	e_stop_inf.e_stop_comm = 0;

	e_stop_inf.e_stop_cmd_inf.bd_btn_stop = 0;
	e_stop_inf.e_stop_cmd_inf.dradio_btn_stop = 0;
	e_stop_inf.e_stop_cmd_inf.sradio_btn_stop = 0;
	e_stop_inf.e_stop_cmd_inf.sp_btn_stop = 0;
}




void Emergency_Stop_Control(int Con_Type)
{

	uint8	e_stop_cmd=0;				//�ֶ���ͣ
	uint8 	e_stop_logic=0;				//����ϵͳ���ϼ�ͣ
	uint8	e_stop_rudder_failure=0;	//��DSP�ϱ����ϼ�ͣ
	uint8	e_stop_comm=0;				//ͨѶ�жϼ�ͣ
	
	uint8	e_stop_last;

		
		e_stop_inf.e_stop_cmd_inf.dradio_btn_stop =		USV_Control.USV_Control_Message[0].Emergency_Stop;		//���ֵ�̨	
		e_stop_inf.e_stop_cmd_inf.sradio_btn_stop =		USV_Control.USV_Control_Message[1].Emergency_Stop;		//�����̨
		e_stop_inf.e_stop_cmd_inf.bradio_btn_stop =		USV_Control.USV_Control_Message[2].Emergency_Stop;		//������̨
		e_stop_inf.e_stop_cmd_inf.bd_btn_stop		=	USV_Control.USV_Control_Message[3].Emergency_Stop;		//����
		e_stop_inf.e_stop_cmd_inf.sp_btn_stop	=	USV_Control.USV_Control_Message[4].Emergency_Stop;		//������弱ͣ�ź�

		e_stop_inf.e_stop_cmd = e_stop_inf.e_stop_cmd_inf.dradio_btn_stop	||
				//			e_stop_inf.e_stop_cmd_inf.sradio_btn_stop	||
							e_stop_inf.e_stop_cmd_inf.bradio_btn_stop	||
							e_stop_inf.e_stop_cmd_inf.bd_btn_stop		||
							e_stop_inf.e_stop_cmd_inf.sp_btn_stop		;


	////����ϵͳͨѶ��ͣ�ź�
	//if(DSP_State_Msg.DSP_Heatbeat == 0 || USV_State.Conrtol_System_Msg.Intelligent_rudder==1)
	//	e_stop_logic = 1;
	//else
	//	e_stop_logic = 0;


	////����ϵͳͨѶ��ͣ
	//�ڲ���ͣ�ź�
	if(DSP_State_Msg.DSP_Heatbeat == 0 || USV_State.Conrtol_System_Msg.Intelligent_rudder==1)
		e_stop_inf.e_stop_logic = 1;
	else
		e_stop_inf.e_stop_logic = 0;

	////����ϵͳ����
	//if((SR_CAN_State.Rudder_Major_Failure & 0x1F)!= 0)
	//	e_stop_rudder_failure = 1;
	//else
	//	e_stop_rudder_failure = 0;

	//����ϵͳ����
	if((SR_CAN_State.Rudder_Major_Failure & 0x1F)!= 0)
		e_stop_inf.e_stop_rudder_failure = 1;
	else
		e_stop_inf.e_stop_rudder_failure = 0;



	////ͨѶ�쳣��ͣ
	//if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod == 2 && 
	//   USV_Control.USV_Control_Message[Radio_Sign].Navigation_Tsak_sign == 1)
	//   e_stop_comm = 0;
	//else
	//{
	//	if(Connect_Sign == 0)
	//	{
	//		e_stop_comm = 1;
	//	}
	//	else
	//	{
	//		e_stop_comm = 0;
	//	}
	//}


	//ͨѶ�쳣��ͣ
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod == 2 && 
		USV_State.USV_Sailing_Intel_Sign == 2)
		e_stop_inf.e_stop_comm = 0;
	else
	{
		if(Connect_Sign == 0)
		{
			e_stop_inf.e_stop_comm = 1;
		}
		else
		{
			e_stop_inf.e_stop_comm = 0;
		}
	}

	monitor_all_inf.usv_equ_state[23] = e_stop_cmd;
	monitor_all_inf.usv_equ_state[24] = e_stop_logic;
	monitor_all_inf.usv_equ_state[25] = e_stop_comm;
	monitor_all_inf.usv_equ_state[26] = e_stop_rudder_failure;

	monitor_all_inf.usv_equ_state[27] = USV_Control.USV_Control_Message[0].Emergency_Stop;	//���ֵ�̨��ͣ
	monitor_all_inf.usv_equ_state[28] = USV_Control.USV_Control_Message[1].Emergency_Stop;	//�����̨��ͣ
	monitor_all_inf.usv_equ_state[29] = USV_Control.USV_Control_Message[2].Emergency_Stop;	//������̨��ͣ
	monitor_all_inf.usv_equ_state[30] = USV_Control.USV_Control_Message[3].Emergency_Stop;	//������̨��ͣ
	monitor_all_inf.usv_equ_state[31] = USV_Control.USV_Control_Message[4].Emergency_Stop;	//��弱ͣ

	monitor_all_inf.monitor_sub_real_data[20] = (float)USV_Control.USV_Control_Message[0].Emergency_Stop;	//���ֵ�̨��ͣ
	monitor_all_inf.monitor_sub_real_data[21] = (float)USV_Control.USV_Control_Message[1].Emergency_Stop;
	monitor_all_inf.monitor_sub_real_data[22] = (float)USV_Control.USV_Control_Message[2].Emergency_Stop;
	monitor_all_inf.monitor_sub_real_data[23] = (float)USV_Control.USV_Control_Message[3].Emergency_Stop;
	monitor_all_inf.monitor_sub_real_data[24] = (float)USV_Control.USV_Control_Message[4].Emergency_Stop;

	//E_Stop = e_stop_cmd || e_stop_logic || e_stop_comm||e_stop_rudder_failure;

	e_stop_last = E_Stop;

	E_Stop =	e_stop_inf.e_stop_cmd			|| 
				e_stop_inf.e_stop_logic			|| 
				e_stop_inf.e_stop_comm			||
				e_stop_inf.e_stop_rudder_failure;

	//������־
	if(E_Stop == 1 && e_stop_last == 0)
	{
	//	SysLogMsgPost("��ͣ���� ��ť:%d,CAN�ж�:%d,ͨѶ�ж�:%d,��DSP�ϱ�:%d",
			// e_stop_inf.e_stop_cmd,
			// e_stop_inf.e_stop_logic,
			// e_stop_inf.e_stop_comm,
			// e_stop_inf.e_stop_rudder_failure
			// );

	//	SysLogMsgPost("��ť���� ���ֵ�̨:%d,������̨:%d,����:%d,�������:%d",
			// e_stop_inf.e_stop_cmd_inf.dradio_btn_stop,
			// e_stop_inf.e_stop_cmd_inf.bradio_btn_stop,	
			// e_stop_inf.e_stop_cmd_inf.bd_btn_stop,		
			// e_stop_inf.e_stop_cmd_inf.sp_btn_stop);

	//	SysLogMsgPost("�ڲ����� ����DSP����:%d,��DSP����:%d",
			// DSP_State_Msg.DSP_Heatbeat,
			// USV_State.Conrtol_System_Msg.Intelligent_rudder);			
	}

	if(E_Stop == 0 && e_stop_last== 1)
	{
	//	SysLogMsgPost("��ͣ�ָ�");
	}
}

void Connect_count()
{
	DSP_Connect_St();
	AIS_Connect_St();
	Dradio_Connect_St();
	BDS_Connect_St();
	Sradio_Connect_St();
	INS_Connect_St();
	MPC_Connect_St();
	Bradio_Connect_St();
	CAN_Connect_St();
}
void DSP_Connect_St()
{
	char errormsg[]="DSP is out of contact !";
	char rightmsg[]="DSP is contact !";
	
	if(DSP_State_Msg.DSP_Heatbeat==1)
	{
		task_time.uart0_task_50ms++;
		task_time.uart0_task_2s++;
		if(E_Stop_DSP==0)
		{
			Wrte_Error(rightmsg);
	//		SysLogMsgPost("MPC ͨѶ�ָ�");
			E_Stop_DSP=1;
		}

	}
	else
	{
		if(E_Stop_DSP==1)
		{
			E_Stop_DSP=0;
			Wrte_Error(errormsg);
		//	SysLogMsgPost("DSP ͨѶ�ж�");
		}
	}
	if(task_time.uart0_task_50ms>6)//60ms�ж�һ���Ƿ�ʧ����
	{
		task_time.uart0_task_50ms=0;
/*cxy*/		//��ʾ���Ķ�ʧ
	}
	if(task_time.uart0_task_2s>200)//��������δ�յ�����
	{
		//printf("��������δ�յ�DSP ��������,����ͣ�����ж���·\n");
		task_time.uart0_task_2s=0;
		DSP_State_Msg.DSP_Heatbeat=0;
		printf("DSP is not connect!\n");
	}
}
void AIS_Connect_St()
{
	if(AIS_Msg_St.AIS_Heatbeat==1)
		task_time.uart1_task_10s++;
	if(task_time.uart1_task_10s>1000)
	{
		AIS_Msg_St.AIS_Heatbeat=0;
		task_time.uart1_task_10s=0;
	}
}
void Dradio_Connect_St()
{
	char errormsg[]="Dradio is out of contact !";
	char rightmsg[]="Dradio is contact !";
	
	if(Dradio_Com1_Sign==1)
	{
		task_time.uart2_task_250ms++;
		task_time.uart2_task_1s++;
		if(E_Stop_Dradio==0)
		{
			Wrte_Error(rightmsg);	
		//	SysLogMsgPost("���ֵ�̨ ͨѶ�ָ�");
			E_Stop_Dradio=1;
		}

	}
	else
	{
		if(E_Stop_Dradio==1)
		{
			E_Stop_Dradio=0;
			Wrte_Error(errormsg);		
		//	SysLogMsgPost("���ֵ�̨ ͨѶ�ж�");
		}
	}

	if(task_time.uart2_task_1s>100)//����һ��δ�յ�����
	{
/*cxy*/			//�澯����־��λ
		task_time.uart2_task_1s=0;
	}
	if(task_time.uart2_task_250ms>500)//250ms��δ�յ�����ָ���������ָ��	//5sδ�յ�ָ�׼����ͣ
	{
		Dradio_Com1_Sign=0;//ʹ������ָ���־��λ�����ΪUSV_Control.USV_Num+1(ע��С��65535)
		task_time.uart2_task_250ms=0;
//		printf("Dradio is not connect!\n");
	}
			
}
void BDS_Connect_St()
{
//	if(BDS_Con_Sign==1)
//		task_time.uart3_task_100s++;
//	if(task_time.uart3_task_100s>10000)//����100��δ�յ�����
//	{
//		//printf("��������δ�յ�DSP ��������,����ͣ�����ж���·\n");
///*cxy*/			//USV_Emergency_Stop();//����ͣ��
///*cxy*/			//USV_Oil_Cut();//�ж���·
//		task_time.uart3_task_100s=0;
//		BDS_Con_Sign=0;
//		USV_Control.USV_Control_Message[3].Emergency_Stop=0;
//	}		
	//���������н�����Ϊ����ͨѶ�ж�
	if((BDS_Con_Sign == 1) &&(USV_State.USV_Sailing_Intel_Sign==3))
	{
		BDS_Con_Sign = 0;
	}
}

void Sradio_Connect_St()
{
	if(SpareDradio_Com1_Sign==1)
	{
		task_time.uart4_task_250ms++;
		task_time.uart4_task_1s++;
	}

	if(task_time.uart4_task_1s>100)//����һ��δ�յ�����
	{
		//�澯����־��λ
		task_time.uart4_task_1s=0;

	}
	if(task_time.uart4_task_250ms>250)//250ms��δ�յ�����ָ���������ָ��
	{
		SpareDradio_Com1_Sign=0;//ʹ�ÿ�����̨��־��λ�����ΪUSV_Control.USV_Num+1(ע��С��65535)
		task_time.uart4_task_250ms=0;
		USV_Control.USV_Control_Message[1].Emergency_Stop=0;
	}
}
void INS_Connect_St()
{
char errormsg[]="INS is out of contact !";
char rightmsg[]="INS is contact !";

	if(Smart_Navigation_St.Smart_Navigation_Heatbeat==1)
	{
		task_time.uart5_task_1s++;
		if(E_Stop_INS==0)
		{
			Wrte_Error(rightmsg);	
		//	SysLogMsgPost("INS ͨѶ�ָ�");
			E_Stop_INS=1;
		}
		
	}
	else
	{
		if(E_Stop_INS==1)
		{
			E_Stop_INS=0;
			Wrte_Error(errormsg);	
		//	SysLogMsgPost("INS ͨѶ�ж�");
		}
	}
	
	if(task_time.uart5_task_1s>100)//1000ms����һ�α��Ĳ��ж�
	{
		Smart_Navigation_St.Smart_Navigation_Heatbeat=0;
		Smart_Navigation_St.Serial_Light=0;
		task_time.uart5_task_1s=0;
		//ͣ�����ж���·
	}
}

void MPC_Connect_St()
{
	char errormsg[]="MPC is out of contact !";
	if(USV_RM_MSG.MPC_Heatbeat==1)
	{
		task_time.lan0_task_50ms++;
		task_time.lan0_task_2s++;	
	}
	if(task_time.lan0_task_50ms>6)
	{
		task_time.lan0_task_50ms=0;

	}
	if(task_time.lan0_task_2s>200)
	{
		task_time.lan0_task_2s=0;
		USV_RM_MSG.MPC_Heatbeat=0;
		memset((char *)&USV_RM_MSG,0,sizeof(USV_RM_MSG));
		//Wrte_Error(errormsg);		
		printf("MPC is outof connected!\n");
	}
}

void Bradio_Connect_St()
{
	char errormsg[]="Bradio is out of contact !";
	if(Bradio_Con_Sign==1)
	{
		task_time.lan1_task_250ms++;
		task_time.lan1_task_1s++;	
	}
	if(task_time.lan1_task_250ms>500)
	{
		task_time.lan1_task_250ms=0;
		Bradio_Con_Sign=0;
	
	}
	if(task_time.lan1_task_1s>100)
	{

		task_time.lan1_task_1s=0;
		USV_Control.USV_Control_Message[2].Emergency_Stop=0;

		Wrte_Error(errormsg);	
	}	
}

void CAN_Connect_St(void)
{
	char errormsg[]="Smart Rudder is out of contact !";
	char rightmsg[]="Smart Rudder is contact !";
	uint8 count,esign;
	if(task_time.CAN_DSP_50ms>200)//����2sδͨѶ
	{
		
		task_time.CAN_DSP_50ms=0;
	}
	if(task_time.CAN_UAV_50ms>200)//����2sδͨѶ
	{
		USV_State.Conrtol_System_Msg.UAV_platform=1;
		task_time.CAN_UAV_50ms=0;
		memset((uint8 *)&UAV_CAN_State.Platform_Hatch_spn520576,0,sizeof(UAV_CAN_State));	

	}		
	if(task_time.CAN_SR_50ms>200)//����2sδͨѶ
	{
		//cxyͣ��
		USV_State.Conrtol_System_Msg.Intelligent_rudder=1;
		task_time.CAN_SR_50ms=0;
		memset((uint8 *)&SR_CAN_State.Rudder_Angle_L_spn520766,0,sizeof(SR_CAN_State));
	//	SysLogMsgPost("��DSP ͨѶ�ж�");

	}
	if(task_time.CAN_POW_50ms>200)//����2sδͨѶ
	{
		USV_State.Conrtol_System_Msg.Power_management=1;
		task_time.CAN_POW_50ms=0;
		memset((uint8 *)&POW_CAN_State.Stable_Platform_Power_spn520958,0,sizeof(POW_CAN_State));
//			printf("CAN_Power is not connect!\n");
	}
	if(task_time.CAN_SP_50ms>200)//����2sδͨѶ
	{
		
		task_time.CAN_SP_50ms=0;
		Get_Control_Sign=0;
		SP_CON_Sign=0;
		SP_CAN_Count=0;	
		memset((uint8 *)&SP_CAN_Model_Con.Emergency_spn521192,0,sizeof(SP_CAN_Model_Con));
//		SysLogMsgPost("������� ͨѶ�ж�");

	}
	if(task_time.CAN_ST_PL_50ms>200)				//����2sδͨѶ
	{
		USV_State.Conrtol_System_Msg.Stabilized_platform=1;
		task_time.CAN_ST_PL_50ms=0;
		memset((uint8 *)&ST_PL_CAN_State.Stable_Platform_RL_spn521342,0,sizeof(ST_PL_CAN_State));
	}
	if(task_time.CAN_BAT_50ms>200)					//����2sδͨѶ
	{
		USV_State.Conrtol_System_Msg.Energy_management=1;
		task_time.CAN_BAT_50ms=0;
		memset((uint8 *)&BAT_CON_CAN_State.USV_Pow_spn521534,0,sizeof(BAT_CON_CAN_State));
//			printf("CAN_Energy is not connect!\n");

	}			
	if(task_time.CAN_Motor_L_10ms>100)				//����1sδͨѶ
	{
		USV_State.Conrtol_System_Msg.Engine_L=1;
		task_time.CAN_Motor_L_10ms=0;
		memset((uint8 *)&Motor_CAN_State.Motor_PGN61444_State[0].Torque_Mod_spn899,0,sizeof(Motor_CAN_State));
	}
	if(task_time.CAN_Motor_R_10ms>100)				//����1sδͨѶ
	{
		USV_State.Conrtol_System_Msg.Engine_R=1;
		task_time.CAN_Motor_R_10ms=0;
		memset((uint8 *)&Motor_CAN_State.Motor_PGN61444_State[1].Torque_Mod_spn899,0,sizeof(Motor_CAN_State));
	}

	task_time.CAN_DSP_50ms++;
	if(USV_State.Conrtol_System_Msg.UAV_platform==0)
		task_time.CAN_UAV_50ms++;
	if(USV_State.Conrtol_System_Msg.Intelligent_rudder==0)
	{
		task_time.CAN_SR_50ms++;	
		if(E_Stop_SmartRudder==0)
		{
			Wrte_Error(rightmsg);			
			E_Stop_SmartRudder=1;
		}
	}

	else
	{
		if(E_Stop_SmartRudder==1)
		{
			E_Stop_SmartRudder=0;
			Wrte_Error(errormsg);			
		}
//		printf("%d-%d-%d-Send_Estop!\n",SR_CAN_State.Config_Answer_spn520778,USV_Control.USV_Control_Message[Radio_Sign].Emergency_Stop,E_Stop);		
		for(count=0;count<4;count++)
		{
			if(1==USV_Control.USV_Control_Message[Radio_Sign].Emergency_Stop)
			{
				esign=1;
				break;
			}
			else
				esign=0;
		}
		if((SR_CAN_State.Config_Answer_spn520778==0)&&(0==esign))//�ȴ������·��ɹ�
		{
			//Get_CAN_SR_Engine_Config();
			//Get_CAN_SR_Gear_Config();
			//Get_CAN_SR_Gear2_Config();
			//Get_CAN_SR_Rudder_Config();
			//Get_CAN_SR_Rudder2_Config();
			//Get_CAN_SR_Rudder3_Config();
		}

	}
	if(SP_CAN_Model_Con.System_Heatbeat==1)		
		task_time.CAN_SP_50ms++;
	if(USV_State.Conrtol_System_Msg.Power_management==0)	
		task_time.CAN_POW_50ms++;
	if(USV_State.Conrtol_System_Msg.Stabilized_platform==0)
		task_time.CAN_ST_PL_50ms++;
	if(USV_State.Conrtol_System_Msg.Energy_management==0)
		task_time.CAN_BAT_50ms++;
	if(USV_State.Conrtol_System_Msg.Engine_L==0)
		task_time.CAN_Motor_L_10ms++;
	if(USV_State.Conrtol_System_Msg.Engine_R==0)
		task_time.CAN_Motor_R_10ms++;		
}
void Wrte_Error(char *ErrorMsg)
{
	uint16 year=0;
	uint8	month=0,date=0,hour=0,minute=0,second=0;
	FILE *pFile;
#ifdef WINNT
	pFile=fopen("../../cfg/USVError.cfg", "a+");
#else
	pFile=fopen("../cfg/USVError.cfg", "a+");
#endif
	if(pFile == NULL)
	{
		printf("open USVError.cfg error\n");
		return ;
	}
	year=2000+Smart_Navigation_St.USV_Year;
	month=Smart_Navigation_St.USV_Month;
	date=Smart_Navigation_St.USV_Date;
	hour=8+Smart_Navigation_St.USV_Hour;
	minute=Smart_Navigation_St.USV_Minute;
	second=Smart_Navigation_St.USV_Second;
	if(hour>=24)
	{
		hour-=24;
		date+=1;
	}
	fprintf(pFile,"%.4d-%.2d-%.2d %.2d:%.2d:%.2d  %s\n",year,month,date,hour,minute,second,ErrorMsg);	
	fclose(pFile);
	return;
}
//��ʼ������
int init(void)
{
	int		initsign;
	int8	ret_val;
	
	initsign=GetName();		
	Msg_Init();	

	ret_val=read_usv_cfg();
	if(ret_val==FALSE){
		printf("read usv cfg failed! set default parameters!\n");
		set_usv_param_default();			
	}

	ret_val=read_usv_state();
	if(ret_val==FALSE){
		printf("read usv_state wrong!\n");
	}
	
	
	if(read_PeriMcuCfg_file()==FALSE)		
	{
		//init_flash_default();
		printf("read PeriMcuCfg failed \n");
	}

	if (iniWarnModel() == FALSE)			
	{
		printf("Warn init failed\n");
	}

	write_usv_state();

	//set_ip();							
	//GPIO_Init();						
	initUn237Gpio();					


	sockid_test=CreateUDPSocket((uint16)TEST_UDP_SEND_PORT);	
	if(sockid_test<0)
			printf("create test sockid failed!\n");

	#ifndef WINNT
	sigset_t signal_mask;
	sigemptyset (&signal_mask);
	sigaddset (&signal_mask, SIGPIPE);
	#endif
//	Watch_Dog();cxy

	AisMsgInit();

	printf("Begin initialize Nanomsg and ZMQ!\n");
	SysPubMsgInit();

	printf("Begin initialize system log!\n");
	SysLogInit();


	switch(ship_version){
	case 1:
	break;
	case 2: 
	break;

	}

	pAutoReturnInst = new CAutoReturn(AUTO_RETURN_CFG_FILE_NAME);	

	// Initialize  new module
	boat.setup();

	initWatchDog();
	return initsign;
}
void  calcCRC(uint16* pSeed, char* buf, int len)
{
	uint16 temp;
	int  i,j;
	char LSB;

	temp = *pSeed;       //0xFFFF;
	for(i = 0;i <len;i++)
	{
		temp = temp ^ *(buf + i);
		
		for(j = 0;j < 8;j++)
		{
			LSB = temp & 0x0001;
			temp = temp >> 1;
			if(LSB)
				temp = temp ^ 0xA001;
		}
	}
	*pSeed=temp;
}

uint16 CalcFileCRC(char* fname)
{
	FILE *pFile;
	int n;
	uint16 seed = 0;
	char buffer[1024];

	memset(buffer,0,sizeof(buffer));
//	bzero(buffer, sizeof(buffer));		
	
	if ( (pFile = fopen(fname, "rb") ) == NULL)
	{
		printf("open S99USV_ARM error\n");				
	     return seed;		
	}
	
	while( !feof( pFile ) )
	{
		n=fread(buffer,1,sizeof(buffer), pFile );

		calcCRC(&seed,buffer,n);
	}	

	fclose( pFile );

	return seed;
}


int GetName(void)
{
	uint16 ArmAppCrc;
	ArmAppCrc= CalcFileCRC("/home/root/run_prog/S99USV_ARM");
    	 sprintf_usv(Version[0],"ARM_V0.10_R255_%04XH",ArmAppCrc );		//����İ汾��
		ARMVersion_sign=1;
	return 0;
}


void set_ip()
{
	char cmd[100];
	uint16 offset;

	memset(cmd,0,sizeof(cmd));

	ip_address.local_ip0[0]=192;
	ip_address.local_ip0[1]=168;
	ip_address.local_ip0[2]=1;
	ip_address.local_ip0[3]=64;

	ip_address.local_netmask0[0]=255;
	ip_address.local_netmask0[1]=255;
	ip_address.local_netmask0[2]=255;
	ip_address.local_netmask0[3]=0;

	ip_address.local_gw0[0]=192;
	ip_address.local_gw0[1]=168;
	ip_address.local_gw0[2]=1;
	ip_address.local_gw0[3]=255;

	ip_address.local_nameserver0[0]=192;
	ip_address.local_nameserver0[1]=168;
	ip_address.local_nameserver0[2]=128;
	ip_address.local_nameserver0[3]=251;	
	
	ip_address.local_ip1[0]=172;
	ip_address.local_ip1[1]=1;
	ip_address.local_ip1[2]=1;
	ip_address.local_ip1[3]=199;	
	
	ip_address.local_netmask1[0]=255;
	ip_address.local_netmask1[1]=255;
	ip_address.local_netmask1[2]=255;
	ip_address.local_netmask1[3]=0;

	ip_address.local_gw1[0]=172;
	ip_address.local_gw1[1]=1;
	ip_address.local_gw1[2]=1;
	ip_address.local_gw1[3]=10;
	
	ip_address.local_nameserver1[0]=192;
	ip_address.local_nameserver1[1]=168;
	ip_address.local_nameserver1[2]=128;
	ip_address.local_nameserver1[3]=251;	
#ifndef WINNT	
	sprintf_usv(cmd,"ifconfig eth0 down");		
	system(cmd); 
	sleep_1(2000);//wait lan up is ok
	offset=sprintf_usv(cmd,"/sbin/ifconfig eth0 %d.%d.%d.%d",ip_address.local_ip0[0],ip_address.local_ip0[1],ip_address.local_ip0[2],ip_address.local_ip0[3]);
	sprintf_usv(&cmd[offset]," netmask %d.%d.%d.%d",ip_address.local_netmask0[0],ip_address.local_netmask0[1],ip_address.local_netmask0[2],ip_address.local_netmask0[3]);
    	system(cmd);
//	sprintf_usv(cmd,"/sbin/route add default gw %d.%d.%d.%d eth0",ip_address.local_gw0[0],ip_address.local_gw0[1],ip_address.local_gw0[2],ip_address.local_gw0[3]);
//    	system(cmd);		
	sprintf_usv(cmd,"ifconfig eth0 up");		//\u5f00\u4ee5\u592a\u7f51
	system(cmd); 
	sleep_1(2000);//wait lan up is ok
		
	sprintf_usv(cmd,"ifconfig eth1 down");		
    	system(cmd);
	sleep_1(2000);//wait lan up is ok		
	offset=sprintf_usv(cmd,"/sbin/ifconfig eth1 %d.%d.%d.%d",ip_address.local_ip1[0],ip_address.local_ip1[1],ip_address.local_ip1[2],ip_address.local_ip1[3]);
	sprintf_usv(&cmd[offset]," netmask %d.%d.%d.%d",ip_address.local_netmask1[0],ip_address.local_netmask1[1],ip_address.local_netmask1[2],ip_address.local_netmask1[3]);
    	system(cmd);
	sprintf_usv(cmd,"/sbin/route add default gw %d.%d.%d.%d eth1",ip_address.local_gw1[0],ip_address.local_gw1[1],ip_address.local_gw1[2],ip_address.local_gw1[3]);
    	system(cmd);
	sprintf_usv(cmd,"ifconfig eth1 up");		//\u5f00\u4ee5\u592a\u7f51
	system(cmd); 
	sleep_1(2000);//wait lan up is ok
#endif
}
void Msg_Init(void)
{
	int icount;
	memset((uint8 *)&USV_Control,0,sizeof(USV_Control));
	memset((uint8 *)&USV_Sailing,0,sizeof(USV_Sailing));
	memset((uint8 *)&USV_State,0,sizeof(USV_State));
	memset((uint8 *)&USV_RM_MSG,0,sizeof(USV_RM_MSG));
	//ͼ����̨�����豸��ϸ��Ϣ�ṹ���ʼ��
	memset((uint8 *)&Motor_Detail_St,0,sizeof(Motor_Detail_St));
	memset((uint8 *)&Rudder_Detail_St,0,sizeof(Rudder_Detail_St));
	memset((uint8 *)&Stable_Platform_St,0,sizeof(Stable_Platform_St));
	memset((uint8 *)&UAV_Detail_St,0,sizeof(UAV_Detail_St));
	memset((uint8 *)&MCU_St,0,sizeof(MCU_St));
	memset((uint8 *)&Hull_St,0,sizeof(Hull_St));
	memset((int8 *)&AIS_Msg_St,0,sizeof(AIS_Msg_St));
	memset((uint8 *)&Smart_Navigation_St,0,sizeof(Smart_Navigation_St));
	memset((uint8 *)&Panel_Control_St,0,sizeof(Panel_Control_St));
	memset((uint8 *)&Energy_Control_St,0,sizeof(Energy_Control_St));
	memset((uint8 *)&USV_Check_Collision[0].User_ID,0,sizeof(USV_Check_Collision[AIS_Collision_Num]));
	memset((uint8 *)&Fault_Code[0],0,sizeof(Fault_Code[2]));
	memset((uint8 *)&Point_Return[0],0,sizeof(Point_Return[2]));
	
	memset((uint8 *)&SR_Config_Msg,0,sizeof(SR_Config_Msg));
	memset((uint8 *)&Dradio_Config,0,sizeof(Dradio_Config));
	memset ( (uint8 *)&vPID,0,sizeof(vPID));
	memset((uint8 *)&vSpd,0,sizeof(vSpd));
	memset ( (uint8 *)&rPID,0,sizeof(rPID));
	memset((uint8 *)&rSrd,0,sizeof(rSrd));

	memset((int8 *)&task_time,0,sizeof(task_time));		//������ʱ����0
	memset((int8 *)&monitor_all_inf,0,sizeof(MONITOR_ALL_INF_STRUCT));			//���߼�����Ϣ

	Dradio_Com1_Sign=0;
	Dradio_Com2_Sign=0;
	SpareDradio_Com1_Sign=0;
	SpareDradio_Com2_Sign=0;
	Bradio_Con_Sign=0;
	BDS_Con_Sign=0;
	Rudder_Con_Angle=0;
	Rudder_Zero_Angle=0;
	Rudder_Count=0;
	AIS_Send_Sign=0;
	SP_CON_Sign=0;
	lau_old=0.0;	
	lnu_old=0.0;
	Mileage=0;
	Radio_Sign=0;
	Radio_Sign_old=0;
	Sailing_Sign=0;
	Sailing_Cnt_Old=1;
	Speed_EXP=0.0;
	Heading_EXP=0.0;
	Collision_Sign=0;
	E_Stop=0;
	Time_Sign=0;
	Accelerator_L=0;
	Accelerator_R=0;
	Rudder_L=0;
	Rudder_R=0;
	Gear_L=0;
	Gear_R=0;
	Accelerator_Coefficient_L=0.0;
	Accelerator_Coefficient_R=0.0;
	Rudder_Coefficient_L=0.0;
	Rudder_Coefficient_R=0.0;
	Gear_Coefficient_L_F=0.0;
	Gear_Coefficient_L_B=0.0;
	Gear_Coefficient_R_F=0.0;
	Gear_Coefficient_R_B=0.0;
	E_Stop_INS=0;
	E_Stop_DSP=0;
	E_Stop_Dradio=0;
	E_Stop_SmartRudder=0;
	sign=0;
	Get_Control_Sign=0;
	count_Estop=0;
	SP_CAN_Count=0;
	Connect_Sign=1;
	 LAN0_Fd=0;
	 LAN1_Fd=0;
	 Heartbeat_Num=0;
	 ARMVersion_sign=0;
	 DSPVersion_sign=0;
	 MPCVersion_sign=0;
	 UAVVersion_sign=0;
	 SRVersion_sign=0;
	 POWVersion_sign=0;
	 SPVersion_sign=0;
	 STVersion_sign=0;
	 BATVersion_sign=0;
	 e_stop_init();		//��ͣ�߼���ʼ��
	for(icount=0;icount<5;icount++)
	{
		USV_Control.USV_Control_Message[icount].Dradio_USV_Stable_Platform.Stable_Platform_AT=90;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Stable_Platform.Stable_Platform_RL=90;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Accelerator_Left=100;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Accelerator_Right=100;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Rudder_Angle_Left=100;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Rudder_Angle_Right=100;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Gear_Left=100;
		USV_Control.USV_Control_Message[icount].Dradio_USV_Drive.Gear_Right=100;		
	}
	return	;
}

int GPIO_Init()
{
	char direction1[]="out",value1[]="1",direction2[]="0",value2[]="0";
	char index[]="204";
	
	if(set_direction(index, direction1)<0)
	{
		printf("set direction error!\n");
		return 0;
	}
	if(2==Dradio_Config.Basic_Cfg.Propeller_Type)
	{
		if(set_value(index, value2)<0)
		{
			printf("set value error!\n");
			return 0;
		}
	}
	else if (3==Dradio_Config.Basic_Cfg.Propeller_Type)
	{
		if(set_value(index, value1)<0)
		{
			printf("set value error!\n");
			return 0;
		}
	}
	if(get_direction(index,direction2)<0)
	{
		printf("get direction error!\n");
		return 0;
	}
	if(get_value(index, value2)<0)
	{
		printf("get value error!\n");
		return 0;
	}
//	printf("direction=%s\n",direction2);
//	printf("value=%s\n",value2);
	return 1;
}




void Watch_Dog()
{
#ifndef WINNT
int ret = 0;
int timeout=10;

	//Open watchdog device
	Watch_fd=open("/dev/watchdog",O_WRONLY);
	if (Watch_fd==-1)
	{
		perror("watchdog");
		return ;
	}
//	printf("set timeout value to %d seconds\n", timeout);

    	ret= ioctl (Watch_fd, WDIOC_SETTIMEOUT, timeout);
    	if (ret)
        	printf ("\nWatchdog Timer : WDIOC_SETTIMEOUT failed");
#endif
}

void exit_deal(void)
{	//ins_zmq_close();
	disableWatchDog();
	printf("**********************Progammer exit,disable watchdog!***********************\n");	
	return	;
}
void sleep_1(int delay_ms)
{
#ifndef WINNT
	long int delay;
	delay=delay_ms*1000;
	usleep(delay);
#else
	Sleep(delay_ms);
#endif
	return  ;
}


void *radar_message_send_thread(void *aa)   
{ 

  for(;;){		
		AP::radar_message()->update_send();	
		sleep_1(50);			
	}
	return ((void *)0);
}


void *radar_message_receive_thread(void *aa)   
{ 

  for(;;){		
		AP::radar_message()->update_receive();	
		sleep_1(10);				
	}
	return ((void *)0);
}


void creat_thread(void)
{
unsigned char Res;


threadnode *pUart5_Ins_Rec;
threadnode *pUart_IRTK_Rec;
threadnode *pCAN0_ARM_Rec;
threadnode *pusv_monitor_thread;
threadnode *pLan_deal_ctrlCenter; 

threadnode *pdockCommunicationUSVRun; 

threadnode *pControl_main_thread;
threadnode *pComm_main_thread;
//threadnode *pZmqGetObstacleThread;
threadnode *pSamplingCommThread;

threadnode *auto_control_thread;
threadnode *radar_message_receive;
threadnode *radar_message_send;


	threadnode_ids_init(&thread_ids); 

	pusv_monitor_thread=NULL;			    	
	Res=CreateNewThread(pusv_monitor_thread,usv_monitor_thread,4);	
	if(Res)
		printf("creat usv_monitor_thread fail???\n");

	// pZmqGetObstacleThread = NULL;
	// Res=CreateNewThread(pZmqGetObstacleThread,zmq_getObstacles,4);
	// if(Res)
	// 	printf("creat zmq_getObstacles fail???\n");

	pSamplingCommThread = NULL;
	Res=CreateNewThread(pSamplingCommThread,samplingComm,4);
	if(Res)
		printf("creat samplingComm fail???\n");


	pUart5_Ins_Rec = NULL;
	Res = CreateNewThread(pUart5_Ins_Rec, uart_deal_ins, 3);
	if (Res)
		printf("Uart5_Ins_Rec creat error!\n"); 


	pUart_IRTK_Rec = NULL;
	Res = CreateNewThread(pUart_IRTK_Rec, uartRTKMsgProcess, 3);
	if (Res)
		printf("pUart_IRTK_Rec creat error!\n");



	pLan_deal_ctrlCenter = NULL;
	Res = CreateNewThread(pLan_deal_ctrlCenter, lan_deal_CtrlCenter, 5);
	if (Res)
		printf("Lan_deal_ctrlCenter error!\n");


	pdockCommunicationUSVRun = NULL;
	Res = CreateNewThread(pdockCommunicationUSVRun, dockCommunicationUSVRun, 5);
	if (Res)
	{
		printf("dockCommunicationUSVRun error!\n");
	}


	pCAN0_ARM_Rec=NULL;				    	
	Res=CreateNewThread(pCAN0_ARM_Rec,can0_rec_thread,2);	
	if(Res)
		printf("Lan1_Bradio_Rec creat error!\n");	

	pControl_main_thread = NULL;
	Res=CreateNewThread(pControl_main_thread,control_main_thread,4);	
	if(Res)
		printf("control_main_thread creat error!\n");	


	radar_message_receive = NULL;
	Res = CreateNewThread(radar_message_receive,radar_message_receive_thread,4);
	if(Res)
		printf("creat radar_message_receive fail???\n");
	
	radar_message_send = NULL;
	Res = CreateNewThread(radar_message_send,radar_message_send_thread,5);
	if(Res)
		printf("creat radar_message_send fail???\n");

	auto_control_thread = NULL;
	Res = CreateNewThread(auto_control_thread,runControlOperation,2);
	if(Res)
		printf("creat auto control thread fail???\n");	

	pComm_main_thread = NULL;
	Res=CreateNewThread(pComm_main_thread,comm_main_thread,5);	
	if(Res)
		printf("comm_main_thread creat error!\n");

	return	;
}

void Get_Return_Point(void)
{

	Point_Return[0].Waypoint_Latitude_Sign=(USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Sign+0x01)&0x01;		//����ά�ȱ�־��0��γ1��γ
	Point_Return[0].Waypoint_Longitude_Sign=(USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Sign+0x01)&0x01;		//���㾭�ȱ�־��0����1����
	Point_Return[0].Waypoint_Latitude_Degree=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Degree;				//ά�ȶ�
	Point_Return[0].Waypoint_Latitude_Minute=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Minute;				//ά�ȷ�
	Point_Return[0].Waypoint_Latitude_Second=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Second;				//ά����
	Point_Return[0].Waypoint_Latitude_Decimal=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Decimal;				//������Ϣά��ֵ��С��λ
	Point_Return[0].Waypoint_Longitude_Degree=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Degree;				//���ȶ�
	Point_Return[0].Waypoint_Longitude_Minute=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Minute;				//���ȷ�
	Point_Return[0].Waypoint_Longitude_Second=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Second;				//������
	Point_Return[0].Waypoint_Longitude_Decimal=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Decimal;				//����С��

	Point_Return[1].Waypoint_Latitude_Sign=(Smart_Navigation_St.Latitude_Sign_St&0x01);					//���˴�λ�þ��ȱ�־
	Point_Return[1].Waypoint_Longitude_Sign=(Smart_Navigation_St.Longitude_Sign_St&0x01);				//���˴�λ��ά�ȱ�־
	Point_Return[1].Waypoint_Latitude_Degree=Smart_Navigation_St.USV_Latitude_Degree;					//���˴�λ�þ��ȶ�
	Point_Return[1].Waypoint_Latitude_Minute=Smart_Navigation_St.USV_Latitude_Minute;					//���˴�λ�þ��ȷ�
	Point_Return[1].Waypoint_Latitude_Second=Smart_Navigation_St.USV_Latitude_Second;					//���˴�λ�þ�����
	Point_Return[1].Waypoint_Latitude_Decimal=Smart_Navigation_St.USV_Latitude_Decimal_2;				////���˴�λ�þ�����С��
	Point_Return[1].Waypoint_Longitude_Degree=Smart_Navigation_St.USV_Longitude_Degree;					////���˴�λ��ά�ȶ�
	Point_Return[1].Waypoint_Longitude_Minute=Smart_Navigation_St.USV_Longitude_Minute;
	Point_Return[1].Waypoint_Longitude_Second=Smart_Navigation_St.USV_Longitude_Second;
	Point_Return[1].Waypoint_Longitude_Decimal=Smart_Navigation_St.USV_Longitude_Decimal_2;	
}

// ############################################################################
//			��̫�����ʹ������� ����
// ##############################################################################
int8 test_udp_send( uint8 * lpBuf, int16 iLen,int16 sockid)
{
	int iSendLen;
	int DestPort;
	struct sockaddr_in server ;

	DestPort =TEST_UDP_SEND_PORT;
	server.sin_family = AF_INET;
	server.sin_port = htons(DestPort);
	server.sin_addr.s_addr =inet_addr("172.1.1.202");
	memset(server.sin_zero, 0, 8);

	iSendLen=sendto(sockid, (int8 *)lpBuf,iLen, 0,(struct sockaddr*)&server, sizeof(server)); 
	if (iSendLen != iLen )
	{
		return FALSE;
	}
	else
	{	  
		return TRUE;
	}
}



uint8 Engine_Run_L_old = 0;
uint8 Engine_Run_R_old = 0;

//����DSP������ת�ٿ���ָ��
void Send_Accelerator(uint16 Heartbeat_Num)
{
uint8	Accelerator_Con[Accelerator_Con_Num];//RAM����DSP�����ſ���ָ��

	memset(Accelerator_Con,0,Accelerator_Con_Num);
	uint8	CRC;
	fireOndelay();	//���������ʱ���ش���
	CRC=0;
	Accelerator_Con[0]='R';
	Accelerator_Con[1]='D';
	Accelerator_Con[2]=',';
	Accelerator_Con[4]=',';
	Accelerator_Con[5]=((Heartbeat_Num&0xff00)>>8);
	Accelerator_Con[6]=(Heartbeat_Num&0x00ff);
	Accelerator_Con[7]=',';
	Accelerator_Con[8]=0x20;
	Accelerator_Con[9]=',';	
	Accelerator_Con[10]=(Accelerator_L&0xff00)>>8;
	Accelerator_Con[11]=(Accelerator_L&0x00ff);
	Accelerator_Con[12]=(Accelerator_R&0xff00)>>8;
	Accelerator_Con[13]=(Accelerator_R&0x00ff);	
	Accelerator_Con[14]=USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L;				//������������Դָ��
	Accelerator_Con[14]+=(USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_R)<<2;			//�ҷ�������Դָ��
	
	Accelerator_Con[14]+=(USV_Control.USV_Control_Message[Radio_Sign].Oil_Con)<<4;				//��·����
	Accelerator_Con[14]+=(E_Stop)<<6;//���ϼ�ͣ��Ӳ����
	Accelerator_Con[15]=USV_State.Engine_run;
//	printf("USV_State.Engine_run=%d\n",USV_State.Engine_run);	

	Accelerator_Con[15]+=(SR_Config_Msg.Motor_Fire_Time_spn520226)<<2;	
//	printf("Accelerator_Con=%d\n",Accelerator_Con[14]);	
	if(DSPVersion_sign==0)
		Accelerator_Con[15]+=0x20;
	GetCheck(&CRC,Accelerator_Con,sizeof(Accelerator_Con));
	Accelerator_Con[30]='*';
	Accelerator_Con[31]=CRC;

	//���ڷ���DSP���ſ���
	write_uart(UART0_Fd, (int8 *)&Accelerator_Con[0], Accelerator_Con_Num);


	//��ʷ��־
	
	//	USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L



	//
	if(Engine_Run_L_old != 0x01 && USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L == 0x01)	//�������������
	{
	//	SysLogMsgPost("�������������,Ϩ����:%d(0-���ֵ�̨,1-���������̨ 2-������̨ 3-������̨  4-���)",Radio_Sign);
	}
	if(Engine_Run_R_old != 0x01 && USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_R == 0x01)    //�ҷ��������
	{
	//	SysLogMsgPost("�ҷ��������,Ϩ����:%d(0-���ֵ�̨,1-���������̨ 2-������̨ 3-������̨  4-���)",Radio_Sign);
	}

	if(Engine_Run_L_old != 0x02 && USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L == 0x02)	//����������Ϩ��
	{
	//	SysLogMsgPost("����������Ϩ��,Ϩ����Դ:%d(0-���ֵ�̨,1-���������̨ 2-������̨ 3-������̨  4-���)",Radio_Sign);
	}
	if(Engine_Run_R_old != 0x02 && USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_R == 0x02)    //�ҷ�����Ϩ��
	{
	//	SysLogMsgPost("�ҷ�����Ϩ��,Ϩ����Դ:%d(0-���ֵ�̨,1-���������̨ 2-������̨ 3-������̨  4-���)",Radio_Sign);
	}




	Engine_Run_L_old = USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L;
	Engine_Run_R_old = USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_R;






#ifdef WINNT
	//printf("%d\n",USV_Control.USV_Control_Message[3].Engine_Run_L);

#endif



#ifdef	debug_print
	if(ret<0)
		printf("Send DSP accelerator speed error!\n");	
#endif
	return	;
}
//����USV״̬
void Update_USV_State(void)
{
	uint8	hour,date;
	uint16 Fuel_ConRate=0;
//	if(==0)//�޲���
//	USV_State.Dradio_USV_Model_State.Sailing_Model_St=
	USV_State.USV_Num=USV_NUM;
	//������Ϣ
	USV_State.Dradio_USV_Sailing_State.USV_Speed=Smart_Navigation_St.USV_Speed;
	USV_State.Dradio_USV_Sailing_State.USV_Heading=Smart_Navigation_St.USV_Heading;
	USV_State.Dradio_USV_Sailing_State.USV_Pitch=Smart_Navigation_St.USV_Pitch;
	USV_State.Dradio_USV_Sailing_State.USV_Roll=Smart_Navigation_St.USV_Roll;
	USV_State.Dradio_USV_Sailing_State.USV_Heave=(Smart_Navigation_St.USV_Heave)/5;
	//����ģʽ
	USV_State.Dradio_USV_Model_State.Latitude_Sign_St=(Smart_Navigation_St.Latitude_Sign_St&0x01);
	USV_State.Dradio_USV_Model_State.Longitude_Sign_St=(Smart_Navigation_St.Longitude_Sign_St&0x01);
	USV_State.Dradio_USV_Model_State.Sailing_Model_St=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Differential_Mod==1)	
		USV_State.Dradio_USV_Model_State.Differential_Model_St=1;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Differential_Mod==2)	
		USV_State.Dradio_USV_Model_State.Differential_Model_St=0;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Set_Return_Point_Mod==1)
		USV_State.Dradio_USV_Model_State.Set_Return_Point_Model_St=1;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Set_Return_Point_Mod==2)
		USV_State.Dradio_USV_Model_State.Set_Return_Point_Model_St=0;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Speed_Constant_Mod==1)
		USV_State.Dradio_USV_Model_State.Speed_Constant_Model_St=1;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Speed_Constant_Mod==2)
		USV_State.Dradio_USV_Model_State.Speed_Constant_Model_St=0;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Direction_Constant_Mod==1)
		USV_State.Dradio_USV_Model_State.Direction_Constant_Model_St=1;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Direction_Constant_Mod==2)
		USV_State.Dradio_USV_Model_State.Direction_Constant_Model_St=0;

	USV_State.Dradio_USV_Model_State.translation_mode_St = USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_mode;
	USV_State.Dradio_USV_Model_State.translation_command_St = USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_command;
	USV_State.Dradio_USV_Model_State.rotation_mode_St = USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_mode;
	USV_State.Dradio_USV_Model_State.rotation_command_St = USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_command;
	
	//λ����Ϣ
	USV_State.Dradio_USV_Location.USV_Latitude_Degree=Smart_Navigation_St.USV_Latitude_Degree;
	USV_State.Dradio_USV_Location.USV_Latitude_Minute=Smart_Navigation_St.USV_Latitude_Minute;
	USV_State.Dradio_USV_Location.USV_Latitude_Second=Smart_Navigation_St.USV_Latitude_Second;
	USV_State.Dradio_USV_Location.USV_Latitude_Decimal=Smart_Navigation_St.USV_Latitude_Decimal_2;
	USV_State.Dradio_USV_Location.USV_Longitude_Degree=Smart_Navigation_St.USV_Longitude_Degree;
	USV_State.Dradio_USV_Location.USV_Longitude_Minute=Smart_Navigation_St.USV_Longitude_Minute;
	USV_State.Dradio_USV_Location.USV_Longitude_Second=Smart_Navigation_St.USV_Longitude_Second;
	USV_State.Dradio_USV_Location.USV_Longitude_Decimal=Smart_Navigation_St.USV_Longitude_Decimal_2;

	//����ϵͳ��Ϣ
	USV_State.Dradio_USV_Drive_State.Accelerator_Left_St=Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190;
	USV_State.Dradio_USV_Drive_State.Accelerator_Right_St=Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190;
//	USV_State.Dradio_USV_Drive_State.Accelerator_Left_St=SR_CAN_State.Motor_Speed_L_spn520770;
//	USV_State.Dradio_USV_Drive_State.Accelerator_Right_St=SR_CAN_State.Motor_Speed_R_spn520771;//cxy
	USV_State.Dradio_USV_Drive_State.Gear_Left_St=SR_CAN_State.Motor_Gear_L_spn520768;
	USV_State.Dradio_USV_Drive_State.Gear_Right_St=SR_CAN_State.Motor_Gear_R_spn520769;
	USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St=(((SR_CAN_State.Rudder_Angle_L_spn520766)>>1)-0x7D);
	USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St=(((SR_CAN_State.Rudder_Angle_R_spn520767)>>1)-0x7D);
	//����
	#ifdef WINNT
	USV_State.USV_Oil = 80;
	#else
	USV_State.USV_Oil=	BAT_CON_CAN_State.Oil_R_spn521565;
	USV_State.USV_Oil+=	BAT_CON_CAN_State.Oil_L_spn521559;
	USV_State.USV_Oil=USV_State.USV_Oil;
	#endif

	//���״̬
	USV_State.Dradio_USV_Battery.Battery_Left=BAT_CON_CAN_State.Battery_Left_spn521554;
	USV_State.Dradio_USV_Battery.Battery_Right=BAT_CON_CAN_State.Battery_Right_spn521560;
	USV_State.Dradio_USV_Battery.Battery_TEMP_Left=BAT_CON_CAN_State.Battery_TEMP_L_spn521555;
	USV_State.Dradio_USV_Battery.Battery_TEMP_Right=BAT_CON_CAN_State.Battery_TEMP_R_spn521561;
	//��������״̬
	//USV_State.USV_Sailing_Intel_Sign�Ѹ���
	//�豸��Դ״̬
	USV_State.Dradio_USV_Device_State.UAV_Power_St=POW_CAN_State.UAV_Power_spn520959;
	USV_State.Dradio_USV_Device_State.Stable_Platform_Power_St=POW_CAN_State.Stable_Platform_Power_spn520958;
	USV_State.Dradio_USV_Device_State.Camera_ahead_Power_St=POW_CAN_State.Camera_ahead_Power_spn520966;
	USV_State.Dradio_USV_Device_State.D_radar_Power_St=POW_CAN_State.D_radar_Power_spn520962;
	USV_State.Dradio_USV_Device_State.Camera_main_Power_St=POW_CAN_State.Camera_main_Power_spn520963;
	USV_State.Dradio_USV_Device_State.Horn_Power_St=POW_CAN_State.Horn_Power_spn520960;
	USV_State.Dradio_USV_Device_State.Searchlight_Power_St=POW_CAN_State.Searchlight_Power_spn520965;
	USV_State.Dradio_USV_Device_State.Camera_lesser_Power_St=POW_CAN_State.Camera_lesser_Power_spn520967;
	USV_State.Dradio_USV_Device_State.Navigationlight_Power_St=POW_CAN_State.Navigationlight_Power_spn520961;
	USV_State.Dradio_USV_Device_State.Camera_tail_Power_St=POW_CAN_State.Camera_tail_Power_spn520968;
	//����¶ȸ澯
	USV_State.Dradio_USV_Battery_Alarm.Battery_Left_Charging=BAT_CON_CAN_State.Battery_Charge_L_spn521535;
	USV_State.Dradio_USV_Battery_Alarm.Battery_Right_Charging=BAT_CON_CAN_State.Battery_Chager_R_spn521536;
	USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_L=BAT_CON_CAN_State.Battery_TEMP_Alarm_L_spn521537;
	USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_R=BAT_CON_CAN_State.Battery_TEMP_ALarm_R_spn521538;
	//��������cxy
	USV_State.Dradio_USV_Fire.Outfire_St=DSP_State_Msg.Outfire_St;
	
	if(DSP_State_Msg.Hull_Pump_st>0)
	//if((DSP_State_Msg.Hull_Pump_st&0xFD)>0)
		USV_State.Dradio_USV_Fire.Water_Level_St=1;
	else
		USV_State.Dradio_USV_Fire.Water_Level_St=0;
	if(DSP_State_Msg.Hull_Fan_st>0)
		USV_State.Dradio_USV_Fire.Ventilation_St=1;
	else
		USV_State.Dradio_USV_Fire.Ventilation_St=0;
	//�������澯��Ϣ
	USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Left=Motor_Detail_St.SuperLoad_Alarm_Left;
	USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Right=Motor_Detail_St.SuperLoad_Alarm_Right;
	USV_State.Dradio_USV_Engine_Alarm.InletPressure_Alarm_Left=Motor_Detail_St.Supercharger_Pressure_Alarm_L;
	USV_State.Dradio_USV_Engine_Alarm.InletPressure_Alarm_Right=Motor_Detail_St.Supercharger_Pressure_Alarm_R;
	USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Left=Motor_Detail_St.InletTEMP_Alarm_Left;
	USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right=Motor_Detail_St.InletTEMP_Alarm_Right;
	USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Left=Motor_Detail_St.Cooling_TEMP_Alarm_Left;
	USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Right=Motor_Detail_St.Cooling_TEMP_Alarm_Right;
	USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Left=Motor_Detail_St.Cooling_Level_Alarm_Left;
	USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Right=Motor_Detail_St.Cooling_Level_Alarm_Right;
	USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Left=Motor_Detail_St.OilPressure_Alarm_Left;
	USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Right=Motor_Detail_St.OilPressure_Alarm_Right;
	USV_State.Dradio_USV_Engine_Alarm.Fuel_Moisture_Alarm=(Motor_Detail_St.Fuel_Moisture_Alarm_L|Motor_Detail_St.Fuel_Moisture_Alarm_R);
	//����
//	USV_State.USV_Height=Smart_Navigation_St.USV_Height/0x14;//�ֱ��ʲ�20��
	USV_State.Hull_Fan_st=DSP_State_Msg.Hull_Fan_st;
	USV_State.Hull_Pump_st=DSP_State_Msg.Hull_Pump_st;
	USV_State.Dradio_USV_Model_State.Panel_Control_Sign_ST=SP_CON_Sign;
	USV_State.Dradio_USV_Model_State.Emergency_St=SP_CAN_Model_Con.Emergency_spn521192;		
	USV_State.APP_12V=POW_CAN_State.Application_12V_spn520969;
	USV_State.APP_24V=POW_CAN_State.Application_24V_spn520964;
	USV_State.Outboard_Engine_st_L=SR_CAN_State.Outboard_Engine_st_L_spn520776;
	USV_State.Outboard_Engine_st_R=SR_CAN_State.Outboard_Engine_st_R_spn520777;
//	USV_State.Sealight_LR_st=0;
//	USV_State.Sealignt_HT_st=0;//cxy
	//USVʱ����Ϣ
	hour=8+Smart_Navigation_St.USV_Hour;
	date=Smart_Navigation_St.USV_Date;
	if(hour>=24)
	{
		hour-=24;
		date+=1;
	}
	USV_State.Dradio_USV_Time.Date_Inside=Smart_Navigation_St.USV_Year*10000;
	USV_State.Dradio_USV_Time.Date_Inside+=Smart_Navigation_St.USV_Month*100;
	USV_State.Dradio_USV_Time.Date_Inside+=date;
	USV_State.Dradio_USV_Time.Time_Inside=hour*10000;
	USV_State.Dradio_USV_Time.Time_Inside+=Smart_Navigation_St.USV_Minute*100;
	USV_State.Dradio_USV_Time.Time_Inside+=Smart_Navigation_St.USV_Second;
	//��������Ϣ
//	USV_State.Dradio_USV_Engine_State.Ldle_Time_Left=Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235;
//	USV_State.Dradio_USV_Engine_State.Ldle_Time_Right=Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235;
//	USV_State.Dradio_USV_Engine_State.Fuel_ConRate_Left=Motor_CAN_State.Motor_PGN65266_State[0].Fuel_Usage_spn183;
//	USV_State.Dradio_USV_Engine_State.Fuel_ConRate_Right=Motor_CAN_State.Motor_PGN65266_State[1].Fuel_Usage_spn183;
	USV_State.USV_ROT=Smart_Navigation_St.USV_ROT;
	Fuel_ConRate=Motor_CAN_State.Motor_PGN65266_State[0].Fuel_Usage_spn183;
	Fuel_ConRate+=Motor_CAN_State.Motor_PGN65266_State[1].Fuel_Usage_spn183;
	Fuel_ConRate=Fuel_ConRate>>8;
	USV_State.Dradio_USV_Engine_State.Fuel_ConRate=(uint8)Fuel_ConRate;
	//�ȶ�ƽ̨�Ƕ���Ϣ
	USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_RL_St=ST_PL_CAN_State.Stable_Platform_RL_spn521342;
	USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_AT_St=ST_PL_CAN_State.Stable_Platform_AT_spn521343;
	//���˻�״̬��Ϣ
	USV_State.Dradio_USV_UAV_State.Platform_Hatch_St=UAV_CAN_State.Platform_Hatch_spn520576;
	USV_State.Dradio_USV_UAV_State.Platform_Lift_St=UAV_CAN_State.Platform_Lift_spn520577;
	USV_State.Dradio_USV_UAV_State.Platform_Open_St=UAV_CAN_State.Platform_Open_spn520578;
	USV_State.Dradio_USV_UAV_State.UAV_Charging_St=UAV_CAN_State.UAV_Charging_spn520582;
	//USV״̬cxy
	//USV_State.USV_Current_Sailing=
	//����������
	//USV_State.Sailing_Nummber=���յ���������ʱ�Ѹ���
	
//	USV_State.Engine_power=DSP_State_Msg.Engine_power_st;
//	USV_State.Engine_run=DSP_State_Msg.Engine_run_st;
	USV_State.USV_oil_sign=DSP_State_Msg.USV_oil_st;
	USV_State.USV_Speed_limit=USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit;
/*cxy	
	USV_State.Conrtol_System_Msg.Intelligent_rudder=
	USV_State.Conrtol_System_Msg.Stabilized_platform=	
	USV_State.Conrtol_System_Msg.Energy_management=
	USV_State.Conrtol_System_Msg.Power_management=POW_CAN_State.System_Check_spn520973;
	USV_State.Conrtol_System_Msg.UAV_platform=
	USV_State.Conrtol_System_Msg.Fire_fighting_system=
	USV_State.Conrtol_System_Msg.Engine_L=
	USV_State.Conrtol_System_Msg.Engine_R=
*/
	return	;
}

//���ֵ�̨����״̬
void Send_Dradio_State(uint16 State_Num)
{
	uint8 CRC;
//	int ret=0,count;
	CRC=0;
	USV_State_Current[0]='$';
	USV_State_Current[1]='U';
	USV_State_Current[2]='S';
	USV_State_Current[3]='V';
	USV_State_Current[4]='R';
	USV_State_Current[5]=USV_State.USV_Num;
	USV_State_Current[6]=((State_Num&0xff00)>>8);
	USV_State_Current[7]=(State_Num&0x00ff);
	USV_State_Current[8]=State_Current_Num;
	USV_State_Current[9]=(((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0xff00)>>8);
	USV_State_Current[10]=((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0x00ff);
	USV_State_Current[11]=(((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0xff00)>>8);
	USV_State_Current[12]=((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0x00ff);
	USV_State_Current[13]=(((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0xff00)>>8);
	USV_State_Current[14]=((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0x00ff);
	USV_State_Current[15]=(((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0xff00)>>8);
	USV_State_Current[16]=((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0x00ff);
	USV_State_Current[17]=(uint8)USV_State.Dradio_USV_Sailing_State.USV_Heave;
	USV_State_Current[18]=USV_State.Dradio_USV_Model_State.Latitude_Sign_St;
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Longitude_Sign_St)<<1);
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Sailing_Model_St)<<2);
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Differential_Model_St)<<4);
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Set_Return_Point_Model_St)<<5);
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Speed_Constant_Model_St)<<6);
	USV_State_Current[18]+=((USV_State.Dradio_USV_Model_State.Direction_Constant_Model_St)<<7);
	USV_State_Current[19]=USV_State.Dradio_USV_Location.USV_Latitude_Degree;
	USV_State_Current[20]=USV_State.Dradio_USV_Location.USV_Latitude_Minute;
	USV_State_Current[21]=USV_State.Dradio_USV_Location.USV_Latitude_Second;
	USV_State_Current[22]=USV_State.Dradio_USV_Location.USV_Latitude_Decimal;
	USV_State_Current[23]=USV_State.Dradio_USV_Location.USV_Longitude_Degree;
	USV_State_Current[24]=USV_State.Dradio_USV_Location.USV_Longitude_Minute;
	USV_State_Current[25]=USV_State.Dradio_USV_Location.USV_Longitude_Second;
	USV_State_Current[26]=USV_State.Dradio_USV_Location.USV_Longitude_Decimal;
	USV_State_Current[27]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0xff00)>>8);
	USV_State_Current[28]=((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0x00ff);
	USV_State_Current[29]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0xff00)>>8);
	USV_State_Current[30]=((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0x00ff);
	USV_State_Current[31]=USV_State.Dradio_USV_Drive_State.Gear_Left_St;
	USV_State_Current[32]=USV_State.Dradio_USV_Drive_State.Gear_Right_St;
	USV_State_Current[33]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St;
	USV_State_Current[34]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St;
	USV_State_Current[35]=USV_State.USV_Oil;
//	printf("USV_State.USV_Oi=%d\n",USV_State.USV_Oil);
	USV_State_Current[36]=USV_State.Dradio_USV_Battery.Battery_Left;
	USV_State_Current[37]=USV_State.Dradio_USV_Battery.Battery_TEMP_Left;
//	printf("Battery_Left=%d-%d\n",USV_State_Current[36],USV_State_Current[37]);
	USV_State_Current[38]=USV_State.Dradio_USV_Battery.Battery_Right;
	USV_State_Current[39]=USV_State.Dradio_USV_Battery.Battery_TEMP_Right;
	USV_State_Current[40]=USV_State.USV_Sailing_Intel_Sign;								//USV��������ִ��״̬��Ϣ
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.UAV_Power_St)<<2);
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.Stable_Platform_Power_St)<<3);
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.Camera_ahead_Power_St)<<4);
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.D_radar_Power_St)<<5);
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.Camera_main_Power_St)<<6);
	USV_State_Current[40]+=((USV_State.Dradio_USV_Device_State.Horn_Power_St)<<7);
	USV_State_Current[41]=USV_State.Dradio_USV_Device_State.Searchlight_Power_St;
	USV_State_Current[41]+=((USV_State.Dradio_USV_Device_State.Camera_lesser_Power_St)<<1);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Device_State.Navigationlight_Power_St)<<2);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Device_State.Camera_tail_Power_St)<<3);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Left_Charging)<<4);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Right_Charging)<<5);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_L)<<6);
	USV_State_Current[41]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_R)<<7);
	USV_State_Current[42]=USV_State.Dradio_USV_Fire.Outfire_St;
	USV_State_Current[42]+=((USV_State.Dradio_USV_Fire.Water_Level_St)<<1);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Fire.Ventilation_St)<<2);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Left)<<3);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Right)<<4);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletPressure_Alarm_Left)<<5);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right)<<6);
	USV_State_Current[42]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Left)<<7);
	USV_State_Current[43]=USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right;
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Left)<<1);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Right)<<2);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Left)<<3);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Right)<<4);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Left)<<5);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Right)<<6);
	USV_State_Current[43]+=((USV_State.Dradio_USV_Engine_Alarm.Fuel_Moisture_Alarm)<<7);
//	USV_State_Current[44]=(((USV_State.USV_Height)&0xff00)>>8);
//	USV_State_Current[45]=((USV_State.USV_Height)&0x00ff);
	USV_State_Current[44]=USV_State.Hull_Fan_st;
	USV_State_Current[44]+=((USV_State.Hull_Pump_st)<<2);
	USV_State_Current[44]+=((USV_State.Dradio_USV_Model_State.Panel_Control_Sign_ST)<<6);
	USV_State_Current[44]+=((USV_State.Dradio_USV_Model_State.Emergency_St)<<7);
	USV_State_Current[45]=USV_State.APP_12V;
	USV_State_Current[45]+=((USV_State.APP_24V)<<1);	
	USV_State_Current[45]+=((USV_State.Outboard_Engine_st_L)<<2);	
	USV_State_Current[45]+=((USV_State.Outboard_Engine_st_R)<<4);	
	USV_State_Current[45]+=(E_Stop<<6);		
	USV_State_Current[46]=(((USV_State.Dradio_USV_Time.Date_Inside)&0xff0000)>>16);
	USV_State_Current[47]=(((USV_State.Dradio_USV_Time.Date_Inside)&0x00ff00)>>8);
	USV_State_Current[48]=((USV_State.Dradio_USV_Time.Date_Inside)&0x0000ff);
	USV_State_Current[49]=(((USV_State.Dradio_USV_Time.Time_Inside)&0xff0000)>>16);
	USV_State_Current[50]=(((USV_State.Dradio_USV_Time.Time_Inside)&0x00ff00)>>8);
	USV_State_Current[51]=((USV_State.Dradio_USV_Time.Time_Inside)&0x0000ff);
//	USV_State_Current[52]=USV_State.Dradio_USV_Engine_State.Ldle_Time_Left;
//	USV_State_Current[53]=USV_State.Dradio_USV_Engine_State.Ldle_Time_Right;
	USV_State_Current[52]=((USV_State.USV_ROT&0xff00)>>8);
	USV_State_Current[53]=USV_State.USV_ROT&0x00ff;
	USV_State_Current[54]=USV_State.Dradio_USV_Engine_State.Fuel_ConRate;
	USV_State_Current[55]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_RL_St;
	USV_State_Current[56]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_AT_St;
	USV_State_Current[57]=USV_State.Dradio_USV_UAV_State.Platform_Hatch_St;
	USV_State_Current[57]+=((USV_State.Dradio_USV_UAV_State.Platform_Lift_St)<<3);
	USV_State_Current[57]+=((USV_State.Dradio_USV_UAV_State.Platform_Open_St)<<6);
	USV_State_Current[58]=USV_State.Dradio_USV_UAV_State.UAV_Charging_St;
	USV_State_Current[58]+=USV_State.USV_Current_Sailing;
	USV_State_Current[58]+=((USV_State.Sailing_Nummber)<<4);
	USV_State_Current[59]=USV_State.Engine_power;
	USV_State_Current[59]+=((USV_State.Engine_run)<<2);
//	printf("Engine_run=%d\n",USV_State.Engine_run);
	USV_State_Current[59]+=((USV_State.USV_oil_sign)<<4);
	USV_State_Current[60]=USV_State.USV_Speed_limit;
	USV_State_Current[61]=USV_State.Conrtol_System_Msg.Intelligent_rudder;
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Stabilized_platform)<<1;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Energy_management)<<2;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Power_management)<<3;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.UAV_platform)<<4;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Fire_fighting_system)<<5;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Engine_L)<<6;		
	USV_State_Current[61]+=(USV_State.Conrtol_System_Msg.Engine_R)<<7;	
	
	//ƽ����ת״̬
	//USV_State_Current[62] = USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_mode ;
	//USV_State_Current[62] +=USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_command<<2;
	//USV_State_Current[62] +=USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_mode<<4;
	//USV_State_Current[62] +=USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_command<<6;

	USV_State_Current[62]  =	USV_State.Dradio_USV_Model_State.translation_mode_St;	
	USV_State_Current[62] +=	USV_State.Dradio_USV_Model_State.translation_command_St<<2;
	USV_State_Current[62] +=    USV_State.Dradio_USV_Model_State.rotation_mode_St<<4;
	USV_State_Current[62] +=	USV_State.Dradio_USV_Model_State.rotation_command_St<<6;

	
	GetCheck(&CRC,USV_State_Current,sizeof(USV_State_Current));
	USV_State_Current[63]='*';
	USV_State_Current[64]=CRC;
	//���ڷ������ֵ�̨״̬����
	//if((1==Dradio_Com1_Sign)||(1==SP_CON_Sign))
	

	//ֹͣ���ʹ��ڱ���
	
	//	write_uart(UART2_Fd, (int8*)&USV_State_Current[0], State_Current_Num);
	
//	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_DRADIO_SN].send_ok_number++;

#ifdef	debug_print
	if(ret<0)
		printf("send radio error!\n");	
#endif
	return	;
}
//�������ֵ�̨����״̬
void Send_Sradio_State()
{
	uint8 CRC;
//	int ret=0,count;
	CRC=0;
	//USV_State_Current[0]='&';
	//USV_State_Current[8] = State_Current_Num-1;
	//USV_State_Current[62]= '*';
	//GetCheck(&CRC,USV_State_Current,(sizeof(USV_State_Current)-1));
	//USV_State_Current[63]=CRC;
	////if(1==SpareDradio_Com1_Sign)
	//	write_uart(UART4_Fd, (int8 *)&USV_State_Current[0], State_Current_Num);	

	USV_State_Current[0]='&';                                              
	GetCheck(&CRC,USV_State_Current,(sizeof(USV_State_Current)));          
	USV_State_Current[State_Current_Num-1]=CRC;                            
	//if(1==SpareDradio_Com1_Sign)                                           
		write_uart(UART4_Fd, (int8 *)&USV_State_Current[0], State_Current_Num);



#ifdef	debug_print
	if(ret<0)
		printf("send spare radio error!\n");	
#endif
	return	;
}


//����У��ֵ
void GetCheck(uint8 *CRC,uint8 *buff,uint32 length)
{
	uint32 i,len,crc;
	len=length;
	crc=buff[0];
	for (i = 1; i < len-2; i++)//���������Ϊcrc����Ҫ���
	{  
		crc =buff[i] ^crc;  
	}  
	*CRC=crc;
	return ;
}

//��������Դ
//�����0--���ֵ�̨,1--���������̨ 2--������̨ 3--������̨  4--���
uint8 Check_Radio(void)
{
	if(1!=SP_CON_Sign)						//������������ȡ����Ȩ�ޱ�־
	{
		if(1==Dradio_Com1_Sign)				//��Դ�����ֵ�̨
		{
			Connect_Sign=1;
			return 0;
		}
		else if(1==SpareDradio_Com1_Sign)	// ���������̨
			return 1;
		else if (1==Bradio_Con_Sign)		//������̨
		{
			Connect_Sign=1;
			return 2;
		}
		else if(1==BDS_Con_Sign)			//������̨
		{
			Connect_Sign=1;
			return 3;
		}
		Connect_Sign=0;						//û���κ�����
		return Radio_Sign_old;//��ʧһ֡��ʱ����50ms��ƫ������Ķ������������һָ�����ݡ���֡����1S�����ͣ����
	}
	else if(1==SP_CON_Sign)				//������Ȩ��
	{
		Connect_Sign=1;		
		return 4;
	}
	return 5;
}


/*****************************************************************************
* help
*/
void help( void )
{
	printf(
		"USV_ARM program\n"
		"Usage: S99USV_ARM [OPTION] [INPUT]\n"
		"-h              help menu (this screen)\n"
		"-v[version]     1 0.1\n"
		"                2 0.2\n"
		);
}
