/*==========================================================*
 * ģ��˵��: can_deal_main.cpp                                             *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա:                                                *
 * ����ʱ��: 				                                *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/can_deal_main.h"
#include "../include/nanoObstacleSender.h"


/**********************************  Include  ********************************/
/******************************  Local Variable  *****************************/
CAN_TYPE can_0;
PERI_MCU_CFG peri_mcu_cfg;
Battery_cfg_t batt_cfg;

uint8 log_b1_IHCcomm_sign=1;
uint8 log_b1_IDUcomm_sign=1;
uint8 log_b1_IOPcomm_sign=1;
uint8 log_b1_DOCKcomm_sign = 1;

/* ���CAN */
#define BMS1_ID                    (uint32)(0x8a6d0d09)
#define BMS2_ID                    (uint32)(0x8ad10d09)
BSM_msg_t bms1_msg = {0,0,0,0};
BSM_msg_t bms2_msg = {0,0,0,0};




/******************************  Extern Variable  ****************************/
extern nanoObstacleSender senderTestV1;
/******************************  Local Function   ****************************/
void *can0_rec_thread(void *aa);
int   can0_init(void);
void  can0_rec_report(void);
void  canRecInit(void);

/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
//CAN��1939�����߳� 
void *can0_rec_thread(void *aa)
{	
	uint8 loop_i=0;
	if(can0_init()<0)
	{
		//SysLogMsgPost("CAN�ڳ�ʼ��ʧ��");
		//SysPubMsgPost("CAN�ڳ�ʼ��ʧ��");
		printf("can start error!\n");
		return((void*)0);
	}
		canRecInit();

		callVersion();
		callVersion();
		callVersion();

		callSelfCheck();
		callSelfCheck();
		callSelfCheck();

		callAllStart();
		callAllStart();
		callAllStart();
	
	for (;;){
		can0_rec_report();
		//	canModelCommCal();
		
		//test code start

		usleep(400);	//0.5ms
		//test code end
	}
	
}
/******************************    Code   ************************************/
//CAN网初始化进程
int can0_init( void )
{
int iret =0;
char name[] = CAN;
#ifndef WINNT

	struct can_frame frame={.can_id = CAN_ARM_ADD};
	struct ifreq ifr;
	struct sockaddr_can addr;
	int family = PF_CAN,type = SOCK_RAW,proto = CAN_RAW;
	frame.can_id = CAN_ARM_ADD;
	
	can_do_stop(name);	//���ò�����ǰ�ر�can
	if(can_set_bitrate(name, CAN0_bitrate)<0)
	{
		printf( "set can CAN0_bitrate fail\n" );
		return -1;
	}
	can_do_start(name);

	can_0 = socket(family,type,proto);	//�����׽���
	if(can_0<0)
	{
		printf( "can0 < 0\n" );
		return -1;
	}
	strcpy(ifr.ifr_name,name);
	if(ioctl(can_0,SIOCGIFINDEX,&ifr))	//�ƶ�CAN�豸
	{
		printf( "set device ioctl fail\n" );
		return -1;
	}
	addr.can_family = family;
	addr.can_ifindex = ifr.ifr_ifindex;
	if(bind(can_0,(struct sockaddr *)&addr,sizeof(addr))<0)	//	���׽�����CAN�豸
	{
		printf( "bind socket fail< 0\n" );
		return -1;
	}
#else
	struct can_frame frame;
	frame.can_id = CAN_ARM_ADD;
	if(init_can_win()<0)
	{
		return -1;
	}
#endif
	return iret;
}
/******************************    Code   ************************************/
//CAN网1939接收线程
void can0_rec_report( void )
{
	struct	can_frame frame;
	int iret = 0 ;
	/****************/
	uint8  can_add;
	uint8  can_pri;
	uint16 can_pgn;
	uint8  can_data[8];
	uint8  loop_i;
	uint8  psID;		//��ַ�ռ� 0~255
	/****************/

	if(iret = read_can(can_0,&frame,sizeof(struct can_frame))<0)
	{
		//SysLogMsgPost("��ȡCAN�豸����");
		//SysPubMsgPost("��ȡCAN�豸����");

	}
	else
	{
		uint32 id = frame.can_id;
		can_add = (frame.can_id)&0x000000ff;
		can_pgn = ((frame.can_id)&0x00ffff00)>>8;
		can_pri = ((frame.can_id)&0x1c000000)>>26;
		psID =  ((frame.can_id)&0x0000ff00)>>8;

		memcpy((uint8 *)can_data,(uint8 *)frame.data,8);

		if(id == BMS1_ID){
			bms1_msg.voltage     = (uint16)(can_data[0])<<8 | (uint16)(can_data[1]);
			bms1_msg.current     = ((int16)(can_data[2])<<8) | (uint16)(can_data[3]);
			bms1_msg.batt_remain = (uint16)(can_data[4])<<8  | (uint16)(can_data[5]);
			bms1_msg.batt_total  = (uint16)(can_data[6])<<8  | (uint16)(can_data[7]);
			//printf("bms1_msg.voltage = %d,bms1_msg.current = %d\n",bms1_msg.voltage,bms1_msg.current);
		}

		if(id == BMS2_ID){
			bms2_msg.voltage     = (uint16)(can_data[0])<<8   | (uint16)(can_data[1]);
			bms2_msg.current     = ((int16)(can_data[2])<<8) | (uint16)(can_data[3]);
			bms2_msg.batt_remain = (uint16)(can_data[4])<<8   | (uint16)(can_data[5]);
			bms2_msg.batt_total  = (uint16)(can_data[6])<<8   | (uint16)(can_data[7]);
			//printf("bms2_msg.voltage = %d,bms2_msg.current = %d\n",bms2_msg.voltage,bms2_msg.current);
		}


		if(can_add == CAN_IDU_ADD)
		{
			IDU_recv(psID,can_data);
			monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IDU_SN].rec_ok_number++;	//接收正确计数
		}
		if(can_add == CAN_IOP_ADD)
		{
			IOP_recv(psID,can_data);
			monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IOP_SN].rec_ok_number++;	//接收正确计数
		}
		if(can_add == CAN_IHC_ADD)
		{
			IHC_recv(psID,can_data);
			monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IHC_SN].rec_ok_number++;	//接收正确计数
		}
		if (can_add == CAN_DOCK_ADD)
		{
			DOCK_recv(psID, can_data);
		}
	}
}
//��֡ģ���ʼ��
void canRecInit( void )
{
	IDU_Init();
	IOP_Init();
	IHC_Init();

	//dock can
	DOCK_Init();
}

void canCommCalInit(void)
{
	addTask(2,canModelCommCal,(void*)0);	//�ӵ�50ms ������
}

//ͨѶ״̬ͳ��
void canModelCommCal(void *)
{

	//IDU_CommCal();
	IOP_CommCal();
	IHC_CommCal();
	DOCK_CommCal(); //dock can ͨ�ų�ʱ����
	if (log_b1_IHCcomm_sign != IHC_comm_sign.comm_sign && poweron_init)
	{
		switch(IHC_comm_sign.comm_sign)
		{
			case COMM_CONNECT_OK:	//SysLogMsgPost("IHCͨѶ�ָ�");
									//SysPubMsgPost("IHCͨѶ�ָ�");
				WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_IHC_RCV_TIMEOUT, WARN_OFF);
				break;
			case COMM_CONNECT_FAIL:	//SysLogMsgPost("IHCͨѶ�ж�");
									//SysPubMsgPost("IHCͨѶ�ָ�");
				WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_IHC_RCV_TIMEOUT, WARN_ON);
				break;
			default:
				break;
		}
	}
	if (log_b1_IDUcomm_sign != IDU_comm_sign.comm_sign && poweron_init)
	{
	
		switch(IDU_comm_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	//SysLogMsgPost("IDUͨѶ�ָ�");
								//SysPubMsgPost("IDUͨѶ�ָ�");
			break;
		case COMM_CONNECT_FAIL:	//SysLogMsgPost("IDUͨѶ�ж�");
								//SysPubMsgPost("IDUͨѶ�ж�");
			break;
		default:
			break;
		}
	}
	//if(log_b1_IOPcomm_sign != IOP_comm_sign.comm_sign)
	//{
	//	switch(IOP_comm_sign.comm_sign)
	//	{
	//	case COMM_CONNECT_OK:	SysLogMsgPost("IOPͨѶ�ָ�");
	//		break;
	//	case COMM_CONNECT_FAIL:	SysLogMsgPost("IOPͨѶ�ж�");
	//	}
	//}
	if (log_b1_DOCKcomm_sign != DOCK_comm_sign.comm_sign && poweron_init)
	{
		
		switch (DOCK_comm_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	
			{
				printf(" dock connect ok\n");
			 }
			break;
		case COMM_CONNECT_FAIL:	
			{
				DOCK_reInit();
				printf(" dock disconnect \n");
			}
			break;
		default:
			break;
		}
		
	}
	log_b1_IHCcomm_sign = IHC_comm_sign.comm_sign;
	log_b1_IDUcomm_sign = IDU_comm_sign.comm_sign;
	log_b1_IOPcomm_sign = IOP_comm_sign.comm_sign;
	log_b1_DOCKcomm_sign = DOCK_comm_sign.comm_sign;
}

//�汾�ٻ�
void callVersion(void)
{
	uint8 pData[8];
	memset(pData,0,8);
	pData[0] = 1;
	pData[1] = 1;
	pData[2] = 1;
	sendCanMsg(can_0,255,128,128,pData);
}

//�Լ��ٻ�
void callSelfCheck(void)
{
	uint8 pData[8];
	memset(pData,0,8);
	pData[0] = 1;
	pData[1] = 1;
	pData[2] = 1;
	sendCanMsg(can_0,255,129,128,pData);
}

//��ʼ����
void callAllStart(void)
{
	uint8 pData[8];
	memset(pData,0,8);
	pData[0] = 1;
	pData[1] = 1;
	pData[2] = 1;
	sendCanMsg(can_0,255,130,128,pData);
}

#define INT_TYPE 2
#define FLOAT_TYPE 1

//�����ļ�����
int8 read_PeriMcuCfg_file( void )
{
	int8 *p_file_memory;				//������
	int32 *p_buffer;
	FILE *pFile;
	uint32 lSize;
	int32 result;
	uint32 len;
	int8 s1[32];
	int8 s2[50];
	int8 ret_val;
	uint16 loop_i;
	ret_val =TRUE;

	if ( (pFile = fopen(PERI_MCU_CFG_FILE_NAME, "r") ) == NULL)		//�����ļ�����
	{														
		sprintf_usv(s1,"PeriMcuConfig.cfg");
		sprintf_usv(s2,"not fond");
		input_cfg_ini_err_sub(s1,s2,0);
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);				//16K
	if (NULL == p_file_memory)
	{
		sprintf_usv(s1,"PeriMcucfg file memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		fclose(pFile);
		return FALSE;
	}

	p_buffer = (int32 *)malloc(0x10000);				//64K
	if (NULL == p_buffer)
	{
		sprintf_usv(s1,"PeriMcucfg explain memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		fclose(pFile);
		return FALSE;
	}

	// ��ȡ�ļ���С 
	fseek (pFile , 0 , SEEK_END);  
	lSize = ftell (pFile);  
	rewind (pFile);					//��ָ��ָ���ļ���ͷ

	if(lSize>=0xffff){
		sprintf_usv(s1,"PeriMcucfg read file");
		sprintf_usv(s2,"too large");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}
	
	 result = fread (p_file_memory,1,lSize,pFile);			 // ���ļ�������buffer��   

	if(32768 < lSize) len = 2 * lSize;
	else len = 1280 + 2 * lSize;
	len=len * 2;
	result=ini_Initialize((char *)p_file_memory, p_buffer, len);

	if(result!=0){										//�ļ���ʼ������
		sprintf_usv(s1,"PeriMcucfg  memory");
		sprintf_usv(s2,"explain error");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

	//��ʼ�����ļ�
	//IDU��������
	sprintf_usv(s1,"IDU_Switch_Cfg");
	sprintf_usv(s2,"IDU_Switch_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&peri_mcu_cfg.IDG_cfg.switch_connect.switchNum,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(peri_mcu_cfg.IDG_cfg.switch_connect.switchNum>IDU_SWITCH_NUM_MAX){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val = FALSE;
		peri_mcu_cfg.IDG_cfg.switch_connect.switchNum=0;
	}
	for(loop_i=0;loop_i<peri_mcu_cfg.IDG_cfg.switch_connect.switchNum;loop_i++)
	{
		sprintf_usv(s2,"IDU_Switch_%d",loop_i);
		if(read_sub_setting(s1,s2,0,(uint32*)&peri_mcu_cfg.IDG_cfg.switch_connect.switch_k[loop_i],INT_TYPE)==FALSE)
		{
			ret_val = FALSE;
		}
	}

	//IOP��������
	sprintf_usv(s1,"IOP_Switch_Cfg");
	sprintf_usv(s2,"IOP_Switch_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&peri_mcu_cfg.IOP_cfg.switch_connect.switchNum,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(peri_mcu_cfg.IDG_cfg.switch_connect.switchNum>IDU_SWITCH_NUM_MAX){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val = FALSE;
		peri_mcu_cfg.IDG_cfg.switch_connect.switchNum=0;
	}
	for(loop_i=0;loop_i<peri_mcu_cfg.IDG_cfg.switch_connect.switchNum;loop_i++)
	{
		sprintf_usv(s2,"IOP_Switch_%d",loop_i);
		if(read_sub_setting(s1,s2,0,(uint32*)&peri_mcu_cfg.IOP_cfg.switch_connect.switch_k[loop_i],INT_TYPE)==FALSE)
		{
			ret_val = FALSE;
		}
	}

	//COMM_Cfg
	sprintf_usv(s1,"COMM_Cfg");

	sprintf_usv(s2,"Docking_zmqCfg");
	read_sub_setting_string(s1, s2, 0,(char*)docking_zmq_cfg);

	sprintf_usv(s2,"MasterCam_zmqCfg");
	read_sub_setting_string(s1, s2, 0, (char*)docking_tracker_cfg);

	sprintf_usv(s2, "Lidar_zmqCfg");
	read_sub_setting_string(s1, s2, 0, (char*)lidar_tracker_cfg);
	sprintf_usv(s2, "Docking_SumlinkIP");
	read_sub_setting_string(s1, s2, 0, (char*)docking_contrl_cfg);

	sprintf_usv(s2, "Docking_Port");
	if (read_sub_setting(s1, s2, 0, (uint32*)&SUMLINK_PORT, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}
	sprintf_usv(s2, "Docking_RecvPort");
	if (read_sub_setting(s1, s2, 0, (uint32*)&SUMLINK_RECVPORT, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}
	sprintf_usv(s2,"Obstacles_zmqCfg");
	read_sub_setting_string(s1,s2,0,(char*)obsCommCfg);
	
	sprintf_usv(s2,"Sampling_zmqCfg");
	read_sub_setting_string(s1,s2,0,(char*)samplingCfg);

	sprintf_usv(s2,"InsPub_zmqCfg");
	read_sub_setting_string(s1,s2,0,(char*)ins_zmq_pub.zmq_cfg);

	sprintf_usv(s2,"PubMsgAddr_nanoCfg");
	read_sub_setting_string(s1,s2,0,(char*)nanoSysMsgAddr);
	
	sprintf_usv(s2,"Monitor_UdpCfg");
	read_sub_setting_string(s1,s2,0,(char*)MoniforIpCfg);

	
	sprintf_usv(s2, "InsTransmit_nanoCfg");
	read_sub_setting_string(s1, s2, 0, (char*)ins_nanoAddr);

	sprintf_usv(s2, "CtrlCenterIP");
	read_sub_setting_string(s1, s2, 0, (char*)CtrlCenterIP);

	sprintf_usv(s2, "CtrlCenterUdpSendPort");
	if (read_sub_setting(s1, s2, 0, (uint32*)&BRCC_UDP_RECV_PORT, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}

	sprintf_usv(s2, "CtrlCenterUdpRecvPort");
	if (read_sub_setting(s1, s2, 0, (uint32*)&BRCC_UDP_SEND_PORT, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}

	char c_Buf[30];
	memset(c_Buf, 0, sizeof(c_Buf));
	sprintf_usv(s2, "ObsPub_nanoCfg");
	read_sub_setting_string(s1, s2, 0, (char*)c_Buf);
	senderTestV1.setNanoSock(c_Buf); //�ϰ�������ת�� ��̨��ͼ��ʾ

	sprintf_usv(s2, "IRTK_udpCfg");
	if (read_sub_setting_string(s1, s2, 0, (char*)irtk_udp_addr) == FALSE)
	{
		ret_val = FALSE;
	}
	sprintf_usv(s2, "IRTK_Port");
	if (read_sub_setting(s1, s2, 0, (uint32*)&irtk_udp_port, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}

	sprintf_usv(s2, "INS_udpCfg");
	if (read_sub_setting_string(s1, s2, 0, (char*)ins_udp_addr) == FALSE)
	{
		ret_val = FALSE;
	}
	sprintf_usv(s2, "INS_Port");
	if (read_sub_setting(s1, s2, 0, (uint32*)&ins_udp_port, INT_TYPE) == FALSE)
	{
		ret_val = FALSE;
	}


	//Ship_Strategy_Cfg
	sprintf_usv(s1, "Ship_Strategy_Cfg");
	sprintf_usv(s2, "Disconnect_cfg");
	read_sub_setting_string(s1, s2, 0, (char*)disconnectCfg);


	
	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	printf("read periMcuCfg ok\n");
	return ret_val;

}


