/*
* algorithm.c --�����㷨
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"
#include <stdarg.h>
#include <string.h>
#ifndef WINNT
#include <linux/sockios.h>
#include <netpacket/packet.h>
#define CloseSocket(x) close(x)
#else
#include <winsock2.h>  
#include <ws2tcpip.h>  
#pragma comment( lib, "WS2_32" )
#define CloseSocket(x) closesocket(x)
#endif
#include "../include/usv_include.h"

MONITOR_UDP_INF_STRUCT	monitor_udp_inf;
MONITOR_UDP_CHECK_SEND_REPORT_STRUCT monitor_udp_check_send_report;
MONITOR_ASK_OPERATION_STRUCT	monitor_ask_operation;

void udp_close(int16  *sockid);
int16 CreateUDPSocket(int16 iPort);
void monitor_init(void);
uint16	monitor_input_report_prog(uint8 *report_buff);
int8 monitor_udp_rec_report(int16  sockid);
void deal_monitor_udp_rec_report(uint8 *net_rec_buff,uint16 report_len);
int8 monitor_udp_send_report( uint8 * lpBuf, int16 iLen,int16 sockid);
void deal_monitor_udp_rec_report(uint8 *net_rec_buff,uint16 report_len);
uint16 Send_arm_master_many_eth(uint8 *report_buff);

//��ģ�鱨�ļ��� ¼������
uint8 msg_record_cmd[MODEL_MAX_NUM];

//���߼����߳�
void *usv_monitor_thread(void *aa)   
{ 
int16    sockid ; //����վ�����õĶ˿�
uint8	 report_buff[200];
uint16	 report_send_len;

	monitor_init();					//���ӳ�ʼ��
	monitor_udp_inf.monitor_udp_rec_flag =0;
	sockid = -1;

	for(;;){		
		if( -1 == sockid ){	
			sockid=CreateUDPSocket((uint16)MONITOR_UDP_REC_PORT);	
			if(sockid<0){						
				sleep_1(1000);
				continue;
			}
		}
		else{													
			if(monitor_udp_check_send_report.send_report_flag!=SWITCH_ON){			
				monitor_udp_rec_report(sockid);	
				report_send_len=monitor_input_report_prog(&report_buff[0]);
				if((report_send_len>0) &&(SWITCH_ON==monitor_udp_inf.monitor_udp_rec_flag)){					
					monitor_udp_send_report((uint8 *)&report_buff[0], report_send_len,sockid);
				}
			}
			else{											
				report_send_len=monitor_input_report_prog(&report_buff[0]);
				if((report_send_len>0) &&(SWITCH_ON==monitor_udp_inf.monitor_udp_rec_flag)){				
					monitor_udp_send_report((uint8 *)&report_buff[0], report_send_len,sockid);
				}
			}
		}					
		sleep_1(10);				//10ms
	}
	return ((void *)0);
}

//��ʼ������
void monitor_init(void)
{					

	memset((int8 *)&monitor_udp_inf,0,sizeof(monitor_udp_inf));
	memset((int8 *)&monitor_udp_check_send_report,0,sizeof(monitor_udp_check_send_report));
	memset((int8 *)&monitor_ask_operation,0,sizeof(monitor_ask_operation));
	memset((int8 *)&msg_record_cmd,0,sizeof(msg_record_cmd));
	monitor_udp_inf.locate_usv_addr =USV_ADDR;						
	if(read_setting_file()==FALSE){							
		//init_flash_setting_default();
		//printf("read setting failed\n");
	}
}



// ############################################################################
//			��̫�����մ���������
// ##############################################################################
//UDP���մ���
void deal_monitor_udp_rec_report(uint8 *net_rec_buff,uint16 report_len)
{
uint8	*p_int8;
uint8	len;
uint8	cal_sum;
uint8	report_sum;
UDP_MONITOR_REPORT_STRUCT *p_udp_monitor_report;


//���У���
	p_int8=net_rec_buff;
	len = *p_int8;
	cal_sum=sys_CalcCheckSum(p_int8,len);
	p_int8 =p_int8+len;
	report_sum = *p_int8;
	if(report_sum!=cal_sum){				//У��ͳ���
		return ;
	}
//��鱨�ĵ�ַ��
	p_udp_monitor_report = (UDP_MONITOR_REPORT_STRUCT *)net_rec_buff;
	if(p_udp_monitor_report->dest_addr !=monitor_udp_inf.locate_usv_addr){	//�ж�Ŀ�ĵ�ַ�Ƿ�����
		return;
	}
	p_int8++;
	if(p_udp_monitor_report->src_addr!=MONITOR_ADDR){	//�ж�Դ��ַ�Ƿ�����
		return;
	}
	//if(p_udp_monitor_report->model_addr!=USV_ARM_MODEL_ADDR){
	//	return;
	//}

//��ȡ������
	if(p_udp_monitor_report->report_type!=DOWN_TPYE_ASK_PC_2_ARM_REPORT ){
		return;
	}

//���Խ���
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_TEST){
		switch(p_udp_monitor_report->func){
			case	DOWN_FUNC_COMM_MONITOR:		//ͨѶ����
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_COMM_MONITOR);						
				break;
			case	DOWN_FUNC_ALARM:			//�澯״̬
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_ALARM);
				break;
			case	DOWN_FUNC_TEST:				//������Ϣ
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_TEST);
				break;
			case	DOWN_FUNC_CFG_ERR_INF:		//�����ļ�������Ϣ
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_CFG_ERR_INF);
				break;
			case	DOWN_FUNC_MAIN_PARAM:		//��Ҫ���в���
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_MAIN_PARAM);
				break;
			case	DOWN_FUNC_OTHER_PARAM:		//�������в���
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_OTHER_PARAM);
				break;
			case	DOWN_FUNC_RUN_STATE:		//�豸����״̬
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_TEST,DOWN_FUNC_RUN_STATE);
				break;
			default:
				break;
		}
	}
//������ʾ
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_CURVE){
		input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_CURVE,DOWN_FUNC_CURVE_REAL_DATA);
	}

//��ģ�鱨�ļ���
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_MODEL_REPORT && p_udp_monitor_report->model_addr == 0){
		input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_MODEL_REPORT,p_udp_monitor_report->func);
	}
	//ģ�鱨��¼��
	if (p_udp_monitor_report->sub_type ==DOWN_SUB_MODEL_REPORT && p_udp_monitor_report->model_addr == 1)
	{
		monitor_rec_deal_record_cmd(p_udp_monitor_report->func);
	}
//��ȡ����
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_ASK_SETTING_PARAM){
		input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_ASK_SETTING_PARAM,p_udp_monitor_report->func);
	}
//�������ò���
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_SETTING_PARAM){
		switch(p_udp_monitor_report->func){
			case	DOWN_FUNC_PID_PARAM:			//PID�����趨
				monitor_rec_deal_PID_param(net_rec_buff);
				break;
			case	DOWN_FUNC_S_FACE_PARAM:			//S������趨
				monitor_rec_deal_S_face_param(net_rec_buff);
				break;
			case	DOWN_FUNC_TRANS_PARAM:			//ƽ�Ʋ����趨
				monitor_rec_deal_trans_param(net_rec_buff);
				break;
			case	DOWN_FUNC_ROTATE_PARAM:			//��ת�����趨
				monitor_rec_deal_rotate_param(net_rec_buff);
				break;			
			case	DOWN_FUNC_SHIP_ROUTE:			//�����趨
				monitor_rec_deal_ship_route(net_rec_buff);
				break;
			case	DOWN_FUNC_OBSTACLE_LOCATE:			//�ϰ����趨
				monitor_rec_deal_obstacle(net_rec_buff);
				break;
			default:
				break;
		}
	}

//����ģ�ͼ���
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_SHIP_MODEL){
		switch(p_udp_monitor_report->func){
			case	DOWN_FUNC_SIM_SHIP_START_STATE:		//����״̬
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_SHIP_MODEL,DOWN_FUNC_SIM_SHIP_START_STATE);						
				break;
			case	DOWN_FUNC_SIM_ROUTE:			//��������
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_SHIP_MODEL,DOWN_FUNC_SIM_ROUTE);
				break;
			case	DOWN_FUNC_SIM_OBSTACLE:				//�ϰ�������
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_SHIP_MODEL,DOWN_FUNC_SIM_OBSTACLE);
				break;
			case	DOWN_FUNC_SIM_INPUT_REAL_DATA:		//ʵʱ������
				input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_SHIP_MODEL,DOWN_FUNC_SIM_INPUT_REAL_DATA);
				break;

			default:
				break;
		}
	}
//��������ٻ�
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_SIMULATE_ASK){
		if(p_udp_monitor_report->func==DOWN_FUNC_MMI){
			input_monitor_udp_life_time(SHORT_LIFE_TIME,UP_TYPE_ARM_2_PC_REPORT,DOWN_SUB_SIMULATE_ASK,DOWN_FUNC_MMI);
		}
	}
//�����������
	if(p_udp_monitor_report->sub_type ==DOWN_SUB_SIMULATE_SET){
		if(p_udp_monitor_report->func==DOWN_FUNC_MMI){
			monitor_rec_deal_mmi_code(net_rec_buff);
		}
	}

}



// ############################################################################
//			��̫�����ʹ���������
// ##############################################################################
int8 monitor_udp_send_report( uint8 * lpBuf, int16 iLen,int16 sockid)
{
int iSendLen;
int DestPort;
struct sockaddr_in server ;
	
   if(monitor_udp_inf.monitor_udp_rec_flag==0) return TRUE;   
   if(iLen>=MAX_UDP_RETORT_NUMBER){			//���ĳ��ȳ���
		return TRUE;
   }
   DestPort =MONITOR_UDP_SEND_PORT;
   server.sin_family = AF_INET;
   server.sin_port = htons(DestPort);
   server.sin_addr.s_addr =monitor_udp_inf.from_udp_ip.sin_addr.s_addr;
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

//��д���߱���
uint16	monitor_input_report_prog(uint8 *report_buff)
{
uint16	report_len;

	report_len =0;
	if(monitor_udp_check_send_report.send_report_flag==SWITCH_ON){				//���ڷ��ͱ���
		if(monitor_udp_check_send_report.sendlife_time>0){
			monitor_udp_check_send_report.sendlife_time	--;
			if(monitor_udp_check_send_report.sendlife_time==0){						//����ʱ�䵽
				monitor_udp_check_send_report.send_report_flag=0;					//�뷢�ͱ�־
			}
		}
		if(monitor_udp_check_send_report.send_report_type==UP_TYPE_ARM_2_PC_REPORT){
			report_len=Send_arm_master_many_eth(report_buff);					//���ͳ�����
		}
		else{
			clear_monitor_udp_life_time();
		}
	}
	else if(monitor_ask_operation.send_flag==SWITCH_ON){
		report_len=send_monitor_ask(report_buff);						//���������
		monitor_ask_operation.send_flag=0;
		return report_len;		
	}

	return report_len;
}


uint16 Send_arm_master_many_eth(uint8 *report_buff)
{
uint16	report_len=0;
	
	//test_monitor();
	monitor_data_fresh();
//���Խ���
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_TEST){
		switch(monitor_udp_check_send_report.send_func){
			case	DOWN_FUNC_COMM_MONITOR:			//ͨѶ����
				report_len=monitor_send_input_comm(report_buff);
				break;
			case	DOWN_FUNC_ALARM:				//�澯״̬
				report_len=monitor_send_input_alarm(report_buff);
				break;
			case	DOWN_FUNC_TEST:					//������Ϣ
				report_len=monitor_send_input_test_inf(report_buff);
				break;
			case	DOWN_FUNC_CFG_ERR_INF:			//�����ļ�������Ϣ
				report_len=monitor_send_input_cfg_error_inf(report_buff);
				break;
			case	DOWN_FUNC_MAIN_PARAM:			//��Ҫ���в���
				report_len=monitor_send_input_main_run_real_data(report_buff);
				break;
			case	DOWN_FUNC_OTHER_PARAM:			//�������в���
				report_len=monitor_send_input_other_run_real_data(report_buff);
				break;
			case	DOWN_FUNC_RUN_STATE:			//�豸����״̬
				report_len=monitor_send_input_run_state(report_buff);
				break;
			default:
				clear_monitor_udp_life_time();		//�巢�ͱ�־
				break;
		}
		return report_len;
	}
//������ʾ
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_CURVE){	
		if(monitor_udp_check_send_report.send_func==DOWN_FUNC_CURVE_REAL_DATA){			
			report_len=monitor_send_input_curve_real_data(report_buff);
		}
		else{
			clear_monitor_udp_life_time();		//�巢�ͱ�־
		}
		return report_len;
	}
//��ģ�鱨�ļ���
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_MODEL_REPORT){	
		report_len=monitor_send_input_sub_equ_report(report_buff,monitor_udp_check_send_report.send_func);
		return report_len;
	}				

//�����趨
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_ASK_SETTING_PARAM){
		switch(monitor_udp_check_send_report.send_func){
			case	DOWN_FUNC_PID_PARAM:			//PID�����趨
				report_len=monitor_send_input_PID_param(report_buff);
				break;
			case	DOWN_FUNC_S_FACE_PARAM:				//S������趨
				report_len=monitor_send_input_S_face_param(report_buff);
				break;
			case	DOWN_FUNC_TRANS_PARAM:				//ƽ�Ʋ����趨
				report_len=monitor_send_input_trans_param(report_buff);
				break;
			case	DOWN_FUNC_ROTATE_PARAM:				//��ת�����趨
				report_len=monitor_send_input_rotate_param(report_buff);
				break;
			case	DOWN_FUNC_SHIP_ROUTE:					//�����趨
				report_len=monitor_send_input_ship_route(report_buff);
				break;
			case	DOWN_FUNC_OBSTACLE_LOCATE:			//�ϰ����趨
				report_len=monitor_send_input_obstacle(report_buff);
				break;	
			case	DOWN_FUNC_LAKE_LARGE_INF:			//�����С��Ϣ
				report_len=monitor_send_input_lake_large_inf(report_buff);
				break;				
			default:
				clear_monitor_udp_life_time();		//�巢�ͱ�־
				break;
		}
		return report_len;
	}

//���洬��ģ�ͼ���
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_SHIP_MODEL){
		switch(monitor_udp_check_send_report.send_func){
			case	DOWN_FUNC_SIM_SHIP_START_STATE:			//���ĳ�ʼ״̬
				report_len=monitor_send_input_ship_start_state(report_buff);
				break;
			case	DOWN_FUNC_SIM_ROUTE:				//��������
				report_len=monitor_send_input_simulate_route(report_buff);
				break;
			case	DOWN_FUNC_SIM_OBSTACLE:					//�ϰ�������
				report_len=monitor_send_input_simulate_obstacle(report_buff);
				break;
			case	DOWN_FUNC_SIM_INPUT_REAL_DATA:			//ʵʱ������
				report_len=monitor_send_input_real_data(report_buff);
				break;			
			default:
				clear_monitor_udp_life_time();		//�巢�ͱ�־
				break;
		}
		return report_len;
	}



//�ٻ�����ͧ�ķ���
	if(monitor_udp_check_send_report.send_data_type==DOWN_SUB_SIMULATE_ASK){
		switch(monitor_udp_check_send_report.send_func){
			case	DOWN_FUNC_MMI:			//�������
				report_len=monitor_send_input_MMI_state(report_buff);
				break;
			default:
				clear_monitor_udp_life_time();		//�巢�ͱ�־
				break;
		}
		return report_len;
	}


//�汾��ȡ

	clear_monitor_udp_life_time();		//�巢�ͱ�־
	return report_len;

}

void	input_monitor_udp_life_time(uint16 life_time,uint8 report_type,uint8 data_type,uint8 func_type)
{
	monitor_udp_check_send_report.send_report_flag=SWITCH_ON;
	monitor_udp_check_send_report.sendlife_time=life_time;			//���ͱ�������ʱ��
	monitor_udp_check_send_report.send_report_type=report_type;;		//���ͱ��ı�������
	monitor_udp_check_send_report.send_data_type=data_type;			//��������
	monitor_udp_check_send_report.send_func=func_type;				//������
	monitor_udp_check_send_report.send_report_seq=0;		//���ͱ������,��0��ʼ	
}


//������ʱ��
void clear_monitor_udp_life_time(void)
{
	monitor_udp_check_send_report.send_report_flag=0;
	monitor_udp_check_send_report.sendlife_time=0;			//���ͱ�������ʱ��
	monitor_udp_check_send_report.send_report_type=0;;		//���ͱ��ı�������
	monitor_udp_check_send_report.send_data_type=0;			//��������
	monitor_udp_check_send_report.send_func=0;				//������
	monitor_udp_check_send_report.send_report_seq=0;		//���ͱ������,��0��ʼ		
}


// #############################################################################
//			UDP �ײ㺯��
// ############################################################################

void udp_close(int16  *sockid) 
{

	shutdown( *sockid, 2 );		//��ֹ��һ���׽ӿ��Ͻ������ݵĽ����뷢��,2--�ر�sockfd�Ķ�д����
	CloseSocket(*sockid);
	*sockid =-1;	
	monitor_udp_inf.monitor_udp_rec_flag =0;
}




#ifdef WINNT
int creatUdp_win_start(void)
{
WORD wVersionRequested;
WSADATA wsaData;
int err;

	 wVersionRequested = MAKEWORD(2, 2);
 
     err = WSAStartup(wVersionRequested, &wsaData);
     if (err != 0) {
         /* Tell the user that we could not find a usable */
         /* Winsock DLL.                                  */
         printf("WSAStartup failed with error: %d\n", err);
         return -1;
     }
 
     if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
         /* Tell the user that we could not find a usable */
         /* WinSock DLL.                                  */
         printf("Could not find a usable version of Winsock.dll\n");
         WSACleanup();
         return -1;
    }
     else
         printf("The Winsock 2.2 dll was found okay\n");
	 return 0;

}
#endif

int16 CreateUDPSocket(int16 iPort)
{
	int sock; 
	int iOpValue;                                                                                                                                                                                                            
    struct sockaddr_in servaddr;                                                          
    int RcvBufLen = 1500;//128k for socket receive buffer  

#ifdef WINNT
	if(creatUdp_win_start()<0){
		return -1;
	}
#endif
                                                                                          
	// Create socket via which get UDP packet from CSS-200/1A or CSFU-107 
    if ( (sock = socket( AF_INET, SOCK_DGRAM, 0 )) < 0 )                                  
    {				                                                                      
	                 
	    return -1;                                                                        
    }                                                                                     
    #ifndef WINNT                                                                                      
	if ( setsockopt( sock, SOL_SOCKET, SO_SNDBUF, &RcvBufLen, sizeof( RcvBufLen ) )<0 )   
	{					                                                                  
		close(sock);                                                                    
	    return -1;                                                                        
	} 
    #endif

	iOpValue=1; 
	if( setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,              
		   (const char *)&iOpValue,  sizeof(iOpValue)) == -1)      
	{                                                           
		 return false;                           
	}                                                                                      
	                                                                                      
     // Create name of sock with wildcards 
    memset( &servaddr, 0, sizeof(servaddr) );                                             
    servaddr.sin_family = AF_INET;                                                        
    servaddr.sin_port = htons(iPort);	
	servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
    memset(&(servaddr.sin_zero), 0, 8);
    if ( bind( sock, (struct sockaddr *)&servaddr, sizeof(servaddr) ) < 0 )               
    {			         
	#ifdef WINNT
		closesocket(sock);
	#else
		close(sock);
	#endif                                                                     
	    return -1;                                                                        
    }                                                                                                                                                                                       	    		                                                              
	return sock;             
}


int8 monitor_udp_rec_report(int16  sockid)
{
fd_set set;
int16 ret;
int16	iLen;
struct timeval timeout;
//sockaddr_in from;
socklen_t sockaddrlen;
int8	net_rec_buff[200];

	FD_ZERO (&set);
    FD_SET(sockid, &set);
    timeout.tv_sec=0;
    timeout.tv_usec=5;					
    ret = select(sockid+1, &set, NULL, NULL, &timeout);  
	if(ret <0){                 		
		return TRUE;   
    }                      					
	if(FD_ISSET(sockid, &set)){				//UDP����	                																
		sockaddrlen = sizeof(sockaddr_in);								
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff), 0,(struct sockaddr *)&monitor_udp_inf.from_udp_ip, &sockaddrlen);		
		if(iLen>=1){
			monitor_udp_inf.monitor_udp_rec_flag =SWITCH_ON;
			deal_monitor_udp_rec_report((uint8 *)&net_rec_buff[0],iLen);
		}
	}	
	return TRUE;
}



// ####################################################################################
//		 							��ӡ����
// ####################################################################################
short int sprintf_usv(char *buffer ,const char *format, ...)
{
int16	iRet;
va_list arglist;
va_start(arglist, format);

#ifdef	WINNT
	iRet=vsprintf(buffer,format,arglist);
//	iRet=sprintf_s(buffer,len,format,arglist);	
#else
	
	iRet=vsprintf(buffer,format,arglist);
#endif
	va_end(arglist);
	return iRet;
	
}

short int sscanf_usv(const char * str, const char * format, ...)
{
int16	iRet;
va_list arglist;
va_start(arglist, format);

	iRet=sscanf(str,  format, arglist);
	return iRet;
}


uint8	sys_CalcCheckSum(const uint8 *dest, uint16 u16len)
{
uint8	u8Sum = 0;
	
	while (u16len--)
	{
		u8Sum += (*dest++);
	}

	return(u8Sum & 0xff);
}

//���ֽ���Ŀ���
void memcpy_32(uint32* dest_buff,uint32* src_buff,uint8 report_len)
{
uint8	loop_i;
uint32  *p_dest_int32;
uint32  *p_src_int32;

	p_dest_int32 =dest_buff;
	p_src_int32 =src_buff;
	for(loop_i=0;loop_i<report_len;loop_i++){
		*p_dest_int32=/*htonl_86_Linux*/(*p_src_int32);
		p_dest_int32++;
		p_src_int32++;
	}
}

//���ֽ���Ŀ���
void memcpy_16(uint16* dest_buff,uint16* src_buff,uint8 report_len)
{
uint8	loop_i;
uint16  *p_dest_int16;
uint16  *p_src_int16;

	p_dest_int16 =dest_buff;
	p_src_int16 =src_buff;
	for(loop_i=0;loop_i<report_len;loop_i++){
		*p_dest_int16=/*htonl_86_Linux*/(*p_src_int16);
		p_dest_int16++;
		p_src_int16++;
	}
}


//����APDU��Э���
//report_len--Len���ĳ���
uint16	input_apdu_report(UDP_MONITOR_REPORT_STRUCT *p_udp_monitor_report,uint8 report_len)
{
uint8	apdu_len;
uint8	*p_uint8;

	apdu_len =report_len+5;
	p_udp_monitor_report->apdu_len=apdu_len;				//APDU����
	p_udp_monitor_report->dest_addr=MONITOR_ADDR;			//PC��ַ
	p_udp_monitor_report->src_addr=USV_ADDR;				//����ͧ��ַ

	p_uint8=(uint8 *)p_udp_monitor_report;
	p_uint8 =p_uint8 +apdu_len;	
	*p_uint8=sys_CalcCheckSum((uint8 *)p_udp_monitor_report, apdu_len);
	return (apdu_len+1);
}

