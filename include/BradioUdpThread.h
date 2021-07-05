#ifndef _BRADIOUPDTHREAD_H
#define _BRADIOUPDTHREAD_H



#define MAX_UDP_REPORT_NUMBER 128

//UDP端口号
#define BRADIO_UDP_REC_PORT  8812   //接收端口
#define BRADIO_UDP_SEND_PORT 8811   //发送端口

//后台IP地址
#define UDP_HOST_IP "172.1.1.80"

typedef struct{
	uint8 local_udp_addr;
	uint8 udp_rec_flag;
	sockaddr_in	from_upd_ip;
}UDP_INF_STRUCT;


void *Lan1_Bradio_UDP_thread(void *aa);
int8 Lan1_Bradio_UDP_send( int sockid ,uint8* msgBuff,uint16 msgLen);
int8  Lan1_Bradio_UDP_rec(int sockid);
void  deal_bradio_udp_rec(uint8 *net_rec_buff,uint16 report_len);
void deal_bradio_udp_send( int sockid ,uint16 State_Num);

void Send_Bradio_State_udp(int sockid,uint16 State_Num);
void Send_Bradio_Ais_udp(int sockid);

void Send_Bradio_Detail_Msg_udp( int sockid,uint8 chd)	;	//发送详细信息
void Send_Motor_Detail_State_udp(int sockid);
void Send_Rudder_Detail_State_udp(int sockid);
void Send_Stable_Platform_Detail_State_udp(int sockid);
void Send_UAV_Detail_State_udp(int sockid);
void Send_MCU_State_udp(int sockid);
void Send_Hull_State_udp(int sockid);
void Send_Smart_Navigation_Msg_udp(int sockid);
void Send_Panel_Control_Msg_udp(int sockid);
void Send_Energy_Control_Msg_udp(int sockid);
void Send_Power_Control_Msg_udp(int sockid);

extern void initLanUdpSendTask(void);
void runLanUdpSendTask(void*);


#endif //_BRADIOUPDTHREAD_H