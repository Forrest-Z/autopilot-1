/*
* lan_ctrl.c --�����߳�
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"

#ifdef WINNT 
#include <stdio.h>  
#include <stdlib.h>  
#include <winsock2.h>  
#include <ws2tcpip.h>  

#pragma comment( lib, "WS2_32" )
#define CloseSocket(x) closesocket(x)
#else
/*
#  include <unistd.h>
#  include <ctype.h> 
#  include <signal.h> 
#  include <sys/types.h> 
#  include <sys/socket.h> 
#  include <sys/file.h> 
#  include <sys/ioctl.h> 
#  include <sys/wait.h> 
#  include <sys/types.h> 
#  include <netdb.h> 
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <time.h>
#  define CloseSocket(x) close(x)
*/
#endif

#include "../include/usv_include.h"

#ifdef WINNT
#define	MSG_DONTWAIT  0
#endif

//MPC���ڱ��Ľ���
void *Lan0_MPC_Rec(void *aa)   
{ 
uint8	buff[1024],MPC_Msg[LAN0_BUFF];
uint8	AIS_MSG[MPC_AIS_BUFF];
bool opt;
struct sockaddr_in  serverAddr;     // server's socket address 
struct sockaddr_in  clientAddr;      	// client's socket address     
struct timeval timeout={0,5000};//5ms		
socklen_t                 sockAddrSize;	// size of socket address structure    
int sFd,re,ico,jco=0,re_sign=0,flags;

int i;

uint8 mpc_radar_valid_old = 0;

#ifdef WINNT
WORD wVersionRequested;
WSADATA wsaData;
int err;

	 wVersionRequested = MAKEWORD(2, 2);
 
//     err = WSAStartup(MAKEWORD(1,1),&wsaData);	//initial Ws2_32.dll by a process;
	 err = WSAStartup(wVersionRequested, &wsaData);
     if (err != 0) {

        printf("WSAStartup failed with error: %d\n", err);
         return ((void *)0);
     }


     if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
              printf("Could not find a usable version of Winsock.dll\n");
         WSACleanup();
         return ((void *)0);
    }
	 else{
         printf("The Winsock 2.2 dll was found okay\n");
	 }
#endif
	memset(MPC_Msg, 0, LAN0_BUFF);
	opt=true;	
	
	sockAddrSize = sizeof (struct sockaddr_in);    
	memset ((char *) &serverAddr, 0,sockAddrSize);    
	serverAddr.sin_family = AF_INET;					 //IPv4 ����Э����׽�������
	serverAddr.sin_port = htons (LAN0_PORT_NUM);		 //�˿ںţ�5001
	serverAddr.sin_addr.s_addr = htonl (INADDR_ANY);	 //���е�ַ������
	sFd = socket (AF_INET, SOCK_STREAM, 0);				 // ����socket
	if(sFd<0)
	{    
	    perror ("lan 0 socket error");    
		return ((void *)0);
	}  


//���÷�����socket
	
#ifdef WINNT
	int  iOpValue;
	iOpValue = 1; 
	setsockopt(sFd, SOL_SOCKET, SO_REUSEADDR, 
	 			(const char *)&iOpValue,  sizeof(iOpValue));
#else
	flags=fcntl(sFd,F_GETFL,0);
	fcntl(sFd,flags|O_NONBLOCK);
	setsockopt(sFd,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout,sizeof(struct timeval));		//
	setsockopt(sFd,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout,sizeof(struct timeval));
#endif

#ifdef WINNT
	if( setsockopt( sFd, IPPROTO_TCP,TCP_NODELAY, 
	 			(const char *)&iOpValue,  sizeof(iOpValue)) == -1)	
#else
	if(setsockopt(sFd,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(struct timeval))<0)  
#endif 
	{
	    perror ("setsockopet error\n");   
#ifdef WINNT
		closesocket(sFd);
#else
		close(sFd);
#endif
    
		return ((void *)0);
	}
	if (bind (sFd, (struct sockaddr *) &serverAddr, sockAddrSize) <0)    
	{    
	    	perror ("bind");    
#ifdef WINNT
			closesocket(sFd);
#else
			close(sFd);
#endif    
		return ((void *)0);
	}   
	if (listen (sFd, SERVER_MAX_CONNECTIONS) <0)    
	{    
	    	perror ("listen");    
#ifdef WINNT
			closesocket(sFd);
#else
			close(sFd);
#endif   
		return ((void *)0);
	}   
	while(1)   
	{    
	    LAN0_Fd=accept (sFd, (struct sockaddr *) (&clientAddr),&sockAddrSize) ;
		if(LAN0_Fd<0)
		{
#ifdef WINNT
			closesocket(LAN0_Fd);
#else
			close(LAN0_Fd);
#endif 	
			sleep_1(100);            //100ms	
			//ͨѶ�жϣ�����ϰ����б�
			
			continue;
		}
		else
		{
  	  		printf("lan0 connect is ok!\n");
		}
		while(1)
		{

//	    	memset(buff, 0, 1024);
//			re=read (LAN0_Fd, buff, 1024) ;
			re=recv(LAN0_Fd,(char *)&buff[0],1024,MSG_DONTWAIT);//���÷�����recv  1024
			if(re<0)
			{
//				close(LAN0_Fd);
				sleep_1(100);            //100ms	
				continue;
			}
			else if(re==0)
			{
				
#ifdef WINNT
				closesocket(LAN0_Fd);
#else
				close(LAN0_Fd);
#endif 
				sleep_1(100);            //100ms	
				break;
			}
			else    
			{    
//				printf("%s",buff);
				for(ico=0;ico<re;ico++)
				{
//�жϽ����Ƿ�ΪCRC�Ͱ汾��
					if((buff[ico]=='V')&&(buff[ico+1]=='E')&&(buff[ico+2]=='R')&&(buff[ico+3]=='S')&&(buff[ico+4]=='I')&&(buff[ico+5]=='O')&&(buff[ico+6]=='N'))
					{
						sprintf_usv(Version[1],"MPC_V0.%d_R%d",buff[ico+7]*256+buff[ico+8],buff[ico+9]*256+buff[ico+10]);
						MPCVersion_sign=1;
					}
//MPC����RAM���ı�־ͷ
					if(((buff[ico]=='R')&&(buff[ico+1]=='M'))||(1==re_sign))
					{
						re_sign=1;
						MPC_Msg[jco++]=buff[ico];
						if(jco>=LAN0_BUFF)
						{
							jco=0;
							re_sign=0;
							if(CheckCRC(1, MPC_Msg,LAN0_BUFF))
							{
//								printf("RM crc  is ok\n");
								LAN0_RM_Message *RM_MSG=&USV_RM_MSG;
								LAN0_RM_Analytical(RM_MSG,MPC_Msg,&Radar_OBS_Msg.Obs_num[0]);
								task_time.lan0_task_50ms=0;
								task_time.lan0_task_2s=0;
								
								Send_MPC_State(Heartbeat_Num);	//������������
								
								//����ת��
								//monitor_all_inf.
								monitor_all_inf.monitor_comm_inf[MONITOR_COMM_MTRADAR_SN].rec_ok_number++;	//���ճɹ�����
								monitor_all_inf.monitor_comm_inf[MONITOR_COMM_MTRADAR_SN].send_ok_number++;	//���ճɹ�����
								
								for(i=0;i<LAN0_BUFF;i++)
								{
									monitor_all_inf.module_report_inf[MONITOR_MPC_MSG].report_detail[i] = MPC_Msg[i];
								}
								mpc_radar_valid_old = USV_RM_MSG.MPC_radar_valid;
								USV_RM_MSG.MPC_radar_valid = MPC_Msg[9];	
								USV_RM_MSG.MPC_Heatbeat=1;
								memset(MPC_Msg, 0, LAN0_BUFF);
							}
							else
							{
//								printf("RM crc  is error\n");
								sleep_1(100);            //100ms	
								break;	
							}
							MCU_St.MPC_Disk=USV_RM_MSG.MPC_Msg_St.MPC_Disk;
							MCU_St.MPC_Memory=USV_RM_MSG.MPC_Msg_St.MPC_Memory;
							MCU_St.MCP_CPU=USV_RM_MSG.MPC_Msg_St.MCP_CPU;




							if(mpc_radar_valid_old = USV_RM_MSG.MPC_radar_valid == 0 && USV_RM_MSG.MPC_radar_valid == 1)
							{
							//	SysLogMsgPost("�״�ͼ��ָ�");
							}
							if(mpc_radar_valid_old = USV_RM_MSG.MPC_radar_valid == 1 && USV_RM_MSG.MPC_radar_valid == 0)
							{
							//	SysLogMsgPost("�״�ͼ���쳣");
							}
						}
					}

					if(((buff[ico]=='^')&&(buff[ico+1]=='A')&&(buff[ico+2]=='I')&&(buff[ico+3]=='S'))||(2==re_sign))
					{
						re_sign=2;
						AIS_MSG[jco++]=buff[ico];
						if(jco>=MPC_AIS_BUFF)
						{
							jco=0;
							re_sign=0;
							if(CheckCRC(1,AIS_MSG,MPC_AIS_BUFF))
							{
								//������
								AisMsgPost((uint8 *)&AIS_MSG[5]);
							//	SysLogMsgPost("�յ�AIS����");
							}
						}
					}
				}
			}   
			sleep_1(100);            //10ms	
		}
	}    
#ifdef WINNT
	closesocket(LAN0_Fd);
#else
	close(LAN0_Fd);
#endif 

	return ((void *)0);
}   
void Send_MPC_State(uint16 Heartbeat_Num)
{
	uint8 MPC_State[MPC_State_Lenght],CRC;
//	int ret=0;
	memset(MPC_State,0,MPC_State_Lenght);

	MPC_State[0]='M';
	MPC_State[1]='R';
	MPC_State[2]=',';
	MPC_State[3]=0x01;
	MPC_State[4]=',';
	MPC_State[5]=((Heartbeat_Num&0xff00)>>8);
	MPC_State[6]=(Heartbeat_Num&0x00ff);
	MPC_State[7]=',';
	MPC_State[8]=0x20;
	MPC_State[9]=',';
	MPC_State[10]=Smart_Navigation_St.USV_Speed/25;
	MPC_State[11]=(((Smart_Navigation_St.USV_Heading/10)&0xff00)>>8);
	MPC_State[12]=((Smart_Navigation_St.USV_Heading/10)&0x00ff);
	MPC_State[13]=Smart_Navigation_St.Latitude_Sign_St;
	MPC_State[13]+=Smart_Navigation_St.Longitude_Sign_St<<2;
	MPC_State[14]=Smart_Navigation_St.USV_Latitude_Degree;
	MPC_State[15]=Smart_Navigation_St.USV_Latitude_Minute;
	MPC_State[16]=Smart_Navigation_St.USV_Latitude_Second;
	MPC_State[17]=Smart_Navigation_St.USV_Latitude_Decimal_2;
	MPC_State[18]=Smart_Navigation_St.USV_Longitude_Degree;
	MPC_State[19]=Smart_Navigation_St.USV_Longitude_Minute;
	MPC_State[20]=Smart_Navigation_St.USV_Longitude_Second;
	MPC_State[21]=Smart_Navigation_St.USV_Longitude_Decimal_2;	
	
	MPC_State[30]='*';
	GetCheck(&CRC, MPC_State, MPC_State_Lenght);
	MPC_State[31]=CRC;

    send (LAN0_Fd,(char *)MPC_State,MPC_State_Lenght,MSG_DONTWAIT);//���ڷ���MPC����

	
	return;
}








void Send_MPC_CRC( )
{
	char MPC_State[7];

	MPC_State[0]='V';
	MPC_State[1]='E';
	MPC_State[2]='R';
	MPC_State[3]='S';
	MPC_State[4]='I';
	MPC_State[5]='O';	
	MPC_State[6]='N';	

    send (LAN0_Fd,MPC_State,7,MSG_DONTWAIT);//���ڷ���MPC����

	
	return;
}


#ifdef WINNT
SOCKET clientSocket ;
#else
int	   clientSocket	;
#endif

//������̨���ڱ��Ľ��� �ͻ��� �趨����������
void *Lan1_Bradio_Client(void *aa)
{
	int ico,jco=0,re_sign=0;
	//uint8 buff[LAN1_BUFF],bradio_Con[Dradio_Con_Msg];
	bool  opt;
	int iret,iret_rd,iret_wr;
	int err;

	struct sockaddr_in  serverAddr;     // server's socket address 
	struct sockaddr_in  clientAddr;      	// client's socket address   
	struct timeval timeout={300,0};//5s	
	socklen_t sockAddrSize;	//size of socket address structure


#ifdef WINNT
	WORD versionRequired;
	WSADATA wsaData;
	versionRequired = MAKEWORD(1,1);
	err = WSAStartup(versionRequired,&wsaData);	//
	if(!err)
	{
		printf("client socket already open!\n");
	}
	else
	{
		printf("ERROR,client socket open failed!\n");
	}
#endif

	opt=1;
	sockAddrSize = sizeof(struct sockaddr_in);
	memset((char *)&clientAddr,0,sockAddrSize);

	
	uint8 receiveBuf[LAN1_BUFF],bradio_Con[Dradio_Con_Msg];
	memset(receiveBuf,0,LAN1_BUFF);

	while(1)
	{
		clientSocket = socket(AF_INET,SOCK_STREAM,0);
		setsockopt(clientSocket,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout,sizeof(timeval));
		setsockopt(clientSocket,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout,sizeof(timeval));
		setsockopt(clientSocket,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(timeval));
		
		struct sockaddr_in clientsock_in;
		clientsock_in.sin_addr.s_addr = inet_addr("172.1.1.200");
		clientsock_in.sin_family=AF_INET;
		clientsock_in.sin_port = htons(LAN1_PORT_SPARE_NUM);
		iret = connect(clientSocket,(struct sockaddr *)&clientsock_in,sizeof(clientsock_in));// ��ʼ����
		
		for(;;)
		{
			memset(receiveBuf,0,LAN1_BUFF);
			if(iret <0)
			{
#ifdef WINNT
				closesocket(clientSocket);
#else
				close(clientSocket);
#endif

				sleep_1(200);
				break;
			}
			else
			{
			//	printf("Send:%s\n","hello,this is client");
				
				//iret_wr = send(clientSocket,"hello,this is client",strlen("hello,this is client")+1,0);
				Send_Bradio_SP_State();
				memset(receiveBuf,0,sizeof(receiveBuf));
				iret_rd = recv(clientSocket,(char *)receiveBuf,101,MSG_DONTWAIT);
			//	printf("Recv:%s\n",receiveBuf);
			}

			if(iret_rd <0)
			{
				sleep_1(200); 
				continue;
			}
			else if(iret_rd == 0)
			{
#ifdef WINNT
				closesocket(clientSocket);
#else
				close(clientSocket);
#endif
				sleep_1(200);
				break;
			}
			else
			{
				//���մ�������
				for(ico=0;ico<iret_rd;ico++)
				{
					if((receiveBuf[ico]=='$')&&(receiveBuf[ico+1]=='U')&&(receiveBuf[ico+2]=='S')&&(receiveBuf[ico+3]=='V')||(1==re_sign))
					{
						re_sign = 1;
						bradio_Con[jco++]=receiveBuf[ico];
						if(jco==Dradio_Con_Msg)
						{
							jco=0;
							re_sign=0;
							if(CheckCRC(1,bradio_Con,Dradio_Con_Msg))
							{
								USV_Control.Radio_Nummber=2;	//������̨����ָ���־
								Dradio_Con_Analytical(2,bradio_Con);
								memcpy((&monitor_all_inf.module_report_inf[MONITOR_BRADIO_MSG].report_detail[0]),(&bradio_Con[0]),Dradio_Con_Msg);	//ת�������̨����ָ��
								task_time.lan1_task_1s=0;
								task_time.lan1_task_250ms=0;
								Bradio_Con_Sign=1;
								memset(bradio_Con, 0, Dradio_Con_Msg);
							}
						}
					}
				}

			}
			sleep_1(200); 
		}	
		//else
		//{
		//	closesocket(clientSocket);
		//	continue;
		//}
		//WSACleanup();
	}
}

#define BRADIO_UDP_REC_PORT 6602
#define BRADIO_UDP_SED_PORT 6601


typedef struct{
	uint8 local_udp_addr;
	uint8 udp_rec_flag;
	sockaddr_in	from_upd_ip;
}SP_UDP_INF_STRUCT;

SP_UDP_INF_STRUCT sp_udp_inf;


//UDP���մ���
void deal_sp_udp_rec(uint8 *net_rec_buff,uint16 report_len)
{
	int ico,jco=0,re_sign=0;
	uint8 bradio_Con[Dradio_Con_Msg];

	for(ico=0;ico<report_len;ico++)
	{
		if((net_rec_buff[ico]=='$')&&(net_rec_buff[ico+1]=='U')&&(net_rec_buff[ico+2]=='S')&&(net_rec_buff[ico+3]=='V')||(1==re_sign))
		{
			re_sign = 1;
			bradio_Con[jco++]=net_rec_buff[ico];
			if(jco == Dradio_Con_Msg)
			{
				jco=0;
				re_sign=0;
				if(CheckCRC(1,bradio_Con,Dradio_Con_Msg))
				{
					USV_Control.Radio_Nummber=2;	//������̨����ָ���־
					Dradio_Con_Analytical(2,bradio_Con);
					memcpy((&monitor_all_inf.module_report_inf[MONITOR_BRADIO_MSG].report_detail[0]),(&bradio_Con[0]),Dradio_Con_Msg);	//ת�������̨����ָ��
					task_time.lan1_task_1s=0;
					task_time.lan1_task_250ms=0;
					Bradio_Con_Sign=1;
					memset(bradio_Con, 0, Dradio_Con_Msg);
				}
			}
		}
	}
	
}


int8 sp_udp_rec(int16 sockid)
{
	fd_set set;
	int16 ret;
	int16 iLen;
	struct timeval timeout;
	socklen_t sockaddrlen;
	int8 net_rec_buff[200];

	FD_ZERO(&set);
	FD_SET(sockid,&set);
	timeout.tv_sec=0;
	timeout.tv_usec=5;
	ret = select(sockid+1,&set,NULL,NULL,&timeout);
	if(ret<0){
		return TRUE;
	}
	if(FD_ISSET(sockid,&set)){		//UDP����
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff),0,(struct sockaddr *)&sp_udp_inf.from_upd_ip, &sockaddrlen);
		if(iLen>=1){
			sp_udp_inf.udp_rec_flag = SWITCH_ON;
			deal_sp_udp_rec((uint8 *)&net_rec_buff[0],iLen);
		}
	}
	return TRUE;
}


// ############################################################################
//			��̫�����ʹ���������
// ##############################################################################
int8 sp_udp_send(int16 sockid)
{
	int iSendLen;
	int DestPort;
	uint8 CRC;
	struct sockaddr_in server;

	DestPort = BRADIO_UDP_SED_PORT;
	server.sin_family = AF_INET;
	server.sin_port = htons(DestPort);
	server.sin_addr.s_addr = sp_udp_inf.from_upd_ip.sin_addr.s_addr;
	memset(server.sin_zero,0,8);


	CRC=0;
	USV_State_Current[0]='$';
	GetCheck(&CRC,USV_State_Current,sizeof(USV_State_Current));
	USV_State_Current[64]=CRC;

	iSendLen=sendto(sockid,(int8 *)&USV_State_Current[0],State_Current_Num,0,(struct sockaddr*)&server, sizeof(server));

	if(iSendLen != State_Current_Num)
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}


//�����������<--->ARM UDP�շ��߳�
void *Lan1_Bradio_UDP(void *aa)
{
	int16	sockid;		//�봮�ڷ����������ö˿�
	uint8	recv_buff[200];
	uint16	send_len;

	//��ʼ��
	memset((int8 *)&recv_buff[0],0,sizeof(recv_buff));
	sockid = -1;

	for(;;)
	{
		if(-1 == sockid){
			sockid = CreateUDPSocket((uint16)BRADIO_UDP_REC_PORT);	//����UDP
			if(sockid<0){
				sleep_1(1000);
				continue;
			}
		}
		else{
			sp_udp_rec(sockid);
		}

		//���ķ���
		sp_udp_send(sockid);
		sleep_1(200);	//200ms
		
	}

}







//������̨���ڱ��Ľ���
void *Lan1_Bradio_Rec(void *aa)   
{   
uint8	buff[LAN1_BUFF],bradio_Con[Dradio_Con_Msg],bradio_Sal[Dradio_Sal_Msg];
uint16	Sail_len;
bool opt;
struct sockaddr_in  serverAddr;     // server's socket address 
struct sockaddr_in  clientAddr;      	// client's socket address     
struct timeval timeout={0,5000};//5ms		
socklen_t                 sockAddrSize;	// size of socket address structure    
int sFd,re,ico,jco=0,re_sign=0,flags,count,icount=0;

#ifdef WINNT
int  iOpValue;
WORD wVersionRequested;
WSADATA wsaData;
int err;

	 wVersionRequested = MAKEWORD(2, 2);
 
//     err = WSAStartup(MAKEWORD(1,1),&wsaData);	//initial Ws2_32.dll by a process;
	 err = WSAStartup(wVersionRequested, &wsaData);
     if (err != 0) {

        printf("WSAStartup failed with error: %d\n", err);
         return ((void *)0);
     }


     if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
              printf("Could not find a usable version of Winsock.dll\n");
         WSACleanup();
         return ((void *)0);
    }
	 else{
         printf("The Winsock 2.2 dll was found okay\n");
	 }
#endif	

	opt=1;
	
	sockAddrSize = sizeof (struct sockaddr_in);    
	memset ((char *) &serverAddr,0, sockAddrSize);    
	serverAddr.sin_family = AF_INET;    
	serverAddr.sin_port = htons (LAN1_PORT_NUM);    
	serverAddr.sin_addr.s_addr = htonl (INADDR_ANY);   
	memset(buff,0,LAN1_BUFF);
	memset(bradio_Con, 0, Dradio_Con_Msg);
	
	sFd = socket (AF_INET, SOCK_STREAM, 0);  
	if(sFd<0)
	{    
#ifdef  debug_print 
		printf("build socket error\n");
#endif
		return ((void *)0);
	}   
	//���÷�����socket	
#ifndef WINNT
	flags=fcntl(sFd,F_GETFL,0);
	fcntl(sFd,flags|O_NONBLOCK);
	setsockopt(sFd,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout,sizeof(struct timeval));
	setsockopt(sFd,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout,sizeof(struct timeval));
	if(setsockopt(sFd,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(struct timeval))<0){
#else
	iOpValue = 1; 
	setsockopt(sFd, SOL_SOCKET, SO_REUSEADDR, 
	 			(const char *)&iOpValue,  sizeof(iOpValue));
	if( setsockopt( sFd, IPPROTO_TCP,TCP_NODELAY, 
		(const char *)&iOpValue,  sizeof(iOpValue))<0){	
#endif	
	   	perror ("setsockopet error\n"); 
#ifdef WINNT
		closesocket(sFd);
#else
		close(sFd);
#endif
   
		return ((void *)0);
	}

	if (bind (sFd, (struct sockaddr *) &serverAddr, sockAddrSize) <0)    
	{    
#ifdef  debug_print 
		printf("bind socket error\n");
#endif
#ifdef WINNT
		closesocket(sFd);
#else
		close(sFd);
#endif   
		return ((void *)0);
	}   

	if (listen (sFd, SERVER_MAX_CONNECTIONS) <0)    
	{    
#ifdef  debug_print 
		printf("listen socket error\n");
#endif
#ifdef WINNT
		closesocket(sFd);
#else
		close(sFd);
#endif   
		return ((void *)0);
	}   
#ifndef WINNT
signal(SIGPIPE,SIG_IGN);
#endif
    while(1)
	{    
		count=0;
	    	LAN1_Fd=accept (sFd, (struct sockaddr *) (&clientAddr),&sockAddrSize) ;    
		if(LAN1_Fd<0)	
		{
#ifdef WINNT
			closesocket(LAN1_Fd);
#else
			close(LAN1_Fd);
#endif
			sleep_1(100);            //100ms	
			continue;
		}
		else 
	    		printf("lan1 connect is ok! MSG_DONTWAIT= %d  MSG_WAITALL=%d\n",MSG_DONTWAIT,MSG_WAITALL);
		while(1)
		{
	    		memset(buff, 0, LAN1_BUFF);
			if(count==0)
				Send_Bradio_State();//������̨״̬�ظ������ݼ���


			//������̨����
			memset(bradio_Con, 0, Dradio_Con_Msg);
			Send_Bradio_State();//������̨״̬�ظ������ݼ���
			//Send_Bradio_Ais();	//AIS��������
			icount++;
			if(icount==5)
			{
				Send_Bradio_Detail_Msg();//���ʹ����豸��ϸ����	
				icount=0;
			}
			count=1;
			//������̨���ͽ���

			re=recv(LAN1_Fd,(char *)buff,LAN1_BUFF,MSG_DONTWAIT);	//���÷�����recv
			if ( re<0) 
			{
				sleep_1(100);            //100ms					
				continue;
			}
			else if(re==0)
			{

#ifdef WINNT
				closesocket(LAN1_Fd);
#else
				close(LAN1_Fd);
#endif
				sleep_1(100);            //100ms					
				break;
			}
			else
			{    
				

				for(ico=0;ico<re;ico++)
				{
//		    			printf("0x%.2x ",buff[ico]);
					if(((buff[ico]=='$')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(1==re_sign))//������̨����ָ��ͷ��־
					{
						re_sign=1;
						bradio_Con[jco++]=buff[ico];
						if(jco==Dradio_Con_Msg)
						{
							jco=0;
							re_sign=0;
							if(CheckCRC(1, bradio_Con,Dradio_Con_Msg))
							{
//								printf("LAN1 rev is ok\n");
								USV_Control.Radio_Nummber=2;//������̨����ָ���־
								Dradio_Con_Analytical(2,bradio_Con);			//������̨����ָ�����
								memcpy((&monitor_all_inf.module_report_inf[MONITOR_BRADIO_MSG].report_detail[0]),(&bradio_Con[0]),Dradio_Con_Msg);	//ת�������̨����ָ��
								//task_time.lan1_task_1s=0;		//�رպ�̨������̨���Ƽ���
								//task_time.lan1_task_250ms=0;
								//Bradio_Con_Sign=1;

								//������̨����
								//memset(bradio_Con, 0, Dradio_Con_Msg);
								//Send_Bradio_State();//������̨״̬�ظ������ݼ���
								//icount++;
								//if(icount==5)
								//{
								//	Send_Bradio_Detail_Msg();//���ʹ����豸��ϸ����	
								//	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_BRADIO_SN].send_ok_number++;
								//	icount=0;
								//}
								//count=1;
							}
						}
					}
					if(((buff[ico]=='#')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(2==re_sign))//������̨��������ָ��ͷ��־
					{
						if(re_sign<2)
							re_sign+=2;
						bradio_Sal[jco++]=buff[ico];
						if(jco>7)
							Sail_len = bradio_Sal[6]*256+bradio_Sal[7];
						if(jco==Sail_len)
						{
							jco=0;
							re_sign=0;
							if(CheckCRC(1,bradio_Sal,Sail_len))
							{
								Sailing_Sign = 2;
								USV_Sailing.Radio_Nummber =2;
								Dradio_Sal_Analytical(bradio_Sal);
								USV_State.Sailing_Nummber = USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber;
								USV_State.USV_Sailing_Intel_Sign = 1; //�յ��������񣬴�����
								Get_Return_Point();
								Sailing_Cnt_Old = 1;
								Get_Compensating_Dst();
							}
							memset(bradio_Sal,0,Dradio_Sal_Msg);
						}
					}
					if((buff[0]=='%')&&(buff[1]=='U')&&(buff[2]=='S')&&(buff[3]=='V')&&(buff[8]==0x0A))//������ͷ���Ʊ���
					{
						//����ʱ��200ms������
					}
				}			
			}


			sleep_1(100);            //100ms	
	    	}   
	}    
#ifdef WINNT
	closesocket(sFd);
#else
	close(sFd);
#endif
	return ((void *)0);
}   






//������̨����״̬
void Send_Bradio_State()
{
	uint8 CRC;
//	int ret=0,count;
	CRC=0;
	USV_State_Current[0]='$';
//	for(count=9;count<62;count++)
//		USV_State_Current[count]=count*2;
	GetCheck(&CRC,USV_State_Current,sizeof(USV_State_Current));
	USV_State_Current[64]=CRC;
//	ret=write(LAN1_Fd, USV_State_Current, State_Current_Num);
       	send (LAN1_Fd,(char *)USV_State_Current,State_Current_Num,MSG_DONTWAIT);   	
 //      		ret=send (LAN1_Fd,USV_State_Current,State_Current_Num,MSG_WAITALL);  
#ifndef WINNT
#ifdef	debug_print
	if(ret<0)
	{
		printf("send kuandai radio error!\n");	
		printf("errno=%d\n",errno);//errno=88,Socket operation on non-socket
	}
		
#endif
#endif
	return	;
}

//������̨����AIS����
void Send_Bradio_Ais()
{
	uint8 aisMsgBuff[AIS_Buff_Num];
	uint8 Msg_Sign,Msg_Num,CRC,count;
	int length;
	StruAisMsg AisMsgTmp;

	Msg_Sign = AIS_Sign	;	//AIS�豸��
	Msg_Num =  AIS_Buff_Num;
	CRC=0;
	if(AisMsgCost(&AisMsgTmp) == 0)
		return;
	
	memset((char *)&aisMsgBuff[0],0,sizeof(aisMsgBuff));
	Fill_Msg_Header(aisMsgBuff,Msg_Sign,Msg_Num);
	
	memcpy((char *)&aisMsgBuff[12],(char *)&AisMsgTmp.frame_buff[0],AisMsgTmp.frame_length);

	
	aisMsgBuff[125] = 1;
	aisMsgBuff[126] = '*';
	GetCheck(&CRC,aisMsgBuff,sizeof(aisMsgBuff));
	
	aisMsgBuff[127] = CRC;

		send (LAN1_Fd,(char *)aisMsgBuff,AIS_Buff_Num,MSG_DONTWAIT);

	return;

}



//����SP��̨����״̬
void Send_Bradio_SP_State()
{
	uint8 CRC;
	//	int ret=0,count;
	CRC=0;
	USV_State_Current[0]='$';
	//	for(count=9;count<62;count++)
	//		USV_State_Current[count]=count*2;
	GetCheck(&CRC,USV_State_Current,sizeof(USV_State_Current));
	USV_State_Current[64]=CRC;
	//	ret=write(LAN1_Fd, USV_State_Current, State_Current_Num);
	send (clientSocket,(char *)USV_State_Current,State_Current_Num,MSG_DONTWAIT);   	
	//      		ret=send (LAN1_Fd,USV_State_Current,State_Current_Num,MSG_WAITALL);  
#ifndef WINNT
#ifdef	debug_print
	if(ret<0)
	{
		printf("send kuandai radio error!\n");	
		printf("errno=%d\n",errno);//errno=88,Socket operation on non-socket
	}

#endif
#endif
	return	;
}



//������̨���ʹ����豸��ϸ����
void Send_Bradio_Detail_Msg(void)
{
	Send_Motor_Detail_State();
	sleep_1(1);
	Send_Rudder_Detail_State();
	sleep_1(1);
	Send_Stable_Platform_Detail_State();
	sleep_1(1);
	Send_UAV_Detail_State();
	sleep_1(1);
	Send_MCU_State();
	sleep_1(1);
	Send_Hull_State();
	sleep_1(1);
	Send_Smart_Navigation_Msg();
	sleep_1(1);
	Send_Panel_Control_Msg();
	sleep_1(1);
	Send_Power_Control_Msg();
	sleep_1(1);
	Send_Energy_Control_Msg();
	sleep_1(1);
	return	;
}
//���ͷ�������ϸ��Ϣ
void Send_Motor_Detail_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Motor_Detail_Buff[Motor_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Motor_Detail_Sign;
	Msg_Num=Motor_Detail_Buff_Num;
	CRC=0;
	memset(Motor_Detail_Buff,0,Motor_Detail_Buff_Num);
	Update_Motor_Detail_State();
	Fill_Msg_Header(Motor_Detail_Buff,Msg_Sign, Msg_Num );
	Motor_Detail_Buff[12]=(((Motor_Detail_St.Motor_Spd_L)&0xff00)>>8);	
	Motor_Detail_Buff[13]=((Motor_Detail_St.Motor_Spd_L)&0x00ff);	
	Motor_Detail_Buff[14]=(((Motor_Detail_St.Motor_Spd_R)&0xff00)>>8);	
	Motor_Detail_Buff[15]=((Motor_Detail_St.Motor_Spd_R)&0x00ff);	
	Motor_Detail_Buff[16]=Motor_Detail_St.Cool_Temp_L;	
	Motor_Detail_Buff[17]=Motor_Detail_St.Cool_Temp_R;	
	Motor_Detail_Buff[18]=Motor_Detail_St.Cool_Lev_L;	
	Motor_Detail_Buff[19]=Motor_Detail_St.Cool_Lev_R;	
	Motor_Detail_Buff[20]=(((Motor_Detail_St.Oil_Temp_L)&0xff00)>>8);	
	Motor_Detail_Buff[21]=((Motor_Detail_St.Oil_Temp_L)&0x00ff);	
	Motor_Detail_Buff[22]=(((Motor_Detail_St.Oil_Temp_R)&0xff00)>>8);	
	Motor_Detail_Buff[23]=((Motor_Detail_St.Oil_Temp_R)&0x00ff);	
	Motor_Detail_Buff[24]=Motor_Detail_St.Fuel_Temp_L;	
	Motor_Detail_Buff[25]=Motor_Detail_St.Fuel_Temp_R;	
	Motor_Detail_Buff[26]=Motor_Detail_St.Inlet_Air_Temp_L;	
	Motor_Detail_Buff[27]=Motor_Detail_St.Inlet_Air_Temp_R;	
	Motor_Detail_Buff[28]=Motor_Detail_St.Oil_Pressure_L;	
	Motor_Detail_Buff[29]=Motor_Detail_St.Oil_Pressure_R;	
	Motor_Detail_Buff[30]=Motor_Detail_St.Supercharger_Pressure_L;	
	Motor_Detail_Buff[31]=Motor_Detail_St.Supercharger_Pressure_R;	
	Motor_Detail_Buff[32]=(((Motor_Detail_St.Fuel_Usage_L)&0xff00)>>8);	
	Motor_Detail_Buff[33]=((Motor_Detail_St.Fuel_Usage_L)&0x00ff);	
	Motor_Detail_Buff[34]=(((Motor_Detail_St.Fuel_Usage_R)&0xff00)>>8);	
	Motor_Detail_Buff[35]=((Motor_Detail_St.Fuel_Usage_R)&0x00ff);	
	Motor_Detail_Buff[36]=(((Motor_Detail_St.Average_Fuel_Usage_L)&0xff00)>>8);	
	Motor_Detail_Buff[37]=((Motor_Detail_St.Average_Fuel_Usage_L)&0x00ff);	
	Motor_Detail_Buff[38]=(((Motor_Detail_St.Average_Fuel_Usage_R)&0xff00)>>8);	
	Motor_Detail_Buff[39]=((Motor_Detail_St.Average_Fuel_Usage_R)&0x00ff);	
	Motor_Detail_Buff[40]=(((Motor_Detail_St.Working_Time_L)&0xff000000)>>24);	
	Motor_Detail_Buff[41]=(((Motor_Detail_St.Working_Time_L)&0x00ff0000)>>16);	
	Motor_Detail_Buff[42]=(((Motor_Detail_St.Working_Time_L)&0x0000ff00)>>8);	
	Motor_Detail_Buff[43]=((Motor_Detail_St.Working_Time_L)&0x000000ff);	
	Motor_Detail_Buff[44]=(((Motor_Detail_St.Working_Time_R)&0xff000000)>>24);	
	Motor_Detail_Buff[45]=(((Motor_Detail_St.Working_Time_R)&0x00ff0000)>>16);	
	Motor_Detail_Buff[46]=(((Motor_Detail_St.Working_Time_R)&0x0000ff00)>>8);	
	Motor_Detail_Buff[47]=((Motor_Detail_St.Working_Time_R)&0x000000ff);	
	Motor_Detail_Buff[48]=(((Motor_Detail_St.Total_turn_L)&0xff000000)>>24);	
	Motor_Detail_Buff[49]=(((Motor_Detail_St.Total_turn_L)&0x00ff0000)>>16);	
	Motor_Detail_Buff[50]=(((Motor_Detail_St.Total_turn_L)&0x0000ff00)>>8);	
	Motor_Detail_Buff[51]=((Motor_Detail_St.Total_turn_L)&0x000000ff);	
	Motor_Detail_Buff[52]=(((Motor_Detail_St.Total_turn_R)&0xff000000)>>24);	
	Motor_Detail_Buff[53]=(((Motor_Detail_St.Total_turn_R)&0x00ff0000)>>16);	
	Motor_Detail_Buff[54]=(((Motor_Detail_St.Total_turn_R)&0x0000ff00)>>8);	
	Motor_Detail_Buff[55]=((Motor_Detail_St.Total_turn_R)&0x000000ff);	
	Motor_Detail_Buff[56]=Motor_Detail_St.Air_preeure_L;	
	Motor_Detail_Buff[57]=(((Motor_Detail_St.Single_Mileage)&0xff00)>>8);	
	Motor_Detail_Buff[58]=((Motor_Detail_St.Single_Mileage)&0x00ff);	
	Motor_Detail_Buff[59]=(((Motor_Detail_St.Total_Mileage)&0x00ff0000)>>16);	
	Motor_Detail_Buff[60]=(((Motor_Detail_St.Total_Mileage)&0x0000ff00)>>8);	
	Motor_Detail_Buff[61]=((Motor_Detail_St.Total_Mileage)&0x000000ff);	
	Motor_Detail_Buff[62]=(((Motor_Detail_St.Single_Fuel_Consume)&0xff00)>>8);	
	Motor_Detail_Buff[63]=((Motor_Detail_St.Single_Fuel_Consume)&0x00ff);	
	Motor_Detail_Buff[64]=(((Motor_Detail_St.Total_Fuel_Consume)&0x00ff0000)>>16);	
	Motor_Detail_Buff[65]=(((Motor_Detail_St.Total_Fuel_Consume)&0x0000ff00)>>8);	
	Motor_Detail_Buff[66]=((Motor_Detail_St.Total_Fuel_Consume)&0x000000ff);	
	Motor_Detail_Buff[67]=(((Motor_Detail_St.Ldle_Spd_L)&0xff00)>>8);	
	Motor_Detail_Buff[68]=((Motor_Detail_St.Ldle_Spd_L)&0x00ff);	
	Motor_Detail_Buff[69]=(((Motor_Detail_St.Ldle_Spd_R)&0xff00)>>8);	
	Motor_Detail_Buff[70]=((Motor_Detail_St.Ldle_Spd_R)&0x00ff);	
	Motor_Detail_Buff[71]=Motor_Detail_St.Ldle_Time_Left;	
	Motor_Detail_Buff[72]=Motor_Detail_St.Ldle_Time_Right;	
	
	Motor_Detail_Buff[73]+=Motor_Detail_St.SuperLoad_Alarm_Left;	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.SuperLoad_Alarm_Right)<<1);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.ECU_Alarm_L)<<2);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.ECU_Alarm_R)<<3);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_TEMP_Alarm_Left)<<4);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_TEMP_Alarm_Right)<<5);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_Level_Alarm_Left)<<6);	
	Motor_Detail_Buff[73]+=((Motor_Detail_St.Cooling_Level_Alarm_Right)<<7);	
	Motor_Detail_Buff[74]+=Motor_Detail_St.OilPressure_Alarm_Left;	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.OilPressure_Alarm_Right)<<1);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Oil_TEMP_Alarm_L)<<2);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Oil_TEMP_Alarm_R)<<3);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Supercharger_Pressure_Alarm_L)<<4);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.Supercharger_Pressure_Alarm_R)<<5);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.InletTEMP_Alarm_Left)<<6);	
	Motor_Detail_Buff[74]+=((Motor_Detail_St.InletTEMP_Alarm_Right)<<7);	
	Motor_Detail_Buff[75]+=Motor_Detail_St.Fuel_Pressure_Alarm_L;	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Pressure_Alarm_R)<<1);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Moisture_Alarm_L)<<2);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Fuel_Moisture_Alarm_R)<<3);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Nozzle_Pressure_L)<<4);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Nozzle_Pressure_R)<<5);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Electricity_Alarm_L)<<6);	
	Motor_Detail_Buff[75]+=((Motor_Detail_St.Electricity_Alarm_R)<<7);	
	Motor_Detail_Buff[76]+=Motor_Detail_St.Whole_Alarm_L;	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Whole_Alarm_R)<<1);	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Heatbeat_L)<<2);	
	Motor_Detail_Buff[76]+=((Motor_Detail_St.Heatbeat_R)<<3);	
	
//	for(count=12;count<77;count++)
//		Motor_Detail_Buff[count]=count;
	GetCheck(&CRC,Motor_Detail_Buff,sizeof(Motor_Detail_Buff));
	Motor_Detail_Buff[94]='*';
	Motor_Detail_Buff[95]=CRC;
       send (LAN1_Fd,(char *)Motor_Detail_Buff,Msg_Num,MSG_DONTWAIT);   
//       ret=send (LAN1_Fd,Motor_Detail_Buff,Msg_Num,MSG_WAITALL);   
/*#ifdef	debug_print
	if(ret<0)
		printf("send motor state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;
}
//����ͼ����̨���ͷ�����״̬
void Update_Motor_Detail_State(void)
{
	//����״̬
	Motor_Detail_St.Motor_Spd_L=Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190;
	Motor_Detail_St.Motor_Spd_R=Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190;
	Motor_Detail_St.Cool_Temp_L=Motor_CAN_State.Motor_PGN65262_State[0].Cool_Temp_spn110;
	Motor_Detail_St.Cool_Temp_R=Motor_CAN_State.Motor_PGN65262_State[1].Cool_Temp_spn110;
	Motor_Detail_St.Cool_Lev_L=Motor_CAN_State.Motor_PGN65263_State[0].Cool_Level_spn111;
	Motor_Detail_St.Cool_Lev_R=Motor_CAN_State.Motor_PGN65263_State[1].Cool_Level_spn111;
	Motor_Detail_St.Oil_Temp_L=Motor_CAN_State.Motor_PGN65262_State[0].Oil_Temp_spn175;
	Motor_Detail_St.Oil_Temp_R=Motor_CAN_State.Motor_PGN65262_State[1].Oil_Temp_spn175;
	Motor_Detail_St.Fuel_Temp_L=Motor_CAN_State.Motor_PGN65262_State[0].Fuel_Temp_spn174;
	Motor_Detail_St.Fuel_Temp_R=Motor_CAN_State.Motor_PGN65262_State[1].Fuel_Temp_spn174;
	Motor_Detail_St.Inlet_Air_Temp_L=Motor_CAN_State.Motor_PGN65270_State[0].Inlet_Air_Temp_spn105;
	Motor_Detail_St.Inlet_Air_Temp_R=Motor_CAN_State.Motor_PGN65270_State[1].Inlet_Air_Temp_spn105;
	Motor_Detail_St.Oil_Pressure_L=Motor_CAN_State.Motor_PGN65263_State[0].Oil_Pressure_spn100;
	Motor_Detail_St.Oil_Pressure_R=Motor_CAN_State.Motor_PGN65263_State[1].Oil_Pressure_spn100;
	Motor_Detail_St.Supercharger_Pressure_L=Motor_CAN_State.Motor_PGN65270_State[0].Supercharger_Pressure_spn102;
	Motor_Detail_St.Supercharger_Pressure_R=Motor_CAN_State.Motor_PGN65270_State[1].Supercharger_Pressure_spn102;
	Motor_Detail_St.Fuel_Usage_L=Motor_CAN_State.Motor_PGN65266_State[0].Fuel_Usage_spn183;
	Motor_Detail_St.Fuel_Usage_R=Motor_CAN_State.Motor_PGN65266_State[1].Fuel_Usage_spn183;
	Motor_Detail_St.Average_Fuel_Usage_L=Motor_CAN_State.Motor_PGN65266_State[0].Average_Fuel_Economy_spn185;
	Motor_Detail_St.Average_Fuel_Usage_R=Motor_CAN_State.Motor_PGN65266_State[1].Average_Fuel_Economy_spn185;
	Motor_Detail_St.Working_Time_L=Motor_CAN_State.Motor_PGN65253_State[0].Working_Time_spn247;
	Motor_Detail_St.Working_Time_R=Motor_CAN_State.Motor_PGN65253_State[1].Working_Time_spn247;
	Motor_Detail_St.Total_turn_L=Motor_CAN_State.Motor_PGN65253_State[0].Total_turn_spn249;
	Motor_Detail_St.Total_turn_R=Motor_CAN_State.Motor_PGN65253_State[1].Total_turn_spn249;
	Motor_Detail_St.Air_preeure_L=Motor_CAN_State.Motor_PGN65269_State[0].Air_Pressure_spn108;
	Motor_Detail_St.Single_Mileage=Get_Mileage();
	Motor_Detail_St.Total_Mileage=Motor_Detail_St.Single_Mileage;//cxy��ô����
	Motor_Detail_St.Single_Fuel_Consume+=Motor_CAN_State.Motor_PGN65257_State[0].Single_Fuel_Consume_spn182;
	Motor_Detail_St.Single_Fuel_Consume+=Motor_CAN_State.Motor_PGN65257_State[1].Single_Fuel_Consume_spn182;
	Motor_Detail_St.Total_Fuel_Consume+=Motor_CAN_State.Motor_PGN65257_State[0].Total_Fuel_Consume_spn250;
	Motor_Detail_St.Total_Fuel_Consume+=Motor_CAN_State.Motor_PGN65257_State[1].Total_Fuel_Consume_spn250;
	Motor_Detail_St.Ldle_Spd_L=Motor_CAN_State.Motor_PGN61444_State[0].Motor_Spd_spn190;
	Motor_Detail_St.Ldle_Spd_R=Motor_CAN_State.Motor_PGN61444_State[1].Motor_Spd_spn190;
	Motor_Detail_St.Ldle_Time_Left=Motor_CAN_State.Motor_PGN65244_State[0].Ldle_Time_spn235;
	Motor_Detail_St.Ldle_Time_Right=Motor_CAN_State.Motor_PGN65244_State[1].Ldle_Time_spn235;
	//���±������ڷ�����CAN���Ĳ�����ʵʱ����
	Motor_Detail_St.SuperLoad_Alarm_Left=Motor_Detail_St.SuperLoad_Alarm_Left;	
	Motor_Detail_St.SuperLoad_Alarm_Right=Motor_Detail_St.SuperLoad_Alarm_Right;	
	Motor_Detail_St.ECU_Alarm_L=USV_State.Conrtol_System_Msg.Engine_L;	
	Motor_Detail_St.ECU_Alarm_R=USV_State.Conrtol_System_Msg.Engine_R;	
	Motor_Detail_St.Cooling_TEMP_Alarm_Left=Motor_Detail_St.Cooling_TEMP_Alarm_Left;	
	Motor_Detail_St.Cooling_TEMP_Alarm_Right=Motor_Detail_St.Cooling_TEMP_Alarm_Right;	
	Motor_Detail_St.Cooling_Level_Alarm_Left=Motor_Detail_St.Cooling_Level_Alarm_Left;	
	Motor_Detail_St.Cooling_Level_Alarm_Right=Motor_Detail_St.Cooling_Level_Alarm_Right;	
	Motor_Detail_St.OilPressure_Alarm_Left=Motor_Detail_St.OilPressure_Alarm_Left;	
	Motor_Detail_St.OilPressure_Alarm_Right=Motor_Detail_St.OilPressure_Alarm_Right;	
	Motor_Detail_St.Oil_TEMP_Alarm_L=Motor_Detail_St.Oil_TEMP_Alarm_L;	
	Motor_Detail_St.Oil_TEMP_Alarm_R=Motor_Detail_St.Oil_TEMP_Alarm_R;	
	Motor_Detail_St.Supercharger_Pressure_Alarm_L=Motor_Detail_St.Supercharger_Pressure_Alarm_L;	
	Motor_Detail_St.Supercharger_Pressure_Alarm_R=Motor_Detail_St.Supercharger_Pressure_Alarm_R;	
	Motor_Detail_St.InletTEMP_Alarm_Left=Motor_Detail_St.InletTEMP_Alarm_Left;	
	Motor_Detail_St.InletTEMP_Alarm_Right=Motor_Detail_St.InletTEMP_Alarm_Right;	
	Motor_Detail_St.Fuel_Pressure_Alarm_L=Motor_Detail_St.Fuel_Pressure_Alarm_L;	
	Motor_Detail_St.Fuel_Pressure_Alarm_R=Motor_Detail_St.Fuel_Pressure_Alarm_R;	
	Motor_Detail_St.Fuel_Moisture_Alarm_L=Motor_Detail_St.Fuel_Moisture_Alarm_L;	
	Motor_Detail_St.Fuel_Moisture_Alarm_R=Motor_Detail_St.Fuel_Moisture_Alarm_R;	
	Motor_Detail_St.Nozzle_Pressure_L=Motor_Detail_St.Nozzle_Pressure_L;	
	Motor_Detail_St.Nozzle_Pressure_R=Motor_Detail_St.Nozzle_Pressure_R;	
	Motor_Detail_St.Electricity_Alarm_L=Motor_Detail_St.Electricity_Alarm_L;	
	Motor_Detail_St.Electricity_Alarm_R=Motor_Detail_St.Electricity_Alarm_R;	
	Motor_Detail_St.Whole_Alarm_L=USV_State.Conrtol_System_Msg.Engine_L;	
	Motor_Detail_St.Whole_Alarm_R=USV_State.Conrtol_System_Msg.Engine_R;	
	Motor_Detail_St.Heatbeat_L=(!USV_State.Conrtol_System_Msg.Engine_L);	
	Motor_Detail_St.Heatbeat_R=(!USV_State.Conrtol_System_Msg.Engine_R);	
	return	;
}
//�������
uint32 Get_Mileage(void)
{
	uint32 uDst;
	double Dst;
	double Lau=Smart_Navigation_St.USV_Latitude_Degree;
	Lau+=Smart_Navigation_St.USV_Latitude_Minute/60.0;
	Lau+=Smart_Navigation_St.USV_Latitude_Second/3600.0;
	Lau+=Smart_Navigation_St.USV_Latitude_Decimal_2/360000.0;
	Lau+=Smart_Navigation_St.USV_Latitude_Decimal_4/36000000.0;
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		Lau=0-Lau;	//����Ϊ��
	double Lnu=Smart_Navigation_St.USV_Longitude_Degree;
	Lnu+=Smart_Navigation_St.USV_Longitude_Minute/60.0;
	Lnu+=Smart_Navigation_St.USV_Longitude_Second/3600.0;
	Lnu+=Smart_Navigation_St.USV_Longitude_Decimal_2/360000.0;
	Lnu+=Smart_Navigation_St.USV_Longitude_Decimal_4/36000000.0;
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		Lnu=0-Lnu;//��γΪ��
	lau_old=Lau;
	lnu_old=Lnu;
	Dst=Get_distance(lau_old, lnu_old, Lau,Lnu);
	if(Dst>60*Nmile/3600)
	{
		Dst=0;
#ifdef	debug_print
		printf("Get mileage error!\n");
#endif
	}
	Mileage+=(uint32)Dst;
	uDst=Mileage/125;//ͼ����̨����״̬�����ÿλ�ֱ���Ϊ0.125km
	return uDst;
}

//�������ܶ���ϸ��Ϣ
void Send_Rudder_Detail_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Rudder_Detail_Buff[Rudder_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Rudder_Detail_Sign;
	Msg_Num=Rudder_Detail_Buff_Num;
	CRC=0;
	memset(Rudder_Detail_Buff,0,Rudder_Detail_Buff_Num);
	Update_Rudder_Detail_State();
	Fill_Msg_Header(Rudder_Detail_Buff,Msg_Sign, Msg_Num );
	Rudder_Detail_Buff[12]=(((Rudder_Detail_St.Rudder_Angle_Left_St)&0xff00)>>8);	
	Rudder_Detail_Buff[13]=((Rudder_Detail_St.Rudder_Angle_Left_St)&0x00ff);	
	Rudder_Detail_Buff[14]=(((Rudder_Detail_St.Rudder_Angle_Right_St)&0xff00)>>8);	
	Rudder_Detail_Buff[15]=((Rudder_Detail_St.Rudder_Angle_Right_St)&0x00ff);	
	Rudder_Detail_Buff[16]=(((Rudder_Detail_St.Rudder_Angle_Left_Con)&0xff00)>>8);	
	Rudder_Detail_Buff[17]=((Rudder_Detail_St.Rudder_Angle_Left_Con)&0x00ff);	
	Rudder_Detail_Buff[18]=(((Rudder_Detail_St.Rudder_Angle_Right_Con)&0xff00)>>8);	
	Rudder_Detail_Buff[19]=((Rudder_Detail_St.Rudder_Angle_Right_Con)&0x00ff);	
	Rudder_Detail_Buff[20]=Rudder_Detail_St.Speed_Limit;	
	Rudder_Detail_Buff[21]=(Rudder_Detail_St.Motor_Speed_L&0xff00)<<8;	
	Rudder_Detail_Buff[22]=Rudder_Detail_St.Motor_Speed_L&0x00ff;	
	Rudder_Detail_Buff[23]=(Rudder_Detail_St.Motor_Speed_R&0xff00)<<8;	
	Rudder_Detail_Buff[24]=Rudder_Detail_St.Motor_Speed_R&0x00ff;	
	Rudder_Detail_Buff[25]=Rudder_Detail_St.System_TEMP;	
	Rudder_Detail_Buff[26]=Rudder_Detail_St.System_Power_5;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_12)<<1;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_3)<<2;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Power_24)<<3;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.System_Check)<<4;	
	Rudder_Detail_Buff[26]+=(Rudder_Detail_St.Heatbeat_Rudder)<<5;	
	
//	for(count=12;count<21;count++)
//		Rudder_Detail_Buff[count]=count;

	GetCheck(&CRC,Rudder_Detail_Buff,sizeof(Rudder_Detail_Buff));
	Rudder_Detail_Buff[30]='*';
	Rudder_Detail_Buff[31]=CRC;
       send (LAN1_Fd,(char *)Rudder_Detail_Buff,Msg_Num,MSG_DONTWAIT);
 //      ret=send (LAN1_Fd,Rudder_Detail_Buff,Msg_Num,MSG_WAITALL);
/*#ifdef	debug_print
	if(ret<0)
		printf("send ruuder state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	Rudder_Detail_St.Heatbeat_Rudder=0;
	return	;
}
//�������ܶ���Ϣ
void Update_Rudder_Detail_State(void)
{
	Rudder_Detail_St.Rudder_Angle_Left_St=(SR_CAN_State.Rudder_Angle_L_spn520766*10-2500);
	Rudder_Detail_St.Rudder_Angle_Right_St=(SR_CAN_State.Rudder_Angle_R_spn520767*10-2500);
	Rudder_Detail_St.Rudder_Angle_Left_Con=(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left*40-2500);
	Rudder_Detail_St.Rudder_Angle_Right_Con=(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Right*40-2500);	
	Rudder_Detail_St.Speed_Limit=USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit;
	Rudder_Detail_St.Motor_Speed_L=SR_CAN_State.Motor_Speed_L_spn520770;
	Rudder_Detail_St.Motor_Speed_R=SR_CAN_State.Motor_Speed_R_spn520771;
	Rudder_Detail_St.System_Power_5=SR_CAN_State.System_Power_5_spn520779;
	Rudder_Detail_St.System_Power_12=SR_CAN_State.System_Power_12_spn520780;
	Rudder_Detail_St.System_Power_3=SR_CAN_State.System_Power_3_spn520781;
	Rudder_Detail_St.System_Power_24=SR_CAN_State.System_Power_24_spn520782;
	Rudder_Detail_St.System_Check=SR_CAN_State.System_Check_spn520783;
	Rudder_Detail_St.System_TEMP=SR_CAN_State.System_TEMP_spn520784;
		
	return	;
}

//�����ȶ�ƽ̨��ϸ��Ϣ
void Send_Stable_Platform_Detail_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Stable_Platform_Detail_Buff[Stable_Platform_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Stable_Platform_Sign;
	Msg_Num=Stable_Platform_Detail_Buff_Num;
	CRC=0;
	memset(Stable_Platform_Detail_Buff,0,Stable_Platform_Detail_Buff_Num);
	Update_Stable_Platform_Detail_State();
	Fill_Msg_Header(Stable_Platform_Detail_Buff,Msg_Sign, Msg_Num );
	Stable_Platform_Detail_Buff[12]=Stable_Platform_St.Stable_Platform_RL_St;	
	Stable_Platform_Detail_Buff[13]=Stable_Platform_St.Stable_Platform_AT_St;	
	Stable_Platform_Detail_Buff[14]=Stable_Platform_St.System_TEMP;	
	Stable_Platform_Detail_Buff[15]=Stable_Platform_St.System_Power_5;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_12)<<1;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_3)<<2;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Power_24)<<3;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.System_Check)<<4;	
	Stable_Platform_Detail_Buff[15]+=(Stable_Platform_St.Stable_Platform_Heatbeat)<<5;	
//	for(count=12;count<15;count++)
//		Stable_Platform_Detail_Buff[count]=count;
	GetCheck(&CRC,Stable_Platform_Detail_Buff,sizeof(Stable_Platform_Detail_Buff));
	Stable_Platform_Detail_Buff[30]='*';
	Stable_Platform_Detail_Buff[31]=CRC;
    	send (LAN1_Fd,(char *)Stable_Platform_Detail_Buff,Msg_Num,MSG_DONTWAIT);
//    	ret=send (LAN1_Fd,Stable_Platform_Detail_Buff,Msg_Num,MSG_WAITALL);
/*#ifdef	debug_print
	if(ret<0)
		printf("send stable platform state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	Stable_Platform_St.Stable_Platform_Heatbeat=0;
	return	;
}
void Update_Stable_Platform_Detail_State(void)
{
	Stable_Platform_St.Stable_Platform_RL_St=ST_PL_CAN_State.Stable_Platform_RL_spn521342;
	Stable_Platform_St.Stable_Platform_AT_St=ST_PL_CAN_State.Stable_Platform_AT_spn521343;
	Stable_Platform_St.System_TEMP=ST_PL_CAN_State.System_TEMP_spn520344;
	Stable_Platform_St.System_Power_5=ST_PL_CAN_State.System_Power_5_spn521345;
	Stable_Platform_St.System_Power_12=ST_PL_CAN_State.System_Power_12_spn521346;
	Stable_Platform_St.System_Power_3=ST_PL_CAN_State.System_Power_3_spn521347;
	Stable_Platform_St.System_Power_24=ST_PL_CAN_State.System_Power_24_spn521348;
	Stable_Platform_St.System_Check=ST_PL_CAN_State.System_Check_spn521349;
	return	;
}

//�������˻�ƽ̨��ϸ��Ϣ
void Send_UAV_Detail_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 UAV_Detail_Buff[UAV_Detail_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=UAV_Detail_Sign;
	Msg_Num=UAV_Detail_Buff_Num;
	CRC=0;
	memset(UAV_Detail_Buff,0,UAV_Detail_Buff_Num);
	Update_UAV_Detail_State();
	Fill_Msg_Header(UAV_Detail_Buff,Msg_Sign, Msg_Num );
	UAV_Detail_Buff[12]=UAV_Detail_St.Platform_Hatch_St;	
	UAV_Detail_Buff[12]+=((UAV_Detail_St.Platform_Lift_St)<<3);	
	UAV_Detail_Buff[12]+=((UAV_Detail_St.Platform_Open_St)<<6);	
	UAV_Detail_Buff[13]=UAV_Detail_St.UAV_Electricity;	
	UAV_Detail_Buff[14]=UAV_Detail_St.UAV_Con_Mod;	
	UAV_Detail_Buff[14]+=((UAV_Detail_St.UAV_Run_Mod)<<2);	
	UAV_Detail_Buff[14]+=((UAV_Detail_St.UAV_Charging_St)<<5);		
	UAV_Detail_Buff[15]=UAV_Detail_St.UAV_TEMP;		
	UAV_Detail_Buff[16]=UAV_Detail_St.UAV_5V_St;	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_12V_St)<<1);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_3V_St)<<2);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_24V_St)<<3);		
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_Check_St)<<4);	
	UAV_Detail_Buff[16]+=((UAV_Detail_St.UAV_Heatbeat)<<5);	
//	for(count=12;count<15;count++)
//		UAV_Detail_Buff[count]=count;
	GetCheck(&CRC,UAV_Detail_Buff,sizeof(UAV_Detail_Buff));
	UAV_Detail_Buff[30]='*';
	UAV_Detail_Buff[31]=CRC;
   	 send (LAN1_Fd,(char *)UAV_Detail_Buff,Msg_Num,MSG_DONTWAIT);
 //  	 ret=send (LAN1_Fd,UAV_Detail_Buff,Msg_Num,MSG_WAITALL);
/*#ifdef	debug_print
	if(ret<0)
		printf("send UAV state error!\n");	
#endif*/
	UAV_Detail_St.UAV_Heatbeat=0;
//	sleep_1(1);            //�ȴ�1ms
	return	;
}
void Update_UAV_Detail_State(void)
{
	UAV_Detail_St.Platform_Hatch_St=UAV_CAN_State.Platform_Hatch_spn520576;
	UAV_Detail_St.Platform_Lift_St=UAV_CAN_State.Platform_Lift_spn520577;
	UAV_Detail_St.Platform_Open_St=UAV_CAN_State.Platform_Open_spn520578;
	UAV_Detail_St.UAV_Electricity=UAV_CAN_State.UAV_Electricity_spn520579;
	UAV_Detail_St.UAV_Con_Mod=UAV_CAN_State.UAV_Con_spn520580;
	UAV_Detail_St.UAV_Run_Mod=UAV_CAN_State.UAV_Run_spn520581;
	UAV_Detail_St.UAV_Charging_St=UAV_CAN_State.UAV_Charging_spn520582;
	UAV_Detail_St.UAV_5V_St=UAV_CAN_State.System_Power_5_spn520583;
	UAV_Detail_St.UAV_12V_St=UAV_CAN_State.System_Power_12_spn520584;
	UAV_Detail_St.UAV_3V_St=UAV_CAN_State.System_Power_3_spn520585;
	UAV_Detail_St.UAV_24V_St=UAV_CAN_State.System_Power_24_spn520586;	
	UAV_Detail_St.UAV_Check_St=UAV_CAN_State.System_Check_spn520587;
	UAV_Detail_St.UAV_TEMP=UAV_CAN_State.System_TEMP_spn502588;
		
	return	;
}

//���Ϳ����������ϸ��Ϣ
void Send_MCU_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 MCU_Buff[MCU_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=MCU_Sign;
	Msg_Num=MCU_Buff_Num;
	CRC=0;
	memset(MCU_Buff,0,MCU_Buff_Num);
	Update_MCU_State();
	Fill_Msg_Header(MCU_Buff,Msg_Sign, Msg_Num );
	MCU_Buff[12]=MCU_St.MCU_TEMP_1;	
	MCU_Buff[13]=MCU_St.MCU_TEMP_2;	
	MCU_Buff[14]=MCU_St.MCU_TEMP_3;	
	MCU_Buff[15]=MCU_St.RAM_Memory;	
	MCU_Buff[16]=MCU_St.RAM_CPU;		//cxy��ȡϵͳ��Դ
	MCU_Buff[17]=MCU_St.MPC_Disk;	
	MCU_Buff[18]=MCU_St.MPC_Memory;	
	MCU_Buff[19]=MCU_St.MCP_CPU;	
	MCU_Buff[20]=MCU_St.Mesh_Bandwidth;	//cxy��ȡ����
//	for(count=12;count<21;count++)
//		MCU_Buff[count]=count;
	GetCheck(&CRC,MCU_Buff,sizeof(MCU_Buff));
	MCU_Buff[30]='*';
	MCU_Buff[31]=CRC;
       send (LAN1_Fd,(char *)MCU_Buff,Msg_Num,MSG_DONTWAIT); 
//       ret=send (LAN1_Fd,MCU_Buff,Msg_Num,MSG_WAITALL); 
/*#ifdef	debug_print
	if(ret<0)
		printf("send MCU state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;
}
void Update_MCU_State(void)
{
	
	MCU_St.MCU_TEMP_1=0;//cxy	
	MCU_St.MCU_TEMP_2=0;	
	MCU_St.MCU_TEMP_3=0;	
	//MCU_St.RAM_Memory=;	
	//MCU_St.RAM_CPU;		//��ȡϵͳ��Դ,��sysinfo�д���
	//MCU_St.MPC_Disk;	
	//MCU_St.MPC_Memory;	
	//MCU_St.MCP_CPU;		//��ȡMPC��Դ����lan0�߳��д���
	MCU_St.Mesh_Bandwidth=50;	//cxy��ȡ����

	return	;
}

//���ʹ�������ϸ��Ϣ
void Send_Hull_State(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Hull_Buff[Hull_Buff_Num];
//	int ret=0;
//	int count;
	Msg_Sign=Hull_Sign;
	Msg_Num=Hull_Buff_Num;
	CRC=0;
	memset(Hull_Buff,0,Hull_Buff_Num);
	Update_Hull_State();
	Fill_Msg_Header(Hull_Buff,Msg_Sign, Msg_Num );
	Hull_Buff[12]=Hull_St.Hull_TEMP_1;	
	Hull_Buff[13]=Hull_St.Hull_TEMP_2;	
	Hull_Buff[14]=Hull_St.Hull_TEMP_3;	
	Hull_Buff[15]+=Hull_St.Outfire_St;	
	Hull_Buff[15]+=((Hull_St.Ventilation_St)<<2);	
	Hull_Buff[15]+=((Hull_St.Water_Pump_St)<<4);
	Hull_Buff[15]+=((Hull_St.Water_Level_St)<<6);
	Hull_Buff[16]=(((Hull_St.Motor_Vibration_1)&0xff000000)>>24);	
	Hull_Buff[17]=(((Hull_St.Motor_Vibration_1)&0x00ff0000)>>16);	
	Hull_Buff[18]=(((Hull_St.Motor_Vibration_1)&0x0000ff00)>>8);	
	Hull_Buff[19]=((Hull_St.Motor_Vibration_1)&0x000000ff);	
	Hull_Buff[20]=(((Hull_St.Motor_Vibration_2)&0xff000000)>>24);	
	Hull_Buff[21]=(((Hull_St.Motor_Vibration_2)&0x00ff0000)>>16);	
	Hull_Buff[22]=(((Hull_St.Motor_Vibration_2)&0x0000ff00)>>8);	
	Hull_Buff[23]=((Hull_St.Motor_Vibration_2)&0x000000ff);	
	Hull_Buff[24]=(((Hull_St.Motor_Vibration_3)&0xff000000)>>24);	
	Hull_Buff[25]=(((Hull_St.Motor_Vibration_3)&0x00ff0000)>>16);	
	Hull_Buff[26]=(((Hull_St.Motor_Vibration_3)&0x0000ff00)>>8);	
	Hull_Buff[27]=((Hull_St.Motor_Vibration_3)&0x000000ff);	
	Hull_Buff[28]=(((Hull_St.Motor_Vibration_4)&0xff000000)>>24);	
	Hull_Buff[29]=(((Hull_St.Motor_Vibration_4)&0x00ff0000)>>16);	
	Hull_Buff[30]=(((Hull_St.Motor_Vibration_4)&0x0000ff00)>>8);	
	Hull_Buff[31]=((Hull_St.Motor_Vibration_4)&0x000000ff);	
	Hull_Buff[32]=(((Hull_St.Motor_Vibration_5)&0xff000000)>>24);	
	Hull_Buff[33]=(((Hull_St.Motor_Vibration_5)&0x00ff0000)>>16);	
	Hull_Buff[34]=(((Hull_St.Motor_Vibration_5)&0x0000ff00)>>8);	
	Hull_Buff[35]=((Hull_St.Motor_Vibration_5)&0x000000ff);	
	Hull_Buff[36]=(((Hull_St.Motor_Vibration_6)&0xff000000)>>24);	
	Hull_Buff[37]=(((Hull_St.Motor_Vibration_6)&0x00ff0000)>>16);	
	Hull_Buff[38]=(((Hull_St.Motor_Vibration_6)&0x0000ff00)>>8);	
	Hull_Buff[39]=((Hull_St.Motor_Vibration_6)&0x000000ff);	
//	for(count=12;count<39;count++)
//		Hull_Buff[count]=count;
	GetCheck(&CRC,Hull_Buff,sizeof(Hull_Buff));
	Hull_Buff[62]='*';
	Hull_Buff[63]=CRC;
	send (LAN1_Fd,(char *)Hull_Buff,Msg_Num,MSG_DONTWAIT);
// ret=send (LAN1_Fd,Hull_Buff,Msg_Num,MSG_WAITALL);
/*#ifdef	debug_print
	if(ret<0)
		printf("send HULL state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;

}
void Update_Hull_State(void)
{
	//cxy
	
	return	;
}

//����AIS������Ϣ
void Send_AIS_Msg(void)
{
	uint8 Msg_Sign,Msg_Num,CRC,count;
 	char AIS_msg[AIS_Buff_Num];
	uint8 AIS_Buff[AIS_Buff_Num];
//	int ret=0;
	Msg_Sign=AIS_Sign;
	Msg_Num=AIS_Buff_Num;
	CRC=0;
	memset(AIS_msg,0,AIS_Buff_Num);
	memset(AIS_Buff,0,AIS_Buff_Num);
	Fill_Msg_Header(AIS_Buff,Msg_Sign, Msg_Num );
	strcpy(&AIS_msg[12],AIS_Msg_St.AIS_Message);
	for(count=0;count<AIS_Buff_Num;count++)
		AIS_Buff[count]=(uint8)(AIS_msg[count]);
	AIS_Buff[125]=AIS_Msg_St.AIS_Heatbeat;	

	GetCheck(&CRC,AIS_Buff,sizeof(AIS_Buff));
	AIS_Buff[126]='*';
	AIS_Buff[127]=CRC;
       send (LAN1_Fd,(char *)AIS_Buff,Msg_Num,MSG_DONTWAIT); 
  //     ret=send (LAN1_Fd,AIS_Buff,Msg_Num,MSG_WAITALL); 
/*#ifdef	debug_print
	if(ret<0)
		printf("send AIS state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;
}
//���Ͷ๦�ܹߵ���ϸ��Ϣ
void Send_Smart_Navigation_Msg(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Smart_Navigation_Buff[Smart_Navigation_Buff_Num];
//	int ret=0;
	Msg_Sign=Smart_Navigation_Sign;
	Msg_Num=Smart_Navigation_Buff_Num;
	CRC=0;
	memset(Smart_Navigation_Buff,0,Smart_Navigation_Buff_Num);
	Fill_Msg_Header(Smart_Navigation_Buff,Msg_Sign, Msg_Num );
	Smart_Navigation_Buff[12]=(((Smart_Navigation_St.USV_Heading)&0xff00)>>8);	
	Smart_Navigation_Buff[13]=((Smart_Navigation_St.USV_Heading)&0x00ff);	
	Smart_Navigation_Buff[14]=(((Smart_Navigation_St.USV_Speed)&0xff00)>>8);	
	Smart_Navigation_Buff[15]=((Smart_Navigation_St.USV_Speed)&0x00ff);	
	Smart_Navigation_Buff[16]=(((Smart_Navigation_St.USV_Roll)&0xff00)>>8);	
	Smart_Navigation_Buff[17]=((Smart_Navigation_St.USV_Roll)&0x00ff);	
	Smart_Navigation_Buff[18]=(((Smart_Navigation_St.USV_Pitch)&0xff00)>>8);	
	Smart_Navigation_Buff[19]=((Smart_Navigation_St.USV_Pitch)&0x00ff);	
	Smart_Navigation_Buff[20]=(((Smart_Navigation_St.USV_Bow)&0xff00)>>8);	
	Smart_Navigation_Buff[21]=((Smart_Navigation_St.USV_Bow)&0x00ff);	
	Smart_Navigation_Buff[22]=(((Smart_Navigation_St.USV_Transverse)&0xff00)>>8);	
	Smart_Navigation_Buff[23]=((Smart_Navigation_St.USV_Transverse)&0x00ff);	
	Smart_Navigation_Buff[24]=(((Smart_Navigation_St.USV_Surge)&0xff00)>>8);	
	Smart_Navigation_Buff[25]=((Smart_Navigation_St.USV_Surge)&0x00ff);	
	Smart_Navigation_Buff[26]=(((Smart_Navigation_St.USV_Heave)&0xff00)>>8);	
	Smart_Navigation_Buff[27]=((Smart_Navigation_St.USV_Heave)&0x00ff);	
	Smart_Navigation_Buff[28]=(((Smart_Navigation_St.USV_Yaw)&0xff00)>>8);	
	Smart_Navigation_Buff[29]=((Smart_Navigation_St.USV_Yaw)&0x00ff);	
	Smart_Navigation_Buff[30]=(((Smart_Navigation_St.USV_ROT)&0xff00)>>8);	
	Smart_Navigation_Buff[31]=((Smart_Navigation_St.USV_ROT)&0x00ff);	
	Smart_Navigation_Buff[32]=(((Smart_Navigation_St.USV_Height)&0x00ff0000)>>16);	
	Smart_Navigation_Buff[33]=(((Smart_Navigation_St.USV_Height)&0x0000ff00)>>8);
	Smart_Navigation_Buff[34]=((Smart_Navigation_St.USV_Height)&0x000000ff);	
	Smart_Navigation_Buff[35]+=Smart_Navigation_St.Latitude_Sign_St;	
	Smart_Navigation_Buff[35]+=((Smart_Navigation_St.Longitude_Sign_St)<<2);	
	Smart_Navigation_Buff[36]=Smart_Navigation_St.USV_Latitude_Degree;	
	Smart_Navigation_Buff[37]=Smart_Navigation_St.USV_Latitude_Minute;	
	Smart_Navigation_Buff[38]=Smart_Navigation_St.USV_Latitude_Second;	
	Smart_Navigation_Buff[39]=Smart_Navigation_St.USV_Latitude_Decimal_2;	
	Smart_Navigation_Buff[40]=Smart_Navigation_St.USV_Latitude_Decimal_4;	
	Smart_Navigation_Buff[41]=Smart_Navigation_St.USV_Longitude_Degree;	
	Smart_Navigation_Buff[42]=Smart_Navigation_St.USV_Longitude_Minute;	
	Smart_Navigation_Buff[43]=Smart_Navigation_St.USV_Longitude_Second;	
	Smart_Navigation_Buff[44]=Smart_Navigation_St.USV_Longitude_Decimal_2;	
	Smart_Navigation_Buff[45]=Smart_Navigation_St.USV_Longitude_Decimal_4;	
	Smart_Navigation_Buff[46]=Smart_Navigation_St.USV_Year;	
	Smart_Navigation_Buff[47]=Smart_Navigation_St.USV_Month;	
	Smart_Navigation_Buff[48]=Smart_Navigation_St.USV_Date;	
	Smart_Navigation_Buff[49]=Smart_Navigation_St.USV_Hour;	
	Smart_Navigation_Buff[50]=Smart_Navigation_St.USV_Minute;	
	Smart_Navigation_Buff[51]=Smart_Navigation_St.USV_Second;	
	Smart_Navigation_Buff[52]=Smart_Navigation_St.Satellite_Num_1;	
	Smart_Navigation_Buff[53]=Smart_Navigation_St.Satellite_Num_2;	
	Smart_Navigation_Buff[54]+=Smart_Navigation_St.Sys_State_1;	
	Smart_Navigation_Buff[54]+=((Smart_Navigation_St.Sys_State_2)<<4);	
	Smart_Navigation_Buff[55]+=Smart_Navigation_St.Power_Light;	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Serial_Light)<<2);	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Main_Antenna_Light)<<4);	
	Smart_Navigation_Buff[55]+=((Smart_Navigation_St.Minor_Antenna_Light)<<6);	
	Smart_Navigation_Buff[56]+=Smart_Navigation_St.Differerntial_Signal_Light;	
	Smart_Navigation_Buff[56]+=((Smart_Navigation_St.Differential_Position_Light)<<2);	
	Smart_Navigation_Buff[56]+=((Smart_Navigation_St.Smart_Navigation_Heatbeat)<<4);	
	Smart_Navigation_Buff[57]=Smart_Navigation_St.Latitude_Sign_St;
	Smart_Navigation_Buff[57]+=(Smart_Navigation_St.Longitude_Sign_St)<<2;
	Smart_Navigation_Buff[58]=Point_Return[1].Waypoint_Latitude_Degree;
	Smart_Navigation_Buff[59]=Point_Return[1].Waypoint_Latitude_Minute;
	Smart_Navigation_Buff[60]=Point_Return[1].Waypoint_Latitude_Second;
	Smart_Navigation_Buff[61]=Point_Return[1].Waypoint_Latitude_Decimal;
	Smart_Navigation_Buff[62]=Point_Return[1].Waypoint_Longitude_Degree;
	Smart_Navigation_Buff[63]=Point_Return[1].Waypoint_Longitude_Minute;
	Smart_Navigation_Buff[64]=Point_Return[1].Waypoint_Longitude_Second;
	Smart_Navigation_Buff[65]=Point_Return[1].Waypoint_Longitude_Decimal;	

	GetCheck(&CRC,Smart_Navigation_Buff,sizeof(Smart_Navigation_Buff));
	Smart_Navigation_Buff[78]='*';
	Smart_Navigation_Buff[79]=CRC;
       send (LAN1_Fd,(char *)Smart_Navigation_Buff,Msg_Num,MSG_DONTWAIT);   
 //      ret=send (LAN1_Fd,Smart_Navigation_Buff,Msg_Num,MSG_WAITALL);   
/*#ifdef	debug_print
	if(ret<0)
		printf("send smart navigation state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;
}

//���ʹ��ؿ��������Ϣ
void Send_Panel_Control_Msg(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Panel_Control_Buff[Panel_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Panel_Control_Sign;
	Msg_Num=Panel_Control_Buff_Num;
	CRC=0;
	memset(Panel_Control_Buff,0,Panel_Control_Buff_Num);
	Update_Panel_Control_Msg();
	Fill_Msg_Header(Panel_Control_Buff,Msg_Sign, Msg_Num );
	Panel_Control_Buff[12]=Panel_Control_St.Panel_Control_St;	

	GetCheck(&CRC,Panel_Control_Buff,sizeof(Panel_Control_Buff));
	Panel_Control_Buff[30]='*';
	Panel_Control_Buff[31]=CRC;
       send (LAN1_Fd,(char *)Panel_Control_Buff,Msg_Num,MSG_DONTWAIT);   
 //      ret=send (LAN1_Fd,Panel_Control_Buff,Msg_Num,MSG_WAITALL);   
/*#ifdef	debug_print
	if(ret<0)
		printf("send panel control state error!\n");	
#endif*/
//	sleep_1(1);            //�ȴ�1ms
	return	;
}
void Update_Panel_Control_Msg(void)
{
	Panel_Control_St.Panel_Control_St=SP_CAN_Model_Con.Get_USV_Con_spn521193;
	return	;
}
//������Դ����ϵͳ��ϸ��Ϣ
void Send_Energy_Control_Msg(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Energy_Control_Buff[Energy_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Energy_Control_Sign;
	Msg_Num=Energy_Control_Buff_Num;
	CRC=0;
	memset(Energy_Control_Buff,0,Energy_Control_Buff_Num);
	Update_Power_Control_Msg();
	Fill_Msg_Header(Energy_Control_Buff,Msg_Sign, Msg_Num );
	Energy_Control_Buff[12]=Energy_Control_St.Battery_Left;	
	Energy_Control_Buff[13]=Energy_Control_St.Battery_TEMP_Left;	
	Energy_Control_Buff[14]=Energy_Control_St.Battery_Power_Left;	
	Energy_Control_Buff[15]=Energy_Control_St.Battery_Current_Left;	
	Energy_Control_Buff[16]=Energy_Control_St.Battery_Voltage_Left;	
	Energy_Control_Buff[17]=Energy_Control_St.Battery_Right;	
	Energy_Control_Buff[18]=Energy_Control_St.Battery_TEMP_Right;	
	Energy_Control_Buff[19]=Energy_Control_St.Battery_Power_Right;	
	Energy_Control_Buff[20]=Energy_Control_St.Battery_Current_Right;	
	Energy_Control_Buff[21]=Energy_Control_St.Battery_Voltage_Right;	
	Energy_Control_Buff[22]=Energy_Control_St.Battery_Charge_Left;	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Chager_Right)<<1);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_TEMP_Alarm_Left)<<2);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_TEMP_ALarm_Right)<<3);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Capacity_Left)<<4);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Battery_Capacity_Right)<<5);	
	Energy_Control_Buff[22]+=((Energy_Control_St.Power_Control_Heatbeat)<<6);	
	Energy_Control_Buff[23]=Energy_Control_St.SYS_DIN1_St;	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN2_St)<<1);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN3_St)<<2);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN4_St)<<3);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN5_St)<<4);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN6_St)<<5);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN7_St)<<6);	
	Energy_Control_Buff[23]+=((Energy_Control_St.SYS_DIN8_St)<<7);	
	Energy_Control_Buff[24]=Energy_Control_St.SYS_5V_St;	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_12V_St)<<1);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_3V_St)<<2);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_24V_St)<<3);	
	Energy_Control_Buff[24]+=((Energy_Control_St.SYS_Check_St)<<4);	
	Energy_Control_Buff[25]=Energy_Control_St.SYS_TEMP;	

	GetCheck(&CRC,Energy_Control_Buff,sizeof(Energy_Control_Buff));
	Energy_Control_Buff[46]='*';
	Energy_Control_Buff[47]=CRC;
      send (LAN1_Fd,(char *)Energy_Control_Buff,Msg_Num,MSG_DONTWAIT); 
//       ret=send (LAN1_Fd,Energy_Control_Buff,Msg_Num,MSG_WAITALL); 
/*#ifdef	debug_print
	if(ret<0)
		printf("send Energy control state error!\n");	
#endif*/
	Energy_Control_St.Power_Control_Heatbeat=0;
	//	sleep_1(1);            //�ȴ�1ms
	return	;
}
void Update_Energy_Control_Msg(void)
{
	Energy_Control_St.Battery_Left=BAT_CON_CAN_State.Battery_Left_spn521554;
	Energy_Control_St.Battery_TEMP_Left=BAT_CON_CAN_State.Battery_TEMP_L_spn521555;
	Energy_Control_St.Battery_Power_Left=BAT_CON_CAN_State.Battery_Power_L_spn521556;
	Energy_Control_St.Battery_Current_Left=BAT_CON_CAN_State.Battery_Current_L_spn521557;
	Energy_Control_St.Battery_Voltage_Left=BAT_CON_CAN_State.Battery_Voltage_L_spn521558;
	Energy_Control_St.Battery_Right=BAT_CON_CAN_State.Battery_Right_spn521560;
	Energy_Control_St.Battery_TEMP_Right=BAT_CON_CAN_State.Battery_TEMP_R_spn521561;
	Energy_Control_St.Battery_Power_Right=BAT_CON_CAN_State.Battery_Power_R_spn521562;
	Energy_Control_St.Battery_Current_Right=BAT_CON_CAN_State.Battery_Current_R_spn521563;
	Energy_Control_St.Battery_Voltage_Right=BAT_CON_CAN_State.Battery_Voltage_R_spn521564;
	Energy_Control_St.Battery_Charge_Left=BAT_CON_CAN_State.Battery_Charge_L_spn521535;
	Energy_Control_St.Battery_Chager_Right=BAT_CON_CAN_State.Battery_Chager_R_spn521536;
	Energy_Control_St.Battery_TEMP_Alarm_Left=BAT_CON_CAN_State.Battery_TEMP_Alarm_L_spn521537;
	Energy_Control_St.Battery_TEMP_ALarm_Right=BAT_CON_CAN_State.Battery_TEMP_ALarm_R_spn521538;
	Energy_Control_St.Battery_Capacity_Left=BAT_CON_CAN_State.Battery_Capacity_L_spn521539;
	Energy_Control_St.Battery_Capacity_Right=BAT_CON_CAN_State.Battery_Capacity_R_spn521540;
	Energy_Control_St.SYS_DIN1_St=BAT_CON_CAN_State.Plug_Connection_St_spn521541;
	Energy_Control_St.SYS_DIN2_St=BAT_CON_CAN_State.Plug_Connection_St_spn521542;
	Energy_Control_St.SYS_DIN3_St=BAT_CON_CAN_State.Plug_Connection_St_spn521543;
	Energy_Control_St.SYS_DIN4_St=BAT_CON_CAN_State.Plug_Connection_St_spn521544;
	Energy_Control_St.SYS_DIN5_St=BAT_CON_CAN_State.Plug_Connection_St_spn521545;
	Energy_Control_St.SYS_DIN6_St=BAT_CON_CAN_State.Plug_Connection_St_spn521546;
	Energy_Control_St.SYS_DIN7_St=BAT_CON_CAN_State.Plug_Connection_St_spn521547;
	Energy_Control_St.SYS_DIN8_St=BAT_CON_CAN_State.Plug_Connection_St_spn521548;
	Energy_Control_St.SYS_5V_St=BAT_CON_CAN_State.System_Power_5_spn521549;
	Energy_Control_St.SYS_12V_St=BAT_CON_CAN_State.System_Power_12_spn521550;
	Energy_Control_St.SYS_3V_St=BAT_CON_CAN_State.System_Power_3_spn521551;
	Energy_Control_St.SYS_24V_St=BAT_CON_CAN_State.System_Power_24_spn521552;
	Energy_Control_St.SYS_Check_St=BAT_CON_CAN_State.System_Check_spn521553;
	Energy_Control_St.SYS_TEMP=BAT_CON_CAN_State.System_TEMP_spn520566;
	
	return;
}

//����Դ�������Ϳ�������
void Send_Power_Control_Msg(void)
{
	uint8 Msg_Sign,Msg_Num,CRC;
 	uint8 Power_Control_Buff[Power_Control_Buff_Num];
//	int ret=0;
	Msg_Sign=Power_Control_Sign;
	Msg_Num=Power_Control_Buff_Num;
	CRC=0;
	memset(Power_Control_Buff,0,Power_Control_Buff_Num);
	Update_Power_Control_Msg();
	Fill_Msg_Header(Power_Control_Buff,Msg_Sign, Msg_Num );
	Power_Control_Buff[12]=Power_Control_St.Stable_Platform_Power;	
	Power_Control_Buff[12]+=((Power_Control_St.UAV_Power)<<1);	
	Power_Control_Buff[12]+=((Power_Control_St.Horn_Power)<<2);	
	Power_Control_Buff[12]+=((Power_Control_St.Navigationlight_Power)<<3);	
	Power_Control_Buff[12]+=((Power_Control_St.D_radar_Power)<<4);	
	Power_Control_Buff[12]+=((Power_Control_St.Camera_main_Power)<<5);	
	Power_Control_Buff[12]+=((Power_Control_St.Application_24V)<<6);	
	Power_Control_Buff[12]+=((Power_Control_St.Searchlight_Power)<<7);	
	Power_Control_Buff[13]=Power_Control_St.Camera_ahead_Power;	
	Power_Control_Buff[13]+=((Power_Control_St.Camera_lesser_Power)<<1);	
	Power_Control_Buff[13]+=((Power_Control_St.Camera_tail_Power)<<2);	
	Power_Control_Buff[13]+=((Power_Control_St.Application_12V)<<3);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_5)<<4);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_12)<<5);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Power_3)<<6);	
	Power_Control_Buff[13]+=((Power_Control_St.System_Check)<<7);	
	Power_Control_Buff[14]=Power_Control_St.System_Voltage;	
	Power_Control_Buff[15]=Power_Control_St.System_Electricity;	
	Power_Control_Buff[16]=Power_Control_St.DIN1_Connection_St;
	Power_Control_Buff[16]+=((Power_Control_St.DIN2_Connection_St)<<1);
	Power_Control_Buff[16]+=((Power_Control_St.DIN3_Connection_St)<<2);
	Power_Control_Buff[16]+=((Power_Control_St.DIN4_Connection_St)<<3);
	Power_Control_Buff[16]+=((Power_Control_St.System_Heatbeat)<<4);
	GetCheck(&CRC,Power_Control_Buff,sizeof(Power_Control_Buff));
	Power_Control_Buff[30]='*';
	Power_Control_Buff[31]=CRC;
       send (LAN1_Fd,(char *)Power_Control_Buff,Msg_Num,MSG_DONTWAIT); 
//       ret=send (LAN1_Fd,Power_Control_Buff,Msg_Num,MSG_WAITALL); 
/*	if(ret<0)
		printf("send Power control state error!\n");	*/
	Power_Control_St.System_Heatbeat=0;
	//	sleep_1(1);            //�ȴ�1ms
	return	;

}
void Update_Power_Control_Msg(void)
{
	Power_Control_St.Stable_Platform_Power=POW_CAN_State.Stable_Platform_Power_spn520958;	
	Power_Control_St.UAV_Power=POW_CAN_State.UAV_Power_spn520959;	
	Power_Control_St.Horn_Power=POW_CAN_State.Horn_Power_spn520960;	
	Power_Control_St.Navigationlight_Power=POW_CAN_State.Navigationlight_Power_spn520961;	
	Power_Control_St.D_radar_Power=POW_CAN_State.D_radar_Power_spn520962;	
	Power_Control_St.Camera_main_Power=POW_CAN_State.Camera_main_Power_spn520963;	
	Power_Control_St.Application_24V=POW_CAN_State.Application_24V_spn520964;	
	Power_Control_St.Searchlight_Power=POW_CAN_State.Searchlight_Power_spn520965;	
	Power_Control_St.Camera_ahead_Power=POW_CAN_State.Camera_ahead_Power_spn520966;	
	Power_Control_St.Camera_lesser_Power=POW_CAN_State.Camera_lesser_Power_spn520967;	
	Power_Control_St.Camera_tail_Power=POW_CAN_State.Camera_tail_Power_spn520968;	
	Power_Control_St.Application_12V=POW_CAN_State.Application_12V_spn520969;	
	Power_Control_St.System_Power_5=POW_CAN_State.System_Power_5_spn520970;	
	Power_Control_St.System_Power_12=POW_CAN_State.System_Power_12_spn520971;	
	Power_Control_St.System_Power_3=POW_CAN_State.System_Power_3_spn520972;	
	Power_Control_St.System_Check=POW_CAN_State.System_Check_spn520973;	
	Power_Control_St.System_Voltage=POW_CAN_State.System_Voltage_spn520974;	
	Power_Control_St.System_Electricity=POW_CAN_State.System_Electricity_spn520975;	
	Power_Control_St.DIN1_Connection_St=POW_CAN_State.DIN1_Connection_St_spn520976;
	Power_Control_St.DIN2_Connection_St=POW_CAN_State.DIN2_Connection_St_spn520977;
	Power_Control_St.DIN3_Connection_St=POW_CAN_State.DIN3_Connection_St_spn520978;
	Power_Control_St.DIN4_Connection_St=POW_CAN_State.DIN4_Connection_St_spn520979;
	Power_Control_St.System_Heatbeat=POW_CAN_State.System_Heatbeat;

}

//��д����ͷ
void Fill_Msg_Header(uint8  *msg_buff,uint8  msg_sign, uint8 msg_num )
{
	msg_buff[0]='%';
	msg_buff[1]='U';
	msg_buff[2]='S';
	msg_buff[3]='V';
	msg_buff[4]='R';
	msg_buff[5]=',';
	msg_buff[6]=USV_State.USV_Num;
	msg_buff[7]=',';
	msg_buff[8]=msg_sign;
	msg_buff[9]=',';
	msg_buff[10]=msg_num;
	msg_buff[11]=',';	
}


//int8 master_udp_rec_report(int16 sockid)
//{
//	fd_set set;
//	int16 ret;
//	int16 iLen;
//	struct timeval timeout;
//	//sockaddr_in from
//	socklen_t sockaddrlen;
//	int8 net_rec_buff[200];
//
//	FD_ZERO(&set);
//	FD_SET(sockid,&set);
//	timeout.tv_sec=0;
//	timeout.tv_usec=5;
//	ret = select(sockid+1,&set,NULL,NULL,&timeout);
//	if(ret<0)
//	{return TRUE;}
//	if(FD_ISSET(sockid,&set)){  //UDP����
//		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff), 0,(struct sockaddr *)&monitor_udp_inf.from_udp_ip, &sockaddrlen);
//		if(iLen>=1)
//		{
//			//deal_master_udp_rec_report((uint8 *)&net_rec_buff[0],iLen);
//		}
//	}
//	return TRUE;
//
//}
//
//
//
//
//
//
//void *Lan1_Bradio_UDP_Rec(void *aa)
//{
//
//	int16   sockid; //����վ
//
//	sockid = -1; 
//	uint16 count_100ms = 0;
//	uint16 count_500ms = 0;
//
//	for(;;)
//	{
//		if(-1 == sockid){
//			sockid = CreateUDPSocket(8888); //����UDP
//			if(sockid<0)
//				sleep_1(1000);
//			continue;
//		}
//		else
//		{
//			//�������ݣ����º�������
//			
//			//����USV state ����
//
//			count_100ms++;
//			count_500ms++;
//			if(count_100ms == 10)    //100ms ����״̬
//			{
//				//������̨״̬�ظ�
//				count_100ms = 0;
//			}
//			if(count_500ms == 50)    //500ms ���ʹ����豸��ϸ����
//			{
//				//���ʹ����豸��ϸ����
//				count_500ms = 0;
//			}
//		}
//		sleep_1(10);   //10ms
//
//	}
//}