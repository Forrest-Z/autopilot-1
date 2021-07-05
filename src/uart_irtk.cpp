/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/uart_irtk.h"
#include "user_time.h"


#define IRTK_REC_TIMESLICE 10
#define UDP_IRTK_REC_PORT 9210
//#define UDP_IRTK_SED_PORT 9211

//port & addr
char irtk_udp_addr[30];
uint32 irtk_udp_port;

int8 rtk_locating_fuc;//��λ��ʽ
/******************************  Local Variable  *****************************/
UART_TYPE irtk_uart_fd;
UART_TYPE irtk_send_uart_fd;

char irtk_msg_buf[128];
char hdt_msg_buf[128]; //���ùߵ���HDT����
uint8 rtk_re_sign = 0;					 //���ձ�־
uint8 rtk_jco = 0;

iRTK_Smart_Navigation_Msg irtk_sn_msg;	//iRTKϵͳ��ϸ��Ϣ
iRTK_DETELL	irtk_msg;					//��������ڵ���ϵͳ����
COMM_SIGN	irtk_sign;					//iRTKͨѶ״̬

uint8	log_irtk_sign;
 uint64_t last_rtk_time=0;

//�ߵ�ת��UDP�˿ڱ���
int16				irtk_sockid;
UDP_INF_STRUCT		irtk_udp_inf;


int8 initIRTK(void);
int8 recRTKProcess(UART_TYPE uartid);

void rtkGPGGADecoding(char  *buff, int length);//GGA
void rtkGPRMCDecoding(char  *buff, int length);//RMC
void rtkGPGSTDecoding(char  *buff, int length);//GST

static int8 irtkUARTSendInit();
static int8 irtkUDPInit();
static int8 irtkRecvDataUDP(int16 sockid);

static void get_header_num(char *buff, char *begin_num, char *end_num, char *begin_sign, char *end_sign, char *count_sign, char *Symbol, int length);
static void get_value(char *buff, double * value, char *begin_num, char *end_num);

//static void jugdeGPSState();


void *uartRTKMsgProcess(void *aa)
{
	rtk_locating_fuc = 0;
	if (initIRTK() >= 0 && irtkUDPInit() >= 0){
		memset((char*)&irtk_msg, 0, sizeof(irtk_msg));					//��ʼ���ߵ�����
		comm_time_return(&(ins_sign.timer), &(ins_sign.comm_sign));
		printf("RTK initialize success!\n");
	}
	else{
	   printf("RTK initialize failed!\n"); 
		return ((void*)0);
	}
	for (;;){
		recRTKProcess(irtk_uart_fd);
		//jugdeGPSState();
		//irtkRecvDataUDP(irtk_sockid);
		sleep_1(IRTK_REC_TIMESLICE);	//10ms
	}
	close_uart(irtk_uart_fd);
	//close_uart(irtk_send_uart_fd);
	return ((void *)0);
}


int8 recRTKProcess(UART_TYPE uartid)
{
	int8	buff[MAX_UART_BUFF_LEN];
	int		ret;
	int		i;
	int		msg_len = 0;

	memset(buff, 0, MAX_UART_BUFF_LEN);
	ret = read_uart(uartid, buff, MAX_UART_BUFF_LEN);


	if (ret > 0)
	{
	//printf(">>>>>%s\n", buff);
	}
	else{
		return TRUE;
	}
	for (i = 0; i < ret; i++)
	{

		if ((buff[i] == '$') || (1 == rtk_re_sign))
		{
			if (rtk_jco >= 127)
			{
				rtk_jco = 0;
				rtk_re_sign = 0;
				break;
			}
			else
			{
				irtk_msg_buf[rtk_jco++] = buff[i];
				rtk_re_sign = 1;
			}
		}
		if (buff[i] == 0x0A)//LF
		{
			//У��
			if ((irtk_msg_buf[0] == '$') && (irtk_msg_buf[rtk_jco - 1] == 10) && (irtk_msg_buf[rtk_jco - 2] == 13))
			{
				//printf("RTK = %s\n",irtk_msg_buf);
				msg_len = rtk_jco - 2;
				rtk_jco = 0;
				if (GetCRC32(irtk_msg_buf, msg_len))
				{
					if ((irtk_msg_buf[0] == '$')  && (irtk_msg_buf[3] == 'G') && (irtk_msg_buf[4] == 'G') && (irtk_msg_buf[5] == 'A')) //GGA
					{
						rtkGPGGADecoding(irtk_msg_buf, msg_len);
					}
					if ((irtk_msg_buf[0] == '$')  && (irtk_msg_buf[3] == 'R') && (irtk_msg_buf[4] == 'M') && (irtk_msg_buf[5] == 'C')) //RMC
					{
						rtkGPRMCDecoding(irtk_msg_buf, msg_len);
					}
					if ((irtk_msg_buf[0] == '$')  && (irtk_msg_buf[3] == 'G') && (irtk_msg_buf[4] == 'S') && (irtk_msg_buf[5] == 'T')) //ZDA
					{
						rtkGPGSTDecoding(irtk_msg_buf, msg_len);
					}
					// printf("irtk uart===>%s\n", irtk_msg_buf);
				}
				if (irtk_sockid > 0){
					irtkSendDataUDP(irtk_sockid, (int8*)irtk_msg_buf, msg_len + 2);
					//write_uart(irtk_send_uart_fd, (int8*)irtk_msg_buf, msg_len + 2);
				}
				memset(irtk_msg_buf, 0, 128);
				rtk_re_sign = 0;
				comm_time_return(&(irtk_sign.timer), &(irtk_sign.comm_sign));
				last_rtk_time=user_time::get_millis();
			}
		}
	}
}
//NEMA0813��ʼ�� ��ȡ
void get_header_num(char *buff, char *begin_num, char *end_num, char *begin_sign, char *end_sign, char *count_sign, char *Symbol, int length)
{
	unsigned char count = 0, symbol_sign;
	int ico, jco;

	for (ico = 0; ico < length; ico++)
	{
		if (buff[ico] == 0x2c)								//\u9017\u53f7 2c--�ָ��� ,
		{
			count++;
		}
		if ((1 == count) && (0 == begin_sign[count - 1]))
		{
			begin_num[count - 1] = ico + 1;
			begin_sign[count - 1] = 1;
		}
		else if (count > 1)
		{
			if (0 == end_sign[count - 2])
			{
				end_num[count - 2] = ico;
				end_sign[count - 2] = 1;
			}
			if (0 == begin_sign[count - 1])
			{
				begin_num[count - 1] = ico + 1;
				begin_sign[count - 1] = 1;
			}
		}
	}
	for (jco = 0; jco < 20; jco++)
	{
		symbol_sign = begin_num[jco];
		if (buff[symbol_sign] == '-')//\u8d1f\u53f7
		{
			Symbol[jco] = 1;
			begin_num[jco]++;
		}
	}
}
//��þ���ֵ
void get_value(char *buff, double * value, char *begin_num, char *end_num)
{
	int ico, jco;
	char min_num[20], count_sign[20];
	memset(min_num, 0, sizeof(min_num));
	memset(count_sign, 0, sizeof(count_sign));

	for (jco = 0; jco < 20; jco++)
	{
		if (0 == end_num[jco])
			return;
		for (ico = begin_num[jco]; ico < end_num[jco]; ico++)
		{
			if (buff[ico] == 0x2e)				//�ԣ�Ϊ�ָ���
			{
				min_num[jco] = 1;
				continue;
			}
			if (1 != min_num[jco])
			{
				value[jco] *= 10.0;
				value[jco] += AscToHex(buff[ico]);			//��ASCII��ת16������
			}
			else
			{
				count_sign[jco]++;
				value[jco] += AscToHex(buff[ico])*(pow(0.1, count_sign[jco]));
			}
		}
	}
}


void rtkCommCal(void *)
{
	comm_time_cal(INS_COMM_TIMEMAX, &(irtk_sign.timer), &(irtk_sign.comm_sign));
	if (log_irtk_sign != irtk_sign.comm_sign && poweron_init && poweron_init)
	{
		switch (ins_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_RCV_TIMEOUT, 0);
			break;
		case COMM_CONNECT_FAIL:	
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_RCV_TIMEOUT, 1);
			break;
		default:
			break;
		}
	}
	log_irtk_sign = irtk_sign.comm_sign;
}

void irtkCommCalInit(void)
{
	addTask(4, rtkCommCal, (void*)0);
}



void irtkSendDataUDP(int16 sockid, int8* pBuf, int len)
{
	int dest_port;
	struct sockaddr_in server;

	dest_port = irtk_udp_port;//UDP_IRTK_SED_PORT;
	server.sin_family = AF_INET;
	server.sin_port = htons(dest_port);
	server.sin_addr.s_addr = inet_addr(irtk_udp_addr);

	memset(server.sin_zero, 0, 8);
	sendto(sockid, pBuf, len, 0, (struct sockaddr*)&server, sizeof(server));
}

int8 irtkRecvDataUDP(int16 sockid)
{
	fd_set	set;
	int16	ret;
	int16	iLen;
	struct timeval timeout;
	socklen_t	sockaddrlen;
	int8		net_rec_buff[200];

	memset(net_rec_buff, 0, sizeof(net_rec_buff));

	FD_ZERO(&set);
	FD_SET(sockid, &set);
	timeout.tv_sec = 0;
	timeout.tv_usec = 5;
	ret = select(sockid + 1, &set, NULL, NULL, &timeout);
	if (ret < 0){
		return 1;
	}
	if (FD_ISSET(sockid, &set)){		//UDP����
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid, net_rec_buff, sizeof(net_rec_buff), 0, (struct sockaddr *)&irtk_udp_inf.from_upd_ip, &sockaddrlen);
		if (iLen >= 1){
			printf("%s\n", net_rec_buff);
			write_uart(irtk_uart_fd, (int8*)net_rec_buff, iLen);
		}
	}
	return 1;
}

int8 irtkUDPInit()
{
	irtk_sockid = CreateUDPSocket(UDP_IRTK_REC_PORT);
	if (irtk_sockid >= 0){
		return 1;
	}
	else{
		return -1;
	}
}

int8 irtkUARTSendInit()
{
	int8 iret = 0;
	irtk_send_uart_fd = open_port(COM3); //COM3  IRTK_SEND_COM
#ifdef WINNT
	if (irtk_send_uart_fd == INVALID_HANDLE_VALUE)
#else
	if (irtk_send_uart_fd < 0)
#endif
	{
		iret = -1;
	}
	if (set_com_config(irtk_send_uart_fd, 115200, 8, 'N', 1) < 0)
		//if(set_com_config(UART5_Fd,38400,8,'N',1)<0)
		//if(set_com_config(UART5_Fd,19200,8,'N',1)<0)
	{
		iret = -1;
	}
	return iret;
}

int8 initIRTK(void)
{
	int8 iret = 0;
	irtk_uart_fd = open_port(COM4); //COM2 IRTK_COMMUNICATION_COM
#ifdef WINNT
	if (irtk_uart_fd == INVALID_HANDLE_VALUE)
#else
	if (irtk_uart_fd < 0)
#endif
	{
		iret = -1;
	}
	if (set_com_config(irtk_uart_fd, 115200, 8, 'N', 1) < 0)
	{
		iret = -1;
	}
	return iret;
}

//λ����Ϣ
//$GPGGA,HHMMSS.SS,DDMM.MMMM,S,DDDMM.MMMM,S,N,QQ,PP.P,SAAAAA.AA,M,��XXXX.XX,M,SSS,AAAA*CC<CR><LF>	5Hz
/*HHMMSS.SS  UTCʱ����
A ��Ч��λ��V��Ч��λ
ddmm.mmmmmά��d��ʾ�ȣ�m��ʾ��
N ��ʾ��γ��S��ʾ��γ
ddmm.mmmmm����d��ʾ�ȣ�m��ʾ��
E����W����
N GPS��λ��־
qq���Ƕ�λ��Ŀ
p.p
SAAAAA.AA ����
M��
*/
void rtkGPGGADecoding(char  *buff, int length)
{
	double value[20];
	char begin_num[20], end_num[20], begin_sign[20], end_sign[20], count_sign[20], Symbol[20];
	//	int ico;
	memset(Symbol, 0, sizeof(Symbol));
	memset(begin_num, 0, sizeof(begin_num));
	memset(end_num, 0, sizeof(end_num));
	memset(begin_sign, 0, sizeof(begin_sign));
	memset(end_sign, 0, sizeof(end_sign));
	memset(count_sign, 0, sizeof(count_sign));
	memset((char*)(&value), 0, sizeof(value));
	get_header_num(buff, begin_num, end_num, begin_sign, end_sign, count_sign, Symbol, length);
	get_value(buff, value, begin_num, end_num);

	irtk_msg.rtk_state.b1_diffSignalValid = value[5]; //GPS��λ��ʶ 0.�޶�λ 1.���㶨λ 2.��ֶ�λ 4.RTK�̶��� 5.RTK����
//	printf("irtk_msg.rtk_state.b1_diffSignalValid  = %d\n",irtk_msg.rtk_state.b1_diffSignalValid );


	irtk_sn_msg.Satellite_Num_1 = (int)value[6];
	irtk_msg.rtk_state.u8_sateliteNum1 = (int)value[6];		//����1�������Ǹ���
	if (Symbol[8])
		irtk_sn_msg.USV_Height = (uint32)(-100 * value[8] + 200000);
	else
		irtk_sn_msg.USV_Height = (uint32)(100 * value[8] + 200000);
	return;
}

void rtkGPRMCDecoding(char *buff, int length)
{
	double value[20], lng = 0.0, lat = 0.0;
	char begin_num[20], end_num[20], begin_sign[20], end_sign[20], count_sign[20], Symbol[20];
	int sys_state_num = 0;
	//	int ico;
	memset(Symbol, 0, sizeof(Symbol));
	memset(begin_num, 0, sizeof(begin_num));
	memset(end_num, 0, sizeof(end_num));
	memset(begin_sign, 0, sizeof(begin_sign));
	memset(end_sign, 0, sizeof(end_sign));
	memset(count_sign, 0, sizeof(count_sign));
	memset((char*)(&value), 0, sizeof(value));
	get_header_num(buff, begin_num, end_num, begin_sign, end_sign, count_sign, Symbol, length);	//��ȡ��ʼ�� ����ֹ��
	get_value(buff, value, begin_num, end_num);

	irtk_msg.speed = value[6];
	irtk_msg.motionDirection = value[7];


	irtk_msg.u16_speed = (uint16)(value[6] * 10);
	irtk_msg.u16_volecityDir = (uint16)(value[7] * 10);
	irtk_msg.u8_month = (uint8)(value[8] / 100 - (100 * (int)(value[8] / 10000)));
	irtk_msg.u8_date = (uint8)(value[8] / 10000);
	irtk_msg.u8_year = (uint8)(value[8] - 10000 * (irtk_msg.u8_date) - 100 * (irtk_msg.u8_month));
	if (irtk_msg.u8_year > 0)
		irtk_msg.rtk_state.b1_dateValid = 1;		//������Ч��־

	//��λ��Ч
	sys_state_num = begin_num[1];
	irtk_msg.rtk_state.c_rmcValid = buff[sys_state_num];

	// printf("irtk_msg.rtk_state.c_rmcValid  = %c\n",irtk_msg.rtk_state.c_rmcValid );

	if (irtk_msg.rtk_state.c_rmcValid == 'A')
	{

		irtk_msg.u8_latiDeg = (uint8)(value[2] / 100);								//value[2]--γ��
		irtk_msg.u8_latiMin = (uint8)(value[2] - 100 * ((int)(value[2] / 100)));
		lat = (value[2] - (int)(value[2])) * 60;
		irtk_msg.u8_latiSec = (int)(lat);										//��
		irtk_msg.u8_latiSecDec = (uint8)((lat - (int)(lat)) * 100);					//��С��λ1��2
		irtk_msg.u8_latiSecDec2 = (uint8)(((lat - (int)(lat)) * 100 - (int)((lat - (int)(lat)) * 100)) * 100);

		sys_state_num = begin_num[3];
		if (buff[sys_state_num] == 'N')						//������
			irtk_msg.u8_latiSt = 1;
		if (buff[sys_state_num] == 'S')						//�ϰ���
			irtk_msg.u8_latiSt = 0;

		//����
		irtk_msg.u8_longiDeg = (uint8)(value[4] / 100);							//����
		irtk_msg.u8_longiMin = (uint8)(value[4] - 100 * ((int)(value[4] / 100)));
		lng = (value[4] - (int)(value[4])) * 60;
		irtk_msg.u8_longiSec = (uint8)(lng);
		irtk_msg.u8_longiSecDec = (uint8)((lng - (int)(lng)) * 100);
		irtk_msg.u8_longiSecDec2 = (uint8)(((lng - (int)(lng)) * 100 - (int)((lng - (int)(lng)) * 100)) * 100);

		sys_state_num = begin_num[5];
		if (buff[sys_state_num] == 'E')						//������
			irtk_msg.u8_longiSt = 1;
		if (buff[sys_state_num] == 'W')						//������
			irtk_msg.u8_longiSt = 0;

		//�����;�γ��
		irtk_msg.latitude = (irtk_msg.u8_latiSt == 0 ? -1 : 1)*(irtk_msg.u8_latiDeg + ((double)(irtk_msg.u8_latiMin)) / 60.0 + \
			((double)(irtk_msg.u8_latiSec)) / 3600.0 + \
			((double)(irtk_msg.u8_latiSecDec)) / 360000.0);
		irtk_msg.longitude = (irtk_msg.u8_longiSt == 0 ? -1 : 1)*(irtk_msg.u8_longiDeg + ((double)(irtk_msg.u8_longiMin)) / 60.0 + \
			((double)(irtk_msg.u8_longiSec)) / 3600.0 + \
			((double)(irtk_msg.u8_longiSecDec)) / 360000.0);

	}

	return;
}

void rtkGPGSTDecoding(char *buff, int length)
{
	char *ptr = NULL;
	float tm;
	if (NULL == (ptr = strstr(buff, "$GNGST")))
	{
		return;
	}
	sscanf(ptr, "$GNGST,%f,%f", &tm, &irtk_msg.pseudorRangeError);
	//printf("rtk_dis == %f",irtk_msg.pseudorRangeError);
	//printf("ins_dis == %f\n", ins_msg.pseudorRangeError);
}

//�ж�iRTK�Ƿ�������߾�������

/*void jugdeGPSState()
{
	uint64_t tnow = user_time::get_millis();
	if(last_time!=0&&(tnow-last_time)>2000)
	{
		irtk_msg.rtk_state.b1_diffSignalValid =0;
		irtk_msg.rtk_state.c_rmcValid = 'N';
		printf("irtk_msg time out\n");
	}
	else
		{
		printf("=======%ld,%ld \n",tnow,last_time);
		}
	

	if(irtk_msg.rtk_state.b1_diffSignalValid >= 2 && irtk_msg.rtk_state.c_rmcValid == 'A')
	{
	//	printf("rtk_locating_fuc = 1\n");
		rtk_locating_fuc = 1;
	}
	else
	{
	//	printf("rtk_locating_fuc = 0\n");
		rtk_locating_fuc = 0;
	}
	//if (irtk_msg.rtk_state.c_rmcValid == 'A')
	//{
	//	rtk_locating_fuc = 1;

	//}

	//if (irtk_msg.rtk_state.c_rmcValid == 'V')
	//{
	//	rtk_locating_fuc = 0;
	//}
}
*/
