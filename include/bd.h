/********************************************************************************

**** Copyright (C), 2017, �ķ��̱� Co., Ltd.                ****

********************************************************************************
* File Name     : BD_USV.h
* Author        : fushuai
* Date          : 2017-06-1
* Description   : .C file function description
* Version       : 1.0
* Function List :
*
* Record        :
* 1.Date        : 2017-06-1
*   Author      : fushuai
*   Modification: Created file
*************************************************************************************************************/
#ifndef BD_USV_H
#define BD_USV_H

#include "uart_api.h"



#define BD_MAX_PAYLOAD_LENGTH		210

#define BD_INSTRUCTION_SIZE		5
#define BD_PACKET_LEN				2
#define BD_USER_ADDR_SIZE			3
#define BD_CHECKSUM_SIZE			1

#define BD_PD  60  //ͨ��Ƶ��
#define BD_IPUC (BD_INSTRUCTION_SIZE+BD_PACKET_LEN+BD_USER_ADDR_SIZE+BD_CHECKSUM_SIZE)

#define BD_TXSQ_INFO_FRAME_SIZE 7 // 1 ����Ϣ��� + 3 ���û���ַ + 2�����ĳ��� + 1��Ӧ���ֽ�
#define BD_TXSQ_FRAME_SIZE (BD_IPUC + BD_TXSQ_INFO_FRAME_SIZE)

#define BD_DWSQ_INFO_FRAME_SIZE 11 // 1 ����Ϣ��� + 4 �߳����ݺ����߸� + 4 ��ѹ����+ 2��վƵ��
#define BD_DWSQ_FRAME_SIZE (BD_IPUC + BD_DWSQ_INFO_FRAME_SIZE)

#define BD_ICJC_INFO_FRAME_SIZE 1 //֡�ţ�ռһ���ֽ�
#define BD_ICJC_FRAME_SIZE (BD_IPUC + BD_ICJC_INFO_FRAME_SIZE)

#define BD_XTZJ_INFO_FRAME_SIZE 2 //�Լ�Ƶ�ȣ�ռ�����ֽ�
#define BD_XTZJ_FRAME_SIZE (BD_IPUC +BD_XTZJ_INFO_FRAME_SIZE)

#define BD_ICXX_INFO_FRAME_SIZE 11 //1��֡��+3��ͨ��ID+1���û�����+2������Ƶ��+1��ͨ�ŵȼ�+1�����ܱ�־+2�������û�����
#define BD_ICXX_FRAME_SIZE (BD_IPUC + BD_ICXX_INFO_FRAME_SIZE)

#define BD_TXXX_INFO_FRAME_SIZE 9 //1����Ϣ���+3�����ŷ���ַ+2������ʱ��+2�����ĳ���+1��CRC��־
#define BD_TXXX_FRAME_SIZE (BD_IPUC + BD_TXXX_INFO_FRAME_SIZE)
#define BD_TXXX_MAX_SIZE (BD_TXXX_FRAME_SIZE + BD_MAX_PAYLOAD_LENGTH)//TXXX�ɹ̶����Ⱥ;����ȹ���


#define BD_DWXX_INFO_FRAME_SIZE 20 // 1����Ϣ���+3��ѯ��ַ+4ʱ��+4����+4ά��+2�û�λ�õĴ�ظ߳�����+2�û�λ�õĸ߳��쳣ֵ
#define BD_DWXX_FRAME_SIZE (BD_IPUC + BD_DWXX_INFO_FRAME_SIZE)

#define BD_FKXX_INFO_FRAME_SIZE 5//1��������־+4��������Ϣ
#define BD_FKXX_FRAME_SIZE (BD_IPUC + BD_FKXX_INFO_FRAME_SIZE)

#define BD_ZJXX_INFO_FRAME_SIZE 10 //1��IC��״̬+1��Ӳ��״̬+1����ص���+1����վ״̬+6������״̬
#define BD_ZJXX_FRAME_SIZE (BD_IPUC + BD_ZJXX_INFO_FRAME_SIZE)

#define BD_SJXX_INFO_FRAME_SIZE  10 //�˴����������,���ڸ���ʵ���޸�
#define BD_SJXX_FRAME_SIZE (BD_IPUC + BD_SJXX_INFO_FRAME_SIZE)

#define BD_BBXX_INFO_FRAME_SIZE 10 //�˴���������ݣ����ڸ���ʵ���޸�
#define BD_BBXX_FRAME_SIZE (BD_IPUC + BD_BBXX_INFO_FRAME_SIZE)

#define BD_RX_MAX_DATA_SIZE BD_TXXX_MAX_SIZE 

#define BD_TXSQ_PAYLOAD_CHINESE 0x44
#define BD_TXSQ_PAYLOAD_BCD 0x46
#define BD_TXSQ_PAYLOAD_MIX 0x00

#define BD_DWSQ_TYPE 0x04

#define BD_DWXX_DW 0x0A //��λ��Ϣ����-������λ���ο�4.0��Э��
#define BD_DWXX_CX 0x2A //��λ��Ϣ����-ָ�ӻ���ѯ�û�����λ���ο�4.0��Э��



#define TIMEOUT 100

enum {
	DWXX_BUF = (1 << 0),
	TXXX_BUF = (1 << 1),
	ICXX_BUF = (1 << 2),
	ZJXX_BUF = (1 << 3),
	SJXX_BUF = (1 << 4),
	BBXX_BUF = (1 << 5),
	FKXX_BUF = (1 << 6),
};




/* ͨ����Ϣ��� */
typedef struct
{
	unsigned char packet_comm : 2;
	unsigned char transfer_format : 1;
	unsigned char ack : 1;
	unsigned char comm_way : 1;
	unsigned char has_key : 1;
	unsigned char rest : 2;
}bd_txxx_info_type;

typedef struct
{
	unsigned char hour;
	unsigned char minute;
}bd_send_time;

typedef struct
{
	bd_txxx_info_type  txxx_info_type;
	unsigned char src_user_addr[3];
	bd_send_time send_time;
	unsigned int payload_len;
	unsigned char payload[BD_MAX_PAYLOAD_LENGTH];
	unsigned char crc;
}bd_txxx_info;

typedef struct
{
	unsigned char instruction[5];
	unsigned int packet_len; //�����ṹ��ʱ���������ݱ�ʾ�䳤��
	unsigned char user_addr[3];
	bd_txxx_info txxx_info;
	unsigned char checksum;
}bd_txxx;

/*��λ��Ϣ���*/
typedef struct
{
	unsigned char local : 2;
	unsigned char type : 1;
	unsigned char has_key : 1;
	unsigned char accuracy : 1;
	unsigned char em_local : 1;
	unsigned char mul_value : 1;
	unsigned char high_type : 1;
}bd_dwxx_info_type;

/* ��λ��Ϣ���ݶ�*/
typedef struct
{
	bd_dwxx_info_type dwxx_info_type;
	unsigned char check_addr[3];
	unsigned char time[4];
	unsigned char longitude[4];
	unsigned char latitude[4];
	unsigned char high[2];
	unsigned char ex_high[2];
} bd_dwxx_info;

typedef struct
{
	unsigned char instruction[5];
	unsigned int packet_len;
	unsigned char user_addr[3];
	bd_dwxx_info dwxx_info;
	unsigned char checksum;
}bd_dwxx;

typedef struct
{
	unsigned char frame_id;
	unsigned char broadcast_id[3];
	unsigned char user_feature;
	unsigned int service_frequency;
	unsigned char comm_level;
	unsigned char encryption_flag;
	unsigned int user_num;
}bd_icxx_info;

/* IC��Ϣ */
typedef struct
{
	unsigned char instruction[5];
	unsigned int packet_len;
	unsigned char user_addr[3];
	bd_icxx_info icxx_info;
	unsigned char checksum;
}bd_icxx;

typedef struct
{
	unsigned char ic_status;
	unsigned char hw_status;
	unsigned char battery_quantity;
	unsigned char in_station_status;
	unsigned char power_status[6];
}bd_zjxx_info;

typedef struct
{
	unsigned char instruction[5];
	unsigned int packet_len;
	unsigned char user_addr[3];
	bd_zjxx_info zjxx_info;
	unsigned char checksum;
}bd_zjxx;

typedef struct
{
	unsigned int todo;//todo
}bd_sjxx;

typedef struct
{
	unsigned int todo;//todo
}bd_bbxx_struct;

typedef struct
{
	unsigned char fk_flag;
	unsigned char extra_info[4];
}bd_fkxx_info;

typedef struct
{
	unsigned char instruction[5];
	unsigned int packet_len;
	unsigned char user_addr[3];
	bd_fkxx_info fkxx_info;
	unsigned char checksum;
}bd_fkxx;

extern unsigned char bd_buf_bitmap;
extern unsigned char src_addr[3]; //= {0x02,0xad,0xf7}; test
extern unsigned char dst_addr[3]; //= {0x02,0xad,0xf7}; test
extern int bd_sign; //������������״̬

extern unsigned char send_txsq_payload[BD_MAX_PAYLOAD_LENGTH];
extern unsigned char transfer_format;
extern unsigned char recv_txxx_payload[BD_MAX_PAYLOAD_LENGTH];

int Recv_Data(unsigned char *rcv_buf, int len);
int Send_Data(unsigned char *buf, int len);

int BD_CheckCRC(int count, unsigned char *buf, int len);
unsigned char BD_GetCRC(int count, unsigned char *buf, int len);

int BD_Init();

void Create_txsq(unsigned char *src_user_addr, unsigned char *dst_user_addr,
	unsigned char transfer_format, unsigned char *payload,
	unsigned int payload_len, unsigned char *send_txsq_data);

void BD_Send_txsq(unsigned char *src_user_addr, unsigned char *dst_user_addr,
	unsigned char transfer_format, unsigned char *send_txsq_payload, unsigned int send_txsq_payload_len);
void Create_dwsq(unsigned char *src_user_addr);
void BD_Send_dwsq(unsigned char *src_user_addr);

void BD_Send_icjc();
void BD_Send_xtzj(unsigned char *src_user_addr, unsigned int frequency);
void BD_Send_txsq(unsigned char * src_user_addr,
	unsigned char * dst_user_addr,
	unsigned char transfer_format,
	unsigned char * send_txsq_payload,
	unsigned int send_txsq_payload_len);

int BD_RecvPacket_Depart(unsigned char *bd_rx_buf, int buf_len);//���յ���һ֡���ݷְ�

void BD_Analytical_txxx(unsigned char *txxx_buf, bd_txxx *txxx);//����һ֡ͨ����Ϣ����
void BD_Analytical_dwxx(unsigned char *dwxx_buf, bd_dwxx *dwxx);//����һ֡��λ��Ϣ����
void BD_Analytical_icxx(unsigned char *icxx_buf, bd_icxx *icxx);//����һ֡ic��Ϣ
void BD_Analytical_zjxx(unsigned char *zjxx_buf, bd_zjxx *zjxx);//����һ֡�Լ���Ϣ
void BD_Analytical_fkxx(unsigned char *fkxx_buf, bd_fkxx *fkxx);//����һ֡������Ϣ

void BD_Analytical_Message(unsigned char *message_buf);


void Print_dwxx(bd_dwxx *_bd_dwxx);
void Print_txxx(bd_txxx *_bd_txxx);
void Print_icxx(bd_icxx *_bd_icxx);
void Print_zjxx(bd_zjxx *_bd_zjxx);
void Print_fkxx(bd_fkxx *_bd_fkxx);
#endif