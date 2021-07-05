/********************************************************************************

**** Copyright (C), 2017, 四方继保 Co., Ltd.                ****

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

#define BD_PD  60  //通信频度
#define BD_IPUC (BD_INSTRUCTION_SIZE+BD_PACKET_LEN+BD_USER_ADDR_SIZE+BD_CHECKSUM_SIZE)

#define BD_TXSQ_INFO_FRAME_SIZE 7 // 1 个信息类别 + 3 个用户地址 + 2个电文长度 + 1个应答字节
#define BD_TXSQ_FRAME_SIZE (BD_IPUC + BD_TXSQ_INFO_FRAME_SIZE)

#define BD_DWSQ_INFO_FRAME_SIZE 11 // 1 个信息类别 + 4 高程数据和天线高 + 4 气压数据+ 2入站频度
#define BD_DWSQ_FRAME_SIZE (BD_IPUC + BD_DWSQ_INFO_FRAME_SIZE)

#define BD_ICJC_INFO_FRAME_SIZE 1 //帧号，占一个字节
#define BD_ICJC_FRAME_SIZE (BD_IPUC + BD_ICJC_INFO_FRAME_SIZE)

#define BD_XTZJ_INFO_FRAME_SIZE 2 //自检频度，占二个字节
#define BD_XTZJ_FRAME_SIZE (BD_IPUC +BD_XTZJ_INFO_FRAME_SIZE)

#define BD_ICXX_INFO_FRAME_SIZE 11 //1个帧号+3个通播ID+1个用户特征+2个服务频度+1个通信等级+1个加密标志+2个下属用户总数
#define BD_ICXX_FRAME_SIZE (BD_IPUC + BD_ICXX_INFO_FRAME_SIZE)

#define BD_TXXX_INFO_FRAME_SIZE 9 //1个信息类别+3个发信方地址+2个发信时间+2个电文长度+1个CRC标志
#define BD_TXXX_FRAME_SIZE (BD_IPUC + BD_TXXX_INFO_FRAME_SIZE)
#define BD_TXXX_MAX_SIZE (BD_TXXX_FRAME_SIZE + BD_MAX_PAYLOAD_LENGTH)//TXXX由固定长度和净长度构成


#define BD_DWXX_INFO_FRAME_SIZE 20 // 1个信息类别+3查询地址+4时间+4经度+4维度+2用户位置的大地高程数据+2用户位置的高程异常值
#define BD_DWXX_FRAME_SIZE (BD_IPUC + BD_DWXX_INFO_FRAME_SIZE)

#define BD_FKXX_INFO_FRAME_SIZE 5//1个反馈标志+4个附加信息
#define BD_FKXX_FRAME_SIZE (BD_IPUC + BD_FKXX_INFO_FRAME_SIZE)

#define BD_ZJXX_INFO_FRAME_SIZE 10 //1个IC卡状态+1个硬件状态+1个电池电量+1个入站状态+6个功率状态
#define BD_ZJXX_FRAME_SIZE (BD_IPUC + BD_ZJXX_INFO_FRAME_SIZE)

#define BD_SJXX_INFO_FRAME_SIZE  10 //此处虚设的数据,后期根据实况修改
#define BD_SJXX_FRAME_SIZE (BD_IPUC + BD_SJXX_INFO_FRAME_SIZE)

#define BD_BBXX_INFO_FRAME_SIZE 10 //此处虚设的数据，后期根据实况修改
#define BD_BBXX_FRAME_SIZE (BD_IPUC + BD_BBXX_INFO_FRAME_SIZE)

#define BD_RX_MAX_DATA_SIZE BD_TXXX_MAX_SIZE 

#define BD_TXSQ_PAYLOAD_CHINESE 0x44
#define BD_TXSQ_PAYLOAD_BCD 0x46
#define BD_TXSQ_PAYLOAD_MIX 0x00

#define BD_DWSQ_TYPE 0x04

#define BD_DWXX_DW 0x0A //定位信息参数-本机定位，参考4.0的协议
#define BD_DWXX_CX 0x2A //定位信息参数-指挥机查询用户机定位，参考4.0的协议



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




/* 通信信息类别 */
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
	unsigned int packet_len; //解析结构体时以整形数据表示其长度
	unsigned char user_addr[3];
	bd_txxx_info txxx_info;
	unsigned char checksum;
}bd_txxx;

/*定位信息类别*/
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

/* 定位信息数据段*/
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

/* IC信息 */
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
extern int bd_sign; //北斗请求卫星状态

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

int BD_RecvPacket_Depart(unsigned char *bd_rx_buf, int buf_len);//对收到的一帧数据分包

void BD_Analytical_txxx(unsigned char *txxx_buf, bd_txxx *txxx);//解析一帧通信信息数据
void BD_Analytical_dwxx(unsigned char *dwxx_buf, bd_dwxx *dwxx);//解析一帧定位信息数据
void BD_Analytical_icxx(unsigned char *icxx_buf, bd_icxx *icxx);//解析一帧ic信息
void BD_Analytical_zjxx(unsigned char *zjxx_buf, bd_zjxx *zjxx);//解析一帧自检信息
void BD_Analytical_fkxx(unsigned char *fkxx_buf, bd_fkxx *fkxx);//解析一帧反馈信息

void BD_Analytical_Message(unsigned char *message_buf);


void Print_dwxx(bd_dwxx *_bd_dwxx);
void Print_txxx(bd_txxx *_bd_txxx);
void Print_icxx(bd_icxx *_bd_icxx);
void Print_zjxx(bd_zjxx *_bd_zjxx);
void Print_fkxx(bd_fkxx *_bd_fkxx);
#endif