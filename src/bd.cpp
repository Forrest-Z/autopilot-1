#include "../include/usv_include.h"

unsigned char bd_buf_bitmap;
unsigned char src_addr[3]; //= {0x02,0xad,0xf7}; test
unsigned char dst_addr[3]; //= {0x02,0xad,0xf7}; test
int bd_sign; //������������״̬

//BD test
unsigned char send_txsq_payload[BD_MAX_PAYLOAD_LENGTH] = { 0xa4, 0xcb, 0xc4, 0xb7, 0xbd, 0xbc, 0xcc, 0xb1, 0xa3, 0xb1, 0xb1, 0xb6, 0xb7, 0xcd, 0xa8, 0xd0, 0xc5 };
unsigned char transfer_format = 0x46;
unsigned char recv_txxx_payload[BD_MAX_PAYLOAD_LENGTH];

#ifndef WINNT
int Recv_Data(unsigned char *rcv_buf, int len)
{
	int ret_read, retval, ret, pos, re_sign;
	fd_set rfds;
	int8 checksum;
	struct timeval _timeval;
	_timeval.tv_sec = TIMEOUT / 1000;
	_timeval.tv_usec = TIMEOUT % 1000 * 1000;//100000us = 0.1s
	pos = 0;
	re_sign = 0;
	while (1)
	{
		FD_ZERO(&rfds);
		FD_SET(UART3_Fd, &rfds);

		retval = select(UART3_Fd + 1, &rfds, NULL, NULL, &_timeval);
		if (retval == -1)
		{
			perror("select()");
			return -1;
		}

		else if (retval)
		{
			ret_read = read_uart(UART3_Fd, (int8*)(rcv_buf + pos), 1);
			if (-1 == ret_read)
			{
				perror("read()");
				close_uart(UART3_Fd);
				return -1;
			}
			pos++;
			if (len <= pos)
			{
				break;
			}

		}

		else
		{
			break;
		}
	}
}
#else
int Recv_Data(unsigned char *rcv_buf, int len)
{
	int re, ico = 0, jco = 0, re_sign = 0, bd_rx_len = 0;
	task_time.uart3_task_100s++;
	unsigned char temp_buf[MAX_UART_BUFF_LEN];
	memset(rcv_buf, 0, MAX_UART_BUFF_LEN);
	memset(temp_buf, 0, MAX_UART_BUFF_LEN);
	if ((re = read_uart(UART3_Fd, (int8*)temp_buf, len)) > 0)
	{
		for (ico = 0; ico < re; ico++)
		{
			if (((temp_buf[ico] == '$') && (temp_buf[ico + 1] == 'T') && (temp_buf[ico + 2] == 'X') && (temp_buf[ico + 3] == 'X') && (temp_buf[ico + 4] == 'X')) || (1 == re_sign))
			{
				re_sign = 1;//re is not end
				bd_rx_len = temp_buf[ico + 5] * 256 + temp_buf[ico + 6];
				memcpy(rcv_buf, temp_buf + ico, bd_rx_len);
			}
		}
	}
	return bd_rx_len;
}
#endif // !WINNT

int Send_Data(unsigned char*buf, int len)
{
	int i;
	uint8 ret;
	ret = write_uart(UART3_Fd, (int8*)buf, len);
	if (ret == -1)
	{
		printf("send msg to BD error\n");
		return -1;

	}
	return 1;
}
int BD_CheckCRC(int count, unsigned char *buf, int len)
{
	int i;
	int8 crc = 0x00;

	for (i = 0; i < len - 1; i++)
	{
		crc = buf[i] ^ crc;
	}
	//printf("\r\ncrc == %2x\n",crc);
	if (crc == buf[len - 1])
		return 1;
	else
		return 0;
}

unsigned char BD_GetCRC(int count, unsigned char *buf, int len)
{
	int i;
	int8 crc = 0x00;
	for (i = 0; i < len; i++)
	{
		crc = buf[i] ^ crc;
	}

	return crc;
}
int BD_Init()
{
	bd_buf_bitmap = 0x00;

	bd_sign = 0;

	src_addr[0] = 0x00;
	src_addr[1] = 0x00;
	src_addr[2] = 0x00;

	dst_addr[0] = 0x04;
	dst_addr[1] = 0xB0;
	dst_addr[2] = 0xD5;


}
void Create_txsq(unsigned char *src_user_addr, unsigned char *dst_user_addr,
	unsigned char transfer_format, unsigned char *payload,
	unsigned int payload_len, unsigned char *send_txsq_data)
{
	send_txsq_data[0] = '$';
	send_txsq_data[1] = 'T';
	send_txsq_data[2] = 'X';
	send_txsq_data[3] = 'S';
	send_txsq_data[4] = 'Q';

	send_txsq_data[5] = (BD_TXSQ_FRAME_SIZE + payload_len) / 256;
	send_txsq_data[6] = (BD_TXSQ_FRAME_SIZE + payload_len) % 256;

	send_txsq_data[7] = *src_user_addr;
	send_txsq_data[8] = *(src_user_addr + 1);
	send_txsq_data[9] = *(src_user_addr + 2);

	/*
	//��Ϣ����
	if(0x44 == transfer_format  )//����
	{

	send_txsq_data[10] = BD_TXSQ_PAYLOAD_CHINESE;//TXSQ_PAYLOAD_CHINESE 0b01000100
	}
	else
	{
	send_txsq_data[10] = BD_TXSQ_PAYLOAD_BCD;//TXSQ_PAYLOAD_BCD 0b01000110
	}
	*/
	//��Ϣ����
	send_txsq_data[10] = transfer_format;
	//��Ϣ-Ŀ���û���ַ
	send_txsq_data[11] = *dst_user_addr;
	send_txsq_data[12] = *(dst_user_addr + 1);
	send_txsq_data[13] = *(dst_user_addr + 2);

	//��Ϣ-���ľ��ɳ���-��λ��bit
	send_txsq_data[14] = (payload_len * 8) / 256;
	send_txsq_data[15] = (payload_len * 8) % 256;

	//��Ϣ-�Ƿ�Ӧ�� 
	send_txsq_data[16] = 0;

	//��Ϣ-��������
	memcpy(&send_txsq_data[17], payload, payload_len);

	//У���
	send_txsq_data[BD_TXSQ_FRAME_SIZE + payload_len - 1] = BD_GetCRC(0, send_txsq_data, (BD_TXSQ_FRAME_SIZE + payload_len - 1));
}

void Create_dwsq(int8 *src_user_addr)
{

	//todo...
}

void BD_Send_txsq(unsigned char *src_user_addr, unsigned char *dst_user_addr,
	unsigned char transfer_format, unsigned char  *send_txsq_payload, unsigned int send_txsq_payload_len)
{

	unsigned int i;

	unsigned int payload_len;
	unsigned long src_user_addr_long;
	unsigned long dst_user_addr_long;


	unsigned char payload[BD_MAX_PAYLOAD_LENGTH];
	unsigned char send_txsq_data[BD_TXSQ_FRAME_SIZE + BD_MAX_PAYLOAD_LENGTH];

	memset(send_txsq_data, 0, BD_TXSQ_FRAME_SIZE + BD_MAX_PAYLOAD_LENGTH);

	Create_txsq(src_user_addr, dst_user_addr, transfer_format, send_txsq_payload, send_txsq_payload_len, send_txsq_data);
	Send_Data(send_txsq_data, (BD_TXSQ_FRAME_SIZE + send_txsq_payload_len));

}


void BD_Send_dwsq(unsigned char *src_user_addr)
{
	unsigned char send_dwsq_data[BD_DWSQ_FRAME_SIZE];

	send_dwsq_data[0] = '$';
	send_dwsq_data[1] = 'D';
	send_dwsq_data[2] = 'W';
	send_dwsq_data[3] = 'S';
	send_dwsq_data[4] = 'Q';

	send_dwsq_data[5] = BD_DWSQ_FRAME_SIZE / 256;//�ȴ���λ
	send_dwsq_data[6] = BD_DWSQ_FRAME_SIZE % 256;//�ٴ���λ

	send_dwsq_data[7] = *src_user_addr;
	send_dwsq_data[8] = *(src_user_addr + 1);
	send_dwsq_data[9] = *(src_user_addr + 2);

	send_dwsq_data[10] = BD_DWSQ_TYPE;

	send_dwsq_data[11] = 0;
	send_dwsq_data[12] = 0;
	send_dwsq_data[13] = 0;
	send_dwsq_data[14] = 0;

	send_dwsq_data[15] = 0;
	send_dwsq_data[16] = 0;
	send_dwsq_data[17] = 0;
	send_dwsq_data[18] = 0;

	send_dwsq_data[19] = 0;
	send_dwsq_data[20] = 0;

	send_dwsq_data[21] = BD_GetCRC(0, send_dwsq_data, BD_DWSQ_FRAME_SIZE - 1);
	Send_Data(send_dwsq_data, BD_DWSQ_FRAME_SIZE);

}

void BD_Send_icjc()
{
	unsigned char send_icjc_data[BD_ICXX_FRAME_SIZE];

	send_icjc_data[0] = '$';
	send_icjc_data[1] = 'I';
	send_icjc_data[2] = 'C';
	send_icjc_data[3] = 'J';
	send_icjc_data[4] = 'C';

	send_icjc_data[5] = BD_ICJC_FRAME_SIZE / 256;  //�ȴ���λ
	send_icjc_data[6] = BD_ICJC_FRAME_SIZE % 256; //�ٴ���λ

	send_icjc_data[7] = 0x00;
	send_icjc_data[8] = 0x00;
	send_icjc_data[9] = 0x00;

	send_icjc_data[10] = 0x00;

	send_icjc_data[11] = BD_GetCRC(0, send_icjc_data, BD_ICJC_FRAME_SIZE);

	Send_Data(send_icjc_data, BD_ICJC_FRAME_SIZE);
	printf("#######################send icjc ......###########################\n");
}

void BD_Send_xtzj(unsigned char *src_user_addr, unsigned int frequency)
{

	unsigned char send_xtzj_data[BD_XTZJ_FRAME_SIZE];

	//��Ҫ�û���ַ
	//ϵͳ�Լ��Ƶ��

	send_xtzj_data[0] = '$';
	send_xtzj_data[1] = 'X';
	send_xtzj_data[2] = 'T';
	send_xtzj_data[3] = 'Z';
	send_xtzj_data[4] = 'J';

	send_xtzj_data[5] = BD_XTZJ_FRAME_SIZE / 256; //�ȴ���λ
	send_xtzj_data[6] = BD_XTZJ_FRAME_SIZE % 256; //�ٴ���λ

	send_xtzj_data[7] = src_user_addr[0];
	send_xtzj_data[8] = src_user_addr[1];
	send_xtzj_data[9] = src_user_addr[2];

	//send_xtzj_data[7] = src_user_addr / 65536;
	//send_xtzj_data[8] = (src_user_addr % 65536) / 256;
	//send_xtzj_data[9] = (src_user_addr % 65536) % 256;

	send_xtzj_data[10] = frequency / 256;
	send_xtzj_data[11] = frequency % 256;

	send_xtzj_data[12] = BD_GetCRC(0, send_xtzj_data, BD_XTZJ_FRAME_SIZE);

	Send_Data(send_xtzj_data, BD_XTZJ_FRAME_SIZE);
}

int BD_RecvPacket_Depart(unsigned char *bd_rx_buf, int buf_len)
{
	unsigned char txxx_buf[BD_TXXX_MAX_SIZE];  //�û����յ�ͨ����Ϣframe
	unsigned char icxx_buf[BD_ICXX_FRAME_SIZE];
	unsigned char zjxx_buf[BD_ZJXX_FRAME_SIZE];
	unsigned char fkxx_buf[BD_FKXX_FRAME_SIZE];
	unsigned char sjxx_buf[BD_SJXX_FRAME_SIZE];
	unsigned char dwxx_buf[BD_DWXX_FRAME_SIZE];
	unsigned char bbxx_buf[BD_BBXX_FRAME_SIZE];

	bd_txxx txxx;
	bd_icxx icxx;
	bd_zjxx zjxx;
	bd_fkxx fkxx;
	bd_dwxx dwxx;

	int n;
	int i;
	int re_sign;
	int packet_len;
	for (i = 0; i <= buf_len; i++)
	{
		if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'D') && (bd_rx_buf[i + 2] == 'W') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ���λ��Ϣ$DWXX
		{
			printf("�յ�dwxx \n");

			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(0, bd_rx_buf + i, packet_len))
			{
				bd_buf_bitmap |= DWXX_BUF;
				memcpy(dwxx_buf, bd_rx_buf + i, sizeof(dwxx_buf));
				BD_Analytical_dwxx(dwxx_buf, &dwxx);
				Print_dwxx(&dwxx);
			}

		}
		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'T') && (bd_rx_buf[i + 2] == 'X') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X'))//�յ�ͨ����Ϣ$TXXX
		{
			printf("�յ�txxx \n");
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(0, bd_rx_buf + i, packet_len))
			{
				bd_buf_bitmap |= TXXX_BUF;
				memcpy(txxx_buf, bd_rx_buf + i, sizeof(txxx_buf));
				BD_Analytical_txxx(txxx_buf, &txxx);
				Print_txxx(&txxx);
			}

		}

		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'I') && (bd_rx_buf[i + 2] == 'C') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ�IC��Ϣ$ICXX
		{
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(i, bd_rx_buf, packet_len))
			{
				bd_buf_bitmap |= ICXX_BUF;
				memcpy(icxx_buf, bd_rx_buf + i, sizeof(icxx_buf));
				BD_Analytical_icxx(icxx_buf, &icxx);
				Print_icxx(&icxx);
			}

		}
		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'Z') && (bd_rx_buf[i + 2] == 'J') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ��Լ���Ϣ$ZJXX
		{
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(i, bd_rx_buf, packet_len))
			{
				bd_buf_bitmap |= ZJXX_BUF;
				memcpy(zjxx_buf, bd_rx_buf + i, sizeof(zjxx_buf));
				BD_Analytical_zjxx(zjxx_buf, &zjxx);
				Print_zjxx(&zjxx);
			}

		}
		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'S') && (bd_rx_buf[i + 2] == 'J') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ�ʱ����Ϣ$SJXX
		{
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(i, bd_rx_buf, packet_len))
			{
				bd_buf_bitmap |= SJXX_BUF;
				memcpy(sjxx_buf, bd_rx_buf + i, sizeof(sjxx_buf));
				//todo;
			}

		}
		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'B') && (bd_rx_buf[i + 2] == 'B') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ��汾��Ϣ$BBXX
		{
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(i, bd_rx_buf, packet_len))
				bd_buf_bitmap |= BBXX_BUF;
			memcpy(bbxx_buf, bd_rx_buf + i, sizeof(bbxx_buf));
			//todo
		}
		else if ((bd_rx_buf[i] == '$') && (bd_rx_buf[i + 1] == 'F') && (bd_rx_buf[i + 2] == 'K') && (bd_rx_buf[i + 3] == 'X') && (bd_rx_buf[i + 4] == 'X')) //�յ�������Ϣ$FKXX
		{
			packet_len = bd_rx_buf[i + 5] * 256 + bd_rx_buf[i + 6];
			if (1 == BD_CheckCRC(i, bd_rx_buf, packet_len))
			{
				bd_buf_bitmap |= FKXX_BUF;
				memcpy(fkxx_buf, bd_rx_buf + i, sizeof(fkxx_buf));
				BD_Analytical_fkxx(fkxx_buf, &fkxx);
				Print_fkxx(&fkxx);
			}

		}
	}

	return 1;
}
void BD_Analytical_txxx(unsigned char *txxx_buf, bd_txxx *txxx)
{

	unsigned int i;
	unsigned int payload_len;
	unsigned char send_data[104];//��;��2

	/* ָ������ */
	for (i = 0; i < 5; ++i)
	{
		txxx->instruction[i] = txxx_buf[i];
	}

	/* ���հ��� */
	txxx->packet_len = txxx_buf[5] * 256 + txxx_buf[6];

	/* Ŀ���û���ַ */
	for (i = 0; i < 3; ++i)
	{
		txxx->user_addr[i] = txxx_buf[i + 7];
	}

	/* ��Ϣ-��Ϣ��� */
	memcpy(&(txxx->txxx_info.txxx_info_type), (txxx_buf + 10), 1);

	/* ��Ϣ-���ͷ���ַ */
	for (i = 0; i < 3; ++i)
	{
		txxx->txxx_info.src_user_addr[i] = txxx_buf[i + 11];
	}

	/* ��Ϣ-����ʱ�� */
	txxx->txxx_info.send_time.hour = txxx_buf[14];
	txxx->txxx_info.send_time.minute = txxx_buf[15];

	/* ��Ϣ-���ĳ��� */
	txxx->txxx_info.payload_len = txxx_buf[16] * 256 + txxx_buf[17];
	payload_len = txxx->txxx_info.payload_len / 8;

	/* ��Ϣ-�������� */
	memcpy(txxx->txxx_info.payload, (txxx_buf + 18), payload_len);

	/* ��Ϣ-CRC */
	txxx->txxx_info.crc = txxx_buf[18 + payload_len];

	/* У��� */
	txxx->checksum = txxx_buf[18 + payload_len + 1];



}

void BD_Analytical_dwxx(unsigned char *dwxx_buf, bd_dwxx *dwxx)
{
	unsigned int i;
	unsigned int payload_len;
	for (i = 0; i < 5; ++i)
	{
		dwxx->instruction[i] = dwxx_buf[i];
	}
	/* ���հ��� */
	dwxx->packet_len = dwxx_buf[5] * 256 + dwxx_buf[6];

	/* �û���ַ */
	for (i = 0; i < 3; ++i)
	{
		dwxx->user_addr[i] = dwxx_buf[i + 7];
	}
	memcpy(&(dwxx->dwxx_info.dwxx_info_type), (dwxx_buf + 10), 1);
	/* ��ѯ��ַ */
	for (i = 0; i < 3; ++i)
	{
		dwxx->dwxx_info.check_addr[i] = dwxx_buf[i + 11];
	}
	/*λ������Time*/
	for (i = 0; i < 4; ++i)
	{

		dwxx->dwxx_info.time[i] = dwxx_buf[i + 14];
	}
	/*λ������ ����*/
	for (i = 0; i < 4; ++i)
	{

		dwxx->dwxx_info.longitude[i] = dwxx_buf[i + 18];
	}
	/*λ������ γ��*/
	for (i = 0; i < 4; ++i)
	{
		dwxx->dwxx_info.latitude[i] = dwxx_buf[i + 22];
	}
	/*λ������ �߶�*/
	for (i = 0; i < 2; ++i)
	{
		dwxx->dwxx_info.high[i] = dwxx_buf[i + 26];
	}
	/*λ������ sigema�߶�*/
	for (i = 0; i < 2; ++i)
	{
		dwxx->dwxx_info.ex_high[i] = dwxx_buf[i + 28];
	}
	dwxx->checksum = dwxx_buf[BD_DWXX_FRAME_SIZE - 1];
}

void BD_Analytical_icxx(unsigned char *icxx_buf, bd_icxx *icxx)
{
	unsigned int i;
	for (i = 0; i < 5; ++i)
	{
		icxx->instruction[i] = icxx_buf[i];
	}

	icxx->packet_len = icxx_buf[5] * 256 + icxx_buf[6];

	for (i = 0; i < 3; ++i)
	{
		icxx->user_addr[i] = icxx_buf[i + 7];
	}

	icxx->icxx_info.frame_id = icxx_buf[10];

	for (i = 0; i < 3; ++i)
	{
		icxx->icxx_info.broadcast_id[i] = icxx_buf[i + 11];
	}

	icxx->icxx_info.user_feature = icxx_buf[14];

	icxx->icxx_info.service_frequency = icxx_buf[15] * 256 + icxx_buf[16];

	icxx->icxx_info.comm_level = icxx_buf[17];

	icxx->icxx_info.encryption_flag = icxx_buf[18];

	icxx->icxx_info.user_num = icxx_buf[19] * 256 + icxx_buf[20];

	icxx->checksum = icxx_buf[21];


}
void BD_Analytical_zjxx(unsigned char *zjxx_buf, bd_zjxx *zjxx)
{
	unsigned int i;

	for (i = 0; i < 5; ++i)
	{
		zjxx->instruction[i] = zjxx_buf[i];
	}

	zjxx->packet_len = zjxx_buf[5] * 256 + zjxx_buf[6];

	for (i = 0; i < 3; ++i)
	{
		zjxx->user_addr[i] = zjxx_buf[i + 7];
	}

	zjxx->zjxx_info.ic_status = zjxx_buf[10];

	zjxx->zjxx_info.hw_status = zjxx_buf[11];

	zjxx->zjxx_info.battery_quantity = zjxx_buf[12];

	zjxx->zjxx_info.in_station_status = zjxx_buf[13];

	for (i = 0; i < 6; ++i)
	{
		zjxx->zjxx_info.power_status[i] = zjxx_buf[14 + i];
	}

	zjxx->checksum = zjxx_buf[20];

}
void BD_Analytical_fkxx(unsigned char *fkxx_buf, bd_fkxx *fkxx)
{
	unsigned int i;

	for (i = 0; i < 5; ++i)
	{
		fkxx->instruction[i] = fkxx_buf[i];
	}

	fkxx->packet_len = fkxx_buf[5] * 256 + fkxx_buf[6];

	for (i = 0; i < 3; ++i)
	{
		fkxx->user_addr[i] = fkxx_buf[i + 7];
	}

	fkxx->fkxx_info.fk_flag = fkxx_buf[10];

	for (i = 0; i < 4; ++i)
	{
		fkxx->fkxx_info.extra_info[i] = fkxx_buf[11 + i];
	}

	fkxx->checksum = fkxx_buf[15];

}

void BD_Analytical_Message(unsigned char *message_buf)
{
}

void  Print_txxx(bd_txxx *_bd_txxx)
{
	unsigned int i;

	printf("\r\n    ========= TXXX��-��ӡ��ʼ=========\r\n");

	printf("\r\n    TXXX���ܳ���:%d", _bd_txxx->packet_len);
	printf("\r\n    TXXX���û���ַ:0x%02x%02x%02x", _bd_txxx->user_addr[0], _bd_txxx->user_addr[1], _bd_txxx->user_addr[2]);
	printf("\r\n    TXXX����Ϣ����-���ͷ���ַ:0x%02x%02x%02x", _bd_txxx->txxx_info.src_user_addr[0], _bd_txxx->txxx_info.src_user_addr[1], _bd_txxx->txxx_info.src_user_addr[2]);
	printf("\r\n    TXXX����Ϣ����-����ʱ��:%02dʱ%02d��", _bd_txxx->txxx_info.send_time.hour, _bd_txxx->txxx_info.send_time.minute);
	printf("\r\n    TXXX����Ϣ����-���ĳ���:%d�ֽ�", _bd_txxx->txxx_info.payload_len / 8);
	printf("\r\n    TXXX����Ϣ����-��������:");
	for (i = 0; i < (_bd_txxx->txxx_info.payload_len / 8); ++i)
	{
		printf("%02x ", _bd_txxx->txxx_info.payload[i]);
		/*
		if(_bd_txxx->txxx_info.payload[i])
		{
		USV_Control.Radio_Nummber=3;//���ֵ�̨����ָ���־
		Dradio_Con_Analytical(0,Dradio_Con);
		}
		*/
	}
	printf("\r\n    TXXX����Ϣ����-CRC:0x%02x", _bd_txxx->txxx_info.crc);
	printf("\r\n    TXXX��У���:0x%02x", _bd_txxx->checksum);

	printf("\r\n    ========= TXXX��-��ӡ����=========\r\n");


}

void Print_icxx(bd_icxx *_bd_icxx)
{


	printf("\r\n    ========= ICXX��-��ӡ��ʼ=========\r\n");

	printf("\r\n    ICXX���ܳ���:%d", _bd_icxx->packet_len);
	printf("\r\n    ICXX���û���ַ:0x%02x%02x%02x", _bd_icxx->user_addr[0], _bd_icxx->user_addr[1], _bd_icxx->user_addr[2]);
	printf("\r\n    ICXX��Ϣ����-֡��:%d", _bd_icxx->icxx_info.frame_id);
	printf("\r\n    ICXX��Ϣ����-ͨ��ID:0x%02x%02x%02x", _bd_icxx->icxx_info.broadcast_id[0], _bd_icxx->icxx_info.broadcast_id[1], _bd_icxx->icxx_info.broadcast_id[2]);
	printf("\r\n    ICXX��Ϣ����-�û�����:%d", _bd_icxx->icxx_info.user_feature);
	printf("\r\n    ICXX��Ϣ����-����Ƶ��:%d��", _bd_icxx->icxx_info.service_frequency);
	printf("\r\n    ICXX��Ϣ����-ͨ�ż���:%d", _bd_icxx->icxx_info.comm_level);
	printf("\r\n    ICXX��Ϣ����-���ܱ�־:%d", _bd_icxx->icxx_info.encryption_flag);
	printf("\r\n    ICXX��Ϣ����-�û���Ŀ:%d", _bd_icxx->icxx_info.user_num);
	printf("\r\n    ICXX��У���:0x%02x\r\n", _bd_icxx->checksum);

	printf("\r\n    ========= ICXX��-��ӡ����=========\r\n");
}

void Print_zjxx(bd_zjxx *_bd_zjxx)
{
	printf("\r\n    ========= ZJXX��-��ӡ��ʼ=========\r\n");

	printf("\r\n    ZJXX���ܳ���:%d", _bd_zjxx->packet_len);
	printf("\r\n    ZJXX���û���ַ:0x%02x%02x%02x", _bd_zjxx->user_addr[0], _bd_zjxx->user_addr[1], _bd_zjxx->user_addr[2]);
	printf("\r\n    ZJXX��Ϣ����-IC��״̬:0x%02x", _bd_zjxx->zjxx_info.ic_status);
	printf("\r\n    ZJXX��Ϣ����-Ӳ��״̬:0x%02x", _bd_zjxx->zjxx_info.hw_status);
	printf("\r\n    ZJXX��Ϣ����-��ص���:0x%02x", _bd_zjxx->zjxx_info.battery_quantity);
	printf("\r\n    ZJXX��Ϣ����-��վ״̬:0x%02x", _bd_zjxx->zjxx_info.in_station_status);
	printf("\r\n    ZJXX��Ϣ����-����״̬:%d-%d-%d-%d-%d-%d", _bd_zjxx->zjxx_info.power_status[0], _bd_zjxx->zjxx_info.power_status[1],
		_bd_zjxx->zjxx_info.power_status[2], _bd_zjxx->zjxx_info.power_status[3], _bd_zjxx->zjxx_info.power_status[4], _bd_zjxx->zjxx_info.power_status[5]);
	printf("\r\n    ZJXX��У���:0x%02x\r\n", _bd_zjxx->checksum);

	printf("\r\n    ========= ZJXX��-��ӡ����=========\r\n");
}

void Print_fkxx(bd_fkxx *_bd_fkxx)
{
	printf("\r\n    ========= FKXX��-��ӡ��ʼ=========\r\n");

	printf("\r\n    FKXX���ܳ���:%d", _bd_fkxx->packet_len);
	printf("\r\n    FKXX���û���ַ:0x%02x%02x%02x", _bd_fkxx->user_addr[0], _bd_fkxx->user_addr[1], _bd_fkxx->user_addr[2]);
	printf("\r\n    FKXX��Ϣ����-������־:0x%02x", _bd_fkxx->fkxx_info.fk_flag);
	printf("\r\n    FKXX��Ϣ����-������Ϣ:0x%02x-0x%02x-0x%02x-0x%02x", _bd_fkxx->fkxx_info.extra_info[0], _bd_fkxx->fkxx_info.extra_info[1], _bd_fkxx->fkxx_info.extra_info[2], _bd_fkxx->fkxx_info.extra_info[3]);
	printf("\r\n    FKXX��У���:0x%02x\r\n", _bd_fkxx->checksum);

	printf("\r\n    ========= FKXX��-��ӡ����=========\r\n");
}
void Print_dwxx(bd_dwxx *_bd_dwxx)
{
	printf("\r\n    ========= DWXX��-��ӡ��ʼ=========\r\n");
	printf("\r\n    DWXX��ѯ��ַ:0x%02x%02x%02x", _bd_dwxx->dwxx_info.check_addr[0], _bd_dwxx->dwxx_info.check_addr[1], _bd_dwxx->dwxx_info.check_addr[2]);
	printf("\r\n    DWXX��λʱ��:d%:%d:%d", _bd_dwxx->dwxx_info.time[0], _bd_dwxx->dwxx_info.time[1], _bd_dwxx->dwxx_info.time[2]);
	printf("\r\n    DWXX��λ����:d%:%d:%d:%d", _bd_dwxx->dwxx_info.longitude[0], _bd_dwxx->dwxx_info.longitude[1], _bd_dwxx->dwxx_info.longitude[2], _bd_dwxx->dwxx_info.longitude[3]);
	printf("\r\n	DWXX��λγ��:d%:%d:%d:%d", _bd_dwxx->dwxx_info.latitude[0], _bd_dwxx->dwxx_info.latitude[1], _bd_dwxx->dwxx_info.latitude[2], _bd_dwxx->dwxx_info.latitude[3]);
	printf("\r\n    DWXX��λ�߶�:%d\r\n", _bd_dwxx->dwxx_info.high[1]);
}
