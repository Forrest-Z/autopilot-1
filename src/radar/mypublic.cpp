#include "mypublic.h"

uint16 calcCRC16(uint16 pseed, const uint8 *buf, uint16 len)
{
	uint16 temp = pseed;
	uint16 i = 0, j = 0;
	uint16 LSB = 0;

	for (i = 0; i < len; i++)
	{
		temp ^= *(buf + i);
		for (j = 0; j < 8; j++)
		{
			LSB = temp & 0x0001;
			temp = temp >> 1;
			if (LSB)
			{
				temp = temp ^ 0xA001;
			}
		}
	}
	return temp;
}

uint32 CRC16(uint8 *arr_buff, uint8 len)
{
	uint32 crc = 0xFFFF;
	uint8 i, j;
	for (j = 0; j < len; j++)
	{
		crc = crc ^ *arr_buff++;
		for (i = 0; i < 8; i++)
		{
			if ((crc & 0x0001) > 0)
			{
				crc = crc >> 1;
				crc = crc ^ 0xa001;
			}
			else
				crc = crc >> 1;
		}
	}
	return(crc);
}

uint8 CRC_XOR(uint8 *arr_buff, uint8 len)
{
	uint32 i, crc;
	crc = arr_buff[0];
	for (i = 1; i < len - 2; i++)//最后两个数为crc不需要异或 
	{
		crc = arr_buff[i] ^ crc;
	}
	uint8 CRC = crc;
	return CRC;
}