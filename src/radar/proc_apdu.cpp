#include "proc_apdu.h"
#include <string.h>

const uint8 gc_uStx = 0xfd;    //起始符
const uint8 gc_uEtx = 0xfe;    //结束符
const uint8 gc_uDel = 0x7d;    //转义字符
const int gc_iLenOther = 11;   //APDU包非数据部分字段长度

ProcAPDU::ProcAPDU()
{
	memset(&m_strAPDU, 0, sizeof(m_strAPDU));
}

ProcAPDU::ProcAPDU(Data_APDU & data)
{
	m_strAPDU = data;
}

ProcAPDU::~ProcAPDU()
{

}

bool ProcAPDU::CheckHasHead(int &index, uint8 *pBuff, int buflen)
{
	//判断是否含有起始符
	for (int i = 0; i < buflen; i++)
	{
		if (pBuff[i] == gc_uStx)
		{
			index = i;
			return true;
		}
	}
	return false;
}

bool ProcAPDU::CheckHasEnd(int &index, uint8 *pBuff, int buflen)
{
	//判断是否含有结束符
	for (int i = 0; i < buflen; i++)
	{
		if (pBuff[i] == gc_uEtx)
		{
			index = i;
			return true;
		}
	}
	return false;
}

bool ProcAPDU::CheckAPDU(uint8 * buf, int buflen, int &nRet)
{
	if (buf == NULL || buflen == 0)
	{
		nRet = 0;
		return false;
	}

	int istart, iend;
	//判断是否包含头，不包含全部丢弃
	if (!CheckHasHead(istart, buf, buflen))
	{
		nRet = buflen;
		return false;
	}
	//判断是否包含尾，不包含继续接收，丢弃报文头之前的部分
	if (!CheckHasEnd(iend, buf + istart, buflen - istart))
	{
		nRet = istart;
		return false;
	}
	nRet = iend + 1;

	//对头尾之间的数据进行处理，还原转义
	uint8 oriDataBuf[STD_BUFFER_LEN];
	memset(oriDataBuf, 0, STD_BUFFER_LEN);
	uint16 oriDataLen = 0;
	for (int i = istart + 1; i < iend; i++)
	{
		if (buf[i] == gc_uDel)
		{
			oriDataBuf[oriDataLen] = buf[i] + buf[i + 1];     //转义计算，与后一位相加
			i++;                                              //跳过后一位
			oriDataLen++;
		}
		else
		{
			oriDataBuf[oriDataLen] = buf[i];
			oriDataLen++;
		}
	}
	//数据校验 校验数据长度 以及 校验码
	uint16 payloadLen;
	payloadLen = oriDataBuf[0];
	if ((payloadLen + gc_iLenOther) != oriDataLen)
		return false;

	return Buffer2APDU(oriDataBuf, oriDataLen);  //内容包含校验
}

Data_APDU * ProcAPDU::GetAPDU()
{
	return &m_strAPDU;
}

void ProcAPDU::SetAPDU(Data_APDU data_apdu)
{
	m_strAPDU = data_apdu;
}

bool ProcAPDU::GetAPDUBuff(uint8 * pAPDUBuffer, int &buflen)
{
	return APDU2Buffer(pAPDUBuffer, buflen);
}

bool ProcAPDU::Buffer2APDU(uint8 * pAPDUBuffer, int buflen)
{
	uint16 crc16;
	m_strAPDU.len   = pAPDUBuffer[0];
	m_strAPDU.rev1  = pAPDUBuffer[1];
	m_strAPDU.rev2  = pAPDUBuffer[2];
	m_strAPDU.seq   = pAPDUBuffer[3];
	m_strAPDU.sysid = pAPDUBuffer[4];
	m_strAPDU.compid = pAPDUBuffer[5];
	m_strAPDU.msgid[0] = pAPDUBuffer[6];
	m_strAPDU.msgid[1] = pAPDUBuffer[7];
	m_strAPDU.msgid[2] = pAPDUBuffer[8];
	memcpy(m_strAPDU.databuf, pAPDUBuffer + 9, m_strAPDU.len);
	memcpy(&m_strAPDU.checksum, &pAPDUBuffer[buflen - 2], 2);

	crc16 = calcCRC16(0xffff, pAPDUBuffer, m_strAPDU.len + 9);
	if (m_strAPDU.checksum == crc16)
		return true;

	return false;
}

bool ProcAPDU::APDU2Buffer(uint8 * pAPDUBuffer, int &buflen)
{
	if (m_strAPDU.len == 0 || pAPDUBuffer == NULL)
		return false;

	uint8 oriDataBuf[STD_BUFFER_LEN], rsltDataBuf[STD_BUFFER_LEN];
	memset(oriDataBuf, 0, STD_BUFFER_LEN);
	memset(rsltDataBuf, 0, STD_BUFFER_LEN);
	int i, len, rsltlen;
	len = m_strAPDU.len + gc_iLenOther;
	memcpy(oriDataBuf, &m_strAPDU, 9);
	memcpy(oriDataBuf + 9, m_strAPDU.databuf, m_strAPDU.len);
	memcpy(oriDataBuf + 9 + m_strAPDU.len, &m_strAPDU.checksum, 2);

	//转义处理
	rsltlen = 0;
	rsltDataBuf[rsltlen++] = 0xfd;                       //报文头
	for (i = 0; i < len; i++)
	{
		if (oriDataBuf[i] == gc_uStx)
		{
			rsltDataBuf[rsltlen++] = gc_uDel;
			rsltDataBuf[rsltlen++] = 0x80;
		}
		else if (oriDataBuf[i] == gc_uEtx)
		{
			rsltDataBuf[rsltlen++] = gc_uDel;
			rsltDataBuf[rsltlen++] = 0x81;
		}
		else if (oriDataBuf[i] == gc_uDel)
		{
			rsltDataBuf[rsltlen++] = gc_uDel;
			rsltDataBuf[rsltlen++] = 0x00;
		}
		else
		{
			rsltDataBuf[rsltlen++] = oriDataBuf[i];
		}
	}
	rsltDataBuf[rsltlen++] = 0xfe;                       //报文尾

	memcpy(pAPDUBuffer, rsltDataBuf, rsltlen);
	buflen = rsltlen;	

	return true;
}
