#pragma once

#include "mypublic.h"
#include <string.h>

#ifndef STD_BUFFER_LEN
#define STD_BUFFER_LEN		512
#endif

struct Data_APDU
{
	uint8	len;			//payload部分长度
	uint8	rev1;			//预留1
	uint8	rev2;			//预留2
	uint8	seq;			//包序号，包递增
	uint8	sysid;			//发送端系统ID，用于区分网络中的不同系统
	uint8	compid;			//发送组件ID， 用于区分系统中的不同组件
	uint8	msgid[3];		//payload中消息类型，用于消息解码
	uint8	databuf[256];	//payload，基于消息类型的内容
	uint16  checksum;		//校验和

	Data_APDU()
	{
		len = 0;
		rev1 = 0;
		rev2 = 0;
		seq = 0;
		sysid = 0;
		compid = 0;
		memset(msgid, 0, 3);
		memset(databuf, 0, 256);
		checksum = 0;
	};
};

class ProcAPDU
{
public:	
	ProcAPDU();
	ProcAPDU(Data_APDU & data);
	~ProcAPDU();

	bool CheckAPDU(uint8 * buf, int buflen, int &nRet);

	Data_APDU * GetAPDU();

	void SetAPDU(Data_APDU data_apdu);

	bool GetAPDUBuff(uint8 * pAPDUBuffer, int &buflen);

private:
	Data_APDU m_strAPDU;

	//判头标识
	bool CheckHasHead(int &index, uint8 *pBuff, int buflen);

	//判尾标识
	bool CheckHasEnd(int &index, uint8 *pBuff, int buflen);

	//buf转数据
	bool Buffer2APDU(uint8 * pAPDUBuffer, int buflen);

	//数据转buf
	bool APDU2Buffer(uint8 * pAPDUBuffer, int &buflen);
};