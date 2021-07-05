/*==========================================================*
 * 模块说明: GlobalFunction.cpp                             *
 * 文件版本: v1.00 (说明本文件的版本信息)                   *
 * 开发人员: shaoyuping                                     *
 * 创建时间: 2018年1月31日 20点23分                         *
 * Copyright(c) sf-auto.ltd                                 *
 *==========================================================*
 * 程序修改记录(最新的放在最前面):                          *
 *  <修改日期>, <修改人员>: <修改功能概述>                  *
 *==========================================================*
 *=========================================================*/
 
 /**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"

 /******************************  Local Variable  *****************************
 /******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void comm_time_cal(uint32 maxTime,uint32* timebuf,uint8* comm_sign)
{

	if(*timebuf>=maxTime)
	{
		*comm_sign = COMM_CONNECT_FAIL;
	}
	else
	{
		*comm_sign = COMM_CONNECT_OK;
		*timebuf= *timebuf+1;
	}
}

void comm_time_return(uint32* timebuf,uint8* comm_sign)
{
	*timebuf = 0;
	*comm_sign = COMM_CONNECT_OK;
}


#define  u16_turn_over(x) (uint16) ( ((((uint16)(x)) & 0x00ff) << 8 ) |((((uint16)(x)) & 0xff00) >> 8 ))

uint16 u8tou16(uint8* src)
{
	uint16 iret;
	//memcpy((uint8 *)&iret,src,2);
	uint16 *p_src_uint16;
	p_src_uint16 = (uint16*)src;
	iret  = (*p_src_uint16);
	return iret;
}

uint64 u8tou64(uint8* src)
{
	uint64 iret;
	//memcpy((uint8 *)&iret,src,4);
	uint64 *p_src_uint64;
	p_src_uint64 = (uint64*)src;
	iret = (*p_src_uint64);
	return iret;
}

uint32 u8tou32(uint8* src)
{
	uint32 iret;
	//memcpy((uint8 *)&iret,src,4);
	uint32 *p_src_uint32;
	p_src_uint32 = (uint32*)src;
	iret = (*p_src_uint32);
	return iret;
}


int16 u8toi16(uint8* src)
{
	int16 iret;
	//memcpy((uint8 *)&iret,src,2);
	uint16 *p_src_uint16;
	p_src_uint16 = (uint16*)src;
	iret = (int16)((*p_src_uint16));
	return iret;
}


float u8tofloat(uint8* src)
{
	float iret;
	uint8 tmp[4] ;
	uint8 loop_i;
	for(loop_i=0;loop_i<4;loop_i++)
	{
		tmp[loop_i] = *(src+3-loop_i);
	}
	iret = *(float*)tmp;
	return iret;
}

//float u8tofloat(uint8* src)-
//{
//	float iret;
//	iret =  memcpy_f(src);
//	//memcpy((uint8 *)&iret,src,4);
//	//float *p_src_float;
//	//p_src_float = (float*)src;
//	//iret = *(p_src_float);
//	return iret;
//}

void sendCanMsg(CAN_TYPE canPort,uint8 canIdPF,uint8 canIdPS,uint8 canIdAddr,uint8* pCanData)
{
	struct can_frame frame;
	memset(&frame,0,sizeof(frame));
	frame.can_id = canIdAddr;
	frame.can_id+= (((uint16)canIdPF)<<8|((uint16)canIdPS))<<8;
	frame.can_id+= 3<<26;	//Priority
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc= 8;
	memcpy((uint8*)&frame.data[0],pCanData,8);
	write_can(canPort,&frame,sizeof(can_frame));
}




int16 i16UpDownLimit( int16 maxLimit,int16 minLimit,int inputData )
{
	if(inputData > maxLimit)
	{
		return maxLimit;
	}
	else if(inputData < minLimit)
	{
		return minLimit;
	}
	return inputData;
}

uint8 u8UpLimit(uint8 maxLimit,uint8 inputData)
{
	if(inputData > maxLimit)
	{
		return maxLimit;
	}
	else 
	{
		return inputData;
	}
}
/*CIIT*/
static const uint16 CRCtable[256]={
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xB37b,0xA35a,0xD3bd,0xC39c,0xF3ff,0xE3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xA56a,0xB54b,0x8528,0x9509,0xE5ee,0xF5cf,0xC5ac,0xD58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xB75b,0xA77a,0x9719,0x8738,0xF7df,0xE7fe,0xD79d,0xC7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xC9cc,0xD9ed,0xE98e,0xF9af,0x8948,0x9969,0xA90a,0xB92b,
	0x5Af5,0x4Ad4,0x7Ab7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
	0xDBfd,0xCBdc,0xFBbf,0xEB9e,0x9B79,0x8B58,0xBB3b,0xAB1a,
	0x6Ca6,0x7C87,0x4Ce4,0x5Cc5,0x2C22,0x3C03,0x0C60,0x1C41,
	0xEDae,0xFD8f,0xCDec,0xDDcd,0xAD2a,0xBD0b,0x8D68,0x9D49,
	0x7E97,0x6Eb6,0x5Ed5,0x4Ef4,0x3E13,0x2E32,0x1E51,0x0E70,
	0xFF9f,0xEFbe,0xDFdd,0xCFfc,0xBF1b,0xAF3a,0x9F59,0x8F78,
	0x9188,0x81a9,0xB1ca,0xA1eb,0xD10c,0xC12d,0xF14e,0xE16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xA3fb,0xB3da,0xC33d,0xD31c,0xE37f,0xF35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xB5ea,0xA5cb,0x95a8,0x8589,0xF56e,0xE54f,0xD52c,0xC50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xA7db,0xB7fa,0x8799,0x97b8,0xE75f,0xF77e,0xC71d,0xD73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xD94c,0xC96d,0xF90e,0xE92f,0x99c8,0x89e9,0xB98a,0xA9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xCB7d,0xDB5c,0xEB3f,0xFB1e,0x8Bf9,0x9Bd8,0xABbb,0xBB9a,
	0x4A75,0x5A54,0x6A37,0x7A16,0x0Af1,0x1Ad0,0x2Ab3,0x3A92,
	0xFD2e,0xED0f,0xDD6c,0xCD4d,0xBDaa,0xAD8b,0x9De8,0x8Dc9,
	0x7C26,0x6C07,0x5C64,0x4C45,0x3Ca2,0x2C83,0x1Ce0,0x0Cc1,
	0xEF1f,0xFF3e,0xCF5d,0xDF7c,0xAF9b,0xBFba,0x8Fd9,0x9Ff8,
	0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3Eb2,0x0Ed1,0x1Ef0
};
/*MODBUS X16+X15+X2+X1*/
//static const uint16 CRCtable[256]={
//	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,     
//	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,     
//	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,      
//	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,     
//	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,       
//	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,     
//	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,     
//	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,     
//	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,     
//	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,        
//	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,     
//	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,     
//	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,     
//	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,     
//	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,        
//	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,     
//	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,     
//	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,     
//	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,     
//	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,        
//	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,     
//	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,     
//	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,     
//	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,     
//	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,       
//	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,     
//	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,     
//	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,     
//	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,     
//	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,       
//	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,     
//	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
//};



static uint16 updateCRC(uint8 byte,uint16 crc)
{
	return( (crc >> 8) ^ CRCtable[ (crc) ^ byte ] );
}

uint16 calcCrc(uint8* p,uint16 len)
{
	uint16 crc;
	uint16 i;
	uint8 da;

	crc = 0;
	for(i = 0; i < len; i++)
	{
		da=crc>>8;
		crc<<=8;
		crc^=CRCtable[*(p+i)^da];
	}
	return crc;
}

uint16 calcTable(uint8 data,uint16 genpoly,uint16 accum)
{
	uint16 i;
	data<<=8;
	for(i=8;i>0;i--)
	{
		if((data^accum)&0x8000)
		{
			accum = (accum<<1)^genpoly;
		}
		else
		{
			accum<<=1;
		}
		data<<=1;
	}
	return accum;
}


uint16 calcCRC(uint16 pseed,uint8 *buf,uint16 len)
{
	uint16 temp=pseed;
	uint16 i=0,j=0;
	uint16 LSB=0;

	for(i=0;i<len;i++)
	{
		temp^=*(buf+i);
		for(j=0;j<8;j++)
		{
			LSB=temp&0x0001;
			temp=temp>>1;
			if(LSB)
			{
				temp = temp^0xA001;
			}
		}
	}
	return temp;
}

//填充报文头
void fill_head(uint8* pBuf,const char* head,uint16 frameNo,uint8 frameLength)
{
	pBuf[0] = head[0];
	pBuf[1] = head[1];
	pBuf[2] = head[2];
	pBuf[3] = head[3];

	pBuf[4] = 0x4d;	//主控发
	pBuf[5] = 1;	//无人船号
	pBuf[6] = frameNo & 0x00ff;
	pBuf[7] = (frameNo & 0xff00)>>8;
	pBuf[8] = frameLength;
}


/***************************************
 * mutex	
 **************************************/

#ifdef WINNT
//windows environment
int usv_mutex_init(HMUTEX *mutex)
{
	InitializeCriticalSection(mutex);
	return 0;
}

int usv_mutex_lock(HMUTEX *mutex)
{
	EnterCriticalSection(mutex);
	return 0;
}

int usv_mutex_unlock(HMUTEX *mutex)
{
	LeaveCriticalSection(mutex);
	return 0;
}

int usv_mutex_destory(HMUTEX *mutex)
{
	DeleteCriticalSection(mutex);
	return 0;
}

#else
// linux environment
int usv_mutex_init(HMUTEX *mutex)
{
	pthread_mutexattr_t mt;
	pthread_mutexattr_init(&mt);
	pthread_mutexattr_settype(&mt, PTHREAD_MUTEX_ERRORCHECK);//set recursive lock like win mutex

	return pthread_mutex_init(mutex, &mt);
}

int usv_mutex_lock(HMUTEX *mutex)
{
	return pthread_mutex_lock(mutex);
}

int usv_mutex_unlock(HMUTEX *mutex)
{
	return pthread_mutex_unlock(mutex);
}

int usv_mutex_destory(HMUTEX *mutex)
{
	return pthread_mutex_destroy(mutex);
}

#endif


/***************************************
* cond
**************************************/
#ifdef WINNT
//windows environment

int usv_cond_init(HCOND *cond)
{
	cond->hEvent = CreateEvent(0,TRUE,FALSE,0);
	cond->count = 0;
	cond->lastop = 0;

	return (cond->hEvent == NULL); //return 0 if successful
}

int usv_cond_wait( HMUTEX *mutex, HCOND *cond, int milliseconds )
{
	if(milliseconds == -1)
		milliseconds = INFINITE;

	if(cond->lastop != 0 && cond->count == 0)
	{
		//last op no effect
		cond->lastop = 0;
		ResetEvent(cond->hEvent);
	}

	cond->count++;

	int retval = 0;
	int wait_one_more_time = 0;

	do
	{
		wait_one_more_time = 0;
		usv_mutex_unlock(mutex);

		time_t start_s = time(0);
		clock_t start_c = clock_t();

		int ret = WaitForSingleObject(cond->hEvent, milliseconds);

		usv_mutex_lock(mutex);

		if (ret == WAIT_OBJECT_0)
		{
			switch(cond->lastop)
			{
			case 0:	//shouldn't awake.
				{
					if(milliseconds == INFINITE)
					{
						wait_one_more_time = 1;
					}
					else
					{
						time_t end_s = time(0);
						clock_t end_c = clock();

						milliseconds -=(end_s - start_s) * 1000 + (end_c -start_c)/(CLOCKS_PER_SEC/ 1000);
						if(milliseconds > 0)
						{
							wait_one_more_time = 1;
						}
						else
							retval = -1;	//time out;
					}
				}
				break;
			case 1:	//signaled
				cond->lastop = 0;
				--cond->count;
				ResetEvent(cond->hEvent);
				break;
			default:	//broadcast
				if(--cond->count == 0)
				{
					cond->lastop = 0;
					ResetEvent(cond->hEvent);
				}
				break;
			}
		}
		else if(ret == WAIT_TIMEOUT)
		{
			cond->count--;
			retval = -1;
		}
		else
			retval = -2;
	} while(wait_one_more_time);

	return retval;

}

int usv_cond_signal( HCOND *cond )
{
	cond->lastop = 1;
	SetEvent(cond->hEvent);
	return 0;
}

int usv_cond_broadcast(HCOND *cond)
{
	cond->lastop = 2;
	SetEvent(cond->hEvent);
	return 0;
}

int usv_cond_destroy( HCOND *cond )
{
	if(CloseHandle(cond->hEvent))
		return 0;

	return -1;
}

#else
//linux environment
int usv_cond_init(HCOND *cond)
{
	return pthread_cond_init(cond, 0);
}

// wait condition until timed out. millisecond == -1 indicates a infinite waiting.
int usv_cond_wait(HMUTEX *mutex, HCOND *cond, int milliseconds)
{
	int ret;
	if (milliseconds == -1)
	{
		ret = pthread_cond_wait(cond, mutex);
	}
	else
	{
		timeval tv;
		timespec tm;

		gettimeofday(&tv, 0);

		tm.tv_sec = tv.tv_sec + milliseconds / 1000;
		tm.tv_nsec = (milliseconds % 1000) * 10000 + tv.tv_usec;
		tm.tv_sec += tm.tv_nsec / 1000000;
		tm.tv_nsec = tm.tv_nsec % 1000000;

		ret = pthread_cond_timedwait(cond, mutex, &tm);
	}

	if (ret != 0)
	{
		if (errno == ETIMEDOUT)
			ret = -1;
		else
			ret = -2;
	}

	return ret;
}


//signal
int usv_cond_signal(HCOND *cond)
{
	return pthread_cond_signal(cond);
}

int usv_cond_broadcast(HCOND *cond)
{
	return pthread_cond_broadcast(cond);
}

int usv_cond_destory(HCOND *cond)
{
	return pthread_cond_destroy(cond);
}





#endif