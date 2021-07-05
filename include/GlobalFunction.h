//GlobalFunction.h
#ifndef __GLOBALFUNCTION__H_
#define __GLOBALFUNCTION__H_
#include "usv_include.h"

//通讯状态
#define COMM_CONNECT_OK 0
#define COMM_CONNECT_FAIL 1

typedef struct{
	uint32	timer;
	uint8	comm_sign;		//定义1:无连接	0:通讯正常
}COMM_SIGN;

extern void comm_time_cal(uint32 maxTime,uint32* timebuf,uint8* comm_sign);
extern void comm_time_return(uint32* timebuf,uint8* comm_sign);
extern float u8tofloat(uint8* src);
extern uint16 u8tou16(uint8* src);
extern int16 u8toi16(uint8* src);
extern uint32 u8tou32(uint8* src);
extern uint64 u8tou64(uint8* src);

extern uint16 calcCrc(uint8* p,uint16 len);
uint16 calcCRC(uint16 pseed,uint8 *buf,uint16 len);

//填充报文头
void fill_head(uint8* pBuf,const char* head,uint16 frameNo,uint8 frameLength);

void sendCanMsg(CAN_TYPE canPort,uint8 canIdPF,uint8 canIdPS,uint8 canIdAddr,uint8* pCanData);

//高低值限制
int16	i16UpDownLimit(int16 maxLimit,int16 minLimit,int inputData);
uint8 u8UpLimit(uint8 maxLimit,uint8 inputData);


/********************************************************************************************/
/* mutex																		
/********************************************************************************************/
#ifdef WINNT
//windows version
typedef CRITICAL_SECTION HMUTEX;
#else
//linux version
typedef pthread_mutex_t HMUTEX;
#endif

//return 0 if successful
int usv_mutex_init(HMUTEX *mutex);

//return 0 if successful
int usv_mutex_lock(HMUTEX *mutex);

//return 0 if successful
int usv_mutex_unlock(HMUTEX *mutex);

//return 0 if successful
int usv_mutex_destory(HMUTEX *mutex);


/********************************************************************************************/
/* cond 条件变量
/********************************************************************************************/
#ifdef WINNT
//windows version
typedef struct 
{
	uint32 count;
	uint32 lastop;	//0 none, 1 signal, 2 broadcast
	HANDLE hEvent;
}HCOND;
#else
//linux version
typedef pthread_cond_t HCOND;
#endif

// return 0 if successful
int usv_cond_init(HCOND *cond);

// wait condition until timed out. milliseconds == -1 indicates an infinite waiting.
// return 0 if successful, -1 for time out, otherwise for errors
int usv_cond_wait(HMUTEX *mutex, HCOND *cond, int milliseconds);

// signal
// return 0 if successful, -1 for time out, otherwise for errors
int usv_cond_signal(HCOND *cond);

// return 0 if successful
int usv_cond_broadcast(HCOND *cond);

// return 0 if successful
int usv_cond_destroy(HCOND *cond);


/********************************************************************************************/
/* UTC time 
/********************************************************************************************/


#endif /*__GLOBALFUNCTION__H_*/