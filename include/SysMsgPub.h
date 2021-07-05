#ifndef __SYS_MSG_POST__H_
#define __SYS_MSG_POST__H_

#include "usv_include.h"
#include "zhelper.h"

#define MAX_POST_QUE_DEPTH_LOG 128



extern char nanoSysMsgAddr[30];	//������־��ַ

extern void		SysPubMsgInit(void);		//ϵͳ������־��ʼ��
extern uint8	SysPubMsgPost(const char *fmt,...);
extern void		PubMsgTest(void);
extern void		SysMsgPub(void);

int gb2312ToUtf8(char *sOut, int iMaxOutLen, const char *sIn, int iInLen);


#endif /*__SYS_MSG_POST__H_*/