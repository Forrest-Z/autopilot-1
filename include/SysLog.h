//SysLog.h
#ifndef _SYSLOG_H
#define _SYSLOG_H


#include "usv_include.h"

#ifdef WINNT
#define MKDIR(a) _mkdir((a))  
#else
#define MKDIR(a) mkdir((a),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) 
#endif

#define MAX_QUE_DEPTH_LOG 128
#define  MAX_LOGFILE_LEN 1024*512    //最大历史文件长度，暂定512Kb

extern void    SysLogInit(void);
extern uint8   SysLogMsgPost(const char *fmt,...);
extern void logTest(void);
extern void SysLogSave(void);

#endif  //_SYSLOG_H