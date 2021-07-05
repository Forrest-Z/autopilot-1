/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// Building 9,Fourth Avenue,Shangdi Information Industry Base,Haidian District
// (C) Copyright 2009,Beijing
// All Rights Reserved
//
//
// FileName:        pii_readIni.h
// Programmer(s):   XiaoZQ ,2009-8-5
// Description:     读取ini配置文件的函数定义
//                  [s1] s2=value ";"后为注释
// 
// added ini_splitStr() XiaoZQ ,2009-8-11
*******************************************************************************************/
#ifndef _READ_USV_CFG_H_
#define _READ_USV_CFG_H_

#ifndef WINNT 
const char USV_CFG_FILE_NAME[]="../cfg/USVconfig.cfg";						//配置文件
const char USV_SETTING_FILE_NAME[]="../cfg/usv_flash_setting.inf";						//配置文件
#else
const char USV_CFG_FILE_NAME[]="../../cfg/USVconfig.cfg";						//配置文件
const char USV_SETTING_FILE_NAME[]="../../cfg/usv_flash_setting.inf";						//定值文件
#endif

typedef struct _tIniS2{
  char *        s2;
  int           ChildCount;
  char *        value;
  struct _tIniS2 * next;
} tIniS2;

typedef struct _tIniS1{
  char *        s1;
  int           ChildCount;
  tIniS2 *      pChildren;
  struct _tIniS1 * next;
} tIniS1;

typedef struct _tIniHandle{
  const char *  iniString;
  const char *  iniLine;
  tIniS1 *      s1List;
  int           ChildCount;
  char *        buff;
  int           buffFree;
} tIniHandle;

/* ini initialize */
/* 0=success,-1=buff overflow,-2=fault */
/* 建议buff的空间为(32+strlen(pIniString)), sizeof(int) */
/* 完成此函数后，调用方可释放pIniString空间 */
extern int  ini_Initialize(const char * pIniString, int * buff, int buffLength);
extern int16 read_flash_mu_ini(void);

/* 读取字符串参数，无对应键值时，返回0 */
/* *pChildCount为值域由“,”分裂的字符串数目*/
extern char *  ini_GetVarStr(const char * s1, const char * s2, int32 * pChildCount);


/* 读取整型参数，无对应键值时，返回0 */
extern int  ini_GetVarInt(const char * s1, const char * s2);


/* 读取浮点参数，无对应键值时，返回0.0 */
extern float  ini_GetVarFloat(const char * s1, const char * s2);


/* 释放内存(可不调用) */
/* 完成此函数后，调用方可释放buff */
/* 调用此函数之前，ini配置读取函数允许全局使用 */
extern void  ini_Finalize();
/* 从str读取整数(支持0x方式的16进制) */
extern int  ini_str2hex(const char * str);
/* 从str读取浮点数 */
extern float  ini_str2float(const char * str);
/* 从str读取双精度浮点数 */
extern double  ini_str2double(const char * str);
/* 读取被字符split分列的第n个字段 */
char *  ini_splitStr(const char * str, char split, int n, int32 * pLength);

void input_cfg_ini_err_sub(int8 *string_title,int8 *string_line1,uint8 param1);
int8	read_usv_cfg(void);
void set_usv_param_default(void);

#endif
