/******************************************************************************************
// Beijing Sifang Automation Co.,Ltd.
// Building 9,Fourth Avenue,Shangdi Information Industry Base,Haidian District
// (C) Copyright 2009,Beijing
// All Rights Reserved
//
//
// FileName:        pii_readIni.h
// Programmer(s):   XiaoZQ ,2009-8-5
// Description:     ��ȡini�����ļ��ĺ�������
//                  [s1] s2=value ";"��Ϊע��
// 
// added ini_splitStr() XiaoZQ ,2009-8-11
*******************************************************************************************/
#ifndef _READ_USV_CFG_H_
#define _READ_USV_CFG_H_

#ifndef WINNT 
const char USV_CFG_FILE_NAME[]="../cfg/USVconfig.cfg";						//�����ļ�
const char USV_SETTING_FILE_NAME[]="../cfg/usv_flash_setting.inf";						//�����ļ�
#else
const char USV_CFG_FILE_NAME[]="../../cfg/USVconfig.cfg";						//�����ļ�
const char USV_SETTING_FILE_NAME[]="../../cfg/usv_flash_setting.inf";						//��ֵ�ļ�
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
/* ����buff�Ŀռ�Ϊ(32+strlen(pIniString)), sizeof(int) */
/* ��ɴ˺����󣬵��÷����ͷ�pIniString�ռ� */
extern int  ini_Initialize(const char * pIniString, int * buff, int buffLength);
extern int16 read_flash_mu_ini(void);

/* ��ȡ�ַ����������޶�Ӧ��ֵʱ������0 */
/* *pChildCountΪֵ���ɡ�,�����ѵ��ַ�����Ŀ*/
extern char *  ini_GetVarStr(const char * s1, const char * s2, int32 * pChildCount);


/* ��ȡ���Ͳ������޶�Ӧ��ֵʱ������0 */
extern int  ini_GetVarInt(const char * s1, const char * s2);


/* ��ȡ����������޶�Ӧ��ֵʱ������0.0 */
extern float  ini_GetVarFloat(const char * s1, const char * s2);


/* �ͷ��ڴ�(�ɲ�����) */
/* ��ɴ˺����󣬵��÷����ͷ�buff */
/* ���ô˺���֮ǰ��ini���ö�ȡ��������ȫ��ʹ�� */
extern void  ini_Finalize();
/* ��str��ȡ����(֧��0x��ʽ��16����) */
extern int  ini_str2hex(const char * str);
/* ��str��ȡ������ */
extern float  ini_str2float(const char * str);
/* ��str��ȡ˫���ȸ����� */
extern double  ini_str2double(const char * str);
/* ��ȡ���ַ�split���еĵ�n���ֶ� */
char *  ini_splitStr(const char * str, char split, int n, int32 * pLength);

void input_cfg_ini_err_sub(int8 *string_title,int8 *string_line1,uint8 param1);
int8	read_usv_cfg(void);
void set_usv_param_default(void);

#endif
