#ifndef __WARN_PROCESS__H_
#define __WARN_PROCESS__H_
#include "usv_include.h"

#ifndef WINNT
const char ARM_WARN_CFG_FILE_NAME[] = "../cfg/arm_warn_setting.inf";		//�����ļ�
const char IHC_WARN_CFG_FILE_NAME[] = "../cfg/ihc_warn_setting.inf";		//�����ļ�
#else
const char ARM_WARN_CFG_FILE_NAME[] = "../../cfg/arm_warn_setting.inf";		//�����ļ�
const char IHC_WARN_CFG_FILE_NAME[] = "../../cfg/ihc_warn_setting.inf";		//�����ļ�
#endif

const uint16 ARM_WARN_SQ_BASE = 0;
const uint16 IHC_WARN_SQ_BASE = 800;

#define WARN_MSG_LEN_MAX 128
typedef enum {
	WARN_SRC_ARM = 0,
	WARN_SRC_IHC,
	WARN_SRC_MPC,
	WARN_SRC_IDU
} WARN_SOURCE;		

typedef enum {
	ARM_WARN_MAIN = 0,
	ARM_WARN_DRADIO_TIMEOUT		,		//���ֵ�̨ͨѶ��ʱ
	ARM_WARN_BRADIO_TIMEOUT		,		//�����̨ͨѶ��ʱ
	ARM_WARN_INS_RCV_TIMEOUT	,		//�ߵ����Ľ��ճ�ʱ
	ARM_WARN_INS_LOC_INVALID	,		//�ߵ���λʧ��
	ARM_WARN_INS_DIF_INVALID	,		//�ߵ����ʧ��
	ARM_WARN_INS_GYRO_HEADING	,		//�ߵ����ݶ���
	ARM_WARN_IHC_RCV_TIMEOUT	,		//IHCͨѶ��ʱ
	ARM_WARN_PREDICTION_TIMEOUT	,		//��֪�豸ͨѶ��ʱ
	ARM_WARN_PREDICTION_INVALID	,		//��֪�豸����
	ARM_WARN_SAMPLE_TASK_BLOCK	,		//����ϵͳ������������
	ARM_WARN_SAMPLE_TIMEOUT		,		//����ϵͳͨѶ�ж�
	ARM_WARN_SAMPLE_REP_ERROR	,		//����ϵͳ����ظ�����
	ARM_WARN_POWER_LOW_RETURN	,		//�͵����Զ���������
	ARM_WARN_COUNT						//
}ARM_WARN_DESCRIPTION_ENUM;

typedef enum{
	WARN_OFF = 0,
	WARN_ON  = 1
}ARM_WARN_STATE;



typedef struct __WARN_TIME__
{
	uint16	year;
	uint16	month;
	uint16	day;
	uint16	hour;
	uint16	minute;
	uint16	second;
	uint16	m_second;
}WARN_TIME;


typedef struct __WARN_MSG__
{
	uint8	warn_source;		//�澯��Ϣ��Դ
	uint16	warn_squence;		//�澯��Ϣ���
	int8	warn_stat;			//�澯��Ϣ״̬
	WARN_TIME warn_time;			//�澯����ʱ��
}WARN_MSG;

//��ȡϵͳʱ��
void ReadWarnTime(WARN_TIME *systime);

//��ʼ���澯ģ��
int iniWarnModel(void);

//��ʼ���澯����
void initWarnMsgQueue(void);	
//�澯��Ϣ���
void WarnMsgQueuePut(int warnSource, int warnSquence, int8 warn_stat);
//��ȡ�����ļ�
int read_warn_setting(void);
//�澯��Ϣ���Ӳ���
void WarnMsgQueuGetPub(void);

//��ʼ��nanoMsg
int initNanoPubSender(char *nanoAddr);
//nanomsg�˿ڷ��͸澯��Ϣ
void warnNanoPub(char *warnMsg);

//UDP ���Ľӿ�
int8 isWarnMsgUdpEmpty(void);
int8 getWarnMsgFromUdpQueue(WARN_MSG *pWarn_msg);
int8 delWarnMsgFromUdpQueue(void);

#endif /*__WARN_PROCESS__H_*/


