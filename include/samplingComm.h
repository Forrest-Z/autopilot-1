/*
* samplingComm.h   ��������ͨѶ
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/



#ifndef __SAMPLING_COMM__H_
#define __SAMPLING_COMM__H_

#include "usv_include.h"


#define SAMPLE_CMD_ASK_START	1
#define SAMPLE_CMD_ASK_STATUS	2
#define SAMPLE_CMD_ASK_CANCEL   3

#define SAMPLE_STATUS_CMD_RECV		1
#define SAMPLE_STATUS_SMP_EXECUTING 2
#define SAMPLE_STATUS_SMP_FINISHED	3
#define SAMPLE_STATUS_ERROR			4
#define SAMPLE_STATUS_BUSY			5
#define SAMPLE_STATUS_CANCELED		6	//���������Ѿ�ȡ��


typedef struct{
	unsigned short	requestSn;
	uint64			sampleID;
	unsigned short	sampleVolume;
	unsigned short	sampleCmd;
}SAMPLE_CMD;

typedef	struct{
	unsigned short	replySn;
	uint64			sampleID;
	unsigned short	sampleVolume;
	unsigned short	sampleStatus;
	unsigned short	errorID;
}SAMPLE_REPLY;

typedef struct{
	int8	askSampleStart;
	int8	repSampleTaskRev;
	int8	askSampleStatus;
	int8	repSampleFinish;
	int8	askSampleCancel;
	int8	repSampleCanceled;
}SAMPLE_CTRL;

extern char samplingCfg[30];
extern SAMPLE_REPLY	smpRep;

int8 myStrcmp(int8* s1,int8* s2,int length);									//�ַ����Ƚ�
void write_sampling_string(char* outString,SAMPLE_CMD smp_cmd);					
void writeSamplingStart(char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd);  //��������
void writeSamplingStatus(char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd); //״̬�ٻ�
void writeSamplingCancel(char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd);	//��������ȡ��
int8 read_sub_value(int8* main_item,int8* sub_item, uint16 *outdata);			//��main_item�ж�ȡsub_item�ؼ��ֵ�����
uint64 ini_str2hex64(char* str);
int8 read_sub_value64(int8* main_item,int8* sub_item,uint64 *outdata);			//��main_item�ж�ȡsub_item�ؼ��ֵ����� ������
int8 read_status(char* recvString,SAMPLE_REPLY* sampleReply);					//��ȡ�ַ����ж�ȡ��عؼ��ֵ�ֵ
//sampling thread;
void *samplingComm(void *aa);	//���������շ��߳�
void samplingInit(void);		

static char *s_recvNoWait (void *socket);	//����������

//�ⲿ�ӿ�
void sampleStart( uint64 sampleId,uint16 sampleVolume );
void sampleStart2(uint64 sampleId,uint8 sample_type,uint8 sample_seq);
int8 isSampleFinished(uint64 sampleId);
void sampleCancel( uint64 sampleId);



#endif /*__SAMPLING_COMM__H_*/