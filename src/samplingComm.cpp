/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/samplingComm.h"

#define REQUEST_TIMEOUT 3000	
#define REQUEST_RETRIES 3		
#define SAMPLING_TIMESLICE 100	//100ms

/******************************  Local Variable  *****************************/
char samplingCfg[30];	
SAMPLE_CMD		smpCmd;
SAMPLE_REPLY	smpRep;
SAMPLE_CTRL		smpCtrl;
uint8 sampleCommState=1;
uint8 sampleTaskBlock = 0;
uint8 sampleRepError = 0;
HMUTEX mutex_smp;

/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/





int8 myStrcmp( int8* s1,int8* s2,int length )
{
	if(s1==NULL || s2==NULL) return FALSE;
	if(strlen(s1)<length || strlen(s2)<length) return FALSE;

	for(int i=0;i<length;i++)
	{
		if(s1[i]!=s2[i])
			return FALSE;
	}
	return TRUE;
}

void write_sampling_string( char* outString,SAMPLE_CMD smp_cmd )
{
	sprintf_usv(outString,"Req=%d;ID=%lld;Sample=%d;",smp_cmd.requestSn,smp_cmd.sampleID,smp_cmd.sampleCmd);
}

void writeSamplingStart( char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd )
{
	smp_cmd->requestSn = requestSn;
	smp_cmd->sampleCmd = SAMPLE_CMD_ASK_START;
	//write_sampling_string(outString,*smp_cmd);
	sprintf_usv(outString,"Req=%d;ID=%lld;Volume=%d;Sample=%d;",smp_cmd->requestSn,smp_cmd->sampleID,smp_cmd->sampleVolume,smp_cmd->sampleCmd);
}

void writeSamplingStatus( char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd )
{
	smp_cmd->requestSn = requestSn;
	smp_cmd->sampleCmd = SAMPLE_CMD_ASK_STATUS;
	write_sampling_string(outString,*smp_cmd);
}


void writeSamplingCancel( char* outString,uint16 requestSn,SAMPLE_CMD *smp_cmd )
{
	smp_cmd->requestSn = requestSn;
	smp_cmd->sampleCmd = SAMPLE_CMD_ASK_CANCEL;
	write_sampling_string(outString,*smp_cmd); 
}



int8 read_sub_value( int8* main_item,int8* sub_item, uint16 *outdata )
{
	int8 iret_val = FALSE;	

	int8 *p;
	int8 *ptrNum;
	p=main_item;
	int i;

	int main_item_len = strlen(main_item);
	int sub_item_len = strlen(sub_item);

	if(main_item_len == 0 || sub_item_len == 0)
		return FALSE;


	for(i=0;i<main_item_len;i++){
		if(myStrcmp(p+i,sub_item,sub_item_len)){
			ptrNum = p+i+sub_item_len;
			main_item_len -= (i+sub_item_len);
			iret_val = TRUE;
			break;
		}
	}

	if(iret_val == FALSE) return iret_val;
	iret_val = FALSE;

	for(i=0;i<main_item_len;i++){
		if(*(ptrNum+i)==' ');
		else if(*(ptrNum+i)=='='){
			p=ptrNum+i+1;
			main_item_len -= (i+1);
			iret_val = TRUE;
			break;
		}
		else{
			iret_val = FALSE;
		}

	}

	if(iret_val == FALSE) return iret_val;

	iret_val = FALSE;


	for(i=0;i<main_item_len;i++){
		if(*(ptrNum+i)==' ');
		else if(*(p+i)>='0'&&*(p+i)<='9'){
			ptrNum = p+i;
			iret_val = TRUE;
			break;
		}
		else{
			iret_val = FALSE;
		}
	}

	if(iret_val == FALSE) return iret_val;

	*outdata = ini_str2hex(ptrNum);
	return TRUE;
}



int8 read_sub_value64( int8* main_item,int8* sub_item,uint64 *outdata )
{
	int8 iret_val = FALSE;	

	int8 *p;
	int8 *ptrNum;
	p=main_item;
	int i;

	int main_item_len = strlen(main_item);
	int sub_item_len = strlen(sub_item);

	if(main_item_len == 0 || sub_item_len == 0)
		return FALSE;


	for(i=0;i<main_item_len;i++){
		if(myStrcmp(p+i,sub_item,sub_item_len)){
			ptrNum = p+i+sub_item_len;
			main_item_len -= (i+sub_item_len);
			iret_val = TRUE;
			break;
		}
	}

	if(iret_val == FALSE) return iret_val;
	iret_val = FALSE;


	for(i=0;i<main_item_len;i++){
		if(*(ptrNum+i)==' ');
		else if(*(ptrNum+i)=='='){
			p=ptrNum+i+1;
			main_item_len -= (i+1);
			iret_val = TRUE;
			break;
		}
		else{
			iret_val = FALSE;
		}

	}

	if(iret_val == FALSE) return iret_val;

	iret_val = FALSE;

	for(i=0;i<main_item_len;i++){
		if(*(ptrNum+i)==' ');
		else if(*(p+i)>='0'&&*(p+i)<='9'){
			ptrNum = p+i;
			iret_val = TRUE;
			break;
		}
		else{
			iret_val = FALSE;
		}
	}

	if(iret_val == FALSE) return iret_val;

	*outdata = ini_str2hex64(ptrNum);
	return TRUE;
}



int8 read_status( char* recvString,SAMPLE_REPLY* sampleReply )
{
	int8 iret = 1;
	char s2[50];

	sprintf_usv(s2,"Rep");
	if(read_sub_value(recvString,s2,&(sampleReply->replySn))==FALSE){
		iret = FALSE;
	}
	sprintf_usv(s2,"ID");
	if(read_sub_value64(recvString,s2,&(sampleReply->sampleID))==FALSE){
		iret = FALSE;
	}
	sprintf_usv(s2,"Volume");
	if(read_sub_value(recvString,s2,&(sampleReply->sampleVolume))==FALSE){
		iret = FALSE;
	}
	sprintf_usv(s2,"Status");
	if(read_sub_value(recvString,s2,&(sampleReply->sampleStatus))==FALSE){
		iret = FALSE;
	}
	if(sampleReply->sampleStatus == 4)
	{
		sprintf_usv(s2,"Error");
		if(read_sub_value(recvString,s2,&(sampleReply->errorID))==FALSE){
			iret = FALSE;
		}
	}
	else{
		sampleReply->errorID = 0;
	}

	return iret;
}

void samplingInit( void )
{
	memset((char*)&smpCmd,0,sizeof(smpCmd));
	memset((char*)&smpRep,0,sizeof(smpRep));
	//memset((char*)&smpCtrl,0,sizeof(smpCtrl));
	//smpCtrl.askSampleStart = 1;
}

void * samplingComm( void *aa )
{
	samplingInit();
	
	usv_mutex_init(&mutex_smp);
	void* ctx	 = zmq_ctx_new();
	void* client = zmq_socket(ctx,ZMQ_REQ);
	zmq_connect(client,samplingCfg);
	int retries_left = REQUEST_RETRIES;
	int8 sampleOnOld=0;
	char request[100];
	uint16 requstSn=0;

	//test
	uint16 testSampleID = 0;

	for(;;){
		if(smpCtrl.askSampleStart == 1 ){	
			retries_left = REQUEST_RETRIES;
			while(retries_left && (smpCtrl.repSampleTaskRev!=1))
			{
				writeSamplingStart(request,requstSn++,&smpCmd);
				s_send(client,request);

				int expect_reply = 1;
				while(expect_reply){
					zmq_pollitem_t items[]={{client,0,ZMQ_POLLIN,0}};
					int rc = zmq_poll(items,1,REQUEST_TIMEOUT*1);
					if(rc==-1) break; 

					if(items[0].revents&ZMQ_POLLIN){
					
						char *reply = s_recvNoWait(client);
						if(!reply){
							break;	
						}
						read_status(reply,&smpRep);
						
						//printf("%s\n",reply);
						free(reply);
						if(smpRep.replySn == smpCmd.requestSn){
						
							
								retries_left = REQUEST_RETRIES;
								
								if(smpRep.sampleStatus == SAMPLE_STATUS_CMD_RECV){
									expect_reply = 0;
									usv_mutex_lock(&mutex_smp);
									smpCtrl.repSampleTaskRev = 1;
									smpCtrl.askSampleStatus = 1;
									smpCtrl.askSampleStart = 0;
									smpCtrl.repSampleFinish = 0;
									usv_mutex_unlock(&mutex_smp);
							
									
									if (sampleCommState == 0){
										WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_TIMEOUT, WARN_OFF);
										sampleCommState = 1;
									}
									if (sampleTaskBlock == 1){
										WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_TASK_BLOCK, WARN_OFF);
										sampleTaskBlock = 0;
									}
									if (sampleRepError == 1){
										WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_REP_ERROR, WARN_OFF);
										sampleRepError = 0;
									}
									break;
								}
							
								else if(smpRep.sampleStatus ==SAMPLE_STATUS_BUSY){
									expect_reply = 0;
									if (sampleTaskBlock == 0){

										WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_TASK_BLOCK, WARN_ON);
										sampleTaskBlock = 1;
									}
									break;
								}
								
								else{
									expect_reply = 0;
									if (sampleRepError == 0){										

										WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_REP_ERROR, WARN_ON);
										sampleRepError = 1;
									}
									break;
								}
						}
					}
					else{
						if(--retries_left == 0){
	
							if (sampleCommState == 1){
								WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_SAMPLE_TIMEOUT, WARN_ON);
								sampleCommState = 0;
							}
							smpCtrl.askSampleStart = 0;
							smpCtrl.askSampleStatus = 0;
							break;
						}
						else{
							//retry
							zmq_close(client);
							client = zmq_socket(ctx,ZMQ_REQ);
							zmq_connect(client,samplingCfg);

							s_send(client,request);
						}
					}
				}
			sleep_1(SAMPLING_TIMESLICE*10);
			}
		}


		if(smpCtrl.askSampleStatus == 1){	//
			retries_left = REQUEST_RETRIES;
			while((retries_left) && (smpCtrl.repSampleFinish != 1)&&(smpCtrl.askSampleStart != 1) && (smpCtrl.askSampleCancel != 1)){		
				writeSamplingStatus(request,requstSn++,&smpCmd);
				s_send(client,request);
			
				int expect_reply = 1;
				while(expect_reply){
					zmq_pollitem_t items_s[]={{client,0,ZMQ_POLLIN,0}};
					int rc = zmq_poll(items_s,1,REQUEST_TIMEOUT*1);
					if(rc==-1) break;	

			
					if(items_s[0].revents&ZMQ_POLLIN){
				
						char *reply = s_recvNoWait(client);
						if(!reply){
							break;
						}
						read_status(reply,&smpRep);
					
						//printf("%s\n",reply);
						free(reply);	
						if(smpRep.replySn == smpCmd.requestSn){
							retries_left = REQUEST_RETRIES;
				
							if(smpRep.sampleStatus == SAMPLE_STATUS_SMP_FINISHED){
						
								usv_mutex_lock(&mutex_smp);
								smpCtrl.askSampleStatus = 0;
								smpCtrl.repSampleFinish = 1;
								usv_mutex_unlock(&mutex_smp);
								break;
							}
							if(smpRep.sampleStatus == SAMPLE_STATUS_ERROR){
							
								usv_mutex_lock(&mutex_smp);
								smpCtrl.askSampleStatus =0;
								usv_mutex_unlock(&mutex_smp);

								break;
							}
							break;
						}
						
					}
					else{
						if(--retries_left == 0){
						
							usv_mutex_lock(&mutex_smp);
							smpCtrl.askSampleStatus = 0;
							usv_mutex_unlock(&mutex_smp);
	
							break;
						}
						else{
							//retry
							zmq_close(client);
							client = zmq_socket(ctx,ZMQ_REQ);
							zmq_connect(client,samplingCfg);
					
							s_send(client,request);
						}
					}
				}
				sleep_1(SAMPLING_TIMESLICE*10);
			}


		}

		if(smpCtrl.askSampleCancel == 1){	
			retries_left = REQUEST_RETRIES;
			while(retries_left && (smpCtrl.repSampleCanceled != 1)){
				writeSamplingCancel(request,requstSn++,&smpCmd);
				s_send(client,request);

	
				int expect_reply = 1;
				while(expect_reply){
					zmq_pollitem_t items[] = {{client,0,ZMQ_POLLIN,0}};
					int rc = zmq_poll(items,1,REQUEST_TIMEOUT*1);
					if(rc==-1) break; 

				
					if(items[0].revents&ZMQ_POLLIN){
						
						char *reply = s_recvNoWait(client);
						if(!reply){
							break;	
						}
						read_status(reply,&smpRep);
				
						free(reply);
						if(smpRep.replySn == smpCmd.requestSn){
						
								retries_left = REQUEST_RETRIES;
								
								if(smpRep.sampleStatus == SAMPLE_STATUS_CANCELED){
									expect_reply = 0;
									usv_mutex_lock(&mutex_smp);
									smpCtrl.repSampleCanceled = 1;
									smpCtrl.askSampleCancel = 0;
									smpCtrl.askSampleStart = 0;
									smpCtrl.askSampleStatus = 0;
									usv_mutex_unlock(&mutex_smp);
							
									break;
								}
								
								else{
									expect_reply = 0;
							
									break;

								}

						}
					}
					else{
						if(--retries_left == 0){
						
							usv_mutex_lock(&mutex_smp);
							smpCtrl.askSampleCancel = 0;
							smpCtrl.repSampleCanceled = 0;
							usv_mutex_unlock(&mutex_smp);
					
							break;
						}
						else{
							//retry
							zmq_close(client);
							client = zmq_socket(ctx,ZMQ_REQ);
							zmq_connect(client,samplingCfg);
						
							s_send(client,request);
						}
					}
				}
			sleep_1(SAMPLING_TIMESLICE*10);
			}

		}


		sleep_1(SAMPLING_TIMESLICE);	
	}

	


	zmq_close(client);
	zmq_ctx_destroy(ctx);
	return ((void *)0);
}

char * s_recvNoWait( void *socket )
{
	char buffer [256];
	//int size = zmq_recv (socket, buffer, 255, 0);
	int size = zmq_recv (socket, buffer, 255, ZMQ_DONTWAIT);
	if (size == -1)
		return NULL;
	buffer[size] = '\0';

#if (defined (WIN32))
	return strdup (buffer);
#else
	return strndup (buffer, sizeof(buffer) - 1);
#endif

	// remember that the strdup family of functions use malloc/alloc for space for the new string.  It must be manually
	// freed when you are done with it.  Failure to do so will allow a heap attack.
}

void sampleStart( uint64 sampleId,uint16 sampleVolume )
{
	samplingInit();
	usv_mutex_lock(&mutex_smp);
	smpCmd.sampleID = sampleId;
	smpCmd.sampleVolume = sampleVolume;
	smpCtrl.askSampleStart = 1;
	smpCtrl.askSampleStatus = 0;
	smpCtrl.repSampleTaskRev = 0;
	//smpCtrl.repSampleFinish = 0;
	usv_mutex_unlock(&mutex_smp);
}

void sampleStart2(uint64 sampleId,uint8 sample_type,uint8 sample_seq)
{
	samplingInit();
	usv_mutex_lock(&mutex_smp);
	smpCmd.sampleID = sampleId;
	smpCmd.sampleVolume = (uint16)(sample_type)*256 + sample_seq;
	smpCtrl.askSampleStart = 1;
	smpCtrl.askSampleStatus = 0;
	smpCtrl.repSampleTaskRev = 0;
	//smpCtrl.repSampleFinish = 0;
	usv_mutex_unlock(&mutex_smp);
}

void sampleCancel( uint64 sampleId )
{
	samplingInit();
	usv_mutex_lock(&mutex_smp);
	smpCmd.sampleID = sampleId;
	smpCtrl.askSampleCancel = 1;
	smpCtrl.askSampleStart = 0;
	smpCtrl.askSampleStatus = 0;
	smpCtrl.repSampleCanceled = 0;
	usv_mutex_unlock(&mutex_smp);
}

int8 isSampleFinished( uint64 sampleId )
{
	if(smpRep.sampleID == sampleId){
		if(smpRep.sampleStatus == SAMPLE_STATUS_SMP_FINISHED){
			printf("sampleId:%lld finished\n",sampleId);
		//	SysPubMsgPost("sampleId:%lld finished",sampleId);
			return TRUE;
		}
		if(smpRep.sampleStatus == SAMPLE_STATUS_ERROR){
			printf("sampleId:%lld Error\n",sampleId);
		}
	}
	return FALSE;
}

uint64 ini_str2hex64(char* str )
{
	uint64 result = 0;

	if(0!=str){
		if('\0'!=str[0]){
			if(('0'==str[0])&&('x'==str[1])){
				sscanf(str,"%x",&result);
				//sprintf_usv(str,"%x",result);
			}
			else{
				sscanf(str,"%Ld",&result);
				//sprintf_usv(str,"%llu",result);
			}
		}
	}
	return result;
}


