/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/zmqGetObstacles.h"
#include "../include/nanoObstacleSender.h"
/******************************  Local Variable  *****************************/
char obsCommCfg[30];
OBS_STRU	obs_var;
COMM_SIGN	obsComm_sign={0,0};
uint8 log_b1_obsComm_sign=0;
uint8 log_b1_obsFault_sign = 1;




/******************************  Extern Variable  ****************************/
extern nanoObstacleSender senderTestV1;
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

 void *zmq_pub_ins(void *aa)
{
	 MotionInputDataMsg send_ins_msg;
	 int option_vale = 1;
	 void *context_pub = zmq_ctx_new();
	 void *pubscriber = zmq_socket(context_pub, ZMQ_PUB);
	 zmq_setsockopt(pubscriber, ZMQ_SNDHWM, &option_vale, sizeof(option_vale));
	 zmq_bind(pubscriber, "tcp://*:5610"); //pub
	 while (1)
	 {
		 zmq_msg_t msg_snd;
		 send_ins_msg.yaw_rate = ins_msg.i16_rot;
		 send_ins_msg.yaw = ins_msg.heading;
		 send_ins_msg.speed = ins_msg.speed;
		 send_ins_msg.motion_direction = ins_msg.motionDirection; //deg

		 std::string header = "usvMotion";
		 zmq_send(pubscriber, header.c_str(), header.length(), ZMQ_SNDMORE);
		 int rc = zmq_msg_init_size(&msg_snd, sizeof(MotionInputDataMsg));
		 memcpy(zmq_msg_data(&msg_snd), &send_ins_msg, sizeof(MotionInputDataMsg));
		 rc = zmq_msg_send(&msg_snd, pubscriber, 0);
		 if (rc == sizeof(MotionInputDataMsg)){
			 //SysLogMsgPost("motion data send successed");
		 }
		 else{
			 //SysLogMsgPost("motion data send failed");
		 }
		 sleep_1(100);	//100ms
	 }
	 zmq_close(pubscriber);
	 zmq_ctx_destroy(context_pub);
}

void * zmq_getObstacles( void *aa )
{

	memset((char*)&obs_var,0,sizeof(obs_var));
	void *context = zmq_ctx_new();
	void *subscriber = zmq_socket(context, ZMQ_SUB);

	int option_vale = 1;
	zmq_setsockopt(subscriber, ZMQ_RCVHWM, &option_vale, sizeof(option_vale));
	zmq_connect(subscriber,obsCommCfg);//sub
	//zmq_connect(subscriber,"tcp://192.158.17.25:5600");
	zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "usvObstacle", ZMQ_DONTWAIT);
	int i;
	StruApfObstacle zmqObsSingle;
	vector <StruApfObstacle> zmqObsVec;	

	//char zmq_send_msg[128];
	while(1)
	{
		int	obsRealNum;
		zmq_msg_t msg;
		char *content = s_recv(subscriber);
		int responseLen = zmq_msg_init(&msg);
		responseLen = zmq_msg_recv(&msg,subscriber,0);

		if(responseLen>1)
		{
			char *pStr = (char*)malloc(responseLen);
			memcpy(pStr,zmq_msg_data(&msg),responseLen);
			memcpy((char*)&obs_var.obsNum,zmq_msg_data(&msg),sizeof(obs_var.obsNum));
			memcpy((char*)&obs_var.obsSenserValid,pStr+2,sizeof(obs_var.obsSenserValid));
			memcpy((char*)&obs_var.obsInsValid,pStr+3,sizeof(obs_var.obsInsValid));
			obsRealNum = obs_var.obsNum > OBS_NUM_MAX ? OBS_NUM_MAX:obs_var.obsNum;

			//printf("obsNum %d....\n", obs_var.obsNum);

			if (log_b1_obsFault_sign != obs_var.obsSenserValid){ 
				if (obs_var.obsSenserValid == 0)
				{
					WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_PREDICTION_INVALID, WARN_ON);
					continue;
				}
				else{ 
					WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_PREDICTION_INVALID, WARN_OFF);
				}
			}
			log_b1_obsFault_sign = obs_var.obsSenserValid;
			for(i=0;i<obsRealNum;i++)
			{
				memcpy((char*)&obs_var.obsAttr[i].lat,(char*)(pStr+i*29+0+8),sizeof(double));
				memcpy((char*)&obs_var.obsAttr[i].lng,(char*)(pStr+i*29+8+8),sizeof(double));
				memcpy((char*)&obs_var.obsAttr[i].radius,(char*)(pStr+i*29+16+8),sizeof(double));
				memcpy((char*)&obs_var.obsAttr[i].velocityMag,(char*)(pStr+i*29+20+8),sizeof(double));
				memcpy((char*)&obs_var.obsAttr[i].velocityDir,(char*)(pStr+i*29+24+8),sizeof(double));
				memcpy((char*)&obs_var.obsAttr[i].type,(char*)(pStr+i*29+28+8),sizeof(uint8));
			}

			monitor_all_inf.monitor_comm_inf[MONITOR_COMM_MWRADAR_SN].rec_ok_number++;			//���ճɹ�����

			comm_time_return(&(obsComm_sign.timer),&(obsComm_sign.comm_sign));

		
			zmqObsVec.clear();
			double dist = 0;
			float angle = 0;
			POSITION ps,p0;
			p0.lat = ins_msg.latitude;
	        p0.lng = ins_msg.longitude;


			for (i = 0; i < obsRealNum; i++)
			{
				dist = obs_var.obsAttr[i].lat;
				angle = wrap_360_cd(obs_var.obsAttr[i].lng + ins_msg.heading);
				ps =  APF_PositionCalc(p0,dist,angle);

				zmqObsSingle.lat = ps.lat;
				zmqObsSingle.lng = ps.lng;
				zmqObsSingle.radius = obs_var.obsAttr[i].radius ;
				zmqObsSingle.infactRadius = obs_var.obsAttr[i].radius;
				zmqObsSingle.speed = obs_var.obsAttr[i].velocityMag;
				zmqObsSingle.heading = obs_var.obsAttr[i].velocityDir;
				zmqObsVec.push_back(zmqObsSingle);
			}
			//printf("obsRealNum = %d\n",obsRealNum);
			senderTestV1.obstacleEnqueue(zmqObsVec);
		}
		sleep_1(50);	//50ms
	}
	//we never get here, but clean up anyhow
	zmq_close(subscriber);
	zmq_ctx_destroy(context);

}

void obsCommCal( void * )
{
	comm_time_cal(OBS_COMM_DISCONNECT_MAX,&(obsComm_sign.timer),&(obsComm_sign.comm_sign));
	if(log_b1_obsComm_sign!=obsComm_sign.comm_sign && poweron_init)
	{
		switch(obsComm_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	
		    printf("Avoidance: connect Radar success!\n");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_PREDICTION_TIMEOUT, WARN_OFF);
			break;
		case COMM_CONNECT_FAIL:	
			obsClear();
			printf("Avoidance: connect Radar failed!\n");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_PREDICTION_TIMEOUT, WARN_ON);
			break;
		default:
			break;
		}
	}

	log_b1_obsComm_sign = obsComm_sign.comm_sign;
}

void obsCommCalInit( void )
{
//	addTask(4,obsCommCal,(void*)0);	//1s����
}

void obsClear( void )
{
	memset((char *)&obs_var,0,sizeof(obs_var));
}

inline uint8 obsPerceptionComm(void)
{
	return log_b1_obsComm_sign;
}
