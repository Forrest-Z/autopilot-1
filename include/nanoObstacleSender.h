#ifndef __NANO_OBSTACLE_SENDER__H_
#define __NANO_OBSTACLE_SENDER__H_

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/ws.h>
#include "ApfMethod.h"
#include "../src/proto/obstacle/obstacle_radar.pb.h"
#include "../src/proto/obstacle//header.pb.h"
#include "../src/proto/obstacle/error_code.pb.h"
#include "../src/proto/obstacle/geometry.pb.h"
#include "commMsgQueue/commMsgQueue.h"

class nanoObstacleSender
{
public:
	nanoObstacleSender();
	virtual ~nanoObstacleSender();

public:
	void obstacleGet(vector<StruApfObstacle> obsVec);
	void obstacleSend();

	void sendHeaderTest(void);
	void recvHeaderTest(void);

	void sendSingleObsTest(void);
	void recvSingleObsTest(void);

	void sendObstaclesTest(void);
	void recvObstaclesTest(void);

	void obstacleSendVec(vector<StruApfObstacle> obsVec);

	//test Msg Queue
	void InitObsQueue(void);
	void obstacleEnqueue(vector<StruApfObstacle> obsVec);
	void obstacleGetHeadOfQueue(void);
	
	void setNanoSock(char *nanoAddr);
	void getNanoSock(char *nanoAddr);

private:
	int	m_nanoSock;
	char m_nanoAddr[30];
	char *pBuf;
	usv::perception::PerceptionObstacles m_obstacles;
	usv::common::Header	*m_pHeader;
	usv::common::ErrorCode m_Error;
	usv::perception::PerceptionObstacle *m_pObstacle;
	usv::perception::PerceptionObstacle m_singleObstacle;
	usv::common::Header hd;
	
	//消息队列
	HMUTEX m_obsMsgMutex;
	HCOND m_obsMsgCondition;
	CMsgQueue <vector<StruApfObstacle> >  m_obsQueue;


private:
	int initNanoSender(char* nanoAddr);		//暂时定义为发布订阅模式
	void initPbHeader(void);				//初始化报文头
	

};

#endif
