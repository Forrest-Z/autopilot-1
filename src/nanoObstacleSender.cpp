#include "stdafx.h"
#include "../include/nanoObstacleSender.h"
#include "../include/ApfMethod.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace usv::perception;
using namespace usv::common;

nanoObstacleSender::nanoObstacleSender()
{
	//initNanoSender("ws://*:6001");	// 初始化发送端口
	initPbHeader();	//初始化报文头
	InitObsQueue();	//初始化队列

}


nanoObstacleSender::~nanoObstacleSender()
{

}

void nanoObstacleSender::obstacleGet(vector<StruApfObstacle> obsVec)
{
	static double timestamp = 0.0;
	static int seq = 1;
	timestamp += 1.0;
	Header *head = new Header;
	head->set_module_name("arm retransit obstacle");
	head->set_timestamp_sec(timestamp);
	head->set_sequence_num(seq++);
	m_obstacles.set_allocated_header(head);

	PointLLH *pos;
	Shape *shap;
	PerceptionObstacle *singleOb;

	//考虑是不是要加个锁
	for (int i = 0; i < obsVec.size(); i++){
		singleOb = m_obstacles.add_perception_obstacle();
		singleOb->set_id(i + 1);
		pos = singleOb->mutable_positon();
		shap = singleOb->mutable_shape();
		pos->set_lat(obsVec[i].lat);
		pos->set_lon(obsVec[i].lng);
		shap->set_radius(obsVec[i].radius);
		shap->set_type(Shape_ShapeType_CIRCLE);
	}
	//cout << "m_obstacle space : " << m_obstacles.SpaceUsed() << endl;
}

void nanoObstacleSender::obstacleSend()
{
	static uint32 seq = 0;
	Header *pHeader = m_obstacles.mutable_header();
	initPbHeader();
	
	//增加报文序号及时标
	pHeader->set_sequence_num(seq++);

	//todo 增加时标
	//序列化及发送
	int bSize = m_obstacles.ByteSize();
	char *buf = (char *)malloc(bSize);

	m_obstacles.SerializeToArray(buf, bSize);
	nn_send(m_nanoSock, buf, bSize, 0);
	free(buf);
	//清空列表
	m_obstacles.Clear();
}

void nanoObstacleSender::sendHeaderTest(void)
{

	static double timestamp = 10.11;

	hd.Clear();
	timestamp += 1.1;
	hd.set_frame_id("frameId");
	hd.set_module_name("usv_name");
	//hd.set_camera_timestamp(1002.111);
	hd.set_version(21);
	hd.set_timestamp_sec(timestamp);

	fstream output("pb.xxx", ios::out | ios::trunc | ios::binary);
	bool flag = hd.SerializeToOstream(&output);	//序列化
	if (!flag)
	{
		return;
	}

	output.close();	//关闭文件

}

void nanoObstacleSender::recvHeaderTest(void)
{

	usv::common::Header tm_hd;
	fstream input("pb.xxx", ios::in | ios::binary);
	tm_hd.ParseFromIstream(&input);
	input.close();
	cout << tm_hd.module_name() << "timestamp: "<< tm_hd.timestamp_sec() << endl;

}

void nanoObstacleSender::sendSingleObsTest(void)
{
	static int32 obsId = 0;
	static double lat = 36.0,lon = 114.0;
	lat += 0.5;
	lon += 0.3;
	obsId++;
	m_singleObstacle.Clear();
	m_singleObstacle.set_id(obsId);
	
	usv::common::PointLLH *pos = m_singleObstacle.mutable_positon();
	pos->set_lat(lat);
	pos->set_lon(lon);
	pos->set_height(0.0);

	fstream output("pbSingleObs.xxx", ios::out | ios::trunc | ios::binary);
	bool flag = m_singleObstacle.SerializeToOstream(&output);	//序列化

	if (!flag)
	{
		return;
	}

	output.close();	//关闭文件

}

void nanoObstacleSender::recvSingleObsTest(void)
{
	usv::perception::PerceptionObstacle tm_singleObs;
	
	fstream input("pbSingleObs.xxx", ios::in | ios::binary);
	tm_singleObs.ParseFromIstream(&input);
	input.close();
	cout << "obs Id : " << tm_singleObs.id() << endl;
	cout << "lat = " << tm_singleObs.positon().lat() << "\t lon = " << tm_singleObs.positon().lon() << endl;

}

void nanoObstacleSender::sendObstaclesTest(void)
{
	static double timestamp = 0.0;
	static int sq = 1;
	static int loop = 1;
	m_obstacles.Clear();
	timestamp += 1.2;
	usv::common::Header *head = new usv::common::Header;
	head->set_module_name("obstacle_radar");
	head->set_timestamp_sec(timestamp);
	head->set_sequence_num(sq++);
	m_obstacles.set_allocated_header(head);

	usv::common::PointLLH *pos;
	usv::perception::Shape *shap;
	usv::perception::PerceptionObstacle *singleOb;

	if (loop++ > 30)
		loop = 1;


	for (int i = 0; i < loop; i++){
		//add obstacle
		singleOb = m_obstacles.add_perception_obstacle();
		singleOb->set_id(i + 1000);
		pos = singleOb->mutable_positon();
		shap = singleOb->mutable_shape();
		pos->set_lat(100 + timestamp + i);
		pos->set_lon(200 + timestamp + i);
		shap->set_radius(20 + timestamp + i);
	}

	//序列化发送文件发送
	fstream output("pbObs.xxx", ios::out | ios::trunc | ios::binary);
	bool flag = m_obstacles.SerializeToOstream(&output);	//序列化
	if (!flag)
	{
		return;
	}

	cout << "test obstacle space: " << m_obstacles.SpaceUsed() << endl;
	//nanomsg 序列化发送
	int nanoSize = m_obstacles.ByteSize();
	char *nanoBuf = (char *)malloc(nanoSize);

	m_obstacles.SerializeToArray(nanoBuf, nanoSize);
	nn_send(m_nanoSock, nanoBuf, nanoSize, 0);

	free(nanoBuf);


	output.close();	//关闭文件

}

void nanoObstacleSender::recvObstaclesTest(void)
{
	usv::perception::PerceptionObstacles tm_obsMsg;
	fstream input("pbObs.xxx", ios::in | ios::binary);
	tm_obsMsg.ParseFromIstream(&input);
	input.close();

	cout << "obs module name : " << tm_obsMsg.header().module_name() << endl;
	cout << "timestamp: " << tm_obsMsg.header().timestamp_sec() << endl;
	cout << "obs sum: " << tm_obsMsg.perception_obstacle_size() << endl;

	for (int i = 0; i < tm_obsMsg.perception_obstacle_size(); i++){
		cout <<"obsId: "<< tm_obsMsg.perception_obstacle(i).id() <<"\t lat = " << tm_obsMsg.perception_obstacle(i).positon().lat() << "\t lon: " << tm_obsMsg.perception_obstacle(i).positon().lon() \
			<< "\t radius = " << tm_obsMsg.perception_obstacle(i).shape().radius() << endl;
	}


}

int nanoObstacleSender::initNanoSender(char* nanoAddr)
{
	m_nanoSock = nn_socket(AF_SP, NN_PUB);
	if (m_nanoSock < 0){
		printf("init nnSock error");
		return 0;
	}

	if (nn_bind(m_nanoSock, nanoAddr) < 0){
		printf("nnSock bind error");
		return 0;
	}

	/*int opt = NN_WS_MSG_TYPE_BINARY;
	nn_setsockopt(m_nanoSock, NN_WS, NN_WS_MSG_TYPE, &opt, sizeof(opt));*/

	return 1;

	

}

void nanoObstacleSender::initPbHeader(void)
{
	Header *pHeader = m_obstacles.mutable_header();
	pHeader->set_frame_id("obstacles from radar");
	pHeader->set_module_name("arm retransmit radar msg");
}

void nanoObstacleSender::obstacleSendVec(vector<StruApfObstacle> obsVec)
{
	static double timestamp = 0.0;
	static int seq = 1;
	timestamp += 1.0;
	PerceptionObstacles tm_obstacles;
	cout << "tm_space new: " << tm_obstacles.SpaceUsed() << endl;
	Header *head = new Header;
	head->set_module_name("arm retransit obstacle");
	head->set_timestamp_sec(timestamp);
	head->set_sequence_num(seq++);
	tm_obstacles.set_allocated_header(head);
	
	PointLLH *pos;
	Shape *shap;
	PerceptionObstacle *singleOb;
	
	//考虑是不是要加个锁
	for (int i = 0; i < obsVec.size(); i++){
		singleOb = tm_obstacles.add_perception_obstacle();
		singleOb->set_id(i + 1);
		pos = singleOb->mutable_positon();
		shap = singleOb->mutable_shape();
		pos->set_lat(obsVec[i].lat);
		pos->set_lon(obsVec[i].lng);
		shap->set_radius(obsVec[i].radius);
		shap->set_type(Shape_ShapeType_CIRCLE);
	}

	//序列化 nanomsg 发送
	//nanomsg 序列化发送
	int nanoSize = tm_obstacles.ByteSize();
	char *nanoBuf = (char *)malloc(nanoSize);

	tm_obstacles.SerializeToArray(nanoBuf, nanoSize);
	nn_send(m_nanoSock, nanoBuf, nanoSize, 0);

	free(nanoBuf);
	cout << "tm_space use: " << tm_obstacles.SpaceUsed() << endl;
	tm_obstacles.Clear();
}

void nanoObstacleSender::obstacleEnqueue(vector<StruApfObstacle> obsVec)
{
	//usv_mutex_lock(&m_obsMsgMutex);
	m_obsQueue.QueueAddElement(&obsVec);
	//usv_mutex_unlock(&m_obsMsgMutex);
}

void nanoObstacleSender::InitObsQueue(void)
{
	m_obsQueue.QueueSetEmpty();
	usv_mutex_init(&m_obsMsgMutex);
	usv_cond_init(&m_obsMsgCondition);
}

void nanoObstacleSender::obstacleGetHeadOfQueue(void)
{
	vector<StruApfObstacle> tmApfObstacle;

	//usv_mutex_lock(&m_obsMsgMutex);
	if (m_obsQueue.QueueDelElement(&tmApfObstacle) == -1)
	{
		//cout << "Obstacle Queue Empty " << endl;
		//usv_mutex_unlock(&m_obsMsgMutex);
		//return;
	}
	//usv_mutex_unlock(&m_obsMsgMutex);

	

	static double timestamp = 0.0;
	static int seq = 1;
	timestamp += 1.0;
	Header *head = new Header;
	head->set_module_name("arm retransit obstacle");
	head->set_timestamp_sec(timestamp);
	head->set_sequence_num(seq++);
	m_obstacles.set_allocated_header(head);

	PointLLH *pos;
	Shape *shap;
	PerceptionObstacle *singleOb;

	for (int i = 0; i < tmApfObstacle.size(); i++)
	{
		singleOb = m_obstacles.add_perception_obstacle();
		singleOb->set_id(i + 1);
		pos = singleOb->mutable_positon();
		shap = singleOb->mutable_shape();
		pos->set_lat(tmApfObstacle[i].lat);
		pos->set_lon(tmApfObstacle[i].lng);
		shap->set_radius(tmApfObstacle[i].radius);
		shap->set_type(Shape_ShapeType_CIRCLE);
	}

}

void nanoObstacleSender::setNanoSock(char *nanoAddr)
{
	memcpy(m_nanoAddr, nanoAddr, strlen(nanoAddr));
	initNanoSender(m_nanoAddr);
}
