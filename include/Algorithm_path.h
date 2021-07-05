#ifndef ALGORITHM_PATH_H
#define ALGORITHM_PATH_H

#include <vector>
#include <string>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "usv_include.h"

using namespace std;

//#include <QtGui>
//#include <QtCore>

//////////////////////////////////////////////////////////////////////////
/* PSO������Ⱥ�⣬��ÿһ������·������һ����ʼ���ӣ���ʼĩ�����߷����ϵ��ϰ������n����γ�ȣ���һ������·����һ��n��γ���ϵ�
����·���ڵ���ɣ�����ÿ������·���ڵ��ϵĽǶȷ�Χang�����Գ�ʼ���ڸ�γ���ϵ��ƶ��ٶȣ�������Ϊ: random(-1,1) * ang / num������num
Ϊ�����ƶ��ٶȹ��������ĵȷ���������·���ܳ���Ϊ��Ӧ������ÿ�ε���ʱ������ÿ�����Ӽ�һ��·���ϣ�����nά�ϸ����ٶ��ƶ������Ž⣬
���¸����ӵ����Ž⣬ͬʱ������Ⱥ���Ž⣬����t�κ󣬼��ɻ������·��
*/
//////////////////////////////////////////////////////////////////////////
struct StruGPSObstacle
{
	double c_lat;
	double c_lng;
	double radius;
	double extend;

	StruGPSObstacle()
	{
		c_lat = 0.0;
		c_lng = 0.0;
		radius = 0.0;
		extend = 0.0;
	}
};

struct StruStaticObstacle
{
	double cx;
	double cy;
	double polarlen;      //����
	double polarang;      //����
	double radius;        //�ϰ���뾶
	double extend;        //���;���

	StruStaticObstacle()
	{
		cx = 0.0;
		cy = 0.0;
		polarlen = 0.0;
		polarang = 0.0;
		radius = 0.0;
		extend = 0.0;
	}
};

struct StruPathPoint
{
	int dim;
	double x;
	double y;
	double polarang;      //����

	StruPathPoint()
	{
		dim = 0;
		x = 0.0;
		y = 0.0;
		polarang = 0.0;
	}
};

struct StruGPSPathPoint
{
	int dim;
	double lat;          //γ��
	double lng;          //����
	double heading;      //����

	StruGPSPathPoint()
	{
		dim = 0;
		lat = 0.0;
		lng = 0.0;
		heading = 0.0;
	}
};

struct StruPath
{
	double len;           //�ܳ���
	double sum_ang;       //��ת���ܺ�
	double cross_ang;     //��ʵ�ʺ����뺽·��һ�κ���ļнǣ���ֹʵʱ����ʱ���µ�һ��������������Ծ
	vector< StruPathPoint > pntlist;

	StruPath()
	{
		len = 0.0;
		sum_ang = 0.0;
		cross_ang = 0.0;
		pntlist.clear();
	}
};

struct StruGPSPath
{
	double len;           //�ܳ���
	double sum_ang;       //��ת���ܺ�
	double cross_ang;	  //��ʵ�ʺ����뺽�ߵ�һ�κ���н�
	vector< StruGPSPathPoint > pntlist;

	StruGPSPath()
	{
		len = 0.0;
		sum_ang = 0.0;
		cross_ang = 0.0;
		pntlist.clear();
	}
};

struct StruSpeed
{
	int dim;
	double speed;

	StruSpeed()
	{
		dim = 0;
		speed = 0.0;
	}
};

struct StruPathSpeed
{
	int pathid;
	vector< StruSpeed > speedlist;

	StruPathSpeed()
	{
		pathid = 0;
		speedlist.clear();
	}
};

//·���滮
class PathCalculatAlgorithm
{
public:
	PathCalculatAlgorithm();
	~PathCalculatAlgorithm();

	//����GPS��������ģ��
	bool BuildSceneDataSetWithGPS( double s_lat, double s_lng, double e_lat, double e_lng, vector< StruGPSObstacle > objlst,double cr_ang=0.0);

	//������������ģ��
	bool BuildSceneDataSet( double sx, double sy, double ex, double ey, vector< StruStaticObstacle > objlst ,double cr_ang=0.0);   //vector to c++

	void ProcessObstacleListValid( vector< StruStaticObstacle > objlst, vector< StruStaticObstacle > &validobjlst ); //�˳���Χ��ĵ�

	vector< StruPath > GetAllPath();                  //���ȫ��·��������ʼ��ʱ������·��

	StruPath GetOptimumPath( int &epv_type );                       //��ȡ����·��

	StruPath GetOptimumPathSmooth( int &epv_type );                 //��ȡƽ���������·��

	StruGPSPath GetOptimumGPSPath( int &epv_type );                 //��ȡGPS����ϵ����·��

private:
	bool m_bIniScene;                                //�����Ƿ��ʼ����־

	double m_dSX;
	double m_dSY;
	double m_dEX;
	double m_dEY;
	time_t t;
	double m_dS_Lat;
	double m_dS_Lng;
	double m_dHeading;
	double m_dCrossAng;

	StruPath m_struOptimumPath;                      //����·��

	int m_iParticleNum;                              //������
	int m_iIterationNum;                             //��������
	double m_dPSO_r1;                                //pso�㷨�����r1
	double m_dPSO_r2;                                //pso�㷨�����r2

	bool InitParticleSwarm();                        //��ʼ������Ⱥ

	void ProcessPSO();                               //PSO�㷨����

	bool CreateOneFeasiblePath( StruPath &path );    //����һ������·��

	//����ά��cur_dim����һ�ε���Ϣ�����㵱ǰcur_dimά�ȵĵ���Ϣ
	bool CalPathPointByDim( int cur_dim, double lastx, double lasty, StruPathPoint &newpnt );

	//����ά��cur_dim���жϵ�ǰά���ϵĵ�(curx, cury)�Ƿ�Ϸ���������һά���ϵĵ�(lastx, lasty)���ɵ�path�Ƿ����
	bool CheckPathPointEnableByDim( int cur_dim, double lastx, double lasty, double curx, double cury );

	//��ȡ����Ƕȣ���Χ��s_ang, e_ang֮��
	double GetRandomAngle( double s_ang, double e_ang );

	//�ϰ��������ɽ���Զ
	void ReorderObstacleList();
	//�����(px, py)������(x1, y1)��(x2, y2)����ֱ�ߵľ���
	//	double CalPointToLineDis( double x1, double y1, double x2, double y2, double px, double py );

	//��������(x1, y1)��(x2, y2)��ͬ��Բ�Ľ���
	//	void CalCrossPointByCirleAndLine( double x1, double y1, double x2, double y2, int dim, double &x, double &y );

	double CalFitnessVal( StruPath path );           //����·������Ӧ��

	void SmoothPath( StruPath &path );               //ƽ��������ĩ�㵹�����Ӵ���
	void SmoothPath2( StruPath &path );              //ƽ������2������������Ӵ���

	void ReCalPathInfo( StruPath &path, bool bPunish=false );            //���¼������·���ĳ���

	bool GetGroupOptimumPath( vector< StruPath > pathgrp, StruPath &path );   //��ȡ��Ⱥ���Ž�·��

	//�жϽ������Ƿ�Ϸ�����Ŀ������������һ���ϰ����ڣ�����·�����һ��Ӧ���������滻Ϊ��������һ��
	int CheckEndPointValidType();                    //-1 ���ϰ��1 ������Ϸ��� 0 �����㲻�Ϸ�                    

	vector< StruStaticObstacle > m_lstObstacle;       //�ϰ����

	vector< StruStaticObstacle> Obs_tanslate(vector< StruStaticObstacle > obs_origin);	//�ϰ��Ｋ����ת��

	vector< StruPath > m_lstPath;                     //��ȫά���ϵĿ���·��
	vector< StruPath > m_lstPathWithSE;               //��ʼĩ�����������·��

	vector< StruPathSpeed > m_lstPathSpeed;           //·���ϵ��ٶȷ����б�
};

//////////////////////////////////////////////////////////////////////////
/* ͨ����ѧ��ʽ�����Ƶ���������ϰ���ֻ������һ����ѧ����ʽ��������ת��������ٶȱ仯v�ͺ���仯ang�����Ž⼯�ϣ�������Ӧ��
����������PSO�㷨����ʼ�������Ӽ��ϣ����㲻��ʽ��Ϊ�������ӽ⣬ͨ�������ݱ������Ž��
*/
//////////////////////////////////////////////////////////////////////////
enum AzimuthFlag
{
	FrontCross = 0,     //����������һ�����ұ���
	LeftCross,          //���Ͻ��棬�������
	RightCross,         //���Ͻ��棬���ұ���
	L_Overtaking,       //���׷Խ���������
	R_Overtaking_TL,    //�Ҳ�׷Խ���ٶȴ���Ŀ�꣬�������
	R_Overtaking_TR     //�Ҳ�׷Խ���ٶ�С��Ŀ�꣬���ұ���
};

struct StruBoatInfo
{
	double x;
	double y;
	double speed;
	double azimuth;
	bool myboat;
	int refgrp_id;       //������ͼԪid
};

struct StruChangeInfo
{
	double speed;
	double azimuth;
};


//
////��̬�����㷨
//class ObstacleAvoidAlgorithm
//{
//public:
//	ObstacleAvoidAlgorithm();
//	~ObstacleAvoidAlgorithm();
//
//	//������������ģ��
//	void BuildSceneDataSet( double ex, double ey, StruBoatInfo myboat, StruBoatInfo otherboat );
//
//	void SetSaveDis( double dis );                   //���ð�ȫ����
//    double GetSaveDis();                             //��ȡ��ȫ����
//
//	//��ȡ���ű仯��Ϣ�����ر��Ͻ�����( ex, ey )
//	StruChangeInfo GetOptimumChangeInfo( double &ex, double &ey );
//
//private:
//	int m_iParticleNum;                              //������
//	int m_iIterationNum;                             //��������
//	int m_iCurAvoidRule;                             //��ǰ����ӵı��Ϲ���
//	int m_iCurAvoidRslt;                             //���ݵ�ǰ���ù���ת�Ǳ�־λ -1 Ϊ��ת��1 Ϊ��ת�� 0 ����
//
//	double m_dEX;                                    //Ŀ���x����
//	double m_dEY;                                    //Ŀ���y����
//	double m_dSaveDis;                               //��ȫ����
//	double m_dPSO_r1;                                //pso�㷨�����r1
//	double m_dPSO_r2;                                //pso�㷨�����r2
//	double m_dFit_m1;                                //��Ӧ�Ⱥ���Ȩ�ز���m1
//	double m_dFit_m2;                                //��Ӧ�Ⱥ���Ȩ�ز���m2
//	double m_dAngle_Q;                               //VR����ٶȵļн�
//	double m_dAngle_R;                               //���ٶ�V���ƶ���ֻ���ĵļн�
//	double m_dAngle_U;                               //�����ƶ���ֻ���ġ���ȫ�������ߵļн�
//	double m_dAngle_X;                               //��ǰ��λ�����յ��������ߵĽǶȣ��뺽��ļн�
//	double m_dRslt_V;                                //���ٶ�
//	double m_dCrossTime;                             //����ʱ��
//	double m_dAvoidEndPntX;
//	double m_dAvoidEndPntY;
//
//	StruChangeInfo m_struOptimumChange;              //���ű仯��
//
//	StruBoatInfo m_stuMyBoat;                        //�洢������Ϣ�ṹ��
//	StruBoatInfo m_stuOtherBoat;                     //�洢��ǰ���Ϲ�����������ֻ��Ϣ�ṹ��
//
//	vector< StruChangeInfo > m_lstChange;             //���б仯�����б�
//	vector< StruChangeInfo > m_lstSpeed;              //���򣬺��ٱ仯������
//
//	void InitBoatInfo();                             //��ʼ����ֻ��Ϣ
//
//	void InitParticleSwarm();                        //��ʼ������Ⱥ
//
//	void ProcessPSO();                               //PSO�㷨����
//
//	//��ȡ����仯�ٶ�
//	double GetRandomChangeSpeed();
//
//	//��ȡ����仯�Ƕȣ��˴���������¹���Լ��
//	double GetRandomChangeAzimuth();
//
//	//���ݺ��¹���ȷ��ƫ��ת�Ƿ���
//	int CalCurAvoidAzimuthByRule( double myboat_azimuth, double otherboat_azimuth, int &rule );  
//
//	//����һ�����б仯
//	bool CreateOneFeasibleChange( StruChangeInfo &change );   
//
//	//�жϱ仯����Ƿ����
//	bool CheckChangeInfoEnable( double speed, double azimuth );
//
//	//����仯ֵ��Ӧ�Ⱥ���
//	double CalFitnessVal( StruChangeInfo change ); 
//
//	//��ȡ��Ⱥ���Ž�·��
//	StruChangeInfo GetGroupOptimumChange( vector< StruChangeInfo > pathgrp ); 
//
//	//������Ͻ�����λ��
//	void CalAvoidEndPnt();
//};
//
//
////////////////////////////////////////////////////////////////////////////
//struct PathPointInfo
//{
//	int index;
//	double x;
//	double y;
//	bool arrived;
//};
//
//class LogicBoatModel
//{
//public:
//	LogicBoatModel();
//	LogicBoatModel( StruBoatInfo boatinfo );
//	~LogicBoatModel();
//
//	//��ȡ�߼���ֻģ�͵ı�����Ϣ
//	void GetCurrentBoatInfo( StruBoatInfo &boatinfo );
//
//	void SetBoatSpeed( double speed );            //���ú���
//	double GetBoatSpeed();                        //��ȡ����
//
//	void SetBoatAzimuth( double azimuth );        //���ú���
//	double GetBoatAzimuth();                      //��ȡ����
//
//	//�����߼���ֻ��·�����б�
//	void SetBoatPath( vector< QPair> pathpntlist );
//
//	void UpdateBoatInfo();                        //���´���Ϣ����λ�á����١������
//	void UpdateSpeedAndAzimuth();                 //���º�����
//
//	//���õ�ǰ����������ϵĶ���ֻ��ָ��
//	void SetRefLogicOhterBoat( LogicBoatModel * otherboat );
//
//	//����м�����ж�����Ҫ���ϣ����ȡ�����ı��Ͻ�����
//	bool IfNeedAvoidGetEndPnt( double &endx, double &endy );
//
//	//��õ�ǰ�����͵�ǰ��·���εĽ����㣬���ڽ������Ӻͱ��ϵ������
//	bool GetCurAndSubPathEndPnt( double &cur_x, double &cur_y, double &sub_ex, double &sub_ey );
//
//private:
//	int m_iRefGrpID;                              //����ͼԪid
//	int m_iCurSubPathID;                          //��ǰ��·��ID
//
//	double m_dX;                                  //��ǰλ��X����
//	double m_dY;                                  //��ǰλ��Y����
//	double m_dAzimuth;                            //����
//	double m_dSpeed;                              //����
//	double m_dCloseDis;                           //��ʵ��λ����滮����Ŀ���ƫ�����
//	double m_dChg_Speed;                          //�����仯�ٶ�ֵ
//	double m_dChg_Azimuth;                        //�����仯�Ƕ�ֵ
//	double m_dCrossTime;
//	double m_dChg_Speed_Done;                     //�ѱ仯�ٶ�ֵ
//	double m_dChg_Azimuth_Done;                   //�ѱ仯�Ƕ�ֵ
//	double m_dAvoidEndPntX;                       //���Ͻ�����X����
//	double m_dAvoidEndPntY;                       //���Ͻ�����Y����
//	double m_dAlarmDis;                           //��������
//
//	bool m_bStop;                                 //�Ƿ�ֹͣ
//	bool m_bShowTrail;                            //�Ƿ���ʾ�켣��־
//	bool m_bIsMyBoat;                             //�Ƿ񱾴���־
//	bool m_bDurAvoid;                             //�Ƿ�����б�־
//	bool m_bReSetChange;                          //���ϱ仯��λ��־
//	bool m_bAddAvoidPnt;                          //��ӱ��ϵ��־�������ⲿ·����Ϣ���£�����ÿ�μ�����±��Ͻ������Ǹ�ֵtrueһ��
//
//	LogicBoatModel * m_pRefOtherBoat;
//
//	vector< PathPointInfo > m_lstPathPnt;
//
//
//	struct QPair{
//		double first;
//		double second;
//	};
//
//	//�жϵ�ǰ�Ӻ��ν�����( ex, ey )�ڣ��Ƿ���Ҫ���б��ϣ�������ö�̬���Ͻ��м���
//	bool ChkNeedAvoid( double ex, double ey );    
//};

#endif // ALGORITHM_PATH_H

