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
/* PSO粒子种群解，将每一条可行路径看做一个初始粒子，将始末点连线方向上的障碍物个数n看做纬度，则一个可行路径由一组n个纬度上的
可行路径节点组成，根据每个可行路径节点上的角度范围ang，可以初始化在该纬度上的移动速度，可设置为: random(-1,1) * ang / num，其中num
为限制移动速度过快或过慢的等分数，设置路径总长度为适应数，则每次迭代时，计算每个粒子即一条路径上，所有n维上根据速度移动的最优解，
更新该粒子的最优解，同时更新种群最优解，迭代t次后，即可获得最优路径
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
	double polarlen;      //极径
	double polarang;      //极角
	double radius;        //障碍物半径
	double extend;        //膨胀距离

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
	double polarang;      //极角

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
	double lat;          //纬度
	double lng;          //经度
	double heading;      //航向

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
	double len;           //总长度
	double sum_ang;       //各转角总和
	double cross_ang;     //船实际航向与航路第一段航向的夹角，防止实时计算时导致第一结果点随机两边跳跃
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
	double len;           //总长度
	double sum_ang;       //各转角总和
	double cross_ang;	  //船实际航向与航线第一段航向夹角
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

//路径规划
class PathCalculatAlgorithm
{
public:
	PathCalculatAlgorithm();
	~PathCalculatAlgorithm();

	//建立GPS场景数据模型
	bool BuildSceneDataSetWithGPS( double s_lat, double s_lng, double e_lat, double e_lng, vector< StruGPSObstacle > objlst,double cr_ang=0.0);

	//建立场景数据模型
	bool BuildSceneDataSet( double sx, double sy, double ex, double ey, vector< StruStaticObstacle > objlst ,double cr_ang=0.0);   //vector to c++

	void ProcessObstacleListValid( vector< StruStaticObstacle > objlst, vector< StruStaticObstacle > &validobjlst ); //滤除范围外的点

	vector< StruPath > GetAllPath();                  //获得全部路径，即初始化时的所有路径

	StruPath GetOptimumPath( int &epv_type );                       //获取最优路径

	StruPath GetOptimumPathSmooth( int &epv_type );                 //获取平滑后的最优路径

	StruGPSPath GetOptimumGPSPath( int &epv_type );                 //获取GPS坐标系最优路径

private:
	bool m_bIniScene;                                //场景是否初始化标志

	double m_dSX;
	double m_dSY;
	double m_dEX;
	double m_dEY;
	time_t t;
	double m_dS_Lat;
	double m_dS_Lng;
	double m_dHeading;
	double m_dCrossAng;

	StruPath m_struOptimumPath;                      //最优路径

	int m_iParticleNum;                              //粒子数
	int m_iIterationNum;                             //迭代次数
	double m_dPSO_r1;                                //pso算法随机数r1
	double m_dPSO_r2;                                //pso算法随机数r2

	bool InitParticleSwarm();                        //初始化粒子群

	void ProcessPSO();                               //PSO算法处理

	bool CreateOneFeasiblePath( StruPath &path );    //生成一条可行路径

	//根据维度cur_dim，上一次点信息，计算当前cur_dim维度的点信息
	bool CalPathPointByDim( int cur_dim, double lastx, double lasty, StruPathPoint &newpnt );

	//根据维度cur_dim，判断当前维度上的点(curx, cury)是否合法，即与上一维度上的点(lastx, lasty)构成的path是否可行
	bool CheckPathPointEnableByDim( int cur_dim, double lastx, double lasty, double curx, double cury );

	//获取随机角度，范围在s_ang, e_ang之间
	double GetRandomAngle( double s_ang, double e_ang );

	//障碍物排序，由近及远
	void ReorderObstacleList();
	//计算点(px, py)到两点(x1, y1)，(x2, y2)所在直线的距离
	//	double CalPointToLineDis( double x1, double y1, double x2, double y2, double px, double py );

	//计算两点(x1, y1)，(x2, y2)到同心圆的交点
	//	void CalCrossPointByCirleAndLine( double x1, double y1, double x2, double y2, int dim, double &x, double &y );

	double CalFitnessVal( StruPath path );           //计算路径的适应度

	void SmoothPath( StruPath &path );               //平滑处理，首末点倒序连接处理
	void SmoothPath2( StruPath &path );              //平滑处理2，间隔两点连接处理

	void ReCalPathInfo( StruPath &path, bool bPunish=false );            //重新计算可行路径的长度

	bool GetGroupOptimumPath( vector< StruPath > pathgrp, StruPath &path );   //获取种群最优解路径

	//判断结束点是否合法，当目标结束点在最后一个障碍物内，最优路径最后一点应将结束点替换为计算的最后一点
	int CheckEndPointValidType();                    //-1 无障碍物，1 结束点合法， 0 结束点不合法                    

	vector< StruStaticObstacle > m_lstObstacle;       //障碍物表

	vector< StruStaticObstacle> Obs_tanslate(vector< StruStaticObstacle > obs_origin);	//障碍物极坐标转换

	vector< StruPath > m_lstPath;                     //仅全维度上的可行路径
	vector< StruPath > m_lstPathWithSE;               //带始末点的完整可行路径

	vector< StruPathSpeed > m_lstPathSpeed;           //路径上的速度分量列表
};

//////////////////////////////////////////////////////////////////////////
/* 通过数学公式计算推导求出逃离障碍物只需满足一个数学不等式，将问题转化成求解速度变化v和航向变化ang的最优解集合，根据适应度
函数，采用PSO算法，初始化解粒子集合，满足不等式的为可行粒子解，通过迭代演变获得最优结果
*/
//////////////////////////////////////////////////////////////////////////
enum AzimuthFlag
{
	FrontCross = 0,     //正面相遇，一律往右避让
	LeftCross,          //左舷交叉，往左避让
	RightCross,         //右舷交叉，往右避让
	L_Overtaking,       //左侧追越，往左避让
	R_Overtaking_TL,    //右侧追越，速度大于目标，往左避让
	R_Overtaking_TR     //右侧追越，速度小于目标，往右避让
};

struct StruBoatInfo
{
	double x;
	double y;
	double speed;
	double azimuth;
	bool myboat;
	int refgrp_id;       //关联的图元id
};

struct StruChangeInfo
{
	double speed;
	double azimuth;
};


//
////动态避障算法
//class ObstacleAvoidAlgorithm
//{
//public:
//	ObstacleAvoidAlgorithm();
//	~ObstacleAvoidAlgorithm();
//
//	//建立场景数据模型
//	void BuildSceneDataSet( double ex, double ey, StruBoatInfo myboat, StruBoatInfo otherboat );
//
//	void SetSaveDis( double dis );                   //设置安全距离
//    double GetSaveDis();                             //获取安全距离
//
//	//获取最优变化信息，返回避障结束点( ex, ey )
//	StruChangeInfo GetOptimumChangeInfo( double &ex, double &ey );
//
//private:
//	int m_iParticleNum;                              //粒子数
//	int m_iIterationNum;                             //迭代次数
//	int m_iCurAvoidRule;                             //当前需遵从的避障规则
//	int m_iCurAvoidRslt;                             //根据当前避让规则，转角标志位 -1 为右转，1 为左转， 0 错误
//
//	double m_dEX;                                    //目标点x坐标
//	double m_dEY;                                    //目标点y坐标
//	double m_dSaveDis;                               //安全距离
//	double m_dPSO_r1;                                //pso算法随机数r1
//	double m_dPSO_r2;                                //pso算法随机数r2
//	double m_dFit_m1;                                //适应度函数权重参数m1
//	double m_dFit_m2;                                //适应度函数权重参数m2
//	double m_dAngle_Q;                               //VR与合速度的夹角
//	double m_dAngle_R;                               //合速度V与移动船只中心的夹角
//	double m_dAngle_U;                               //船与移动船只中心、安全区域连线的夹角
//	double m_dAngle_X;                               //当前船位置与终点坐标连线的角度，与航向的夹角
//	double m_dRslt_V;                                //合速度
//	double m_dCrossTime;                             //相遇时间
//	double m_dAvoidEndPntX;
//	double m_dAvoidEndPntY;
//
//	StruChangeInfo m_struOptimumChange;              //最优变化解
//
//	StruBoatInfo m_stuMyBoat;                        //存储本船信息结构体
//	StruBoatInfo m_stuOtherBoat;                     //存储当前避障关联的其他船只信息结构体
//
//	vector< StruChangeInfo > m_lstChange;             //可行变化粒子列表
//	vector< StruChangeInfo > m_lstSpeed;              //航向，航速变化分量表
//
//	void InitBoatInfo();                             //初始化船只信息
//
//	void InitParticleSwarm();                        //初始化粒子群
//
//	void ProcessPSO();                               //PSO算法处理
//
//	//获取随机变化速度
//	double GetRandomChangeSpeed();
//
//	//获取随机变化角度，此处需包含海事规则约束
//	double GetRandomChangeAzimuth();
//
//	//根据海事规则确认偏航转角方向
//	int CalCurAvoidAzimuthByRule( double myboat_azimuth, double otherboat_azimuth, int &rule );  
//
//	//生成一个可行变化
//	bool CreateOneFeasibleChange( StruChangeInfo &change );   
//
//	//判断变化结果是否可行
//	bool CheckChangeInfoEnable( double speed, double azimuth );
//
//	//计算变化值适应度函数
//	double CalFitnessVal( StruChangeInfo change ); 
//
//	//获取种群最优解路径
//	StruChangeInfo GetGroupOptimumChange( vector< StruChangeInfo > pathgrp ); 
//
//	//计算避障结束点位置
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
//	//获取逻辑船只模型的本船信息
//	void GetCurrentBoatInfo( StruBoatInfo &boatinfo );
//
//	void SetBoatSpeed( double speed );            //设置航速
//	double GetBoatSpeed();                        //获取航速
//
//	void SetBoatAzimuth( double azimuth );        //设置航向
//	double GetBoatAzimuth();                      //获取航向
//
//	//设置逻辑船只的路径点列表
//	void SetBoatPath( vector< QPair> pathpntlist );
//
//	void UpdateBoatInfo();                        //更新船信息，含位置、航速、航向等
//	void UpdateSpeedAndAzimuth();                 //更新航向航速
//
//	//设置当前关联的需避障的对象船只的指针
//	void SetRefLogicOhterBoat( LogicBoatModel * otherboat );
//
//	//如果中间过程判定了需要避障，则获取计算后的避障结束点
//	bool IfNeedAvoidGetEndPnt( double &endx, double &endy );
//
//	//获得当前坐标点和当前子路径段的结束点，用于界面增加和避障点的连线
//	bool GetCurAndSubPathEndPnt( double &cur_x, double &cur_y, double &sub_ex, double &sub_ey );
//
//private:
//	int m_iRefGrpID;                              //关联图元id
//	int m_iCurSubPathID;                          //当前子路径ID
//
//	double m_dX;                                  //当前位置X坐标
//	double m_dY;                                  //当前位置Y坐标
//	double m_dAzimuth;                            //航向
//	double m_dSpeed;                              //航速
//	double m_dCloseDis;                           //船实际位置与规划航点的靠近偏差距离
//	double m_dChg_Speed;                          //期望变化速度值
//	double m_dChg_Azimuth;                        //期望变化角度值
//	double m_dCrossTime;
//	double m_dChg_Speed_Done;                     //已变化速度值
//	double m_dChg_Azimuth_Done;                   //已变化角度值
//	double m_dAvoidEndPntX;                       //避障结束点X坐标
//	double m_dAvoidEndPntY;                       //避障结束点Y坐标
//	double m_dAlarmDis;                           //警戒区域
//
//	bool m_bStop;                                 //是否停止
//	bool m_bShowTrail;                            //是否显示轨迹标志
//	bool m_bIsMyBoat;                             //是否本船标志
//	bool m_bDurAvoid;                             //是否避障中标志
//	bool m_bReSetChange;                          //避障变化归位标志
//	bool m_bAddAvoidPnt;                          //添加避障点标志，用于外部路径信息更新，仅在每次计算出新避障结束点是赋值true一次
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
//	//判断当前子航段结束点( ex, ey )内，是否需要进行避障，是则调用动态避障进行计算
//	bool ChkNeedAvoid( double ex, double ey );    
//};

#endif // ALGORITHM_PATH_H

