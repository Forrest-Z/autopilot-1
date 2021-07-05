

//#include "GraphicSceneObj.h"
#include "stdafx.h"
#include "math.h"
#include "../include/Algorithm_path.h"

#define M_PI	3.1415926
using namespace std;

const long gc_Earth_Radius = 6378137; //����뾶

const double gc_PSO_w = 1.2;          //PSO�㷨����Ȩֵ
const double gc_PSO_c1 = 2.0;         //PSO�㷨����Ȩֵ
const double gc_PSO_c2 = 2.0;         //PSO�㷨����Ȩֵ
const double gc_PSO_num = 100;        //PSO�㷨���ʱ仯�ȷ���

const double gc_AngleRange = 80.0;    //ǰ������·��ת�Ƿ�Χ������80��
const double gc_SpeedRange = 40.0;    //���������ٶ�������40����/Сʱ
const int gc_MaxRandomNum = 500;

//�����(px, py)������(x1, y1)��(x2, y2)����ֱ�ߵľ���
double CalPointToLineDis( double x1, double y1, double x2, double y2, double px, double py )
{
	double para_A, para_B, para_C, dis;
	para_A = y2 - y1;
	para_B = x1 - x2;
	para_C = x2*y1 - x1*y2;

	dis = fabs( para_A*px + para_B*py + para_C ) / sqrt( pow( para_A, 2 ) + pow( para_B, 2 ) );
	return dis;
}

//��������(x1, y1)��(x2, y2)��ͬ��Բ�Ľ���
void CalCrossPointByCirleAndLine( double x1, double y1, double x2, double y2, double cx, double cy, double cr, double &x, double &y )
{
	//Բ���̲�����������scene��ƽ����ת���������ʼ�ս�ͬ��Բ��Ϊ[0, 0]��
	double p_A, p_B, p_R, ang1, ang2;
	p_A = cx;       //Բ��xƫ��
	p_B = cy;       //Բ��yƫ��
	p_R = cr;       //Բ�뾶
	//���㽻��
	if ( fabs( y2 - y1 ) > 1e-6 && fabs( x2 - x1 ) < 1e-6 )       //x1����x2��y1������y2
	{
		//(x1,b+[R^2 - (x1-a)^2]^(1/2))��(x1,b-[R^2 - (x1-a)^2]^(1/2))
		x = x1;
		y = p_B + sqrt( pow( p_R, 2 ) - pow( (x1-p_A), 2 ) );
		//��֤�������Ƿ���ϣ�����ȡ����
		ang1 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
		ang2 = atan2( y - y1, x - x1 ) * 180.0 / M_PI;
		if ( fabs( ang1 - ang2 ) > 90 )
		{
			y = p_B - sqrt( pow( p_R, 2 ) - pow( (x1-p_A), 2 ) );
		}
	}
	if ( fabs( y2 - y1 ) < 1e-6 && fabs( x2 - x1 ) > 1e-6 )       //x1������x2��y1����y2
	{
		//(a+[R^2 - (y1-b)^2]^(1/2),y1)��(a-[R^2 - (y1-b)^2]^(1/2),y1)
		x = p_A + sqrt( pow( p_R, 2 ) - pow( (y1-p_B), 2 ) );
		y = y1;
		//��֤�������Ƿ���ϣ�����ȡ����
		ang1 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
		ang2 = atan2( y - y1, x - x1 ) * 180.0 / M_PI;
		if ( fabs( ang1 - ang2 ) > 90 )
		{
			x = p_A - sqrt( pow( p_R, 2 ) - pow( (y1-p_B), 2 ) );
		}
	}
	else if ( fabs( y2 - y1 ) > 1e-6 && fabs( x2 - x1 ) > 1e-6 )  //x1������x2��y1������y2
	{
		double para_U, para_V, para_T1, para_T2;
		//�Ƶ������ʽ
		//u = [(y1-b) + (x1-a)(x2-x1)/(y2-y1)]/{1 + [(x2-x1)/(y2-y1)]^2}
		//v = [(x1-a)^2 + (y1-b)^2 - R^2]/{1 + [(x2-x1)/(y2-y1)]^2}
		//t = (u^2 - v)^(1/2) - u �� -(u^2 - v)^(1/2) - u
		para_U = ( (y1-p_B) + (x1-p_A)*(x2-x1)/(y2-y1) ) / ( 1 + pow( (x2-x1)/(y2-y1), 2 ) );
		para_V = ( pow( (x1-p_A), 2 ) + pow( (y1-p_B), 2 ) - pow( p_R, 2 ) ) / ( 1 + pow( (x2-x1)/(y2-y1), 2 ) );
		para_T1 = sqrt( pow( para_U, 2 ) - para_V ) - para_U;     //��
		para_T2 = - sqrt( pow( para_U, 2 ) - para_V ) - para_U;   //��
		//���
		//(x1 + t(x2-x1)/(y2-y1),y1+t)
		x = x1 + para_T1*(x2-x1)/(y2-y1);
		y = y1 + para_T1;
		//��֤�������Ƿ���ϣ�����ȡ����
		ang1 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
		ang2 = atan2( y - y1, x - x1 ) * 180.0 / M_PI;
		if ( fabs( ang1 - ang2 ) > 90 )
		{
			x = x1 + para_T2*(x2-x1)/(y2-y1);
			y = y1 + para_T2;
		}
	}
}

//����ת��
double gc_Radian(double d)
{
	return d * M_PI / 180.0; 
}

//�������
double gc_Get_Distance(double lat1,double lng1,double lat2,double lng2)
{
	double radLat1=gc_Radian(lat1);
	double radLat2=gc_Radian(lat2);
	double a=radLat1-radLat2;
	double b=gc_Radian(lng1)-gc_Radian(lng2);
	double dst=2*asin((sqrt(pow(sin(a/2),2)+cos(radLat1)*cos(radLat2)*pow(sin(b/2),2))));
	dst=dst*gc_Earth_Radius;
	return dst;
}

//���㷽λ��
double gc_Get_Heading(double fStarPtx,double fStarPty,double fEndPtx,double fEndPty)
{
	double e=0.081813369f; 
	double delta_fy=fEndPtx-fStarPtx;
	double delta_lnmg=fEndPty-fStarPty;
	if(delta_lnmg < -180.0)
		delta_lnmg += 360.0;
	if(delta_lnmg > 180.0)
		delta_lnmg -= 360.0;
	unsigned char bGoEast=FALSE,bGoNorth=FALSE;
	if(delta_lnmg >= 0.0)
		bGoEast=TRUE;
	else
		bGoEast=FALSE;
	if(delta_fy>=0.0)
		bGoNorth=TRUE;
	else
		bGoNorth=FALSE;

	if(delta_fy==0)
	{
		if(delta_lnmg==0)return 0;
		return bGoEast?90:270;
	}
	double d1=7915.7045*(e/2*log10((1-e*sin(fStarPtx*M_PI/180))/(1+e*sin(fStarPtx*M_PI/180)))+log10(tan((45+fStarPtx/2)*M_PI/180.0)));
	double d2=7915.7045*(e/2*log10((1-e*sin(fEndPtx*M_PI/180))/(1+e*sin(fEndPtx*M_PI/180)))+log10(tan((45+fEndPtx/2)*M_PI/180.0)));
	double delta_d=d2-d1;

	double dbDir=atan(delta_lnmg*60/delta_d)*180/M_PI;
	if(!bGoEast&&bGoNorth)
		dbDir=360+dbDir;
	if(!bGoEast&&!bGoNorth)
		dbDir=180+dbDir;
	if(bGoEast&&!bGoNorth)
		dbDir=180+dbDir;
	return dbDir;
}

//���ݾ�γ��(src_lat, src_lng)�Լ�����ڸ�λ�õľ���dis������heading���������λ�õľ�γ��(rslt_lat, rslt_lng)
void gc_Get_LAT_LNG( double src_lat, double src_lng, double dis, double heading, double &rslt_lat, double &rslt_lng )
{
	double c = dis / gc_Earth_Radius;
	double a = acos(cos(M_PI*(90.0-src_lat)/180.0)*cos(c)+sin(M_PI*(90-src_lat)/180.0)*sin(c)*cos(M_PI*(heading)/180.0));
	double C = asin(sin(c)*sin(M_PI*(heading)/180.0)/sin(a));

	rslt_lat = 90 - a * 180.0 / M_PI;      //γ��
	rslt_lng = src_lng + C * 180.0 / M_PI; //����  

	return ;
}

//////////////////////////////////////////////////////////////////////////
PathCalculatAlgorithm::PathCalculatAlgorithm()
{
	m_bIniScene = false;

	m_dSX = 0.0;
	m_dSY = 0.0;
	m_dEX = 0.0;
	m_dEY = 0.0;
	m_dS_Lat = 0.0;
	m_dS_Lng = 0.0;
	m_dHeading = 0.0;

	m_iParticleNum = 80;
	m_iIterationNum = 50;

	m_lstObstacle.clear();
	m_lstPath.clear();
	m_lstPathWithSE.clear();

	//qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));  //change to c++
	srand((unsigned) time(&t));
}

PathCalculatAlgorithm::~PathCalculatAlgorithm()
{

}

void PathCalculatAlgorithm::ReorderObstacleList()
{
	if ( m_lstObstacle.size() == 0 )
		return;

	StruStaticObstacle tmpobstacle;
	int i, j, count;
	count = m_lstObstacle.size();
	for ( j=0; j<count-1; j++ )
	{
		for ( i=0; i<count-j-1; i++ )
		{
			if ( m_lstObstacle[i].polarlen > m_lstObstacle[i+1].polarlen )
			{
				tmpobstacle = m_lstObstacle[i];
				m_lstObstacle[i] = m_lstObstacle[i+1];
				m_lstObstacle[i+1] = tmpobstacle;
			}
		}
	}
}
bool PathCalculatAlgorithm::BuildSceneDataSetWithGPS( double s_lat, double s_lng, double e_lat, double e_lng, vector< StruGPSObstacle > objlst,double cr_ang)
{
	m_bIniScene = true;

	m_dS_Lat = s_lat;
	m_dS_Lng = s_lng;

	//������ʼ��ľ��룬����
	double len, heading, sx, sy, ex, ey, tmplen, tmpheading;
	len = gc_Get_Distance( s_lat, s_lng, e_lat, e_lng );
	heading = gc_Get_Heading( s_lat, s_lng, e_lat, e_lng );

	if ( heading > 180.0 )
		heading -= 360.0;
	else if ( heading < -180.0 )
		heading += 360.0;

	sx = 0.0;
	sy = 0.0;
	ex = sx + len;
	ey = sy;                    //ey��ת��sy����ת�Ƕȼ�-heading

	m_dHeading = heading;

	//��GPS�ϰ������ת��
	vector< StruStaticObstacle > staticobjlst;
	staticobjlst.clear();
	for ( int i=0; i<objlst.size(); i++ )
	{
		tmplen = gc_Get_Distance( s_lat, s_lng, objlst[i].c_lat, objlst[i].c_lng );
		tmpheading = gc_Get_Heading( s_lat, s_lng, objlst[i].c_lat, objlst[i].c_lng );

		if ( tmpheading > 180.0 )
			tmpheading -= 360.0;
		else if ( tmpheading < -180.0 )
			tmpheading += 360.0;

		tmpheading -= heading;  //��ת

		StruStaticObstacle tmpobj;
		tmpobj.cx = sx + tmplen * cos( tmpheading / 180.0 * M_PI ); 
		tmpobj.cy = sy + tmplen * sin( tmpheading / 180.0 * M_PI );
		tmpobj.polarlen = tmplen;
		tmpobj.polarang = tmpheading;
		tmpobj.radius = objlst[i].radius;
		tmpobj.extend = objlst[i].extend;

		staticobjlst.push_back( tmpobj );
	}

	//���㵱ǰ������ת��ĽǶȣ�����׼������Χ�޸�Ϊ-180��180��
	double ang;
	ang = cr_ang - m_dHeading;
	if ( ang > 180.0 )
        ang -= 360.0;
	else if ( ang < -180.0 )
        ang += 360.0;

	//SysPubMsgPost("ƫת��:%f �����:%f",m_dHeading,cr_ang);

	return BuildSceneDataSet( sx, sy, ex, ey, staticobjlst, ang );
}

bool PathCalculatAlgorithm::BuildSceneDataSet( double sx, double sy, double ex, double ey, vector< StruStaticObstacle > objlst, double cr_ang )
{
	m_bIniScene = true;

	m_dSX = sx;
	m_dSY = sy;
	m_dEX = ex;
	m_dEY = ey;
	m_dCrossAng = cr_ang;
    
	//�ϰ����б��Ϸ���
	ProcessObstacleListValid( objlst, m_lstObstacle );
	ReorderObstacleList();

	if (m_lstObstacle.size() > 0)
	{
		//��ʼ����Ⱥ����
		if (!InitParticleSwarm())
			return false;

		//PSO����
		ProcessPSO();
	}
	else //���ϰ�������ʼ��ֱ��Ϊ����·��
	{
		m_struOptimumPath.len = sqrt(pow(m_dEY - m_dSY, 2) + pow(m_dEX - m_dSX, 2));
		m_struOptimumPath.cross_ang = m_dCrossAng;
		m_struOptimumPath.sum_ang = 0;
	}


	return true;
}

vector< StruPath > PathCalculatAlgorithm::GetAllPath()
{
	return m_lstPathWithSE;
}

StruPath PathCalculatAlgorithm::GetOptimumPath( int &epv_type )
{
	StruPath path = m_struOptimumPath;

	epv_type = CheckEndPointValidType();

	StruPathPoint startpnt, endpnt;
	startpnt.dim = -1;
	startpnt.x = m_dSX;
	startpnt.y = m_dSY;
	startpnt.polarang = 0;
	endpnt.dim = -1;
	endpnt.x = m_dEX;
	endpnt.y = m_dEY;
	endpnt.polarang = 0;
	//������㣬�յ�
	path.pntlist.insert( path.pntlist.begin(), startpnt );
	path.pntlist.push_back( endpnt );

	return path;
}

StruPath PathCalculatAlgorithm::GetOptimumPathSmooth(int &epv_type)
{
	StruPath smoothpath = m_struOptimumPath;
	SmoothPath2( smoothpath );

	epv_type = CheckEndPointValidType();

	//��ʼ������������õ���path������ĩ�㡣(����δ��ʼ������ʱ����������ĩ�㣬path�ϵĵ��б�Ϊ��)
	if ( m_bIniScene )
	{
		StruPathPoint startpnt, endpnt;
		startpnt.dim = -1;
		startpnt.x = m_dSX;
		startpnt.y = m_dSY;
		startpnt.polarang = 0;
		endpnt.dim = -1;
		endpnt.x = m_dEX;
		endpnt.y = m_dEY;
		endpnt.polarang = 0;
		//������㣬�յ�
		smoothpath.pntlist.insert( smoothpath.pntlist.begin(), startpnt );
		smoothpath.pntlist.push_back( endpnt );		
	}

	return smoothpath;
}

StruGPSPath PathCalculatAlgorithm::GetOptimumGPSPath( int &epv_type )
{
	StruPath path = m_struOptimumPath;

	epv_type = CheckEndPointValidType();

	StruPathPoint startpnt, endpnt;
	startpnt.dim = -1;
	startpnt.x = m_dSX;
	startpnt.y = m_dSY;
	startpnt.polarang = 0;
	endpnt.dim = -1;
	endpnt.x = m_dEX;
	endpnt.y = m_dEY;
	endpnt.polarang = 0;
	//������㣬�յ�
	path.pntlist.insert( path.pntlist.begin(), startpnt );
	path.pntlist.push_back( endpnt );

	//ת��
	StruGPSPath gps_path;
	gps_path.len = path.len;
	gps_path.sum_ang = path.sum_ang;
	gps_path.cross_ang = path.cross_ang;
	double dis;
	for ( int i=0; i<path.pntlist.size(); i++ )
	{
		StruGPSPathPoint gps_pnt;
		gps_pnt.dim = path.pntlist[i].dim;
		gps_pnt.heading = path.pntlist[i].polarang + m_dHeading;
		dis = sqrt( pow( path.pntlist[i].y-m_dSY, 2 ) + pow( path.pntlist[i].x-m_dSX, 2  ) );
		gc_Get_LAT_LNG( m_dS_Lat, m_dS_Lng, dis , gps_pnt.heading, gps_pnt.lat, gps_pnt.lng );

		gps_path.pntlist.push_back( gps_pnt );
	}

	return gps_path;
}

void PathCalculatAlgorithm::ProcessObstacleListValid( vector< StruStaticObstacle > objlst, vector< StruStaticObstacle > &validobjlst )
{
	validobjlst.clear();

	double dis;
	dis = sqrt( pow( m_dEY - m_dSY, 2 ) + pow( m_dEX - m_dSX, 2 ) );
	//�����ϰ����б�
	for ( int i=0; i<objlst.size(); i++ )
	{
		//�ϰ���Բ�ĵ����ľ���С����ʼ����룬��λ��ǰ��������Χ�ڵ��ϰ���Ϸ�
		if ( objlst[i].polarlen < dis && objlst[i].polarang <= gc_AngleRange && objlst[i].polarang >= -gc_AngleRange )
		{
			validobjlst.push_back( objlst[i] );
		}
	}
}

bool PathCalculatAlgorithm::InitParticleSwarm()
{
	m_lstPath.clear();
	m_lstPathWithSE.clear();

	int i;
	for ( i=0; i<m_iParticleNum; i++  )
	{
		StruPath newpath, newpath_se;
		//����һ��·���ϵĿ���·���㣬��ά�ȡ�������������
		if ( CreateOneFeasiblePath( newpath ) )
		{
			m_lstPath.push_back( newpath );

			//���Ƴ�ʼ·����������ʼĩ��
			newpath_se = newpath;
			StruPathPoint startpnt, endpnt;
			startpnt.dim = -1;
			startpnt.x = m_dSX;
			startpnt.y = m_dSY;
			endpnt.dim = -1;
			endpnt.x = m_dEX;
			endpnt.y = m_dEY;
			//������㣬�յ�
			newpath_se.pntlist.insert( newpath_se.pntlist.begin(), startpnt );
			newpath_se.pntlist.push_back( endpnt );

			m_lstPathWithSE.push_back( newpath_se );
		}
	}

	bool rslt = m_lstPathWithSE.size()>0? true: false;
	return rslt;


}

void PathCalculatAlgorithm::ProcessPSO()
{
	//��ʼ��r1, r2��Χ 0 - 1 ֮��
	m_dPSO_r1 = double( rand() % 1000 ) / 1000;    //change to c++
	m_dPSO_r2 = double( rand() % 1000 ) / 1000;

	StruPath grp_optimum_path, tmpgrp_optimum_path;
	vector< StruPath > cur_optimum_path_lst;

	int i, j, k, dim;
	double randompara, tmpspeed, lastx, lasty, x, y;
	double cur_optimum_ang, grp_optimum_ang, tmp_ang;
	dim = m_lstObstacle.size();

	//��ʼ����Ϣ
	m_lstPathSpeed.clear();
	//���һ������·������Ϊ��ʼ��Ⱥ���Ž�
	if ( !GetGroupOptimumPath( m_lstPath, grp_optimum_path ) )
		return;

	//��ʼ��ÿһ��·����ÿһά�����ϵ��ٶ�v
	for ( j=0; j<m_lstPath.size(); j++ )
	{
		//��ʼ��ÿһ��·���ĵ�ǰ���Ž�
		StruPath tmp_path;
		tmp_path = m_lstPath[j];
		cur_optimum_path_lst.push_back( tmp_path );

		StruPathSpeed pathspeed;
		pathspeed.pathid = j;
		for ( k=0; k<dim; k++ )
		{
			StruSpeed dimspeed;
			dimspeed.dim = k;
			//�ٶȳ�ʼ����[-1,1]*speed֮�ڣ�speedΪ��λʱ����ת�ĽǶȣ�ʹ�ÿ�������Եȷ���
			randompara = double( rand() % 2000 ) / 1000 - 1;
			dimspeed.speed = randompara * gc_AngleRange / gc_PSO_num;

			pathspeed.speedlist.push_back( dimspeed );
		}
		m_lstPathSpeed.push_back( pathspeed );
	}

	//���������Ž⣬��С�ڵ�����������δ����ʱ��ѭ��
	bool bPunish;  //�ͷ���־
	i = 0;
	while ( i<m_iIterationNum )
	{
		//����Ⱥ����һ�ε���
		for ( j=0; j<m_lstPath.size(); j++ )
		{
			lastx = m_dSX;
			lasty = m_dSY;
			bPunish = false;  
			//����ÿ��·����ÿ��ά�ȵĵ���Ϣ��λ����Ϣ�����ȫ��ת���ɽǶ�����
			for ( k=0; k<dim; k++ )
			{
				tmp_ang = m_lstPath[j].pntlist[k].polarang;
				cur_optimum_ang = cur_optimum_path_lst[j].pntlist[k].polarang;	
				grp_optimum_ang = grp_optimum_path.pntlist[k].polarang;

				//����speed v(t+1) = gc_PSO_w*v(t) + gc_PSO_c1*m_dPSO_r1*( cur_optimum_path - x_id ) + gc_PSO_c2*m_dPSO_r2( grp_optimum_path - xid );
				tmpspeed = m_lstPathSpeed[j].speedlist[k].speed;
				tmpspeed = gc_PSO_w*tmpspeed + gc_PSO_c1*m_dPSO_r1*( cur_optimum_ang - tmp_ang ) + gc_PSO_c2*m_dPSO_r2*( grp_optimum_ang - tmp_ang );

				//������λ�ã�ת����x,y��Ϣ��xid(t+1) = xid(t) + vid(t+1)
				tmp_ang = tmp_ang + tmpspeed;
				x = m_lstObstacle[k].polarlen * cos( tmp_ang / 180.0 * M_PI );
				y = m_lstObstacle[k].polarlen * sin( tmp_ang / 180.0 * M_PI );

				//�жϵ�kά�ȵ���λ���Ƿ�Ϊ���н�
				if ( CheckPathPointEnableByDim( k, lastx, lasty, x, y ) )
				{
					//�ǿ��н⣬�����λ����Ϣ
					m_lstPath[j].pntlist[k].polarang = tmp_ang;
					m_lstPath[j].pntlist[k].x = x;
					m_lstPath[j].pntlist[k].y = y;
				}
				else //������ʱ��������һ��λ�����¼���һ���µ����
				{
					StruPathPoint newpnt;
					if ( CalPathPointByDim( k, lastx, lasty, newpnt ) )
					{
						m_lstPath[j].pntlist[k].polarang = newpnt.polarang;
						m_lstPath[j].pntlist[k].x = newpnt.x;
						m_lstPath[j].pntlist[k].y = newpnt.y;
					}
					else  //�޿��н�ʱ�����ӳͷ�
					{
						bPunish = true;
					}
				}
				//����������λ�õ���Ϣ
				lastx = m_lstPath[j].pntlist[k].x;
				lasty = m_lstPath[j].pntlist[k].y;
				//�����ٶ�
				m_lstPathSpeed[j].speedlist[k].speed = tmpspeed;
			}
			//���¼���仯���·����Ϣ
			ReCalPathInfo( m_lstPath[j], bPunish );
			//������Ӧ��������path���ȣ����µ�ǰ���Ž�
			if ( CalFitnessVal( m_lstPath[j] ) < CalFitnessVal( cur_optimum_path_lst[j] ) )
			{
				cur_optimum_path_lst[j] = m_lstPath[j];
			}
		}
		//�ӵ�ǰ���Ž��л�ȡ��Ⱥ���ŽⲢ����
		GetGroupOptimumPath( cur_optimum_path_lst ,tmpgrp_optimum_path);
		if ( CalFitnessVal( tmpgrp_optimum_path ) < CalFitnessVal( grp_optimum_path ) )
		{
			grp_optimum_path = tmpgrp_optimum_path;
		}
		i++;
	}
	//��ȡ����������Ⱥ���Ž�
	m_struOptimumPath = grp_optimum_path;
}

void PathCalculatAlgorithm::SmoothPath( StruPath &path )
{
	//path��ά�����ϰ������һ�£���path�ĵ�������ϰ���һ��(������·��������ʼĩ��)��һ���ϰ����ǲ���ƽ����path��Ϊ�գ�
	if ( path.pntlist.size() != m_lstObstacle.size() || m_lstObstacle.size() == 1 || path.pntlist.size() == 0 ) 
        return;

	int i, j, dim, index;
	double dis, ang;
	bool bcross;
	//����ʼ�㿪ʼ��õ�����·���㵹�����������ö����߲����ϰ����ཻ����ѡȡ�������Ϊ��·����
	dim = m_lstObstacle.size(); 
	index = -1;
	for ( i=dim-1; i>=1; i-- )  //ֻ��ȡ������ʼ����ĵ����ڶ���
	{
		bcross = false;
		//�жϵ�ǰά�ȵ�path�����������ߣ��Ƿ��������ϰ��ﶼ���ཻ
		for ( j=0; j<dim; j++ )
		{
			//�����ϰ���j��Բ�ĵ�path��[i]�������ֱ�ߵľ���
			dis = CalPointToLineDis( m_dSX, m_dSY, path.pntlist[i].x, path.pntlist[i].y, m_lstObstacle[j].cx, m_lstObstacle[j].cy );
			//��Ϊpath�ĵ㶼�ǿ����򣬹�path�ϵĵ�ض������ϰ���Բ�ڣ�ֻ���жϸ��ϰ��ﵽֱ�߾����Ƿ�С���ϰ��������İ뾶�����ɶ϶��Ƿ��ཻ
			if ( dis < m_lstObstacle[j].radius )  
			{
				bcross = true;
				break;
			}
		}
		//������ཻ�����ƽ��
		if ( !bcross )
		{
			index = i;
			break;
		}
	}
	if ( index != -1 )
	{
		//�ɽ�path�ϵ����index��֮ǰ�ĵ㣬ȫ�����µ�path��index������������ϣ�ά��ͬ��Բ�����ߵĽ��㼴Ϊ���µ�
		ang = atan2( path.pntlist[index].y - m_dSY, path.pntlist[index].x - m_dSX );  //����
		for ( i=0; i<index; i++ )
		{
			path.pntlist[i].polarang = ang * 180 / M_PI;
			path.pntlist[i].x = m_lstObstacle[i].polarlen * cos( ang ); //����
			path.pntlist[i].y = m_lstObstacle[i].polarlen * sin( ang ); //����
		}
	}
}

void PathCalculatAlgorithm::SmoothPath2( StruPath &path )
{
	//path��ά�����ϰ������һ�£���path�ĵ�������ϰ���һ��(������·��������ʼĩ��)��һ���ϰ����ǲ���ƽ����path��Ϊ�գ�
	if ( path.pntlist.size() != m_lstObstacle.size() || m_lstObstacle.size() == 1 || path.pntlist.size() == 0 )
		return;

	int i, j, dim;
	double dis, ang, x1, y1, x2, y2, x, y;
	bool bcross;
	//����㿪ʼ���������pathά���ϵĵ㣬�������Ϊ[1,3],[2,4],[3,5]...
	dim = m_lstObstacle.size(); 
	for ( i=0; i<dim; i++ )      //���������ǣ��ϰ������2����
	{
		bcross = false;
		if ( i == 0 )            //�����ϰ���i��Բ�ĵ�path��[i+1]�������ֱ�ߵľ���
		{
			x1 = m_dSX;
			y1 = m_dSY;
			x2 = path.pntlist[i+1].x;
			y2 = path.pntlist[i+1].y;
		}
		else if ( i == dim-1 )   //�����ϰ���i��Բ�ĵ�path��[i-1]����յ��ֱ�ߵľ���
		{
			x1 = path.pntlist[i-1].x;
			y1 = path.pntlist[i-1].y;
			x2 = m_dEX;
			y2 = m_dEY;
		}
		else    //�����ϰ���i��Բ�ĵ�path��[i-1]��͵�[i+1]���ֱ�ߵľ���
		{
			x1 = path.pntlist[i-1].x;
			y1 = path.pntlist[i-1].y;
			x2 = path.pntlist[i+1].x;
			y2 = path.pntlist[i+1].y;
		}

		//�жϵ�ǰά�ȵ�path�����������ߣ��Ƿ��������ϰ��ﶼ���ཻ
		for ( j=0; j<dim; j++ )
		{
			//�����ϰ���j��Բ�ĵ�(x1, y1),(x2, y2)���ڵ�ֱ�ߵľ���
			dis = CalPointToLineDis( x1, y1, x2, y2, m_lstObstacle[j].cx, m_lstObstacle[j].cy );
			//��Ϊpath�ĵ㶼�ǿ����򣬹�path�ϵĵ�ض������ϰ���Բ�ڣ�ֻ���жϸ��ϰ��ﵽֱ�߾����Ƿ�С���ϰ��������İ뾶�����ɶ϶��Ƿ��ཻ
			if ( dis < m_lstObstacle[j].radius )  
			{
				bcross = true;
				break;
			}
		}

		//���߲����ϰ����ཻ�ɸ��£����������е���ͬ��Բ�Ľ��㣬�����м�ά�ȵ���Ϣ
		if ( !bcross )
		{
			CalCrossPointByCirleAndLine( x1, y1, x2, y2, 0.0, 0.0, m_lstObstacle[i].polarlen, x, y );
			ang = atan2( y - m_dSY, x - m_dSX ) * 180 / M_PI;
			path.pntlist[i].polarang = ang;
			path.pntlist[i].x = x;
			path.pntlist[i].y = y;
		}
	}
	ReCalPathInfo( path );
}

void PathCalculatAlgorithm::ReCalPathInfo( StruPath &path, bool bPunish )
{
	int i, count;
	count = path.pntlist.size();
	if ( count == 0 )
        return;

	double x1, y1, x2, y2, len, sum_ang, ang1, ang2;
	len = 0;
	ang1 = 0.0;
	sum_ang = 0.0;
	x1 = m_dSX;
	y1 = m_dSY;
	for ( i=0; i<count; i++ )
	{
		x2 = path.pntlist[i].x;
		y2 = path.pntlist[i].y;
		len += sqrt( pow( y2 - y1, 2 ) + pow( x2 - x1, 2 ) );

		ang2 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
		sum_ang += fabs( ang2 - ang1 );

		ang1 = ang2;
		x1 = x2;
		y1 = y2;
	}
	len += sqrt( pow( m_dEY - y1, 2 ) + pow( m_dEX - x1, 2 ) );
	path.len = len;

	//�����յ�ת��
	ang2 = atan2( m_dEY - y1, m_dEX - x1 ) * 180.0 / M_PI;
	sum_ang += fabs( ang2 - ang1 );
	path.sum_ang = sum_ang;

	//���µ�ǰ�����뺽·��һ�κ���ļн�
	ang2 = atan2( path.pntlist[0].y - m_dSY, path.pntlist[0].x - m_dSX ) * 180.0 / M_PI;
	path.cross_ang = fabs( ang2 - m_dCrossAng );

	//���ӳͷ������ȳ���10��
	if ( bPunish )
        path.len = path.len * 10;
}

bool PathCalculatAlgorithm::GetGroupOptimumPath( vector< StruPath > pathgrp, StruPath &path )
{
	if ( pathgrp.size() > 0 )
	{
		StruPath grp_optimum_path;
		grp_optimum_path = pathgrp[0];
		for ( int i=0; i<pathgrp.size()-1; i++ )
		{
			if ( CalFitnessVal( grp_optimum_path ) > CalFitnessVal( pathgrp[i+1] ) )
			{
				grp_optimum_path = pathgrp[i+1];
			}
		}
		path = grp_optimum_path;
		return true;
	}

	return false;
}

bool PathCalculatAlgorithm::CreateOneFeasiblePath( StruPath &path )
{
	StruPath newpath;
	int i, dim;
	dim = m_lstObstacle.size();
	if ( dim == 0 )
		return false;

	double x1, y1, x2, y2, len, sum_ang, ang1, ang2;
	len = 0.0;
	ang1 = 0.0;
	sum_ang = 0.0;
	x1 = m_dSX;
	y1 = m_dSY;
	for ( i=0; i<dim; i++ )
	{
		//����ÿ��ά���ϵĿ���·����
		StruPathPoint nextpnt;
		if ( CalPathPointByDim( i, x1, y1, nextpnt ) )
		{
			x2 = nextpnt.x;
			y2 = nextpnt.y;			
			len += sqrt( pow( y2 - y1, 2 ) + pow( x2 - x1, 2 ) );

			ang2 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
			sum_ang += fabs( ang2 - ang1 );

			ang1 = ang2;
			x1 = x2;
			y1 = y2;

			newpath.pntlist.push_back( nextpnt );
		}
		else
		{
			return false;
		}
	}
	len += sqrt( pow( m_dEY - y1, 2 ) + pow( m_dEX - x1, 2 ) );
	newpath.len = len;

	//�����յ�ת��
	ang2 = atan2( m_dEY - y1, m_dEX - x1 ) * 180.0 / M_PI;
	sum_ang += fabs( ang2 - ang1 );
	newpath.sum_ang = sum_ang;

	//���µ�ǰ�����뺽·��һ�κ���ļн�
	ang2 = atan2( newpath.pntlist[0].y - m_dSY, newpath.pntlist[0].x - m_dSX ) * 180.0 / M_PI;
	newpath.cross_ang = fabs( ang2 - m_dCrossAng );

	path = newpath;

	return true;
}

bool PathCalculatAlgorithm::CalPathPointByDim( int cur_dim, double lastx, double lasty, StruPathPoint &newpnt )
{
	if ( cur_dim > m_lstObstacle.size() || cur_dim < 0 )
		return false;

	double se_angle, random_angle;
	double x1, y1, x2, y2, tmpdis;

	int i, count;
	i = cur_dim;
	count = 0;      //ѭ������
	x1 = lastx;
	y1 = lasty;
	se_angle = atan2( m_dEY - m_dSY, m_dEX - m_dSX ) * 180.0 / M_PI;

	while( count < gc_MaxRandomNum )
	{
		//����ڵ�iά����ȡһ�㣬�ж������i-1ά���ϵĵ������Ƿ��ڵ�iά�ȵĽ������
		tmpdis = m_lstObstacle[i].polarlen;
		random_angle = GetRandomAngle( se_angle-gc_AngleRange, se_angle+gc_AngleRange );
		x2 = tmpdis * cos( random_angle / 180.0 * M_PI );
		y2 = tmpdis * sin( random_angle / 180.0 * M_PI );

		if ( !CheckPathPointEnableByDim( i,  x1, y1, x2, y2 ) )
			count ++;
		else
			break;
	}
	newpnt.dim = i;
	newpnt.x = x2;
	newpnt.y = y2;
	newpnt.polarang = random_angle;

	if ( count == gc_MaxRandomNum )
		return false;
	else
		return true;
}

bool PathCalculatAlgorithm::CheckPathPointEnableByDim( int cur_dim, double lastx, double lasty, double curx, double cury )
{
	if ( cur_dim > m_lstObstacle.size() || cur_dim < 0 )
		return false;

	int i = cur_dim;

	double disable_angle, se_angle,dis;
	double x1, y1, x2, y2, tmpang, tmpdis, ang_x1x2;
	x1 = lastx;
	y1 = lasty;
	x2 = curx;
	y2 = cury;
	se_angle = atan2( m_dEY - m_dSY, m_dEX - m_dSX ) * 180.0 / M_PI;

	//�жϵ�ǰ���Ƿ�����һά�ȵ��ϰ�����
	if ( i+1 < m_lstObstacle.size() )
	{
		tmpdis =  sqrt( pow( m_lstObstacle[i+1].cy - y2, 2 ) + pow( m_lstObstacle[i+1].cx - x2, 2 ) );
		//��ǰ������һά���ϰ����ڣ�������
		if ( tmpdis < m_lstObstacle[i+1].radius )
			return false;
	}
	//�ж���һ���Ƿ��ڵ�ǰά���ϰ�����
	tmpdis =  sqrt( pow( m_lstObstacle[i].cy - y1, 2 ) + pow( m_lstObstacle[i].cx - x1, 2 ) );
	//��һ���ڵ�ǰ�ϰ����ڣ�������
	if ( tmpdis < m_lstObstacle[i].radius )
        return false;
	//�ж���ǰ����������һ��ά���ϵĵ㣬�����յ�������Ƿ�Խ�˵�ǰά���ϵ��ϰ���	20180914
	if ( i+1 == m_lstObstacle.size() )
	{
		dis = CalPointToLineDis(x2, y2, m_dEX, m_dEY, m_lstObstacle[i].cx, m_lstObstacle[i].cy);
		if (dis < m_lstObstacle[i].radius)
			return false;
	}


	//�жϵ�ǰ������һ��������Ƿ��ڽ���Ƿ�Χ��
	tmpang = atan2( m_lstObstacle[i].cy - y1, m_lstObstacle[i].cx - x1 ) * 180.0 / M_PI;
	disable_angle = asin( m_lstObstacle[i].radius / tmpdis ) * 180.0 / M_PI;
	ang_x1x2 = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
	if ( ang_x1x2 > tmpang-disable_angle && ang_x1x2 < tmpang+disable_angle )
	{
		return false;
	}
	else
	{
		//�жϵ�ǰ������һ��������Ƿ񳬹���ǰ��Լ���Ƕȣ��������
		if ( ang_x1x2 < se_angle-gc_AngleRange || ang_x1x2 > se_angle+gc_AngleRange  )
			return false;

		//�жϵ�ǰ������һ��������Ƿ�Խ��ǰ����ά�ȵ��ϰ��������ǰ����ά���ϰ����е㵽���ߵĴ��߳����Ƿ�С���ϰ���뾶
		if ( i > 0 )
		{
			double dis;
			for ( int j=0; j<i; j++ )
			{
				dis = CalPointToLineDis( x1, y1, x2, y2, m_lstObstacle[j].cx, m_lstObstacle[j].cy );
				if ( dis < m_lstObstacle[j].radius )
					return false;
			}
		}//����0ά��ʱ��Ϊ��㿪ʼ��������������ƣ������ж�

		return true;
	}
}

double PathCalculatAlgorithm::GetRandomAngle( double s_ang, double e_ang )
{
	double angle = s_ang + rand() % (int)( e_ang - s_ang + 1 );

	return angle;
}

double PathCalculatAlgorithm::CalFitnessVal( StruPath path )
{
	double para_w, val;    //Ȩ�ز���
	para_w = 0.9;
	val = path.len * para_w + path.sum_ang * ( 1 - para_w ) + path.cross_ang*0.3;

	return val;
}

int PathCalculatAlgorithm::CheckEndPointValidType()
{
    //�жϽ������Ƿ������һ���ϰ�����, -1 ���ϰ��1 ������Ϸ��� 0 �����㲻�Ϸ� 
	int dim = m_lstObstacle.size();

	if ( dim == 0 )
		return -1;

	double dis;
	dis =  sqrt( pow( m_lstObstacle[dim-1].cy - m_dEY, 2 ) + pow( m_lstObstacle[dim-1].cx - m_dEX, 2 ) );
	//�����㲻�Ϸ����ü������һ���滻
	if ( dis < m_lstObstacle[dim-1].radius )
	{
		int index = m_struOptimumPath.pntlist.size() - 1;
		m_dEX = m_struOptimumPath.pntlist[index].x;
		m_dEY = m_struOptimumPath.pntlist[index].y;
		return 0;
	}

	return 1;
}

//////////////////////////////////////////////////////////////////////////
//ObstacleAvoidAlgorithm::ObstacleAvoidAlgorithm()
//{
//	m_iParticleNum = 100;
//	m_iIterationNum = 50;
//
//	m_iCurAvoidRule = -1;
//	m_iCurAvoidRslt = 0;
//
//	m_dEX = 0.0;
//	m_dEY = 0.0;
//	m_dSaveDis = 50.0;
//	m_dCrossTime = 0.0;
//	m_dAvoidEndPntX = 0.0;
//	m_dAvoidEndPntY = 0.0;
//
//	m_dFit_m1 = 1;
//	m_dFit_m2 = 10;
//
//	m_lstChange.clear();
//	m_lstSpeed.clear();
//}
//
//ObstacleAvoidAlgorithm::~ObstacleAvoidAlgorithm()
//{
//
//}
//
//void ObstacleAvoidAlgorithm::BuildSceneDataSet( double ex, double ey, StruBoatInfo myboat, StruBoatInfo otherboat )
//{
//	m_dEX = ex;
//	m_dEY = ey;
//	m_stuMyBoat = myboat;
//	m_stuOtherBoat = otherboat;
//
//	//��ʼ������Ϣ
//	InitBoatInfo();
//
//	//��ʼ����Ⱥ����
//	InitParticleSwarm();
//
//	//PSO����
//	ProcessPSO();
//
//	//������Ͻ�����
//	CalAvoidEndPnt();
//}
//
//void ObstacleAvoidAlgorithm::SetSaveDis( double dis )
//{
//	m_dSaveDis = dis;
//}
//
//double ObstacleAvoidAlgorithm::GetSaveDis()
//{
//	return m_dSaveDis;
//}
//
//StruChangeInfo ObstacleAvoidAlgorithm::GetOptimumChangeInfo( double &ex, double &ey  )
//{
//	ex = m_dAvoidEndPntX;
//	ey = m_dAvoidEndPntY;
//	return m_struOptimumChange;
//}
//
//void ObstacleAvoidAlgorithm::InitBoatInfo()
//{
//	//����ٶȵķ��򣬴˴�Ϊ�������㣬�м�ת��������ȥ
//	double speed1_x, speed1_y, speed2_x, speed2_y;
//	speed1_x = m_stuMyBoat.speed * cos( m_stuMyBoat.azimuth / 180.0 * M_PI );
//	speed1_y = m_stuMyBoat.speed * sin( m_stuMyBoat.azimuth / 180.0 * M_PI );
//	speed2_x = m_stuOtherBoat.speed * cos( m_stuOtherBoat.azimuth / 180.0 * M_PI );
//	speed2_y = m_stuOtherBoat.speed * sin( m_stuOtherBoat.azimuth / 180.0 * M_PI );
//    //tan( ang ) = ( sp1 * sin(ang1) + sp2 * sin( 180-ang2 ) ) / ( sp1 * cos(ang1) + sp2 * cos( 180-ang2 ) )
//	//           = ( sp1 * sin(ang1) - sp2 * sin(ang2) ) / ( sp1 * cos(ang1) - sp2 * cos(ang2) )
//	double ang_resultant_velocity;
//	ang_resultant_velocity = atan2( speed1_y-speed2_y, speed1_x-speed2_x ) * 180.0 / M_PI;
//	m_dRslt_V = sqrt( pow( speed1_y-speed2_y, 2 ) + pow( speed1_x-speed2_x, 2 ) );
//
//	//VR����ٶȵļн�
//	m_dAngle_Q = m_stuMyBoat.azimuth - ang_resultant_velocity;
//
//	//�������������ߵĽǶ�                               
//	double ang_boats;
//	ang_boats = atan2( m_stuOtherBoat.y-m_stuMyBoat.y, m_stuOtherBoat.x-m_stuMyBoat.x ) * 180.0 / M_PI;
//
//	//���ٶ�V�������������ߵļн�
//	m_dAngle_R = ang_resultant_velocity - ang_boats;
//
//	//�����ƶ���ֻ���ġ���ȫ�������ߵļн�
//	double dis;
//	dis = sqrt( pow( m_stuOtherBoat.y-m_stuMyBoat.y, 2 ) + pow( m_stuOtherBoat.x-m_stuMyBoat.x, 2 ) );
//	m_dAngle_U = asin( m_dSaveDis / dis ) * 180.0 / M_PI;
//
//	//��ǰ��λ�����յ��������ߵĽǶȣ��뺽��ļн�
//	double ang;
//	ang = atan2( m_dEY-m_stuMyBoat.y, m_dEX-m_stuMyBoat.x ) * 180 / M_PI;
//	ang = m_stuMyBoat.azimuth - ang;
//	m_dAngle_X = ang;
//
//	//��������ʱ��
//	m_dCrossTime = dis / m_dRslt_V;
//	//��ʼ�����¹��������Ϣ
//	m_iCurAvoidRslt = CalCurAvoidAzimuthByRule( m_stuMyBoat.azimuth, m_stuOtherBoat.azimuth, m_iCurAvoidRule );
//}
//
//void ObstacleAvoidAlgorithm::InitParticleSwarm()
//{
//	m_lstChange.clear();
//
//	int i;
//	for ( i=0; i<m_iParticleNum; i++  )
//	{
//		StruChangeInfo newchange;
//		//����һ��·���ϵĿ���·���㣬��ά�ȡ�������������
//		if ( CreateOneFeasibleChange( newchange ) )
//		{
//			m_lstChange.push_back( newchange );
//		}
//	}
//}
//
//void ObstacleAvoidAlgorithm::ProcessPSO()
//{
//	//��ʼ��r1, r2��Χ 0 - 1 ֮��
//	m_dPSO_r1 = double( rand() % 1000 ) / 1000;
//	m_dPSO_r2 = double( rand() % 1000 ) / 1000;
//
//	StruChangeInfo grp_optimum_change, tmpgrp_optimum_change;
//	vector< StruChangeInfo > cur_optimum_change_lst;
//
//	int i, j;
//	double cur_optimum_ang, grp_optimum_ang, tmp_ang;
//	double cur_optimum_speed, grp_optimum_speed, tmp_speed;
//	double randompara, change_speed, change_azimuth;
//
//	//��ʼ����Ϣ
//	m_lstSpeed.clear();
//	//���һ�����ű仯�⣬��Ϊ��ʼ��Ⱥ���Ž�
//	grp_optimum_change = GetGroupOptimumChange( m_lstChange );
//
//	//��ʼ��ÿһ���仯��ĵ��ٶ�v
//	for ( j=0; j<m_lstChange.size(); j++ )
//	{
//		//��ʼ��ÿһ���仯�⣬��ֵΪ��ǰ���Ž�
//		StruChangeInfo tmp_change;
//		tmp_change = m_lstChange[j];
//		cur_optimum_change_lst.push_back( tmp_change );
//
//		StruChangeInfo change_speed;   //�˴��仯��ı仯��
//		randompara = double( rand() % 2000 ) / 1000 - 1;
//		change_speed.speed = randompara * gc_SpeedRange / gc_PSO_num;
//		change_speed.azimuth = randompara * gc_AngleRange / gc_PSO_num;
//
//		m_lstSpeed.push_back( change_speed );
//	}
//
//	//���������Ž⣬��С�ڵ�����������δ����ʱ��ѭ��
//	i = 0;
//	while ( i<m_iIterationNum )
//	{
//		//����Ⱥ����һ�ε���������ÿ�����Ž�ı仯��Ϣ��������
//		for ( j=0; j<m_lstChange.size(); j++ )
//		{
//			//�ٶȱ�����ֵ
//			tmp_speed = m_lstChange[j].speed;
//			cur_optimum_speed = cur_optimum_change_lst[j].speed;
//			grp_optimum_speed = grp_optimum_change.speed;
//			//�Ƕȱ�����ֵ
//			tmp_ang = m_lstChange[j].azimuth;
//			cur_optimum_ang = cur_optimum_change_lst[j].azimuth;
//			grp_optimum_ang = grp_optimum_change.azimuth;
//
//			//��ʽ v(t+1) = gc_PSO_w*v(t) + gc_PSO_c1*m_dPSO_r1*( p_id - x_id ) + gc_PSO_c2*m_dPSO_r2( p_gd - xid );
//			//��ʽ x_id(t+1) = x_id(t) + v(t+1)
//
//			//�����ٶȱ仯�� 
//			change_speed = m_lstSpeed[j].speed;
//			change_speed = gc_PSO_w*change_speed + gc_PSO_c1*m_dPSO_r1*( cur_optimum_speed - tmp_speed ) + gc_PSO_c2*m_dPSO_r2*( grp_optimum_speed - tmp_speed );
//			tmp_speed = tmp_speed + change_speed;
//			//����Ƕȱ仯��
//			change_azimuth = m_lstSpeed[j].azimuth;
//			change_azimuth = gc_PSO_w*change_azimuth + gc_PSO_c1*m_dPSO_r1*( cur_optimum_ang - tmp_ang ) + gc_PSO_c2*m_dPSO_r2*( grp_optimum_ang - tmp_ang );
//			tmp_ang = tmp_ang + change_azimuth;
//
//            //�ж��Ƿ���н⣬������£����򲻸���
//			if ( CheckChangeInfoEnable( tmp_speed, tmp_ang ) )
//			{
//				m_lstChange[j].speed = tmp_speed;
//				m_lstChange[j].azimuth = tmp_ang;
//			}
//
//			//������Ӧ���������µ�ǰ���Ž�
//			if ( CalFitnessVal( m_lstChange[j] ) < CalFitnessVal( cur_optimum_change_lst[j] ) )
//			{
//				cur_optimum_change_lst[j] = m_lstChange[j];
//			}
//		}
//		//�ӵ�ǰ���Ž��л�ȡ��Ⱥ���ŽⲢ����
//		tmpgrp_optimum_change = GetGroupOptimumChange( cur_optimum_change_lst );
//		if ( CalFitnessVal( tmpgrp_optimum_change ) < CalFitnessVal( grp_optimum_change ) )
//		{
//			grp_optimum_change = tmpgrp_optimum_change;
//		}
//		i++;
//	}
//	//��ȡ����������Ⱥ���Ž�
//	m_struOptimumChange = grp_optimum_change;
//}
//
//double ObstacleAvoidAlgorithm::GetRandomChangeSpeed()
//{ 
//	int speedrange;
//	speedrange = qRound( gc_SpeedRange ) - 1;    //ȡ�����ٶȣ����������-1��֤���ᳬ���ٶ����ƣ�
//
//	//�ٶȿ��÷�Χ[0, speedrange]�������ٶȱ仯��Χ[-curspeed, speedrange-curspeed]
//    double speed, curspeed;
//	curspeed = m_stuMyBoat.speed * 10; //ˢ��Ϊ0.1�룬��ʵ���ٶ�ת������Ҫ����10
//	speed = - curspeed + rand() % qRound( speedrange-curspeed );
//	speed = speed / 10.0;
//
//	return speed;
//}
//
//double ObstacleAvoidAlgorithm::GetRandomChangeAzimuth()
//{
//	double para, rsltrang;
//	para = m_iCurAvoidRslt * double( rand() % 1000 ) / 1000;
//
//	rsltrang = para * gc_AngleRange;
//	return rsltrang;
//}
//
//int ObstacleAvoidAlgorithm::CalCurAvoidAzimuthByRule( double myboat_azimuth, double otherboat_azimuth, int &rule )
//{
//	int result = 0;
//
//	double ang;
//	ang = otherboat_azimuth - myboat_azimuth;
//	//��׼��
//	if ( ang < 0 )
//		ang += 360.0;
//
//	//��������������ǰ������15�ȷ�Χ��һ�����ұ���
//	if ( ang >= 165.0 && ang <= 195.0 )
//	{
//		rule = FrontCross;
//		result = -1;   
//	}
//	else if ( ang > 195.0 && ang < 315.0 )  //���Ͻ��棬�������
//	{
//		rule = LeftCross;
//		result = 1;  
//	}
//	else if ( ang > 45.0 && ang < 165 )     //���Ͻ��棬���ұ���
//	{
//		rule = RightCross;
//		result = -1; 
//	}
//	else if ( ang >= 315.0 && ang < 360.0 ) //���׷Խ��������� 
//	{
//		rule = L_Overtaking;
//		result = 1; 
//	}
//	else if ( ang >= 0.0 && ang <= 45.0 )   //�Ҳ�׷Խ�������ٶ��жϣ��ٶȴ���Ŀ�����׷Խ�������Ҳ�ͨ��
//	{
//		if ( m_stuMyBoat.speed > m_stuOtherBoat.speed )
//		{
//			rule = R_Overtaking_TL;
//			result = 1; 
//		}
//		else
//		{
//			rule = R_Overtaking_TR;
//			result = -1; 
//		}
//	}
//
//	return result;
//}
//
//bool ObstacleAvoidAlgorithm::CreateOneFeasibleChange( StruChangeInfo &change )
//{
//	int count;
//	count = 0;      //ѭ������
//	double speed, ang;
//	while( count < gc_MaxRandomNum )
//	{
//		//���ȡ�仯�ٶȣ��仯�Ƕ�
//		speed = GetRandomChangeSpeed();
//		ang = GetRandomChangeAzimuth();
//
//		if ( !CheckChangeInfoEnable( speed, ang ) )
//			count ++;
//		else
//			break;
//	}
//	change.speed = speed;
//	change.azimuth = ang;
//
//	if ( count == gc_MaxRandomNum )
//		return false;
//	else
//		return true;
//}
//
//bool ObstacleAvoidAlgorithm::CheckChangeInfoEnable( double speed, double azimuth )
//{
//	//�ж��ٶ�ֵ���Ƕ�ֵ�Ƿ�Ϸ�
//	if ( speed < -m_stuMyBoat.speed || speed > ( gc_SpeedRange / 10.0 - m_stuMyBoat.speed ) )
//        return false;
//	if ( azimuth < m_stuMyBoat.azimuth-gc_AngleRange || azimuth > m_stuMyBoat.azimuth + gc_AngleRange )
//        return false;
//
//	//�ж�ת���Ƿ���Ϻ��¹�����
//	if ( m_iCurAvoidRslt == -1  )
//	{
//		if ( azimuth > 0 )            //���¹���Ҫ����תʱ����ת������
//            return false;
//	}
//	else if ( m_iCurAvoidRslt == 1 )
//	{
//		if ( azimuth < 0 )            //���¹���Ҫ����תʱ����ת������
//			return false;
//	}
//
//	//�ж�����ʽ r >= u - R ���� r < = -( u + R ) עԭʽΪ R + r >= u ���� R + r <= -u
//	double r, ang, tmpazimuth, tmpang_R, tmpang_U;
//	ang = m_dAngle_Q / 180.0 * M_PI;
//	tmpazimuth = azimuth / 180.0 * M_PI;
//	tmpang_R = m_dAngle_R/* / 180.0 * M_PI*/;
//	tmpang_U = m_dAngle_U/* / 180.0 * M_PI*/;
//	r = -sin( ang ) * speed / m_dRslt_V + m_stuMyBoat.speed * cos( ang ) * azimuth / m_dRslt_V ;
//
//	if ( tmpang_R + r >= tmpang_U || tmpang_R + r <= -tmpang_U )
//        return true;
//
//	return false;
//}
//
//double ObstacleAvoidAlgorithm::CalFitnessVal( StruChangeInfo change )
//{
//	//��Ӧ�Ⱥ���
//	double val = m_dFit_m1 * fabs( change.speed ) + m_dFit_m2 * fabs( m_dAngle_X + change.azimuth );
//	return val;
//}
//
//StruChangeInfo ObstacleAvoidAlgorithm::GetGroupOptimumChange( vector< StruChangeInfo > changegrp )
//{
//	StruChangeInfo grp_optimum_change;
//	grp_optimum_change = changegrp[0];
//	for ( int i=0; i<changegrp.size()-1; i++ )
//	{
//		if ( CalFitnessVal( grp_optimum_change ) > CalFitnessVal( changegrp[i+1] ) )
//		{
//			grp_optimum_change = changegrp[i+1];
//		}
//	}
//
//	return grp_optimum_change;
//}
//
//void ObstacleAvoidAlgorithm::CalAvoidEndPnt()
//{
//	//����������ĺ��٣�����
//	double speed, azimuth;
//	speed = m_stuMyBoat.speed + m_struOptimumChange.speed;
//	azimuth = m_stuMyBoat.azimuth + m_struOptimumChange.azimuth;
//
//	//����ٶȵķ��򣬴˴�Ϊ�������㣬�м�ת��������ȥ
//	double speed1_x, speed1_y, speed2_x, speed2_y;
//	speed1_x = speed * cos( azimuth / 180.0 * M_PI );
//	speed1_y = speed * sin( azimuth / 180.0 * M_PI );
//	speed2_x = m_stuOtherBoat.speed * cos( m_stuOtherBoat.azimuth / 180.0 * M_PI );
//	speed2_y = m_stuOtherBoat.speed * sin( m_stuOtherBoat.azimuth / 180.0 * M_PI );
//	//tan( ang ) = ( sp1 * sin(ang1) + sp2 * sin( 180-ang2 ) ) / ( sp1 * cos(ang1) + sp2 * cos( 180-ang2 ) )
//	//           = ( sp1 * sin(ang1) - sp2 * sin(ang2) ) / ( sp1 * cos(ang1) - sp2 * cos(ang2) )
//	double ang_resultant_velocity, speed_resultant_velocity;
//	ang_resultant_velocity = atan2( speed1_y-speed2_y, speed1_x-speed2_x );
//	speed_resultant_velocity = sqrt( pow( speed1_y-speed2_y, 2 ) + pow( speed1_x-speed2_x, 2 ) );
//
//	//������������
//	double dis;
//	dis = sqrt( pow( m_stuOtherBoat.y-m_stuMyBoat.y, 2 ) + pow( m_stuOtherBoat.x-m_stuMyBoat.x, 2 ) );
//	dis += m_dSaveDis;
//
//	//������ٶȵ����λ�õ�ʱ��
//	double crosstime;
//	crosstime = dis / speed_resultant_velocity;
//
//	//���㱾�����޸ĺ�ĺ����٣���ʻʱ��crosstime�󵽴��λ�õ㼴Ϊ������
//	dis = speed * crosstime;
//
//	m_dAvoidEndPntX = dis * cos( azimuth / 180.0 * M_PI );
//	m_dAvoidEndPntY = dis * sin( azimuth / 180.0 * M_PI );
//}
//
////////////////////////////////////////////////////////////////////////////
//LogicBoatModel::LogicBoatModel()
//{
//	m_bStop = false;
//    m_bIsMyBoat = false;
//	m_bDurAvoid = false;
//	m_bReSetChange = false;
//	m_bAddAvoidPnt = false;
//
//	m_iRefGrpID = -1;
//	m_iCurSubPathID = -1;
//	m_dX = 0.0;
//	m_dY = 0.0;
//	m_dAzimuth = 0.0;
//	m_dSpeed = 20.0;
//	m_dCloseDis = 4.0;
//	m_dChg_Speed = 0.0;
//	m_dChg_Azimuth = 0.0;
//	m_dCrossTime = 1.0;
//	m_dChg_Speed_Done = 0.0;
//	m_dChg_Azimuth_Done = 0.0;
//	m_dAlarmDis = 200.0;
//	m_dAvoidEndPntX = 0.0;
//	m_dAvoidEndPntY = 0.0;
//
//	m_pRefOtherBoat = NULL;
//}
//
//LogicBoatModel::LogicBoatModel( StruBoatInfo boatinfo )
//{
//	m_bStop = false;
//	m_bDurAvoid = false;
//	m_bReSetChange = false;
//	m_bAddAvoidPnt = false;
//
//	m_iCurSubPathID = -1;
//	m_dX = boatinfo.x;
//	m_dY = boatinfo.y;
//	m_dAzimuth = boatinfo.azimuth;
//	m_dSpeed = boatinfo.speed;
//	m_bIsMyBoat = boatinfo.myboat;
//	m_iRefGrpID = boatinfo.refgrp_id;
//
//	m_dCloseDis = 2.0;
//	m_dChg_Speed = 0.0;
//	m_dChg_Azimuth = 0.0;
//	m_dCrossTime = 1.0;
//	m_dChg_Speed_Done = 0.0;
//	m_dChg_Azimuth_Done = 0.0;
//	m_dAlarmDis = 200.0;
//	m_dAvoidEndPntX = 0.0;
//	m_dAvoidEndPntY = 0.0;
//
//	m_pRefOtherBoat = NULL;
//}
//
//LogicBoatModel::~LogicBoatModel()
//{
//
//}
//
//void LogicBoatModel::GetCurrentBoatInfo( StruBoatInfo &boatinfo )
//{
//	boatinfo.x = m_dX;
//	boatinfo.y = m_dY;
//	boatinfo.azimuth = m_dAzimuth;
//	boatinfo.speed = m_dSpeed;
//	boatinfo.myboat = m_bIsMyBoat;
//	boatinfo.refgrp_id = m_iRefGrpID;
//}
//
//void LogicBoatModel::SetBoatSpeed( double speed )
//{
//	m_dSpeed = speed;
//}
//
//double LogicBoatModel::GetBoatSpeed()
//{
//	return m_dSpeed;
//}
//
//void LogicBoatModel::SetBoatAzimuth( double azimuth )
//{
//	m_dAzimuth = azimuth;
//}
//
//double LogicBoatModel::GetBoatAzimuth()
//{
//	return m_dAzimuth;
//}
//
//void LogicBoatModel::SetBoatPath( vector< QPair< double, double > > pathpntlist )
//{
//	if ( pathpntlist.size() < 1 )
//		return;
//
//	for ( int i=0; i<pathpntlist.size(); i++ )
//	{
//		PathPointInfo pntinfo;
//		pntinfo.index = i;
//		pntinfo.x = pathpntlist[i].first;
//		pntinfo.y = pathpntlist[i].second;
//		pntinfo.arrived = false;
//
//		m_lstPathPnt.push_back( pntinfo );
//	}
//}
//
//void LogicBoatModel::UpdateBoatInfo()
//{
//	if ( m_bStop )
//		return;
//
//    UpdateSpeedAndAzimuth();
//
//	m_dX += m_dSpeed * cos( m_dAzimuth / 180.0 * M_PI );
//	m_dY += m_dSpeed * sin( m_dAzimuth / 180.0 * M_PI );
//}
//
//bool LogicBoatModel::ChkNeedAvoid( double ex, double ey )
//{
//	if ( m_pRefOtherBoat != NULL )
//	{
//		StruBoatInfo myboat, otherboat;
//		GetCurrentBoatInfo( myboat );
//		m_pRefOtherBoat->GetCurrentBoatInfo( otherboat );  //����������������Ϣ
//		double dis;
//		dis = sqrt( pow( otherboat.y - m_dY, 2 ) + pow( otherboat.x - m_dX, 2 ) );
//		if ( dis < m_dAlarmDis ) //���뾯������
//		{
//			//�ж��Ƿ������
//			double k1, k2, cross_x, cross_y, tx, ty, cross_t, len1, len2, len3;
//			k1 = tan( myboat.azimuth / 180.0 * M_PI );
//			k2 = tan( otherboat.azimuth / 180.0 * M_PI );
//			if ( fabs( k1 - k2 ) < 1e-1 )  //����ƽ�У�������
//			{
//				return false;
//			}
//			else //�������ߵĽ���
//			{
//				cross_x = ( myboat.y - otherboat.y - k1 * myboat.x + k2 * otherboat.x ) / ( k2 - k1 );
//				cross_y = myboat.y + k1 * ( cross_x - myboat.x );
//				//�жϽ����Ƿ��ڱ����Ӻ�����
//				len1 = sqrt( pow( myboat.y - ey, 2 ) + pow( myboat.x - ex, 2 ) );             //���κ���ʣ�����
//				len2 = sqrt( pow( myboat.y - cross_y, 2 ) + pow( myboat.x - cross_x, 2 ) );   //������ľ���
//				len3 = sqrt( pow( ey - cross_y, 2 ) + pow( ex - cross_x, 2 ) );               //���㵽���ν�������ľ���
//				if ( len2 + len3 > len1 + 1.0 )  //���Ȳ����1.0˵�����ڱ�����
//				{
//					return false;
//				}
//				//Ŀ�괬ֻ�����߽���ľ���
//				dis = sqrt( pow( otherboat.y - cross_y, 2 ) + pow( otherboat.x - cross_x, 2 ) );   
//				//Ŀ�괬ֻ�����߽����ʱ��
//				cross_t = dis / otherboat.speed;  
//				//Ԥ�Ȿ������ʱ��ʱ��λ��
//				dis = myboat.speed * cross_t;                                          
//				tx = myboat.x + dis * cos( myboat.azimuth / 180.0 * M_PI );
//				ty = myboat.y + dis * sin( myboat.azimuth / 180.0 * M_PI );
//				//�жϱ�������ʱ���λ�þ��������߽���ľ����Ƿ�С�ڰ�ȫ����
//				dis = sqrt( pow( cross_y - ty, 2 ) + pow( cross_x - tx, 2 ) );
//				ObstacleAvoidAlgorithm avoidalgorithm;
//				if ( dis < avoidalgorithm.GetSaveDis() )
//				{
//					avoidalgorithm.BuildSceneDataSet( ex, ey, myboat, otherboat );
//					StruChangeInfo changeinfo;
//					changeinfo = avoidalgorithm.GetOptimumChangeInfo( m_dAvoidEndPntX, m_dAvoidEndPntY );
//					m_dAvoidEndPntX += m_dX;
//					m_dAvoidEndPntY += m_dY;
//					m_dChg_Speed = changeinfo.speed;
//					m_dChg_Azimuth = changeinfo.azimuth;
//
//					return true;
//				}
//			}
//		}
//	}
//
//	return false;
//}
//
//void LogicBoatModel::UpdateSpeedAndAzimuth()
//{
//	if ( m_lstPathPnt.size() < 1 )
//		return;
//
//	//��̬·��
//	int i, index;
//	index = -1;
//	double dis;
//	for ( i=1; i<m_lstPathPnt.size(); i++ )
//	{
//		//�жϵ�ǰ�Ƿ񵽴��Ӧ����
//		dis = sqrt( pow( m_lstPathPnt[i].y - m_dY, 2 ) + pow( m_lstPathPnt[i].x - m_dX, 2 ) );
//		if ( dis < m_dCloseDis )
//		{
//			m_lstPathPnt[i].arrived = true;
//		}
//		//ȡ����һ��δ�ƶ�������index
//		if ( m_lstPathPnt[i].arrived == false )
//		{
//			index = i;
//			break;
//		}
//	}
//	m_iCurSubPathID = index;
//	if ( index == -1 )  //���к��㶼�Ѿ����ֹ꣬ͣ
//	{
//		m_bStop = true;
//	}
//	else  //������һ�κ�·
//	{
//		if ( !m_bDurAvoid )
//		{
//			//�ж�ǰ���Ƿ���Ҫ��̬����
//			if ( !ChkNeedAvoid( m_lstPathPnt[index].x, m_lstPathPnt[index].y ) )
//			{
//				double x1, y1, x2, y2;
//				x1 = m_lstPathPnt[index-1].x;
//				y1 = m_lstPathPnt[index-1].y;
//				x2 = m_lstPathPnt[index].x;
//				y2 = m_lstPathPnt[index].y;
//				m_dAzimuth = atan2( y2 - y1, x2 - x1 ) * 180.0 / M_PI;
//			}
//			else
//			{
//				m_bDurAvoid = true;
//				m_bReSetChange = false;
//				m_bAddAvoidPnt = true;    //���ڸո��ж���Ҫ���ϵ�һ����������trueһ�Σ���֤�ⲿֻ����һ�θõ�
//			}
//		}
//		else
//		{
//			m_bAddAvoidPnt = false;      //�¸����ڼ���Ϊfalse�������ⲿ�����ظ����ӱ��Ͻ�����
//			double setp_speed, setp_azimuth;
//			setp_speed = m_dChg_Speed / m_dCrossTime;     
//			setp_azimuth = m_dChg_Azimuth / m_dCrossTime;
//			//�ѱ仯�ٶ�δ�ﵽ����ֵ����������ٶ�
//			if ( fabs( m_dChg_Speed_Done ) < fabs( m_dChg_Speed ) )
//			{
//				m_dChg_Speed_Done += setp_speed;
//				m_dSpeed += setp_speed;
//			}
//			//�ѱ仯�Ƕ�ֵδ�ﵽ����ֵ����������Ƕ�
//			if ( fabs( m_dChg_Azimuth_Done ) < fabs( m_dChg_Azimuth ) )
//			{
//				m_dChg_Azimuth_Done += setp_azimuth;
//				m_dAzimuth += setp_azimuth;
//			}
//			//�ж��Ƿ񵽴���Ͻ�����
//			if ( fabs( m_dX - m_dAvoidEndPntX ) < 2.0 && fabs( m_dY - m_dAvoidEndPntY ) < 10.0 && !m_bReSetChange )
//			{
//				m_dSpeed = m_dSpeed - m_dChg_Speed;
//				//�����µ��յ�͵�ǰλ�ü��㺽��
//				m_dAzimuth = atan2( m_lstPathPnt[index].y - m_dY, m_lstPathPnt[index].x - m_dX ) * 180.0 / M_PI;
//				m_bReSetChange = true;
//			}
//			//�ж��Ƿ񵽴ﵱǰ��·���յ�
//			dis = sqrt( pow( m_lstPathPnt[index].y - m_dY, 2 ) + pow( m_lstPathPnt[index].x - m_dX, 2 ) );
//			if ( dis < m_dCloseDis * 2 )
//			{
//				m_bDurAvoid = false;
//			}
//		}
//	}
//}
//
//void LogicBoatModel::SetRefLogicOhterBoat( LogicBoatModel * otherboat )
//{
//	m_pRefOtherBoat = otherboat;
//}
//
//bool LogicBoatModel::IfNeedAvoidGetEndPnt( double &endx, double &endy  )
//{
//	endx = m_dAvoidEndPntX;
//	endy = m_dAvoidEndPntY;
//	return m_bAddAvoidPnt;
//}
//
//bool LogicBoatModel::GetCurAndSubPathEndPnt( double &cur_x, double &cur_y, double &sub_ex, double &sub_ey )
//{
//	if ( m_iCurSubPathID != -1 )
//	{
//		cur_x = m_dX;
//		cur_y = m_dY;
//		sub_ex = m_lstPathPnt[m_iCurSubPathID].x;
//		sub_ey = m_lstPathPnt[m_iCurSubPathID].y;
//
//		return true;
//	}
//	return false;
//}