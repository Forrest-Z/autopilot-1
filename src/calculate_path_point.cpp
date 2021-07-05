//2017��5��8�� 10:18:21
#include "stdafx.h"                                                                                                                                                     
#include "../include/usv_include.h"  

double temp_point_lat;
double temp_point_lng;
int	   point_sum;
int		epv_type;		//-1 ���ϰ��� 0 �ϰ���������� 1 �������Ϻ���
OBSTACLE_LOCATE_INF_STRUCT moving_obs;
double get_velocity_heading(double vx,double vy);

void *calculate_path_point(void *aa)
{
	StruGPSPath local_path;                                                                                                                                             
	PathCalculatAlgorithm obj_path; 
	vector <StruGPSObstacle> OBS;
	StruGPSObstacle obstacle; 
	int i;
	double lat_local,lng_local;
	double lat_dst,lng_dst;
	double heading_local;
	int iret_pso = 0;

	//initial random seed                                                                                                                                               
	srand((unsigned)time(NULL));  

	for(;;)
	{
		if(command_signal.sail_mode_cmd.b2_sailMode == SAIL_MODE_AUTO && command_signal.sail_feedBack.b2_sailTask == SAIL_TASK_ON)
		{
			if(Avoid_Point_Arrive == 1)
			{
				sailTask.u8_PointNum++;
				Avoid_Point_Arrive = 0;
			}

			//����λ��
			lat_local = ins_msg.latitude;
			lng_local = ins_msg.longitude;
			//��һ����λ��
			lat_dst = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude;
			lng_dst = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude;

			OBS.clear();
			int ob_num;
			if(USV_RM_MSG.Obstacles_Num>5)
				ob_num = 5;
			else
				ob_num = USV_RM_MSG.Obstacles_Num;

			for(i=0;i<ob_num;i++)
			{
				obstacle.c_lat	= USV_RM_MSG.RM_radar_Msg[i].obstacle_locate.lat/360000.0;
				obstacle.c_lng	= USV_RM_MSG.RM_radar_Msg[i].obstacle_locate.lng/360000.0;
				obstacle.radius = USV_RM_MSG.RM_radar_Msg[i].obstacle_radius+5.0;		//��ȫ��������30m
				OBS.push_back(obstacle);
			}

			heading_local = ins_msg.heading;

			//����·������
			iret_pso  = obj_path.BuildSceneDataSetWithGPS(lat_local,lng_local,lat_dst,lng_dst,OBS,heading_local);
			if(iret_pso == 0)
			{
			//	SysPubMsgPost("PSO�滮·��ʧ��");
				epv_type = -2;
				sleep_1(4000);
				continue;
			}
			

			local_path = obj_path.GetOptimumGPSPath(epv_type);

			temp_point_lat = local_path.pntlist[1].lat;
			temp_point_lng = local_path.pntlist[1].lng;

			point_sum = local_path.pntlist.size();	//�����ж��Ƿ����ϰ����ں�����

			//�жϽ������Ƿ������һ���ϰ�����, -1 ���ϰ��1 ������Ϸ��� 0 �����㲻�Ϸ� -2 PSO�޽�
			if(epv_type>=0 && point_sum>2)
			{
				switch(epv_type)
				{
				case(1):
				//	SysPubMsgPost("�����������ɵ��ﺽ��");
					break;
				case(0):
				//	SysPubMsgPost("�������������ɵ��ﺽ��");
					break;
				default:
					break;
				}


				
			}

		}
		else;
		sleep_1(4000);
	}
}






uint8 get_radar_moving_obs(void)	//�����Ƿ����ƶ��ϰ���
{
	int loop_i,iret;
	iret = 0;
	for(loop_i=0;loop_i<OBS_NUM;loop_i++)
	{
		if(USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_speed>1.0)	//�ٶȴ���1m/s
		{
			iret = 1;
			moving_obs.obstacle_locate.lat = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lat;
			moving_obs.obstacle_locate.lng = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lng;
			moving_obs.obstacle_direction  = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_direction;
			moving_obs.obstacle_radius	   = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_radius;
			moving_obs.obstacle_speed	   = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_speed	;
			break;
		}
		else
			iret = 0;
	}
	return iret;	
}

double get_velocity_heading(double vx,double vy)	//�����ٶȵķ�λ��
{
	double dir,rtan;
//	uint8  dir_num;
	
	if((vx>=0)&&(vy>=0))
		dir = 0.0;
	else if((vx<0)&&(vy<0))
		dir = 180.0;
	else if((vx>=0)&&(vy<0))
		dir = 180.0;
	else if((vx<0)&&(vy>=0))
		dir = 360.0;

	rtan = atan(vx/vy)*180.0/Pi + dir;
	return rtan;

}




void delay_relay(uint8 *afterdelayflag,uint8 delayflag,int32 *timebuf,int32 timeset)
{
	if(delayflag)
	{
		*timebuf = *timebuf + 1;
		if(*timebuf>=timeset)
		{
			*afterdelayflag = 1;
			*timebuf = timeset;
		}
		else *afterdelayflag = 0;
	}
	else
	{
		*timebuf = 0;
		*afterdelayflag = 0;
	}
}


void ext_relay(uint8 *outflag,uint8 extflag,int32 *timebuf,int32 timeset)
{
	if(extflag) *timebuf = timeset;

	if(*timebuf>0) *timebuf = *timebuf - 1;
	if(*timebuf == 0) 
	{
		*outflag = 0;
	}
	if(*timebuf>0)
	{
		*outflag = 1;
	}
}




//
//uint8 get_radar_collision(void)
//{
//	sint16  TCPA;
//	int32	Safe_DCPA,DCPA;
//	uint8	collision_sign;
//	static uint8 collision_sign_once = 0;
//	uint8	collision_style=0;		//������ʽ 0-������ 1-׷�� 2-�󽻲� 3-�ҽ��� 4-����
//
//	//����RadarĿ��DCPA/TCPA
//	double	Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
//	double	Dst;
//	double	Qu_Deg,Qt_Deg,Qut_Deg;
//
//
//
//
//if(get_radar_moving_obs())
//{
//
//	Vu = (Smart_Navigation_St.USV_Speed*Nmile)/(3600*100);	//���٣���λm/s
//	Qu_Deg = Smart_Navigation_St.USV_Heading/100.0;				//���򣬵�λ��
//	Qu = Radian(Qu_Deg);										//ת��Ϊ����
//	VXu = Vu * sin(Qu);		//x���ٶ�
//	VYu = Vu * cos(Qu);		//y���ٶ�
//
//	Vt = moving_obs.obstacle_speed;
//	Qt_Deg = moving_obs.obstacle_direction;
//	Qt = Radian(Qt_Deg);
//	VXt= Vt * sin(Qt);		//x���ٶ�
//	VYt= Vt * cos(Qt);		//y���ٶ�
//
//	VXut =  VXu -VXt;
//	VYut =  VYu -VYt;
//
//	AQ = Get_AQ(VXut,VYut);
//
//	Vut = sqrt(VXut*VXut + VYut*VYut);		//����ٶ�
//	Qut= atan(VXut/VYut)*180.0/Pi + AQ;			//����ٶȷ��� ��Ҫ������
//
//
//	
//
//
//	
//
//	double lat_obs = moving_obs.obstacle_locate.lat /360000.0;
//	double lng_obs = moving_obs.obstacle_locate.lng /360000.0;
//
//	double lat_local = Smart_Navigation_St.USV_Lng;	//ԭʼ�����ж�ȡ��γ�����Ʒ��ˣ�׼���޸� syp
//	double lng_local = Smart_Navigation_St.USV_Lat;
//
//	Dst = Get_distance(lat_local,lng_local,lat_obs,lng_obs);
//	double AUT= Get_heading(lat_local,lng_local,lat_obs,lng_obs);
//	
//	DCPA = (int32)(abs(Dst*sin(Radian(Qut-AUT))));
//	TCPA = (int32)(Dst*cos(Radian(Qut-AUT))/Vut);
//
//	Safe_DCPA = (int32)(moving_obs.obstacle_radius + 30.0);	//��ȫ����+30m
//
//	//ת����ʾ��
//	{
//		GLB_TCPA = TCPA;
//		GLB_DCPA = DCPA;
//		GLB_Safe_DCPA = Safe_DCPA;
//	}
//
//	if(DCPA > Safe_DCPA || TCPA < 0)
//	{
//		collision_sign = 0;
//	}
//	else
//	{
//		collision_sign = 1;
//	}
//
//	//else if(TCPA < 30)
//	//{
//	//	collision_sign = 1;
//	//}
//	//else
//	//{
//	//	collision_sign = 0;
//	//}
//
//
//	if(collision_sign == 1 && collision_sign_once == 0)
//	{
//		collision_sign_once = 1;
//
//		Qut_Deg = (Qu_Deg-Qt_Deg);
//		if(Qut_Deg > 180)
//			Qut_Deg = Qut_Deg-360;
//	
//		if(Qut_Deg<-180)
//			Qut_Deg=360+Qut_Deg;
//	
//
//		if(Qut_Deg<=30 && Qut_Deg>= -30)	//׷��
//		{
//			//������������
//			radar_avoid_type = 1;
//			Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP - Heading_Con_Angle_Max;//��ת
//		}
//		else if(Qut_Deg>=150 || Qut_Deg<=-150 )//����
//		{
//			//������������
//			radar_avoid_type = 2;
//			Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP + Heading_Con_Angle_Max;//��ת
//		}
//		else	//����
//		{
//			if(Qut_Deg < 0) //������
//			{
//				Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP - Heading_Con_Angle_Max;//��ת
//				radar_avoid_type = 3;
//			}
//			else			//������
//			{
//				Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP + Heading_Con_Angle_Max;//��ת
//				radar_avoid_type = 4;
//			}
//		}
//		//�߽�����
//		if(Radar_Collision_Heading<0)
//			Radar_Collision_Heading = Radar_Collision_Heading+360;
//		if(Radar_Collision_Heading>360)
//			Radar_Collision_Heading = Radar_Collision_Heading-360;
//	}
//	
//}
//else
//{
//	collision_sign = 0;
//	radar_avoid_type = 0;
//}
//
//
//	//����չ��
//	ext_relay(&collision_flag,collision_sign,&collision_timebuf,5);
//	
//	if(collision_flag == 0)
//		collision_sign_once = 0;
//
//
//	return collision_flag;
//}

uint8  collision_flag		=0;
int32  collision_timebuf	=0;
uint8	collision_sign      = 0;
uint8	collision_flag_old	= 0;



uint8 get_radar_collision(void)
{
	sint16  TCPA;
	int32   Safe_DCPA,DCPA;
	uint8   collision_style;
	uint8   is_moving_obs_flag;


	//����RadarĿ��DCPA/TCPA
	double	Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double	Dst;
	double	Qu_Deg,Qt_Deg,Qut_Deg;
	double  AUT;

	double lat_obs;
	double lng_obs;
	double lat_local;
	double lng_local;

	//�ж��Ƿ����ƶ��ϰ���
	if(get_radar_moving_obs())
		is_moving_obs_flag = 1;
	else
		is_moving_obs_flag = 0;

	if(is_moving_obs_flag == 1)
	{
		lat_obs = moving_obs.obstacle_locate.lat /360000.0;
		lng_obs = moving_obs.obstacle_locate.lng /360000.0;
		//Safe_DCPA = (int32)(moving_obs.obstacle_radius + 30.0);	//��ȫ����+30m

		lat_local = Smart_Navigation_St.USV_Lng;	//ԭʼ�����ж�ȡ��γ�����Ʒ��ˣ�׼���޸� syp
		lng_local = Smart_Navigation_St.USV_Lat;

		Dst = Get_distance(lat_local,lng_local,lat_obs,lng_obs);
		AUT = Get_heading(lat_local,lng_local,lat_obs,lng_obs);


		//��������ٶ�
		Vu = (Smart_Navigation_St.USV_Speed*Nmile)/(3600*100);	//���٣���λm/s
		//Qu_Deg = Smart_Navigation_St.USV_Heading/100.0;				//���򣬵�λ��
		//ʹ����������������
		Qu_Deg = Heading_EXP;				//���򣬵�λ��

		Qu = Radian(Qu_Deg);										//ת��Ϊ����
		VXu = Vu * sin(Qu);		//x���ٶ�
		VYu = Vu * cos(Qu);		//y���ٶ�

		Vt = moving_obs.obstacle_speed;
		Qt_Deg = moving_obs.obstacle_direction;
		Qt = Radian(Qt_Deg);
		VXt= Vt * sin(Qt);		//x���ٶ�
		VYt= Vt * cos(Qt);		//y���ٶ�
		VXut =  VXu -VXt;
		VYut =  VYu -VYt;
		AQ = Get_AQ(VXut,VYut);
		Vut = sqrt(VXut*VXut + VYut*VYut);		//����ٶ�
		Qut= atan(VXut/VYut)*180.0/Pi + AQ;			//����ٶȷ��� ��Ҫ������

		DCPA = (int32)(fabs(Dst*sin(Radian(Qut-AUT))));
		TCPA = (int32)(Dst*cos(Radian(Qut-AUT))/Vut);

		float dst_tmp;
		dst_tmp = Radian(Qut-AUT);
		dst_tmp = cos(Radian(Qut-AUT));
		dst_tmp = Dst*cos(Radian(Qut-AUT));
		
		Safe_DCPA = (int32)(moving_obs.obstacle_radius + 30.0);	//��ȫ����+30m
		//Safe_DCPA = 45;

		//��ʾ������
		GLB_TCPA = TCPA;
		GLB_DCPA = DCPA;
		GLB_Safe_DCPA = Safe_DCPA;

		
		if((DCPA < Safe_DCPA) && (TCPA < TCPA_MIN) && (TCPA>0))
		{
			collision_sign = 1;
		}
		else
		{
			collision_sign = 0;
		}

	}
	else
	{
		collision_sign = 0;
	}
	
	collision_flag_old = collision_flag;

	//������־����2s
	ext_relay(&collision_flag,collision_sign,&collision_timebuf,10);


	//���ڳ����ж���ײ����¼����ܺ���
	if(collision_flag == 1 && collision_flag_old == 0)
	{

		Qut_Deg = (Qu_Deg-Qt_Deg);
		if(Qut_Deg > 180)
			Qut_Deg = Qut_Deg-360;
		if(Qut_Deg<-180)
			Qut_Deg=360+Qut_Deg;

		if(Qut_Deg<=30 && Qut_Deg>= -30)	//׷��
		{
			//������������
			radar_avoid_type = 1;
			Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP - Heading_Con_Angle_Max;//��ת
		}
		else if(Qut_Deg>=150 || Qut_Deg<=-150 )//����
		{
			//������������
			radar_avoid_type = 2;
			Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP + Heading_Con_Angle_Max;//��ת
		}
		else	//����
		{
			if(Qut_Deg < 0) //�󽻲�
			{
				Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP - Heading_Con_Angle_Max;//��ת
				radar_avoid_type = 3;
			}
			else			//�ҽ���
			{
				Radar_Collision_Heading = /*Smart_Navigation_St.USV_Heading*0.01*/ Heading_EXP + Heading_Con_Angle_Max;//��ת
				radar_avoid_type = 4;
			}
		}
		//�߽�����
		if(Radar_Collision_Heading<0)
			Radar_Collision_Heading = Radar_Collision_Heading+360;
		if(Radar_Collision_Heading>360)
			Radar_Collision_Heading = Radar_Collision_Heading-360;
	}


	return collision_flag;
}






//ͨѶ�̣߳���ʱ��ţ�CAN��ͨѶ
void *COMM_Task(void *aa)
{
	uint8 task_200ms,task_50ms;
	uint16	task_1s,task_5s,task_60s,State_Num;
	uint16	task_500ms;
	uint32  task_1min;
	State_Num = 0;
	task_200ms = 0;
	task_1min =0;
	task_500ms = 1;

	//д��־����
	uint16	test_num = 0;


	while(1)
	{
		task_1min++;
		task_200ms++;
		task_500ms++;
		
		if(task_500ms >= 10)
		{
			task_500ms = 0;


			//	Send_Sradio_State();			//�������ֵ�̨״̬�ظ������ݼ���
			//	SysLogMsgPost("���ͺ�̨����");
				test_num ++;


		}




		//200ms����һ��
		if(task_200ms>=4)
		{
		task_200ms=0;

		Radio_Sign_old=Radio_Sign;
		Radio_Sign=Check_Radio();//����̨ͨѶ���cxy
	//	Oil_Control();
		Emergency_Stop_Control(0);


		Update_USV_State();//����USV״̬
		
		//���ֵ�̨���ͱ���(ֱ��)
		State_Num++;							//����״̬���ı��
		memset(USV_State_Current,0,State_Current_Num);
		Send_Dradio_State(State_Num);	//���ֵ�̨״̬�ظ������ݼ���

		

		//Ȩ���л���־
		if(Radio_Sign != Radio_Sign_old)
		{
			switch(Radio_Sign){
				case(0): //SysLogMsgPost("Ȩ���л����л���-���ֵ�̨ ");
						 break;
				case(1): //SysLogMsgPost("Ȩ���л����л���-���������̨");
						 break;
				case(2): //SysLogMsgPost("Ȩ���л����л���-������̨ ");
						 break;
				case(3): //SysLogMsgPost("Ȩ���л����л���-������̨ ");
						 break;
				case(4)://SysLogMsgPost("Ȩ���л����л���-������� ");
						 break;
				default:
						 break;

			}
		}
		}
		if(task_1min>=1240)
		{
			task_1min = 0;
			//Send_BD_State();				//��������
		}

		//50ms ����
		{
			Heartbeat_Num++;	//�����������ı��
/*			Send_CAN_Cfg();
			Send_Accelerator(Heartbeat_Num);//���ڷ��ͷ���������ָ��
			Send_CAN_Control();//����CAN����ָ��
			Send_CAN_State();//����CAN״̬��Ϣ
20180201*/
			//ͨѶ���ӣ�CANͨѶ���ͼ���



			//�����ã�����UDP���ͱ��ļ���
			//memset(&sockid_test_buf[0],0,sizeof(sockid_test_buf));
			//sockid_test_buf[0] = 0xEB;
			//sockid_test_buf[1] = 0x91;
			//sockid_test_buf[2] =  Radio_Sign;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[0].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[1].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[2].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[3].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[4].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[3] =  USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Stable_Platform_Power;
			//sockid_test_buf[18] = 0x91;
			//sockid_test_buf[19] = 0xEB;
			//
			//if(sockid_test>0)
			//	test_udp_send((uint8*)&sockid_test_buf[0],sizeof(sockid_test_buf),sockid_test);

		}

		
#ifdef WINNT
		sleep_1(50);
#else
		sleep_1(50); 
#endif
		
	}

}
