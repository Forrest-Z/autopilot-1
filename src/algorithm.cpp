/*
* algorithm.c --�����㷨
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"
#include "../include/usv_include.h"


#define	Deviation_Dst_Right				70			//�һ�ת�뾶
#define	Deviation_Dst_Left				60			//���ת�뾶
#define	Deceleration_Dst				30			//���پ���
#define Dst_arrive						10			//�ﵽ����
#define Dst_temp						45.0		//��ʱ����ת������
#define Dst_temp_last_stop				70			//���յ�ͣ�����پ���
#define Dst_Slow_Down					85			//���پ���
#define TRACK_ERROR_TH					20			//����ƫ����ֵ
#define TH10M							10
#define TH10M1D							1
#define	TH20M							20
#define TH20M3D							3
#define TH30M							30
#define TH30M5D							5
#define TH70M							70
#define	TH70M10D						10
#define	TRACK_CORRCTION_ANGEL			10		//����ƫ�����

const double e=0.081813369f; 

uint8   Sailing_Mod = 0;
uint8	Sailing_Mod_old =0;

uint8	Sail_Count=0;
uint32	Stop_Time_Count=0;
uint8	Stop_Time_Sign=0;
int Compensating_Dst[255];
double heading_Safe_Min1=0.0,heading_Safe_Max1=0.0,heading_Safe_Min2=0.0,heading_Safe_Max2=0.0;
FILE *PFile;	
int speed_sign,heading_sign,Intellingent_sign;
double Auto_Return(uint8 type);
void Get_Accelerator_L();
void Get_Rudder_L();
void Get_Gear_L(uint8 type);		
void Get_Accelerator_R();
void Get_Rudder_R();
void Get_Gear_R(uint8 type);	
void Rudder_Limit();
void Heading_PID2(double heading);
void Speed_PID2(double speed);
void Get_Compensating_Dst();
double Get_Yaw_Dst();
void Get_Radar_DCPA_TCPA(double speed,double heading,uint8 OBS_Count,double*DCPA,double *TCPA);//��ȡ�״��ϰ�Ŀ�����ײ����
int cmp ( const void *a , const void *b);
void Get_lat_lng(double lat,double lng,double dst,double heading,double *Lat,double *Lng);
int Check_Recovery();
int Check_Obstacle();
void Yaw_correction(double Yaw_Dst,double DST);
int Obstacle_Test();
double Get_Obstacle_Dst(int count);
void  Set_Accelerator_Deceleration(float multi_idling);		// ���ٵ��ٱ���
void Remote_Control();
int Automatic_Return();
void Cruise_control(int heading_sign);
void Directional_Cruise(int Speed_sign);
int Automatic_Navigation();
void Update_Parameter();
int Get_Rudder();
int Get_Accelerator();

//���ܺ����㷨
void *Intelligent_Navigation(void *aa)
{
uint8 Intellingent_Switch;		//������������־ 0--���� 1--���������� 2--����������ͣ
uint8	Intellingent_Mod;		//����ģʽ��0--�ֶ�ģʽ ��1--���Զ� 2--ȫ�Զ���3--����ģʽ�����٣�
uint8	Speed_Const;			// ���ٺ��б�־ 1--���ٺ���  0--
uint8    Heading_Const;			//�����б�־ 1--������
uint8    Auto_Return_Const;		//�Զ�������һ��������־
uint8    U_speed=0;				//���� ���ӹߵ��ı����л�ȡ����ȡʱ *100 
int32	 Return_Point_sign=0;			//��һ�ν�����Զ���־
int32  Auto_Return_sign1=0;				//��һ�ν���ȫ�Զ���־
double Dst=0.0;
	
//	int Recovery_sign=0;
	while(1)
	{
		memset((uint8*)&Radar_OBS_Msg.Obs_num[0],0,sizeof(Radar_OBS_Msg));		//���״��ϰ������Ϣ
		memset((uint8*)&Path_Coordinate_Msg.lat[0],0,sizeof(Path_Coordinate_Msg));		//���Ϲ滮������
		Intellingent_Mod=Check_Intelligent_Mod();					// ���ݿ�����̨����ָ����ء�0--�ֶ�ģʽ ��1--���Զ� 2--ȫ�Զ���3--����ģʽ�����٣�
		Intellingent_Switch=Check_Intelligent_Switch();				//�����������жϣ���ȫ�Զ�ģʽ�£����������Ƿ��·����������ֵ�̨��0-���� ��1-������������2-����������ͣ
		Speed_Const=Check_Speed_Const();							//���ݳ��ػ�����·�������ٺ����ж�
		Heading_Const=Check_Heading_Const();						//���ݳ��ػ�����·�����������ж�
		Auto_Return_Const=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod;		//�Զ�������һ������
		Con_Rudder();									//�ó���ı��� û���� ���ݺ��ټ�����ƶ�� ��ö��Rudder_Con_Angle�����ڶ�Ŀ������Ƽ��� ���ó���û����
		
		//test
		Intellingent_Mod = 1;
		Speed_Const = 1;
		//test end
		if(Intellingent_Mod!=1)										//0--�ֶ�ģʽ ��1--���Զ� 2--ȫ�Զ���3--����ģʽ�����٣�
			Return_Point_sign=0;									//��һ�ν�����Զ�
		if((Intellingent_Mod!=2)||(Auto_Return_Const!=1))
			Auto_Return_sign1=0;									//��һ�ν���һ������
		if((Intellingent_Mod!=1)||(Speed_Const!=1))
			speed_sign=0;											//��һ�ν��붨�ٺ���
		if((Intellingent_Mod!=1)||(Heading_Const!=1))
		{
			heading_sign=0;											//��һ�ν��붨����
		}
		if((Intellingent_Mod!=2)||(Intellingent_Switch==0))
			Intellingent_sign=0;									//��һ�ν���ȫ�Զ�ģʽ
		
		if(Intellingent_Mod==0)			//�ֶ�ģʽ
		{
			Remote_Control();					//���ݿ����豸�·��Ĳ�����������ҷ��������� ����� ����λ
			sleep_1(Algorithm_Samp_Time);
			continue;
		}
		else if(Intellingent_Mod==1)			//�ж��Ƿ�Ϊ������ģʽ��
		{
			if(0==Return_Point_sign)			//��1�ν�����Զ�����ʼ������
			{
				Get_Return_Point();				//���˴��������񣬻�����˴��趨����ʼλ�ã�0��  �͵�ǰ��λ��(���ȣ�ά�ȣ�����Դ�ڹߵ�ϵͳ��ϸ��Ϣ
				Update_Parameter();				//�������ļ��л��PID��������
				Return_Point_sign=1;			//�����һ�ν����־
			}
			if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Set_Return_Point_Mod)			//���÷�����
				Get_Return_Point();							//������˴� ������˴��趨����ʼλ�ã�0�� �͵�ǰ��λ��

			if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod)		//һ������cxy
			{
				if(Automatic_Return()==1)			//�Զ����� ����㶨���� 1
				{
					sleep_1(Algorithm_Samp_Time);
					continue;
				}
			}
			if(2==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod)
			{
				USV_State.USV_Current_Sailing=0x00;											//USV״̬
			}			
			if(1==Speed_Const){							//����
				Cruise_control(Heading_Const);			//Heading_Const --����Ѳ����־
			}
			if(1==Heading_Const)						//����
			{
				Directional_Cruise(Speed_Const);		//Speed_Const --����Ѳ����־ 1--����
			}
			if((1!=Heading_Const)&&(1!=Speed_Const))		//���Ƕ���ͬʱ���Ƕ���
				Remote_Control();							//��� �� ���ؿ���
			sleep_1(Algorithm_Samp_Time);
			continue;
		}
		else if(Intellingent_Mod==2)//�ж��Ƿ�Ϊ����ģʽ��
		{
			
			if((0==USV_State.USV_Sailing_Intel_Sign)||(3==USV_State.USV_Sailing_Intel_Sign))	//�ж��Ƿ��к������� ��û�к�������
			{
				Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);			//���� *8
				Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
				Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
				Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
				Rudder_L=0x7d;//0���
				Rudder_R=0x7d;
				Sail_Count=0;
				sleep_1(Algorithm_Samp_Time);
				continue;
			}
			else
			{
				if(0==Intellingent_Switch)					//0--���٣�
				{
					Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//����
					Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
					Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
					Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
					Rudder_L=0x7d;//0���
					Rudder_R=0x7d;
					sleep_1(Algorithm_Samp_Time);
					continue;
				}
				if(1==Intellingent_Switch)				//1--�ж��Ƿ��յ�����������
				{
					if(Automatic_Navigation()==1)			//�Զ������㷨
					{
						sleep_1(Algorithm_Samp_Time);
						continue;
					}
				}
				if(2==Intellingent_Switch)				//����������ͣ
				{
					if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod)			//һ������cxy
					{
						USV_State.USV_Current_Sailing=0x03;
						if(Auto_Return_sign1==0)
						{
							memset ( (uint8 *)&vPID.SetPoint,0,sizeof(vPID));
							memset((uint8 *)&vSpd.SetPoint,0,sizeof(vSpd));
							memset ( (uint8 *)&rPID.SetPoint,0,sizeof(rPID));
							memset((uint8 *)&rSrd.SetPoint,0,sizeof(rSrd));
						}
						Auto_Return_sign1=1;
//Algorithm_Cfg0 ��	�ٶȺ���PID �������ļ����	
						vPID.Proportion = Dradio_Config.Algorithm_Cfg[1].PID_P/((float)100.0);             	//�趨��ʼPֵ
						vPID.Integral   = Dradio_Config.Algorithm_Cfg[1].PID_I/((float)100.0);            	 	//�趨��ʼIֵ
						vPID.Derivative = Dradio_Config.Algorithm_Cfg[1].PID_D/((float)100.0);             	//�趨��ʼDֵ
				
						vSpd.k1=Dradio_Config.Algorithm_Cfg[1].S_K1/((float)10.0);							//���ϵ��
						vSpd.k2=Dradio_Config.Algorithm_Cfg[1].S_K2/((float)10.0);							//���仯��ϵ��
						vSpd.k3=Dradio_Config.Algorithm_Cfg[1].S_K3/((float)100.0);//0~1֮��				//ѧϰ��
						vSpd.k4=8.0;//40kn/5v																/�������ϵ��
						rPID.Proportion = Dradio_Config.Algorithm_Cfg[0].PID_P/((float)100.0);             	//�趨��ʼPֵ
						rPID.Integral   = Dradio_Config.Algorithm_Cfg[0].PID_I/((float)100.0);            	 	//�趨��ʼIֵ
						rPID.Derivative = Dradio_Config.Algorithm_Cfg[0].PID_D/((float)100.0);             	//�趨��ʼDֵ
//�ٶȺ���S����� �������ļ����						
						rSrd.k1=Dradio_Config.Algorithm_Cfg[0].S_K1/((float)10.0);
						rSrd.k2=Dradio_Config.Algorithm_Cfg[0].S_K2/((float)10.0);
						rSrd.k3=Dradio_Config.Algorithm_Cfg[0].S_K3/((float)100.0);//0~1֮��
						rSrd.k4=(float)8.0;//40kn/5v
						
						Dst=Auto_Return(0);									//���ص�ǰ�㵽������֮��ľ���
						//if(Dst<Compensating_Dst[Sailing_Cnt_Old])			
						if(Dst<Dst_arrive)
						{
							Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//����
							Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
							Rudder_L=0x7d;									//125  =  0���
							Rudder_R=0x7d;
							Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
							Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;	
							
							USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod=0;
							sleep_1(Algorithm_Samp_Time);
							continue;
						}
						if(Dst<Deceleration_Dst)
						{
							if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)
								Heading_PID2(Heading_EXP);							//���ݷ����ƫ���PID���㷨������Ҷ�Ŀ��ƶ��Rudder_L��Rudder_R
							else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)
								Heading_S(Heading_EXP);								//���ݺ���ƫ��������Ҷ��Rudder_L��Rudder_R
							if(Accelerator_L>=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3)))  //�󷢶������� ���󷢶�������ת��
								Accelerator_L=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3));
							else
								Accelerator_L=(int)(Accelerator_L*(float)0.6);
							if(Accelerator_R>=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3)))	//�ҷ���������
								Accelerator_R=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3));
							else
								Accelerator_R=(int)(Accelerator_R*(float)0.6);	
							Get_Gear_L(1);							//����õ��󷢶�����λ
							Get_Gear_R(1);							//����õ��ҷ�������λ
							sleep_1(Algorithm_Samp_Time);
							continue;
						}
						Calculate_Accelerator_Rudder();						
						sleep_1(Algorithm_Samp_Time);			//Algorithm_Samp_Time=200
						continue;
							//cxy
					}

					Get_Accelerator_L();				//���ݿ����������õ��󷢶������� Accelerator_L
					Get_Rudder_L();
					Get_Accelerator_R();
					Get_Rudder_R();
					Get_Gear_L(1);						//����õ��󷢶�����λ
					Get_Gear_R(1);						//����õ��ҷ�������λ	

					Sail_Count=0;
//					printf("Suspend sailing task!-%d-%d\n",Sailing_Cnt_Old,Gear_L);				

					if(U_speed>USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit)//���ٶ�Ӧת��
					{
						Accelerator_L = (int)(Accelerator_L*(float)Acc_Coefficient);//����ת�ٻ򷭸�����cxy
						Accelerator_R = (int)(Accelerator_R*(float)Acc_Coefficient);
					}
					//Rudder_Limit();//����ʱ�������
				
					USV_State.USV_Sailing_Intel_Sign=1;				//����������ͣ
					sleep_1(Algorithm_Samp_Time);
					continue;
				}
			}
		}
		else if(Intellingent_Mod==3)				//����ģʽ����ͬ
		{
			Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//����
			Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
			Rudder_L=0x7d;//0���
			Rudder_R=0x7d;
		}
		sleep_1(Algorithm_Samp_Time);
	}
	return ((void *)0);
}

//������ҷ��������� ����� ����λ
void Remote_Control()
{
	double	U_speed;
	int		control_rudder;		//ҡ��λ�ö�Ӧ��� 0~250
	int		control_accelerator;
	
	int		trans_l_rudder;		//
	int		trans_r_rudder;		//
	

	U_speed=Smart_Navigation_St.USV_Speed*0.01;			//USV_Speed--�ӹߵ��ı����л�ȡ����ȡʱ *100 

	
	//�������ģʽ
	if(USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_mode == 0 && USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_mode == 0)
	{
		//������
		Get_Rudder_L();										//���ݿ��������������Rudder_L
		Get_Gear_L(0);										//����õ��󷢶�����λ
		Get_Rudder_R();
		Get_Gear_R(0);	
		Get_Accelerator_L();								//�����������������õ��󷢶������� Accelerator_L
		Get_Accelerator_R();
	}
	else		//ƽ����תģʽ
	{
		control_rudder = Get_Rudder();				//ƽ�ƶ�ǵ���
		control_accelerator = Get_Accelerator();	//ƽ�����ŵ���

		if(USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_mode == 1)	//ƽ��ģʽ
		{
			if(1==USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_command) //��ƽ��
			{
				trans_l_rudder = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].rudder_L;
				trans_r_rudder = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].rudder_R;
				
				if(control_rudder>=125)
				{
					Rudder_L = trans_l_rudder + (control_rudder-125)/61.0 * (186-trans_l_rudder);
					Rudder_R = trans_r_rudder + (control_rudder-125)/61.0 * (186-trans_r_rudder);
				}
				else
				{
					Rudder_L = trans_l_rudder - (125-control_rudder)/63.0 * (trans_l_rudder-62);
					Rudder_R = trans_r_rudder - (125-control_rudder)/63.0 * (trans_r_rudder-62);
				}

				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].gear_R;
				Accelerator_L =  monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].accelerator_L<<3;
				Accelerator_R =  (monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0].accelerator_R - control_accelerator)<<3;

			}
			else if(2==USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.translation_command)	//��ƽ��
			{
				//
				trans_l_rudder = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].rudder_L;
				trans_r_rudder = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].rudder_R;

				if(control_rudder>=125)
				{
					Rudder_L = trans_l_rudder + (control_rudder-125)/61.0 * (186-trans_l_rudder);
					Rudder_R = trans_r_rudder + (control_rudder-125)/61.0 * (186-trans_r_rudder);
				}
				else
				{
					Rudder_L = trans_l_rudder - (125-control_rudder)/63.0 * (trans_l_rudder-62);
					Rudder_R = trans_r_rudder - (125-control_rudder)/63.0 * (trans_r_rudder-62);
				}

				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].gear_R;
				Accelerator_L = (monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].accelerator_L - control_accelerator)<<3;
				Accelerator_R =monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[1].accelerator_R<<3;
			}
			else	//ֹͣ
			{
				/*Rudder_L = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].rudder_L;
				Rudder_R = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].rudder_R;
				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].gear_R;
				Accelerator_L =monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].accelerator_L<<3;
				Accelerator_R =monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[2].accelerator_R<<3;	*/		
				//ƽ��ģʽ��ֹͣ״̬��Ȼ���Կ��� 2017��8��28�� 19:29:56
				Get_Rudder_L();										//���ݿ��������������Rudder_L
				Get_Gear_L(0);										//����õ��󷢶�����λ
				Get_Rudder_R();
				Get_Gear_R(0);	
				Get_Accelerator_L();								//�����������������õ��󷢶������� Accelerator_L
				Get_Accelerator_R();
			}
		}
		if(USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_mode == 1)	//��תģʽ
		{
			if(1==USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_command)	//��ʱ����ת
			{
				//
				Rudder_L = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_L;
				Rudder_R = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_R;
				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].gear_R;
				Accelerator_L =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].accelerator_L<<3;
				Accelerator_R =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].accelerator_R<<3;

			}
			else if(2 == USV_Control.USV_Control_Message[Radio_Sign].trans_rot_cmd.rotation_command) //˳ʱ����ת
			{
				//
				Rudder_L = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].rudder_L;
				Rudder_R = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].rudder_R;
				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].gear_R;
				Accelerator_L =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].accelerator_L<<3;
				Accelerator_R =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[1].accelerator_R<<3;
			}
			else	//ֹͣ
			{
				Rudder_L = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].rudder_L;
				Rudder_R = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].rudder_R;
				Gear_L   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].gear_L;
				Gear_R   = monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].gear_R;
				Accelerator_L =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].accelerator_L<<3;
				Accelerator_R =monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[2].accelerator_R<<3;
			}				
		}
	}

	
	if(U_speed>USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit)		//��ǰ�ٶȳ������� ���ٶ�Ӧת�� Radio_Sign--��ǰ���ƴ�������
	{
		Accelerator_L=(int)(Accelerator_L*(float)Acc_Coefficient);//����ת�� ȡ0.95
		Accelerator_R=(int)(Accelerator_R*(float)Acc_Coefficient);
	}
	//Rudder_Limit();//����ʱ�������
/*			printf("Radio_Sign=%d\n",Radio_Sign);
	printf("Gear_Left=%d\n",USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left);
	printf("Gear_Right=%d\n",USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right);
	printf("Rudder_Angle_Left=%d\n",USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left);
	printf("Rudder_Angle_Right=%d\n",USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Right);
*/					
}

//�Զ�����
int Automatic_Return()
{
	double Dst;
	USV_State.USV_Current_Sailing=0x03;	
	Dst=Auto_Return(1);								//���2���ľ��룻��������ֵ,�ٶ�����ֵ
	if(Dst<Compensating_Dst[Sailing_Cnt_Old])		//����Ŀ�ĵ�,
	{
		Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//���� �󷢶������ſ���ת��
		Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);	//�ҷ��������ſ���ת��
		Rudder_L=0x7d;												//��0��� 125
		Rudder_R=0x7d;												//�Ҷ�� 125
		Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;			//�󷢶�����λ����λ�յ��Ƕ�
		Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			//�ҷ�������λ���ҵ�λ�յ��Ƕ�
		
		USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Return_Mod=0;	//����Ŀ�ĵ�
		return 1;
	}
	if(Dst<Deceleration_Dst)							//С�ڼ��پ��� 30		
	{
		if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)			//��������㷨���� PID�㷨
			Heading_PID2(Heading_EXP);								//���ݷ����ƫ���PID���㷨������Ҷ�Ŀ��ƶ��
		else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)		//S���㷨
			Heading_S(Heading_EXP);
		if(Accelerator_L>=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3)))		//����ת�� *16
			Accelerator_L=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3));
		else
			Accelerator_L=(int)(Accelerator_L*(float)0.6);											//���ű�Ϊ 0.6 ��
		if(Accelerator_R>=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3)))
			Accelerator_R=(uint16)(2.0*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3));
		else
			Accelerator_R=(int)(Accelerator_R*(float)0.6);											//���ű�Ϊ 0.6 ��
		Get_Gear_L(1);				//���ݷ�����ת�ټ�����λ
		Get_Gear_R(1);				//���ݷ�����ת�ټ����ҵ�λ			
		return 1;
	}
//����������
	Calculate_Accelerator_Rudder();					//�������õ��㷨��PID��S�������������ת�� �� ���	
	return 1;
}

//����Ѳ�� 
//Heading_Const --����Ѳ����־
void Cruise_control(int Heading_sign_input)
{
	if(speed_sign==0)						//�Ƿ�Ϊ��һ�ν���
	{
		Speed_EXP=USV_State.Dradio_USV_Sailing_State.USV_Speed*0.01;		//�ٶ�����ֵȡ�ߵ��ٶ� ���ߵ��ٶ� *100��
		speed_sign=1;							//���һ�ν����־
	}
	if(Speed_EXP>USV_State.USV_Speed_limit)
		Speed_EXP=USV_State.USV_Speed_limit;
//				printf("speed_sign=%d-Speed_EXP=%f\n",speed_sign,Speed_EXP);
	if(Dradio_Config.Algorithm_Cfg[1].Control_Type==0)		//��PID�㷨�����ٶ�
	{
		Speed_PID2(Speed_EXP);
//					printf("Speed_EXP=%f-%f-%d-%d-%d-%d-\n",Speed_EXP,vPID.Proportion,Accelerator_L/8,Accelerator_R/8,Gear_L,Gear_R);
	}
	//	Speed_PID(Speed_EXP,&Accelerator_L,&Accelerator_R);
	else if (Dradio_Config.Algorithm_Cfg[1].Control_Type==1)		//��S��������㷨�����ٶ�
		Speed_S(Speed_EXP);
//				printf("Speed_EXP=%f-%d\n",Speed_EXP,Accelerator_L/8);
	if(1!=Heading_sign_input)					//���Ƕ���Ѳ��
	{
		Get_Rudder_L();				//�������ͳ��ص��趨ֵ��ö��
		Get_Rudder_R();
		Get_Gear_L(1);				//���ݵ�ǰת�ٻ�õ�λ
		Get_Gear_R(1);							
	}
}

//������
void Directional_Cruise(int Speed_sign_input)
{
	double	U_speed;
	U_speed=Smart_Navigation_St.USV_Speed*0.01;
	double heading_exp_cal;
	//heading_exp_cal = Heading_EXP;

	if(1!=Speed_sign_input)				//���Ƕ��ٺ���
	{
		Get_Accelerator_L();		//�������ͳ��ص��趨ֵ���ת��
		Get_Gear_L(0);				//�������ͳ��ص��趨ֵ��õ�λ
		Get_Accelerator_R();
		Get_Gear_R(0);						
	}

//				USV_State.Dradio_USV_Sailing_State.USV_Heading=1000;
	if(heading_sign==0)					//��һ�ν��붨���� 
	{
		Heading_EXP=USV_State.Dradio_USV_Sailing_State.USV_Heading*0.01;		//����������
		heading_sign=1;								//���һ�ν����־
	}
	

	//��̬����Ͷ��
	radar_obstacle_sign =	get_radar_collision();

	//���붯̬����
	if(1==radar_obstacle_sign)
		heading_exp_cal = Radar_Collision_Heading;
	else
		heading_exp_cal = Heading_EXP;



//				printf("%d-heading_sign=%d-Heading_EXP=%f\n",Dradio_Config.Algorithm_Cfg[0].Control_Type,heading_sign,Heading_EXP);
	if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)					//����PID�㷨 ������������
	{
//					Heading_PID(Heading_EXP,&Rudder_L,&Rudder_R);
		Heading_PID2(heading_exp_cal);
/*		if((rPID.Error>10.0)||(rPID.Error<-10.0))
		{
			if((U_speed>USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit)||(U_speed>20.0))
			{
				Accelerator_L*=Acc_Coefficient;//����ת�ٻ򷭸�����cxy
				Accelerator_R*=Acc_Coefficient;
			}
		}*/
	}
	else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)				//����S��������㷨 ������������
	{
		Heading_S(heading_exp_cal);
/*		if((rSrd.Error>10.0)||(rSrd.Error<-10.0))
		{
			if((U_speed>USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit)||(U_speed>20.0))
			{
				Accelerator_L*=Acc_Coefficient;//����ת�ٻ򷭸�����cxy
				Accelerator_R*=Acc_Coefficient;
			}
		}*/
	}
	
//				printf("Heading_EXP=%f-0x%.4x-0x%.4x-0x%.2x-0x%.2x\n",Heading_EXP,Accelerator_L,Accelerator_R,Rudder_L,Rudder_R);

}

//�Զ�����
int  Automatic_Navigation()
{
double Dst,Yaw_Dst,avoid_Dst;
static double Yaw_Dst_Max=0.0;
uint32 Stop_Time;
static int32 Yaw_Count=0;

	if(Sail_Count<(3000/Algorithm_Samp_Time))			
	{
		Sail_Count++;
		return 1;
	}
//3��ִ��1��
	if(Intellingent_sign==0)				//��1�ν����Զ�����ʼ������
	{
//��PID�Ĺ��̲�����0
		memset ( (uint8 *)&vPID.SetPoint,0,sizeof(vPID));
		memset((uint8 *)&vSpd.SetPoint,0,sizeof(vSpd));
		memset ( (uint8 *)&rPID.SetPoint,0,sizeof(rPID));
		memset((uint8 *)&rSrd.SetPoint,0,sizeof(rSrd));
//Algorithm_Cfg1
//�ٶȺ���PID �������ļ����
		vPID.Proportion = Dradio_Config.Algorithm_Cfg[1].PID_P/((float)100.0);             	//�趨��ʼPֵ
		vPID.Integral   = Dradio_Config.Algorithm_Cfg[1].PID_I/((float)100.0);            	//�趨��ʼIֵ
		vPID.Derivative = Dradio_Config.Algorithm_Cfg[1].PID_D/((float)100.0);             	//�趨��ʼDֵ
//�ٶȺ���S����� �������ļ����		
		vSpd.k1=Dradio_Config.Algorithm_Cfg[1].S_K1/((float)10.0);							//���ϵ��
		vSpd.k2=Dradio_Config.Algorithm_Cfg[1].S_K2/((float)10.0);							//���仯��ϵ��
		vSpd.k3=Dradio_Config.Algorithm_Cfg[1].S_K3/((float)100.0);//0~1֮��				//ѧϰ��
		vSpd.k4=8.0;//40kn/5v						//�������ϵ��
//Algorithm_Cfg0*
//��ǵ� �ٶȺ���PID �������ļ����
		rPID.Proportion = Dradio_Config.Algorithm_Cfg[0].PID_P/((float)100.0);             	//�趨��ʼPֵ
		rPID.Integral   = Dradio_Config.Algorithm_Cfg[0].PID_I/((float)100.0);            	 	//�趨��ʼIֵ
		rPID.Derivative = Dradio_Config.Algorithm_Cfg[0].PID_D/((float)100.0);             	//�趨��ʼDֵ
//�ٶȺ���S����� �������ļ����		
		rSrd.k1=Dradio_Config.Algorithm_Cfg[0].S_K1/((float)10.0);						//���ϵ��
		rSrd.k2=Dradio_Config.Algorithm_Cfg[0].S_K2/((float)10.0);						//���仯��ϵ��
		rSrd.k3=Dradio_Config.Algorithm_Cfg[0].S_K3/((float)100.0);//0~1֮��			//ѧϰ��
		rSrd.k4=((float)8.0);//40kn/5v													//�������ϵ��
	}
	
	Intellingent_sign=1;				//���һ�����Զ����б�־
	Dst=Calculate_Speed_Heading();		//���ݵ�ǰλ�ã�Ŀ��λ�� �����ٶ�����ֵ �ͺ�������ֵ�����룬Speed_EXP=15--�ٶ�����ֵ Heading_EXP--��������ֵ
	Dst*=Nmile;							//ת��Ϊ��
//					printf("Start sailing task!-%f-%f-%f-%f-%d\n",Yaw_Dst_Max,Yaw_Dst,Dst,Heading_EXP,Compensating_Dst[Sailing_Cnt_Old]);									
	Dst_monitor = (float)Dst;	//��ص�Ŀ�ĵصľ���

	USV_State.USV_Sailing_Intel_Sign=2;			//�Զ�����		
	if(Sailing_Cnt_Old>=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber)	//����ִ����ϣ�����ߵ����ִ����ϡ�
	{
		Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//����
		Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
		Rudder_L=0x7d;						//0���
		Rudder_R=0x7d;
		Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
		Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
		Sailing_Cnt_Old=1;
		USV_State.USV_Sailing_Intel_Sign=3;					//�����������
		USV_State.Sailing_Nummber=0;
		return 1;
	}
	//NoStopPoint_Sign=Obstacle_Test();					//�ϰ���־

	//��̬����Ͷ��
	//if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0].bak_32_1 == 1)
	if(1)
	{
		if(epv_type >= 0 && point_sum>2)
			Obstacle_Sign = 1;
		else
			Obstacle_Sign = 0;
	}
	else
	{
		Obstacle_Sign = 0;
	}
	if(Obstacle_Sign == 1)	//
	{
		avoid_Dst = Calculate_Speed_Heading_tem();					//�����ٶ�7��
		Calculate_Accelerator_Rudder_temp();			//����ת�ٺͶ��
	//	Gear_L=0xA0;//60,D dang		
	//	Gear_R=0xA0;
		Get_Gear_L(1);
		Get_Gear_R(1);

		if(avoid_Dst < Dst_temp && epv_type== 0)
		{
			Avoid_Point_Arrive = 1;
		}
		return 1;
	}
	
	
	//�ޱ���������������
	
	////�ж��Ƿ�����ʱ����
	//if(USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Stop_Time == 0)
	//	NoStopPoint_Sign= 1;
	//else
	//	NoStopPoint_Sign= 0;
	NoStopPoint_Sign= 1;

	if(NoStopPoint_Sign==1)	//��ͣ������
	{
		//todo ������ʱ�����
		//Calculate_Speed_Heading_tem();

		//��ʱͣ����70�״� ����
		if(Sailing_Cnt_Old==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber)
		{
			if(Dst < Dst_temp_last_stop)
			{
				Speed_EXP= 3;
			}
		}

		if(Dst<Dst_Slow_Down)
		{
			if(Speed_EXP>=5.0)
				Speed_EXP = 5.0;
			else
				;
		}

		if(Dst<Dst_temp)
		{							
			Sailing_Cnt_Old++;							//�������+1
		}
		Calculate_Accelerator_Rudder_temp();			//����ת�ٺͶ��
		Get_Gear_L(1);
		Get_Gear_R(1);
		//Gear_L=0xA0;//60,D dang		
		//Gear_R=0xA0;	
		return 1;
	}

	if(NoStopPoint_Sign==0)						//ͣ������
	{
		if(1==Stop_Time_Sign)					//�뺽�����С��Ԥ��ֵ��ͣ����־
		{
			Dst=Calculate_Speed_Heading();	//���ݵ�ǰλ�ã�Ŀ��λ�� �����ٶ�����ֵ �ͺ�������ֵ���������롣
			Dst*=Nmile;						//ת������
			Stop_Time=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Stop_Time;  //����ͣ��ʱ��
			Stop_Time*=(1000/Algorithm_Samp_Time);//ת��Ϊ���룬����ת��������ִ�м��
			Stop_Time_Count++;						//��ʱ
			if(Stop_Time_Count>=Stop_Time)			//����ͣ����ʱʱ�䵽
			{
				Yaw_Dst_Max=0.0;					//
				Stop_Time_Sign=0;					//���뺽�����С��Ԥ��ֵ��ͣ����־
				Sailing_Cnt_Old++;					//�������+1
			}
			else									//�����ڴ���״̬
			{
				Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);//����
				Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
				Rudder_L=0x7d;//0���
				Rudder_R=0x7d;
				Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
				Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
			}
			//if(Dst>Compensating_Dst[Sailing_Cnt_Old])			//�ж�Ʈ�����㷶Χ�Զ��غ���
			if(Dst>Dst_arrive)
				Calculate_Accelerator_Rudder_temp();					//���㺽��								
			return 1;
		}
		//if(Dst<(Compensating_Dst[Sailing_Cnt_Old]))				//С��ʮ�ף�������
		if(Dst<Dst_arrive)
		{
			Stop_Time_Sign=1;									//��Ŀ������С��Ԥ��ֵ								
			return 1;
		}

		if(Dst<(Deceleration_Dst+Dst_arrive))			//�����������cxy 30��
		{
			if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)					//PID�㷨
				Heading_PID2(Heading_EXP);
			else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)
				Heading_S(Heading_EXP);
			Set_Accelerator_Deceleration(1.2);									//1.2 ������
			//if(Accelerator_L>=(uint16)(1.2*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3)))		//1.2 ������
			//	Accelerator_L=(uint16)(1.2*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3));
			//if(Accelerator_R>=(uint16)(1.2*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3)))
			//	Accelerator_R=(uint16)(1.2*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3));
			Get_Gear_L(1);		
			Get_Gear_R(1);	
			
			return 1;
		}
		if(Dst<(Deceleration_Dst*1.5+Dst_arrive))
		{
			if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)
				Heading_PID2(Heading_EXP);
			else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)
				Heading_S(Heading_EXP);
			Set_Accelerator_Deceleration(2);									//2������

			//if(Accelerator_L>=(uint16)(2*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3)))		//2������
			//	Accelerator_L=(uint16)(2*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3));
			//if(Accelerator_R>=(uint16)(2*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3)))
			//	Accelerator_R=(uint16)(2*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3));
			Get_Gear_L(1);		
			Get_Gear_R(1);	
			
			return 1;
		}	
		////�Ƴ���������
		//Yaw_Dst=Get_Yaw_Dst();						//Yaw_Dst ????  AIS�����㷨
		//if(Yaw_Dst_Max<fabs(Yaw_Dst))
		//	Yaw_Dst_Max=fabs(Yaw_Dst);
		//if((fabs(Yaw_Dst))>10)
		//	Yaw_correction(Yaw_Dst,Dst);			//����AIS ���� �޸ĺ�������ֵ
		//Yaw_Count++;
		//if(Yaw_Count==2000/Algorithm_Samp_Time)		//��ʱ2��
		//{
		//	if(Yaw_Dst>10)
		//	{
		//	//	Yaw_correction();
		//	}	
		//}
		Calculate_Accelerator_Rudder();			//����ת�ٺͶ��

		Get_Gear_L(1);
		Get_Gear_R(1);

	//	Gear_L=0xA0;//60,D dang		
	//	Gear_R=0xA0;	
	}	
	return 0;
}

void  Set_Accelerator_Deceleration(float multi_idling)		// ���ٵ��ٱ���
{
	if(Accelerator_L>=(uint16)(multi_idling*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3)))		//multi_idling������
		Accelerator_L=(uint16)(multi_idling*((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3));
	if(Accelerator_R>=(uint16)(multi_idling*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3)))
		Accelerator_R=(uint16)(multi_idling*((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3));
}


double Get_Obstacle_Dst(int count)
{
	double lat1,lng1,lat2,lng2,lat3,lng3,Dst1,Dst2,Dst3,Heading1,Heading2,Heading_error,Yaw_Dst,Perimeter,Measure;
	Dst1=0.0;
	Dst2=0.0;
	Dst3=0.0;
	lat1=Smart_Navigation_St.USV_Lat;
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//����Ϊ��
	lng1=Smart_Navigation_St.USV_Lng;
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;//��γΪ��
	lng2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Degree;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Minute/60.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Second/3600.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Sign)
		lng2=0-lng2;//����Ϊ��
	lat2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Degree;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Minute/60.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Second/3600.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Sign)
		lat2=0-lat2;//��γΪ��
	lng3=Radar_OBS_Msg.lat[count];
	lat3=Radar_OBS_Msg.lng[count];

//	printf("lat1=%f,lng1=%f,lat2=%f,lng2=%f\n",lat1,lng1,lat2,lng2);	
	Dst1=Get_distance(lng1,lat1 , lng2, lat2);
	Dst2=Get_distance(lng1,lat1 , lng3, lat3);
	Dst3=Get_distance(lng3,lat3 , lng2, lat2);
	Heading1=Get_heading(lng2,lat2 , lng1, lat1);
	Heading2=Get_heading(lng3,lat3 , lng1, lat1);
	Perimeter=(Dst1+Dst2+Dst3)/2.0;
	Measure=sqrt(Perimeter*(Perimeter-Dst1)*(Perimeter-Dst2)*(Perimeter-Dst3));
	Yaw_Dst=2*Measure/Dst1;
	Heading_error=Heading1-Heading2;
	if(Heading_error>180.0)
		Heading_error=Heading_error-360.0;
	if(Heading_error<-180.0)
		Heading_error=360.0+Heading_error;
	if(Heading_error<0)
		Yaw_Dst=0.0-Yaw_Dst;
	return Yaw_Dst;
}


//����AIS ���� �޸ĺ�������ֵ
void Yaw_correction(double Yaw_Dst,double DST)
{
	double heading_error,speed_c;
	if(Smart_Navigation_St.USV_Speed>10)
	speed_c=0.6;
	if(Yaw_Dst>0)//��ת
	{
		if(Yaw_Dst>=43*speed_c)
			heading_error=45.0;
		else
		{
	//		heading_error=asin(Yaw_Dst/DST);
			heading_error=asin(Yaw_Dst/(65*speed_c));//
			heading_error=heading_error*180.0/Pi;			
		}
		Heading_EXP+=heading_error;
		if(Heading_EXP>360.0)
			Heading_EXP-=360.0;
	}
	else//��ת
	{
		Yaw_Dst=0.0-Yaw_Dst;
		if(Yaw_Dst>=43*speed_c)
			heading_error=45.0;
		else
		{
	//		heading_error=asin(Yaw_Dst/DST);
			heading_error=asin(Yaw_Dst/(65*speed_c));
			heading_error=heading_error*180.0/Pi;
	
		}
		Heading_EXP-=heading_error;
		if(Heading_EXP<0.0)
			Heading_EXP+=360.0;		
	}
	
}

//�Զ�����
//���2���ľ��룻
//��������ֵ
//�ٶ�����ֵ
//����2���ľ���
double Auto_Return(uint8 type)
{
	double lat1,lng1,lat2,lng2,Dst;
	Dst=0.0;
	lat1=Smart_Navigation_St.USV_Lat;					//��ǰ��λ�þ���
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//����Ϊ��
	lng1=Smart_Navigation_St.USV_Lng;					//��ǰ��λ��ά��
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;//��γΪ��
//�����ľ���
	lat2=(double)Point_Return[type].Waypoint_Latitude_Degree;
	lat2+=(double)Point_Return[type].Waypoint_Latitude_Minute/60.0;
	lat2+=(double)Point_Return[type].Waypoint_Latitude_Second/3600.0;
	lat2+=(double)Point_Return[type].Waypoint_Latitude_Decimal/360000.0;
	if(0==Point_Return[type].Waypoint_Latitude_Sign)
		lat2=0-lat2;//����Ϊ��
//������ά��
	lng2=(double)Point_Return[type].Waypoint_Longitude_Degree;
	lng2+=(double)Point_Return[type].Waypoint_Longitude_Minute/60.0;
	lng2+=(double)Point_Return[type].Waypoint_Longitude_Second/3600.0;
	lng2+=(double)Point_Return[type].Waypoint_Longitude_Decimal/360000.0;
	if(0==Point_Return[type].Waypoint_Longitude_Sign)
		lng2=0-lng2;//��γΪ��
	Dst=Get_distance(lng1,lat1 , lng2, lat2);							//��2���ľ���
	Heading_EXP=Get_heading(lng2,lat2 , lng1, lat1);					//��ú��з���	����ֵ
	Speed_EXP=USV_State.USV_Speed_limit;								//���˴������� �ٶ�����ֵ ����ֵ
	return Dst;
}

//�������ļ��л��PID��������
void Update_Parameter()
{
	memset ( (uint8 *)&vPID.SetPoint,0,sizeof(vPID));
	memset((uint8 *)&vSpd.SetPoint,0,sizeof(vSpd));
	memset ( (uint8 *)&rPID.SetPoint,0,sizeof(rPID));
	memset((uint8 *)&rSrd.SetPoint,0,sizeof(rSrd));
	vPID.Proportion = Dradio_Config.Algorithm_Cfg[1].PID_P/((float)100.0);             	//�趨��ʼPֵ��Dradio_Config --�������ļ�����
	vPID.Integral   = Dradio_Config.Algorithm_Cfg[1].PID_I/((float)100.0);            	 	//�趨��ʼIֵ
	vPID.Derivative = Dradio_Config.Algorithm_Cfg[1].PID_D/((float)100.0);             	//�趨��ʼDֵ
	
	vSpd.k1=Dradio_Config.Algorithm_Cfg[1].S_K1/((float)10.0);
	vSpd.k2=Dradio_Config.Algorithm_Cfg[1].S_K2/((float)10.0);
	vSpd.k3=Dradio_Config.Algorithm_Cfg[1].S_K3/((float)100.0);//0~1֮��
	vSpd.k4=8.0;																		//40kn/5v
	rPID.Proportion = Dradio_Config.Algorithm_Cfg[0].PID_P/((float)100.0);             	//�趨��ʼPֵ
	rPID.Integral   = Dradio_Config.Algorithm_Cfg[0].PID_I/((float)100.0);            	 	//�趨��ʼIֵ
	rPID.Derivative = Dradio_Config.Algorithm_Cfg[0].PID_D/((float)100.0);             	//�趨��ʼDֵ
	
	rSrd.k1=Dradio_Config.Algorithm_Cfg[0].S_K1/((float)10.0);
	rSrd.k2=Dradio_Config.Algorithm_Cfg[0].S_K2/((float)10.0);
	rSrd.k3=Dradio_Config.Algorithm_Cfg[0].S_K3/((float)100.0);//0~1֮��
	rSrd.k4=8.0;//40kn/5v
}

//���ݸ�����Դ�Ŀ����������õ��󷢶������� Accelerator_L
void Get_Accelerator_L()
{
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Left>=100)						//�󷢶�������,��Դ�ڿ�����̨
		Accelerator_L=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Left-100;		//����󷢶�������
	else if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Left<100)
		Accelerator_L=100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Left;
	Accelerator_L=(int)(Accelerator_Coefficient_L*Accelerator_L);
	Accelerator_L+=SR_Config_Msg.Motor_Idling_Speed_L_spn520221*0x0f;
	Accelerator_L=(Accelerator_L<<3);			//*8
	Accelerator_L+=15*8;						//120  ���������� ��+
}

//���ݿ����������õ��ҷ��������� Accelerator_R
void Get_Accelerator_R()
{
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right>=100)
		Accelerator_R=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right-100;
	else if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right<100)
		Accelerator_R=100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right;
	Accelerator_R=(int)(Accelerator_Coefficient_R*Accelerator_R);
	Accelerator_R+=SR_Config_Msg.Motor_Idling_Speed_R_spn520223*0x0f;
	Accelerator_R=(Accelerator_R<<3);

}

//���ݿ��������������Rudder_L
void Get_Rudder_L()
{
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left>=100)
	{
		Rudder_L=(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left-100);
		Rudder_L=(int)(Rudder_L*Rudder_Coefficient_L*2.5+125);						//��125Ϊ0
	}
	else
	{
		Rudder_L=(100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left);
		Rudder_L=(int)(125-Rudder_L*Rudder_Coefficient_L*2.5);
	}
}

//���ݿ�����������Ҷ��Rudder_R
void Get_Rudder_R()
{
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Right>=100)
	{
		Rudder_R=(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Right-100);
		Rudder_R=(int32)(Rudder_R*Rudder_Coefficient_R*2.5+125);
	}
	else
	{
		Rudder_R=(100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Right);
		Rudder_R=(int32)(125-Rudder_R*Rudder_Coefficient_R*2.5);
	}
}


int Get_Rudder()	//���ݿ��������������Rudder ����ƽ��
{
	int iret_Rudder;
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left>=100)
	{
		iret_Rudder=(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left-100);
		iret_Rudder=(int)(iret_Rudder*Rudder_Coefficient_L*2.5+125);						//��125Ϊ0
	}
	else
	{
		iret_Rudder=(100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Rudder_Angle_Left);
		iret_Rudder=(int)(125-iret_Rudder*Rudder_Coefficient_L*2.5);
	}
	return iret_Rudder;
}


int Get_Accelerator()	//���ݿ���������㷢����ת��ֵ ����ƽ��
{
	int iret_Accelerator;
	//if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right>=100)
	//	iret_Accelerator=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right-100;
	//else if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right<100)
	//	iret_Accelerator=100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right;
	//iret_Accelerator=(int)(Accelerator_Coefficient_R*iret_Accelerator);
	//iret_Accelerator+=SR_Config_Msg.Motor_Idling_Speed_R_spn520223*0x0f;
	//iret_Accelerator=(iret_Accelerator<<3);
	iret_Accelerator = (100 -USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Accelerator_Right )*2;	// -200 ~ + 200
	return iret_Accelerator;
}




//type --�㷨
//����õ��󷢶�����λ
void Get_Gear_L(uint8 type)
{
	if(type==0)
	{
		if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left>=100)		//�󷢶�����λ
		{
			if(SR_Config_Msg.Gear_Direction_L_spn520250==0)								//��λ���� 0--ǰ������
				Gear_L=(int32) ((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left-100)*Gear_Coefficient_L_F);
			else if(SR_Config_Msg.Gear_Direction_L_spn520250==1)					//��λ���� 1--����
				Gear_L=(int32)((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left-100)*Gear_Coefficient_L_B);
			Gear_L=Gear_L+SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;				// �����Ƕ�+��λ�յ��Ƕ�(�������ļ���ã�
		}
		else if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left<100)			
		{
			if(SR_Config_Msg.Gear_Direction_L_spn520250==0)
				Gear_L=(int32)((100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left)*Gear_Coefficient_L_B);
			else if(SR_Config_Msg.Gear_Direction_L_spn520250==1)
				Gear_L=(int32)((100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Left)*Gear_Coefficient_L_F);
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248-Gear_L;				//��λ�յ��Ƕ�-�����Ƕ�
		}
	}
	if(type==1)
	{
		Gear_L=(int32)((Accelerator_L/8-SR_Config_Msg.Motor_Idling_Speed_L_spn520221*0x0f)/Accelerator_Coefficient_L);
		Gear_L=(int32)(Gear_L*Gear_Coefficient_L_F);
		Gear_L+=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
	}
//���е�λ����
	if(Gear_L>=(SR_Config_Msg.Gear_Neutral_Angle_L_spn520248+SR_Config_Msg.Gear_Forward_Angle_L_spn520244))
		Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248+SR_Config_Msg.Gear_Forward_Angle_L_spn520244;
	if(Gear_L<=(SR_Config_Msg.Gear_Neutral_Angle_L_spn520248-SR_Config_Msg.Gear_Back_Angle_L_spn520246))
		Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248-SR_Config_Msg.Gear_Back_Angle_L_spn520246;			
	
}

//����õ��ҷ�������λ
void Get_Gear_R(uint8 type)
{
	if(type==0)
	{
		if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right>=100)
		{
			if(SR_Config_Msg.Gear_Direction_R_spn520251==0)
				Gear_R=(int32)((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right-100)*Gear_Coefficient_R_F);
			else if(SR_Config_Msg.Gear_Direction_R_spn520251==1)
				Gear_R=(int32)((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right-100)*Gear_Coefficient_R_B);
			Gear_R=Gear_R+SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;			
		}
		else if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right<100)
		{
			if(SR_Config_Msg.Gear_Direction_R_spn520251==0)
				Gear_R=(int32)((100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right)*Gear_Coefficient_R_B);
			else if(SR_Config_Msg.Gear_Direction_R_spn520251==1)
				Gear_R=(int32)((100-USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Drive.Gear_Right)*Gear_Coefficient_R_F);
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249-Gear_R;		
		}
	}
	if(type==1)
	{
		Gear_R=(int32)((Accelerator_R/8-SR_Config_Msg.Motor_Idling_Speed_R_spn520223*0x0f)/Accelerator_Coefficient_R);
		Gear_R=(int32)(Gear_R*Gear_Coefficient_R_F);
		Gear_R+=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;
	}
	if(Gear_R>=(SR_Config_Msg.Gear_Neutral_Angle_R_spn520249+SR_Config_Msg.Gear_Forward_Angle_R_spn520245))
		Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249+SR_Config_Msg.Gear_Forward_Angle_R_spn520245;
	if(Gear_R<=(SR_Config_Msg.Gear_Neutral_Angle_R_spn520249-SR_Config_Msg.Gear_Back_Angle_R_spn520247))
		Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249-SR_Config_Msg.Gear_Back_Angle_R_spn520247;			
}

//���ƶ��
void Rudder_Limit()
{
	double U_speed;
	U_speed=Smart_Navigation_St.USV_Speed*0.01;
	if(U_speed>30.0)//����30kn��Ҫ�������
	{
		if(Rudder_L<(65+U_speed))//30kn����ת�����Ϊ12��45knΪ6��
			Rudder_L=(int32)(65+U_speed);
		if(Rudder_L>(185-U_speed))
			Rudder_L=(int32)(185-U_speed);
		if(Rudder_R<(65+U_speed))//����ת�����Ϊ10��
			Rudder_R=(int32)(65+U_speed);
		if(Rudder_R>(185-U_speed))
			Rudder_R=(int32)(185-U_speed);				
	}
}

//��ú��ߵ���Χ
void Get_Compensating_Dst()
{
	double lat1,lng1,lat2,lng2;
	double heading_x,heading_y;
	uint8 count=0,num;
	int	heading_exp[255],heading_error;
	memset((uint8 *)&Compensating_Dst,0,sizeof(Compensating_Dst));
	memset((uint8 *)&heading_exp,0,sizeof(heading_exp));
	
	num=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber;		//��������
	for(count=0;count<num+1;count++)
	{
		lng1=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Latitude_Degree;
		lng1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Latitude_Minute/60.0;
		lng1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Latitude_Second/3600.0;
		lng1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Latitude_Decimal/360000.0;		//������ľ���
		if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Latitude_Sign)
			lng1=0-lng1;//����Ϊ��
		lat1=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Longitude_Degree;
		lat1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Longitude_Minute/60.0;
		lat1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Longitude_Second/3600.0;
		lat1+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Longitude_Decimal/360000.0;		//������ά��
		if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count].Waypoint_Longitude_Sign)
			lat1=0-lat1;//��γΪ��
		
		lng2=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Latitude_Degree;				//��һ����ľ���
		lng2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Latitude_Minute/60.0;
		lng2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Latitude_Second/3600.0;
		lng2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Latitude_Decimal/360000.0;
		if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Latitude_Sign)
			lng2=0-lng2;//����Ϊ��
		lat2=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Longitude_Degree;				//��һ�����ά��
		lat2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Longitude_Minute/60.0;
		lat2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Longitude_Second/3600.0;
		lat2+=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Longitude_Decimal/360000.0;
		if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count+1].Waypoint_Longitude_Sign)
			lat2=0-lat2;//��γΪ��
		heading_exp[count]=(int)(Get_heading(lng1,lat1 , lng2, lat2));						//��������ķ���
		if(count>0)
		{
			heading_error=heading_exp[count]-heading_exp[count-1];					//����ƫ��
			if(heading_error>180)
				heading_error=heading_error-360;
			if(heading_error<-180)
				heading_error=360+heading_error;
			heading_x=Radian(heading_error);							//���Ƕȱ�ɻ���
			heading_y=Radian(180-heading_error);						//���Ƕȱ�ɻ���

			if(heading_error>=0)//��ת								//Deviation_Dst_Right--�һ�ת�뾶 ,Deviation_Dst_Left--���ת�뾶
			{
				Compensating_Dst[count]=(int32)(fabs(Deviation_Dst_Right*(sin(heading_x)*sin(heading_x/2.0)/(sin(heading_y)*sin(heading_y/2.0)))));
			}
			else
			{
				Compensating_Dst[count]=(int32)(fabs(Deviation_Dst_Left*(sin(heading_x)*sin(heading_x/2.0)/(sin(heading_y)*sin(heading_y/2.0)))));
			}
			printf("Compensating_Dst[%d]=%d\n",count,Compensating_Dst[count]);
		}
	}

}

//���������㷨��Ĭ�ϵ�ǰ��Ϊ���
void Sailing_Intelligent_algorithm(double *dst,uint8 sailing_cnt)
{

	//�˲��㷨Filter_abc(int n,double *x,Filter_a,Filter_b,Filter_c);
	*dst=Calculate_Speed_Heading();				//���ݵ�ǰλ�ã�Ŀ��λ�� �����ٶ�����ֵ �ͺ�������ֵ������
	//���ټ���ת��cxy,���Dst<50m,PID����
	//����ͣ��ʱ���ʱ
	//����PID�㷨,������������
	//����PID�㷨������ʱ3000rpm���ٶ��ȶ���΢��ת��
	//���ﺽ��ͣ����ͣ��
	//S����Ʒ�
	return;
}
//abc�˲�
void Filter_abc(int n,double *x,double a,double b,double c)
{ 
	int i;
	double s1,ss,v1,vv,a1,aa,t;
	aa=0.0; vv=0.0;ss=0.0;
	for (i=0; i<=n-1; i++)
	{ 
		t=(i+1)*Dradio_T;
		s1=ss+t*vv+t*t*aa/2.0;
		v1=vv+t*aa; 
		a1=aa;
		ss=s1+a*(x[i]-s1);
		vv=v1+b*(x[i]-s1);
		aa=a1+2.0*c*(x[i]-s1)/(t*t);
	}
	return;
}


//���ݵ�ǰλ�ã�Ŀ��λ�� �����ٶ�����ֵ �ͺ�������ֵ���������롣
//�����Speed_EXP=15--�ٶ�����ֵ Heading_EXP--��������ֵ ����������
double Calculate_Speed_Heading_tem()
{
	double lat1,lng1,lat2,lng2,Dst;
	Dst=0.0;
	lat1=Smart_Navigation_St.USV_Lat;					//��ǰ����
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//����Ϊ��
	lng1=Smart_Navigation_St.USV_Lng;					//��ǰά��
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;//��γΪ��
	////��һ�����е��λ��
	lng2 = temp_point_lat;
	lat2 = temp_point_lng;	


	Dst=Get_distance(lng1,lat1 , lng2, lat2);						//���ݾ��Ⱥ�ά�ȣ�����2��ľ��� ��λΪ ��
	Dst=Dst/Nmile;													//�������׻��㱶��ֵ ��ת��Ϊ����
	Speed_EXP=	7;//USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_speed/100.0;		//��������֮��ĺ����ٶȣ��������صõ�
	//Speed_EXP=15.0;													//�̶ֹ�Ϊ 15����
	if(Speed_EXP>USV_State.USV_Speed_limit)							//�Ƿ񳬹�����
		Speed_EXP=USV_State.USV_Speed_limit;
	Heading_EXP=Get_heading(lng1,lat1 , lng2, lat2);				//���ݵ�ǰ�㣬��Ŀ�����㺽������ֵ
	return Dst;
}

//���㺽��ƫ�����
//ֱ�������
void tracking_correction(void)
{
	double angle_exp,angle_real,angle_diff;
	double local_distance;

	if(Sailing_Cnt_Old <= 1)
	{
		track_control.heading_change = 0;	//��һ����ǰ��ʹ�ú�������
		return;
	}
	//����ת��
	//��һ�����е��λ��
	track_control.wp_next_lati=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Degree;
	track_control.wp_next_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Minute/60.0;
	track_control.wp_next_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Second/3600.0;
	track_control.wp_next_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Sign)
		track_control.wp_next_lati=0-track_control.wp_next_lati;//����Ϊ��
	track_control.wp_next_lngi=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Degree;
	track_control.wp_next_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Minute/60.0;
	track_control.wp_next_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Second/3600.0;
	track_control.wp_next_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Sign)
		track_control.wp_next_lngi=0-track_control.wp_next_lngi;//��γΪ��
	//��һ�����е��λ��
	track_control.wp_last_lati=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Degree;
	track_control.wp_last_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Minute/60.0;
	track_control.wp_last_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Second/3600.0;
	track_control.wp_last_lati+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Sign)
		track_control.wp_last_lati=0-track_control.wp_last_lati;//����Ϊ��
	track_control.wp_last_lngi=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Degree;
	track_control.wp_last_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Minute/60.0;
	track_control.wp_last_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Second/3600.0;
	track_control.wp_last_lngi+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Sign)
		track_control.wp_last_lngi=0-track_control.wp_last_lngi;//��γΪ��
	//����λ��
	track_control.local_lngi=Smart_Navigation_St.USV_Lat;					//��ǰ����
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		track_control.local_lngi=0-track_control.local_lngi;				//����Ϊ��
	track_control.local_lati=Smart_Navigation_St.USV_Lng;					//��ǰά��
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		track_control.local_lati=0-track_control.local_lati;				//��γΪ��
	

	angle_exp  = Get_heading(track_control.wp_last_lati ,track_control.wp_last_lngi,track_control.wp_next_lati,track_control.wp_next_lngi);	//������һ���㵽��һ���㺽���
	angle_real = Get_heading(track_control.wp_last_lati ,track_control.wp_last_lngi,track_control.local_lati,track_control.local_lngi);	//������һ���㵽����λ�õĺ����
	angle_diff = angle_exp - angle_real;		//����ǶȲ�

	if(angle_diff > 180 || angle_diff < -180)
	{
		track_control.heading_change = 0;
		return;
	}

	local_distance = Get_distance(track_control.wp_last_lati,track_control.wp_last_lngi,track_control.local_lati,track_control.local_lngi);	//���㺽�о���
	track_control.track_error = local_distance * sin(Radian(angle_diff));

	//if(track_control.track_error > TRACK_ERROR_TH)	//������� 20��Ϊ�ż�
	//{
	//	track_control.heading_change =  -TRACK_CORRCTION_ANGEL;
	//	return;
	//}
	//if (track_control.track_error < -TRACK_ERROR_TH)
	//{
	//	track_control.heading_change =	TRACK_CORRCTION_ANGEL;
	//	return;
	//}
	
	
	if(track_control.track_error > TH70M)
	{
		track_control.heading_change = -TH70M10D;
		return;
	}
	if(track_control.track_error > TH30M)
	{
		track_control.heading_change = -TH30M5D;
		return;
	}
	if(track_control.track_error > TH20M)
	{
		track_control.heading_change = -TH20M3D;
		return;
	}
	if(track_control.track_error > TH10M)
	{
		track_control.heading_change = -TH10M1D;
		return;
	}
	
	//��ƫ
	if(track_control.track_error < -TH70M)
	{
		track_control.heading_change = TH70M10D;
		return;
	}
	if(track_control.track_error < -TH30M)
	{
		track_control.heading_change = TH30M5D;
		return;
	}
	if(track_control.track_error < -TH20M)
	{
		track_control.heading_change = TH20M3D;
		return;
	}
	if(track_control.track_error < TH10M)
	{
		track_control.heading_change = TH10M1D;
		return;
	}


	track_control.heading_change =  0;
	return;
}



//���ݵ�ǰλ�ã�Ŀ��λ�� �����ٶ�����ֵ �ͺ�������ֵ���������롣
//�����Speed_EXP=15--�ٶ�����ֵ Heading_EXP--��������ֵ ����������
double Calculate_Speed_Heading()
{
	double lat1,lng1,lat2,lng2,Dst;
	Dst=0.0;
	lat1=Smart_Navigation_St.USV_Lat;					//��ǰ����
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//����Ϊ��
	lng1=Smart_Navigation_St.USV_Lng;					//��ǰά��
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;//��γΪ��
//��һ�����е��λ��
	lng2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Degree;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Minute/60.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Second/3600.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Sign)
		lng2=0-lng2;//����Ϊ��
	lat2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Degree;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Minute/60.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Second/3600.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Sign)
		lat2=0-lat2;//��γΪ��
//	printf("lat1=%f,lng1=%f,lat2=%f,lng2=%f\n",lat1,lng1,lat2,lng2);	
	Dst=Get_distance(lng1,lat1 , lng2, lat2);						//���ݾ��Ⱥ�ά�ȣ�����2��ľ��� ��λΪ ��
	Dst=Dst/Nmile;													//�������׻��㱶��ֵ ��ת��Ϊ����
	//Speed_EXP= 10;
	Speed_EXP =((double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_speed)/100.0;		//��������֮��ĺ����ٶȣ��������صõ�
	//Speed_EXP=15.0;													//�̶ֹ�Ϊ 15����
	if(Speed_EXP>USV_State.USV_Speed_limit)							//�Ƿ񳬹�����
		Speed_EXP=USV_State.USV_Speed_limit;
	Heading_EXP=Get_heading(lng1,lat1 , lng2, lat2);				//���ݵ�ǰ�㣬��Ŀ�����㺽������ֵ

	//��������
	tracking_correction();
	Heading_EXP -= track_control.heading_change;
	if(Heading_EXP > 360)
		Heading_EXP = Heading_EXP - 360;
	if(Heading_EXP < 0)
		Heading_EXP = Heading_EXP + 360;

	return Dst;
}

//���㺽��(������)
//��1�㾭�Ⱥ�ά�� ����2�㾭�Ⱥ�ά��
//fStarPtx --ά�� ��fStarPty--����
//��� ������Ƕ� ��λΪ��
double Get_heading(double fStarPtx,double fStarPty,double fEndPtx,double fEndPty)

{
	double delta_fy=fEndPtx-fStarPtx;					//���Ȳ�
	double delta_lnmg=fEndPty-fStarPty;					//ά�Ȳ�
	//���Ȳ�ӦС��180��
	if(delta_lnmg < -180.0)
		delta_lnmg += 360.0;
	if(delta_lnmg > 180.0)
		delta_lnmg -= 360.0;
	//delta_lnmg > 0.0 �� �� ---> ��  delta_lnmg < 0.0 �� �� ---> ��
	uint8 bGoEast=_FALSE,bGoNorth=_FALSE;
	if(delta_lnmg >= 0.0)
		bGoEast=_TRUE;								//��
	else
		bGoEast=_FALSE;								//����
	//delta_fy > 0.0 �� �� ---> ��  delta_fy < 0.0 �� �� ---> ��
	if(delta_fy>=0.0)
		bGoNorth=_TRUE;								//��
	else
		bGoNorth=_FALSE;							//����
     
	if(delta_fy==0)									//��ͬһ������
	{
		if(delta_lnmg==0)							//��ͬһά����
			return 0;
		return bGoEast?90:270;						// 90--�� ��270 ����
	}
	double d1=7915.7045*(e/2*log10((1-e*sin(fStarPtx*Pi/180))/(1+e*sin(fStarPtx*Pi/180)))+log10(tan((45+fStarPtx/2)*Pi/180.0)));//γ�Ƚ�����
	double d2=7915.7045*(e/2*log10((1-e*sin(fEndPtx*Pi/180))/(1+e*sin(fEndPtx*Pi/180)))+log10(tan((45+fEndPtx/2)*Pi/180.0)));//γ�Ƚ�����
	double delta_d=d2-d1;////γ�Ƚ����ʲ�(��)
	double dbDir=atan(delta_lnmg*60/delta_d)*180/Pi;			//���� ��λΪ��
	if(!bGoEast&&bGoNorth)
		dbDir=360+dbDir;
	if(!bGoEast&&!bGoNorth)
		dbDir=180+dbDir;
	if(bGoEast&&!bGoNorth)
		dbDir=180+dbDir;
	return dbDir;
}

//����ת�ٺͶ��
uint8 Calculate_Accelerator_Rudder()
{
//	float rot;
//	rot=(Smart_Navigation_St.USV_ROT-10000)/100;
	if(Dradio_Config.Algorithm_Cfg[1].Control_Type==0)			//PID�����㷨�����������ټ���ת��
//		Speed_PID(Speed_EXP,&Accelerator_L,&Accelerator_R);
		Speed_PID2(Speed_EXP);									//�����ٶ�����ֵ PID�����ٶ�
	else if (Dradio_Config.Algorithm_Cfg[1].Control_Type==1)	//��S�����������ת��
		Speed_S(Speed_EXP);										//�����ٶ�����ֵ S������������ٶ�
	if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)			//PID�����㷨���ݷ��������ֵ������
	{
//		Heading_PID(Heading_EXP,&Rudder_L,&Rudder_R);
		Heading_PID2(Heading_EXP);								//���ݷ�������ֵ����PID�㷨������
		if((rPID.Error>30.0)||(rPID.Error<-30.0))				//����Χ
		{
/*			Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)*8);//��������㷽��ת��
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
			Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)*8);
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;*/
			Accelerator_L=(1000*8);//��������㷽��ת��
			Get_Gear_L(1);			//��õ�λ
			Accelerator_R=(1000*8);
			Get_Gear_R(1);	
		}
	}
	else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)				//���ݷ�������ֵ����S��������㷨������
	{
		Heading_S(Heading_EXP);											//���ݷ�������ֵ����S���㷨������
		if((rPID.Error>15.0)||(rPID.Error<-15.0))
		{
			Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)*8);//����
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
//			Gear_L+=SR_Config_Msg.Gear_Forward_Angle_L_spn520244*0.3;
			Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)*8);
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;
//			Gear_R+=SR_Config_Msg.Gear_Forward_Angle_R_spn520245*0.3;			
		}
	}

	return 0;
}

//����ת�ٺͶ��
uint8 Calculate_Accelerator_Rudder_temp()
{
//	float rot;
//	rot=(Smart_Navigation_St.USV_ROT-10000)/100;
	if(Dradio_Config.Algorithm_Cfg[1].Control_Type==0)			//PID�����㷨�����������ټ���ת��
//		Speed_PID(Speed_EXP,&Accelerator_L,&Accelerator_R);
		Speed_PID2(Speed_EXP);									//�����ٶ�����ֵ PID�����ٶ�
	else if (Dradio_Config.Algorithm_Cfg[1].Control_Type==1)	//��S�����������ת��
		Speed_S(Speed_EXP);										//�����ٶ�����ֵ S������������ٶ�
	if(Dradio_Config.Algorithm_Cfg[0].Control_Type==0)			//PID�����㷨���ݷ��������ֵ������
	{
//		Heading_PID(Heading_EXP,&Rudder_L,&Rudder_R);
		Heading_PID2(Heading_EXP);								//���ݷ�������ֵ����PID�㷨������

		if((rPID.Error>60.0)||(rPID.Error<-60.0))				//����Χ60������
		{	
			Speed_PID2(2.0);	// 2���ٶ�ת��
			Get_Gear_L(1);
			Get_Gear_R(1);
		}
		else if((rPID.Error>30.0)||(rPID.Error<-30.0))				//����Χ
		{
/*			Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)*8);//��������㷽��ת��
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
			Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)*8);
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;*/
			//Accelerator_L=(1000*8);//��������㷽��ת��
			//Get_Gear_L(1);			//��õ�λ
			//Accelerator_R=(1000*8);
			//Get_Gear_R(1);	
			Speed_PID2(Speed_EXP*0.3);	
			Get_Gear_L(1);
			Get_Gear_R(1);
		}
	}
	else if(Dradio_Config.Algorithm_Cfg[0].Control_Type==1)				//���ݷ�������ֵ����S��������㷨������
	{
		Heading_S(Heading_EXP);											//���ݷ�������ֵ����S���㷨������
		if((rPID.Error>15.0)||(rPID.Error<-15.0))
		{
			Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)*8);//����
			Gear_L=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
//			Gear_L+=SR_Config_Msg.Gear_Forward_Angle_L_spn520244*0.3;
			Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)*8);
			Gear_R=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;
//			Gear_R+=SR_Config_Msg.Gear_Forward_Angle_R_spn520245*0.3;			
		}
	}

	return 0;
}

//����ת��Ϊ����PID�㷨
void Heading_PID(double heading)
{
	float Increment;
	Increment=0.0;
	rPID.SetPoint =(float)heading;                 	//����ʵ������趨
	rPID.Actual=(float)(Smart_Navigation_St.USV_Heading*0.01);               		//�õ���ǰ����
	rPID.Error =rPID.SetPoint- rPID.Actual;   	//���趨ֵ�Ƚϣ��õ����ֵ
	if(rPID.Error>180.0)
		rPID.Error=rPID.Error-360;
	else if (rPID.Error<-180.0)
		rPID.Error=rPID.Error+360;		
	if((rPID.Error<=1.0)&&(rPID.Error>=-1.0))
		return;
	rPID.Ec=rPID.Error-rPID.LastError;
	Increment=rPID.Proportion*(rPID.Error-rPID.LastError)+rPID.Integral*rPID.Error+rPID.Derivative*(rPID.Error-2*rPID.LastError+rPID.PreError);
	rPID.PreError=rPID.LastError;
	rPID.LastError=rPID.Error;
//	vPID.Proportion=fuzzy_kp(vPID.Error/5,vPID.Ec);       //E��������5 
//	vPID.Integral=fuzzy_ki(vPID.Error/5,vPID.Ec); 
//	vPID.Derivative=fuzzy_kd(vPID.Error/5,vPID.Ec); 
/*	if(Increment>Rudder_Con_Angle)//ÿ�ζ������ܳ��������ƶ��
		Increment=Rudder_Con_Angle;
	if((Increment<0)&&(Increment<(0-Rudder_Con_Angle)))
		Increment=0-Rudder_Con_Angle;//cxy
*/	
	Rudder_L+=(int32)((Increment)*2.5);
	Rudder_R+=(int32)((Increment)*2.5);
//	printf("Rudder=%f-%f-%f-%d-%d\n",rPID.SetPoint,rPID.Actual,Increment,Rudder_L,Rudder_R);
	
	if(Rudder_L>((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=(int32)((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125);
	if(Rudder_L<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)))
		Rudder_L=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5));
	if((Rudder_L<((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L*2.5)+125))&&(Rudder_L>(125-Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L*2.5)))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=0x7d;	
	if(Rudder_R>Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125)
		Rudder_R=(int32)(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125);
	if(Rudder_R<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5)))
		Rudder_R=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5));
	if((Rudder_R<((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R*2.5)+125))&&(Rudder_L>(125-Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R*2.5)))//��������ǶȲ��ܳ��������ƶ��
		Rudder_R=0x7d;	
	
}

//����ת��Ϊ����S���㷨
//���ݺ���ƫ��������Ҷ��Rudder_L��Rudder_R
void Heading_S(double heading)
{
	float dIncrement;

	dIncrement=0.0;//�̶�����

	rSrd.SetPoint=(float)heading;
	rSrd.Actual=(float)(Smart_Navigation_St.USV_Heading*0.01);
	rSrd.Error=rSrd.SetPoint-rSrd.Actual;					//ƫ��
	if(rPID.Error>180.0)
		rPID.Error=rPID.Error-360;
	else if (rPID.Error<-180.0)
		rPID.Error=rPID.Error+360;		
	
	if((rSrd.Error<=1.0)&&(rSrd.Error>=-1.0))
		return;
	rSrd.dError=(float)((rSrd.Error-rSrd.LastError)/0.05);//��������Ϊ50ms ��ƫ���΢��
	rSrd.out=(float)(2.0/(1.0+exp(-rSrd.k1*rSrd.Error-rSrd.k2*rSrd.dError))-1.0+dIncrement);		//�������
	rSrd.eout=rSrd.Lastout+rSrd.k4*rSrd.Error;											//������� ��rSrd.Lastout--ǰһ�εĿ������
	//����k1��k2ֵ
	rSrd.k1+=rSrd.k3*(rSrd.eout-rSrd.Lastout)*2*(-rSrd.k1*rSrd.Error-rSrd.k2*rSrd.dError)*rSrd.Error;
	rSrd.k1=(float)(rSrd.k1/(pow((1.0+exp(-rSrd.k1*rSrd.Error-rSrd.k2*rSrd.dError)),2)));				//���ϵ��
	rSrd.k2+=rSrd.k3*(rSrd.eout-rSrd.Lastout)*2*(-rSrd.k1*rSrd.Error-rSrd.k2*rSrd.dError)*rSrd.dError;	
	rSrd.k2=(float)(rSrd.k2/(pow((1.0+exp(-rSrd.k1*rSrd.Error-rSrd.k2*rSrd.dError)),2)));					//���仯��ϵ��

	rSrd.LastError=rSrd.Error;
	rSrd.Lastout=rSrd.out;
	Rudder_L=(int32)(rSrd.out*(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5+125));//�������������������ϵ��
	Rudder_R=(int32)(rSrd.out*(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125));
	
	if(Rudder_L>((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=(int32)((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125);	//������
	if(Rudder_L<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)))
		Rudder_L=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5));
	if((Rudder_L<((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L*2.5)+125))&&(Rudder_L>(125-Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L*2.5)))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=0x7d;	
	if(Rudder_R>Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125)
		Rudder_R=(int32)(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125);			//������
	if(Rudder_R<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5)))
		Rudder_R=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5));
	if((Rudder_R<((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R*2.5)+125))&&(Rudder_L>(125-Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R*2.5)))//��������ǶȲ��ܳ��������ƶ��
		Rudder_R=0x7d;									//����ֵ

/*	if(Rudder_L>((Rudder_Con_Angle*2.5)+125))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=((Rudder_Con_Angle*2.5)+125);
	if(Rudder_L<(125-(Rudder_Con_Angle*2.5)))
		Rudder_L=(125-(Rudder_Con_Angle*2.5));
	if(Rudder_R>Rudder_Con_Angle*2.5+125)
		Rudder_R=Rudder_Con_Angle*2.5+125;
	if(Rudder_R<(125-(Rudder_Con_Angle*2.5)))
		Rudder_R=(125-(Rudder_Con_Angle*2.5));*/
}

//���ݷ����ƫ���PID���㷨������Ҷ�Ŀ��ƶ��
void Heading_PID2(double heading)
{
	Rudder_L=0x7d;
	Rudder_R=0x7d;
	rPID.SetPoint =(float)heading;                 	//����ʵ������趨
	rPID.Actual=(float)(Smart_Navigation_St.USV_Heading*0.01);               		//�õ���ǰ����
	rPID.Error =rPID.SetPoint- rPID.Actual;   	//���趨ֵ�Ƚϣ��õ����ֵ
	if(rPID.Error>180.0)
		rPID.Error=rPID.Error-360;
	else if (rPID.Error<-180.0)
		rPID.Error=rPID.Error+360;		
	if((rPID.Error<=1.0)&&(rPID.Error>=-1.0))		//��ƫ��С��1��ά����״
		return;
	if(Smart_Navigation_St.USV_Speed<5)		//���ٶ�С��5 ����΢��ϵ��
		rPID.Derivative*=2.0;				//΢��ϵ��

	rPID.LastError=rPID.Error;				//����ƫ��
	rPID.Integral_error+=rPID.Error;		//ƫ�����
//���������ƶ��	��PID�㷨
	Rudder_L+=(int32)(rPID.Proportion*rPID.Error*2.5+rPID.Integral*rPID.Integral_error*2.5+rPID.Derivative*(rPID.Error-rPID.LastError)*2.5);
//�����Ҷ���ƶ��
	Rudder_R+=(int32)(rPID.Proportion*rPID.Error*2.5+rPID.Integral*rPID.Integral_error*2.5+rPID.Derivative*(rPID.Error-rPID.LastError)*2.5);
//����Ŀ��ƽ�������1����Χ��
	if(Rudder_L>((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125))//��������ǶȲ��ܳ��������ƶ��
		Rudder_L=(int32)((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125);
	if(Rudder_L<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)))
		Rudder_L=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5));
	if(Rudder_R>Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125)
		Rudder_R=(int32)(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125);
	if(Rudder_R<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5)))
		Rudder_R=(int32)(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5));

	
//	printf("Rudder=%f-%f-%f-0x%.2x\n",rPID.SetPoint,rPID.Actual,rPID.Error,Rudder_L);
/*	if((rPID.Error >30)||(rPID.Error <-30))
	{
		if(Rudder_L>((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125))//��������ǶȲ��ܳ��������ƶ��
			Rudder_L=((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)+125);
		if(Rudder_L<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5)))
			Rudder_L=(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*2.5));
		if(Rudder_R>Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125)
			Rudder_R=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5+125;
		if(Rudder_R<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5)))
			Rudder_R=(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*2.5));
	}
	else
	{
		if(Rudder_L>((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*1.25)+125))//��������ǶȲ��ܳ��������ƶ��
			Rudder_L=((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*1.25)+125);
		if(Rudder_L<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*1.25)))
			Rudder_L=(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L*1.25));
		if(Rudder_R>Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*1.25+125)
			Rudder_R=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*1.25+125;
		if(Rudder_R<(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*1.25)))
			Rudder_R=(125-(Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R*1.25));
	}*/
}

//����ת��Ϊת��PID�㷨
void Speed_PID(double speed)
{
	float Increment;
	Increment=0.0;
	vPID.SetPoint =(float)speed;                 	//����ʵ������趨
	vPID.Actual=(float)(Smart_Navigation_St.USV_Speed*0.01);               		//�õ���ǰ�ٶ�ֵ
	vPID.Error =vPID.SetPoint- vPID.Actual;   	//���趨ֵ�Ƚϣ��õ����ֵ
	if((vPID.Error <=1.0)&&(vPID.Error>=-1.0))
		return;
	vPID.Ec=vPID.Error-vPID.LastError;
	Increment=vPID.Proportion*(vPID.Error-vPID.LastError)+vPID.Integral*vPID.Error+vPID.Derivative*(vPID.Error-2*vPID.LastError+vPID.PreError);
	vPID.PreError=vPID.LastError;
	vPID.LastError=vPID.Error;
//	vPID.Proportion=fuzzy_kp(vPID.Error/5,vPID.Ec);       //E��������5 
//	vPID.Integral=fuzzy_ki(vPID.Error/5,vPID.Ec); 
//	vPID.Derivative=fuzzy_kd(vPID.Error/5,vPID.Ec); 
	Accelerator_L+=(int32)(Increment*8);
	Accelerator_R+=(int32)(Increment*8);
//	printf("accelerator_L=%f-%f-%f-%f-%d\n",vPID.SetPoint ,vPID.Actual,Increment,vPID.Proportion,Accelerator_L/8);
	if(Accelerator_L<((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3))
		Accelerator_L=((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);
	if(Accelerator_L>((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3))
		Accelerator_L=((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3);
	if(Accelerator_R<((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
	if(Accelerator_R>((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3);
}
//����ת��Ϊת�ٵ�S���㷨
void Speed_S(double speed)
{
	float dIncrement;
	dIncrement=0.0;//�̶�����

	vSpd.SetPoint=(float)speed;
	vSpd.Actual=(float)(Smart_Navigation_St.USV_Speed*0.01);

	vSpd.Error=vSpd.SetPoint-vSpd.Actual;
	if((vSpd.Error<1.0)&&(vSpd.Error>=-1.0))
		return;
	vSpd.dError=(float)((vSpd.Error-vSpd.LastError)/0.05);//��������Ϊ50ms
	vSpd.out=(float)(2.0/(1.0+exp(-vSpd.k1*vSpd.Error-vSpd.k2*vSpd.dError))-1.0+dIncrement);
	vSpd.eout=vSpd.Lastout+vSpd.k4*vSpd.Error;
	//����k1��k2ֵ
	vSpd.k1+=vSpd.k3*(vSpd.eout-vSpd.Lastout)*2*(-vSpd.k1*vSpd.Error-vSpd.k2*vSpd.dError)*vSpd.Error;
	vSpd.k1=(float)(vSpd.k1/(pow((1.0+exp(-vSpd.k1*vSpd.Error-vSpd.k2*vSpd.dError)),2)));
	vSpd.k2+=vSpd.k3*(vSpd.eout-vSpd.Lastout)*2*(-vSpd.k1*vSpd.Error-vSpd.k2*vSpd.dError)*vSpd.dError;
	vSpd.k2=(float)(vSpd.k2/(pow((1.0+exp(-vSpd.k1*vSpd.Error-vSpd.k2*vSpd.dError)),2)));

	vSpd.LastError=vSpd.Error;
	vSpd.Lastout=vSpd.out;
	Accelerator_L=(int32)((vSpd.out*SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)*8);//�ֱ���Ϊ0.125rpm
	Accelerator_R=(int32)((vSpd.out*SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)*8);
	if(Accelerator_L<((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3))
		Accelerator_L=(int32)((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);
	if(Accelerator_L>((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3))
		Accelerator_L=(int32)((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3);
	if(Accelerator_R<((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3))
		Accelerator_R=(int32)((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
	if(Accelerator_R>((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3))
		Accelerator_R=(int32)((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3);
}
//PID����Kp�ļ���
void Speed_PID2(double speed)
{
//	float rot;
	vPID.SetPoint =(float)speed;                 	//����ʵ������趨
	vPID.Actual=(float)(Smart_Navigation_St.USV_Speed*0.01);               		//�õ���ǰ�ٶ�ֵ
	vPID.Error =vPID.SetPoint- vPID.Actual;   	//���趨ֵ�Ƚϣ��õ����ֵ
	if((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St<500)||(USV_State.Dradio_USV_Drive_State.Accelerator_Right_St<500))
		return;
	if((vPID.Error <=0.5)&&(vPID.Error>=-0.5))
		return;
//	rot=(Smart_Navigation_St.USV_ROT-10000)/100;
//	printf("rot=%f\n",rot);	
/*	if((rot>2)||(rot<-2))
		return;		
	if((rPID.Error>10.0)||(rPID.Error<-10.0))
		return;	*/
	vPID.Ec=vPID.Error-vPID.LastError;
	vPID.PreError=vPID.LastError;
	vPID.LastError=vPID.Error;
	vPID.Integral_error+=vPID.Error;
	Accelerator_L+=(int32)(vPID.Proportion*vPID.Error*8+vPID.Integral*vPID.Integral_error*8+vPID.Derivative*vPID.Ec*8);
	Accelerator_R+=(int32)(vPID.Proportion*vPID.Error*8+vPID.Integral*vPID.Integral_error*8+vPID.Derivative*vPID.Ec*8);
//	fprintf(PFile,"accelerator_L=%f-%f-%f-%d-%d\n",vPID.SetPoint ,vPID.Actual,vPID.Proportion,Accelerator_L/8,Accelerator_R/8);
//	printf("accelerator_L=%f-%f-%f-%d",vPID.SetPoint ,vPID.Actual,vPID.Proportion,Accelerator_L/8);
	if(Accelerator_L<((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3))
		Accelerator_L=(int32)((SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)<<3);
	if(Accelerator_L>((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3))
		Accelerator_L=(int32)((SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30)<<3);
	if(Accelerator_L>((78*30)<<3))
		Accelerator_L=((78*30)<<3);

	if(Accelerator_R<((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
	if(Accelerator_R>((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3);
	if(Accelerator_R>((78*30)<<3))
		Accelerator_R=((78*30)<<3);

}

float fuzzy_kp(float e, float ec)    //e,ec����ʾ�����仯�� 

{                

	float Kp_calcu; 	
	uint8  num,pe,pec; 
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};   //���E��ģ������ 	
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; //���仯��EC��ģ������ 	
	float eFuzzy[2]={0.0,0.0};        //���������E�������̶� 	
	float ecFuzzy[2]={0.0,0.0};            //���������仯��EC�������̶� 	
	float kpRule[4]={0.0,8.0,16.0,24.0};   //Kp��ģ���Ӽ� 	cxy
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};    //������Kp�������̶� 	cxy
	int KpRule[7][7]=         //Kp��ģ�����Ʊ�  cxy
	{ 
		{3,3,3,3,3,3,3}, 
		{2,2,2,2,1,2,2}, 
		{1,1,1,1,1,1,1}, 
		{1,1,0,1,0,1,1}, 
		{0,0,1,0,0,1,0}, 
		{0,1,0,1,0,0,2}, 
		{3,3,3,3,3,3,3} 
	};  
	/*****���E������������*****/   
	if(e<eRule[0])        
	{ 
		eFuzzy[0] =1.0;  
		pe = 0; 
	} 
	else if(eRule[0]<=e && e<eRule[1]) 
	{ 
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]); 
		pe = 0; 
	} 
	else if(eRule[1]<=e && e<eRule[2]) 
	{ 
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]); 
		pe = 1; 
	} 
	else if(eRule[2]<=e && e<eRule[3]) 
	{   
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]); 
		pe = 2; 
	} 
	else if(eRule[3]<=e && e<eRule[4]) 
	{   
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]); 
		pe = 3; 
	} 
	else if(eRule[4]<=e && e<eRule[5]) 
	{ 
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]); 
		pe = 4; 
	} 
	else if(eRule[5]<=e && e<eRule[6]) 
	{ 
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]); 
		pe = 5; 
	} 
	else 
	{ 
		eFuzzy[0] =0.0; 
		pe =5; 
	} 
	eFuzzy[1] =(float)(1.0 - eFuzzy[0]); 
 /*****���仯��EC������������*****/        
	if(ec<ecRule[0])        
	{ 
		ecFuzzy[0] =1.0; 
		pec = 0; 
	} 
	else if(ecRule[0]<=ec && ec<ecRule[1]) 
	{ 
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]); 
		pec = 0 ; 
	} 
	else if(ecRule[1]<=ec && ec<ecRule[2]) 
	{ 
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]); 
		pec = 1; 
	} 
	else if(ecRule[2]<=ec && ec<ecRule[3]) 
	{ 
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]); 
		pec = 2 ; 
	}     
	else if(ecRule[3]<=ec && ec<ecRule[4]) 
	{   
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]); 
		pec=3; 
	} 

	else if(ecRule[4]<=ec && ec<ecRule[5]) 
	{   
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]); 
		pec=4; 
	} 
	else if(ecRule[5]<=ec && ec<ecRule[6]) 
	{   
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]); 
		pec=5; 
	} 
	else 
	{ 
		ecFuzzy[0] =0.0; 
		pec = 5; 
	} 
	ecFuzzy[1] =(float)(1.0 - ecFuzzy[0]);  

 /*********��ѯģ�������*********/    
	num =KpRule[pe][pec]; 
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[0]; 
	num =KpRule[pe][pec+1]; 
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[1];  
	num =KpRule[pe+1][pec]; 
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[0]; 
	num =KpRule[pe+1][pec+1]; 
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[1]; 
/*********�� Ȩ ƽ �� �� �� ģ ��*********/  
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3]; 
	return(Kp_calcu); 
} 
//PID����Ki�ļ���
float fuzzy_ki(float e, float ec)             
{ 
	float Ki_calcu; 
	uint8 num,pe,pec; 
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};  
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; 
	float eFuzzy[2]={0.0,0.0};      
	float ecFuzzy[2]={0.0,0.0};              
	float kiRule[4]={0.00,(float)0.01,(float)0.02,(float)0.03};       
	float KiFuzzy[4]={0.0,0.0,0.0,0.0};    
	int KiRule[7][7]=             
	{ 
		{0,0,0,0,0,0,0}, 
		{0,0,0,0,0,0,0}, 
		{0,0,0,0,0,0,0}, 
		{0,0,0,0,0,0,0}, 
		{0,0,0,0,0,0,0}, 
		{2,0,0,0,0,0,1}, 
		{3,3,3,3,3,3,3} 
	}; 
   /*****���������������*****/ 
	if(e<eRule[0])        
	{ 
		eFuzzy[0] =1.0;  
		pe = 0; 
	} 
	else if(eRule[0]<=e && e<eRule[1]) 
	{ 
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]); 
		pe = 0; 
	} 
	else if(eRule[1]<=e && e<eRule[2]) 
	{ 
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]); 
		pe = 1; 
	} 
	else if(eRule[2]<=e && e<eRule[3]) 
	{ 
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]); 
		pe = 2; 
	} 
	else if(eRule[3]<=e && e<eRule[4]) 
	{   
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]); 
		pe = 3; 
	} 
	else if(eRule[4]<=e && e<eRule[5]) 
	{ 
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]); 
		pe = 4; 
	} 
	else if(eRule[5]<=e && e<eRule[6]) 
	{   
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]); 
		pe = 5; 
	} 
	else 
	{ 
		eFuzzy[0] =0.0; 
		pe =5; 
	} 
	eFuzzy[1] =(float)(1.0 - eFuzzy[0]); 
 /*****���仯������������*****/  
	if(ec<ecRule[0])        
	{ 
		ecFuzzy[0] =(float)1.0; 
		pec = 0; 
	} 
	else if(ecRule[0]<=ec && ec<ecRule[1]) 
	{ 
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]); 
		pec = 0 ; 
	} 
	else if(ecRule[1]<=ec && ec<ecRule[2]) 
	{ 
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]); 
		pec = 1; 
	} 
	else if(ecRule[2]<=ec && ec<ecRule[3]) 
	{ 
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]); 
		pec = 2 ; 
	} 
	else if(ecRule[3]<=ec && ec<ecRule[4]) 
	{   
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]); 
		pec=3; 
	} 
	else if(ecRule[4]<=ec && ec<ecRule[5]) 
	{   
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]); 
		pec=4; 
	} 
	else if(ecRule[5]<=ec && ec<ecRule[6]) 
	{   
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]); 
		pec=5; 
	} 
	else 
	{   
		ecFuzzy[0] =0.0; 
		pec = 5; 
	} 
	ecFuzzy[1] = (float)(1.0 - ecFuzzy[0]);  
 /***********��ѯģ�������***************/    
	num =KiRule[pe][pec]; 
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[0]; 
	num =KiRule[pe][pec+1]; 
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[1];  
	num =KiRule[pe+1][pec]; 
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[0]; 
	num =KiRule[pe+1][pec+1]; 
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[1]; 
 /********�� Ȩ ƽ �� �� �� ģ ��********/   
	Ki_calcu=KiFuzzy[0]*kiRule[0]+KiFuzzy[1]*kiRule[1]+KiFuzzy[2]*kiRule[2]+KiFuzzy[3]*kiRule[3];  
	return(Ki_calcu); 
} 

//PID����Kd�ļ���
float fuzzy_kd(float e, float ec)           
{ 
	float Kd_calcu; 
	uint8 num,pe,pec;     
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};  
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; 
	float eFuzzy[2]={0.0,0.0};      
	float ecFuzzy[2]={0.0,0.0};                   
	float kdRule[4]={0.0,1.0,2.0,3.0};    
	float KdFuzzy[4]={0.0,0.0,0.0,0.0};   
	int KdRule[7][7]=      
	{ 
		{3,3,3,2,2,2,2}, 
		{2,2,2,1,1,1,1}, 
		{1,1,2,1,1,2,1}, 
		{1,1,0,1,0,1,1}, 
		{1,1,0,0,0,1,1}, 
		{2,2,1,0 ,1,1,1}, 
		{3,3,3,3,2,3,2 }   
	}; 
   /*****���������������*****/ 
	if(e<eRule[0])        
	{   
		eFuzzy[0] =1.0;  
		pe = 0; 
	} 
	else if(eRule[0]<=e && e<eRule[1]) 
	{ 
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]); 
		pe = 0; 
	} 
	else if(eRule[1]<=e && e<eRule[2]) 
	{ 
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]); 
		pe = 1; 
	} 
	else if(eRule[2]<=e && e<eRule[3]) 
	{ 
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]); 
		pe = 2; 
	} 
	else if(eRule[3]<=e && e<eRule[4]) 
	{   
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]); 
		pe = 3; 
	} 
	else if(eRule[4]<=e && e<eRule[5]) 
	{ 
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]); 
		pe = 4; 
	} 
	else if(eRule[5]<=e && e<eRule[6]) 
	{ 
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]); 
		pe = 5; 
	} 
	else 
	{ 
		eFuzzy[0] =0.0; 
		pe =5; 
	} 
	eFuzzy[1] =(float)(1.0 - eFuzzy[0]); 
 /*****���仯������������*****/    
	if(ec<ecRule[0])         
	{ 
		ecFuzzy[0] =(float)1.0; 
		pec = 0;  
	} 
	else if(ecRule[0]<=ec && ec<ecRule[1]) 
	{ 
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]); 
		pec = 0 ; 
	} 
	else if(ecRule[1]<=ec && ec<ecRule[2]) 
	{ 
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]); 
		pec = 1; 
	} 
	else if(ecRule[2]<=ec && ec<ecRule[3]) 
	{ 
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]); 
		pec = 2 ; 
	} 
	else if(ecRule[3]<=ec && ec<ecRule[4]) 
	{   
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]); 
		pec=3; 
	} 
	else if(ecRule[4]<=ec && ec<ecRule[5]) 
	{   
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]); 
		pec=4; 
	} 
	else if(ecRule[5]<=ec && ec<ecRule[6]) 
	{   
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]); 
		pec=5; 
	} 
	else 
	{ 
		ecFuzzy[0] =0.0; 
		pec = 5; 
	} 
	ecFuzzy[1] = (float)(1.0 - ecFuzzy[0]);  
 /***********��ѯģ�������*************/  
	num =KdRule[pe][pec]; 
	KdFuzzy[num] += eFuzzy[0]*ecFuzzy[0]; 
	num =KdRule[pe][pec+1]; 
	KdFuzzy[num] += eFuzzy[0]*ecFuzzy[1];  
	num =KdRule[pe+1][pec]; 
	KdFuzzy[num] += eFuzzy[1]*ecFuzzy[0]; 
	num =KdRule[pe+1][pec+1]; 
	KdFuzzy[num] += eFuzzy[1]*ecFuzzy[1]; 
 /********��Ȩƽ������ģ��********/    
	Kd_calcu=KdFuzzy[0]*kdRule[0]+KdFuzzy[1]*kdRule[1]+KdFuzzy[2]*kdRule[2]+KdFuzzy[3]*kdRule[3];  
	return(Kd_calcu); 
} 



//�������ֵ�̨���񣬺��������־
//��� 0--����  1--�յ����������� ��2--����������ͣ
uint8 Check_Intelligent_Switch(void)
{
//#ifndef WINNT
//	if((1==Dradio_Com1_Sign || 1==SP_CON_Sign || 1==BDS_Con_Sign || 1==Bradio_Con_Sign)&&(1==USV_RM_MSG.MPC_radar_valid && 1==USV_RM_MSG.MPC_Heatbeat))//(�����������ͨ��������ͨ�����������,����Ȩ��)&&MPC���״�ͼ����Ч		//���ֵ�̨COM1����״̬ 1--������״̬   //��������ȡȨ��
//#else
	if(1==Dradio_Com1_Sign || 1==SP_CON_Sign || 1==BDS_Con_Sign || 1==Bradio_Con_Sign)
//#endif
	{
		if((1==USV_Control.USV_Control_Message[Radio_Sign].Navigation_Tsak_sign))		//����������
			return 1;
		else if((2==USV_Control.USV_Control_Message[Radio_Sign].Navigation_Tsak_sign))	//����������ͣ
			return 2;
		else
			return 0;			//����
	}
	else
		return 0;
}
//�����·����� ���ٺ����ж�
//���� 1--����  0--
uint8 Check_Speed_Const(void)
{
	if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
	{
		if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Speed_Constant_Mod)
			return 1;
		else
			return 0;
	}
	else
		return 0;
	
}
//�����·��Ŀ������ȷ���������ж�
//��������������򡡡�������
uint8 Check_Heading_Const(void)
{
	if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
	{
		if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Direction_Constant_Mod)
			return 1;
		else
			return 0;
	}
	else
		return 0;

}
//����ģʽ�ж�
//0--�ֶ�ģʽ ��1--���Զ� 2--ȫ�Զ���3--����ģʽ�����٣�
// ���ݿ�����̨����ָ����ء���,��,��,��
uint8 Check_Intelligent_Mod(void)
{
	Sailing_Mod_old = Sailing_Mod;
	Sailing_Mod = USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod;
	
	if(Sailing_Mod != Sailing_Mod_old)
	{
		//Ȩ���л���־
			switch(Sailing_Mod){
				case(0): //SysLogMsgPost("����ģʽ�л����л���-�ֶ�ģʽ ");
						// SysPubMsgPost("����ģʽ�л����л���-�ֶ�ģʽ ");
					break;
				case(1): //SysLogMsgPost("����ģʽ�л����л���-���Զ�");
						// SysPubMsgPost("����ģʽ�л����л���-���Զ�");
					break;
				case(2): //SysLogMsgPost("����ģʽ�л����л���-ȫ�Զ� ");
						 //SysPubMsgPost("����ģʽ�л����л���-ȫ�Զ� ");
					break;
				case(3): //SysLogMsgPost("����ģʽ�л����л���-��ģʽ");
						 //SysPubMsgPost("����ģʽ�л����л���-��ģʽ");
					break;
				default:
					break;
			}
	}


	if(3==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
		return 3;
	else if(2==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
		return 2;		
	else if(1==USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
		return 1;
	else
		return 0;
}

//������ǿ���ֵ�����ݺ��ټ�����ƶ�� ��ö��Rudder_Con_Angle
void Con_Rudder(void)
{
	Rudder_Con_Angle=Calculate_Rudder();		//���ݺ��ټ�����ƶ�� 5,5~25 25 ȥ�����  ����ֵ  ��С���
	if(1)										//��ǰʵ�ʶ������һ��ʵ�ʶ��ͬ��cxy ���²��ֳ���û����
		Rudder_Count++;
	else
		Rudder_Count=0;
	if(Rudder_Count>200)
		Rudder_Zero_Angle=Calculate_Rudder_Zero();		//��������� ���� ���� 0		
	//���ݶ���������������ֵ�������ݿ��ƶ�ǽ�������cxy
	return;
}

//���ݺ��ټ�����ƶ�� 5 ,5~25 >25 ��ȡ������ �����ݺ�������ֵ ��  ����
uint8 Calculate_Rudder(void)
{
	float Rudder_Act,USV_Speed_Act;
	uint8 GetRudder;
	double	 Ks;
	Rudder_Act=0.0;
	USV_Speed_Act=0.0;
	Ks=0.0;
	GetRudder=0;
	
	USV_Speed_Act=(float)(USV_State.Dradio_USV_Sailing_State.USV_Speed*0.01);			//�ٶ���Դ�ڹߵ�ϵͳ
	if(USV_Speed_Act<0)
	{
#ifdef	debug_print
		printf("Get control ruuder error!\n");	
#endif
		return	Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L;
	}
	if(USV_Speed_Act<=5.0)														//�ٶ�С�� 5��
		GetRudder=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L;			//=��������ƶ��
	else if(USV_Speed_Act<=25.0)												//�ٶ�С��25��
	{
		Ks=exp((5.0-USV_Speed_Act)/Calculate_Rudder_ks);						//e�η�
		Rudder_Act+=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L;		//����С�ǵĻ�����+����ֵ
		Rudder_Act+=(float)((Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L-Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L)*Ks);
		GetRudder=(uint8)(Rudder_Act*10);
	}
	else if(USV_Speed_Act>25.0)												//�ٶ�>25��
		GetRudder=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L;		//=��С���
	return GetRudder;

}

//���������
uint8 Calculate_Rudder_Zero(void)
{
		/*cxy*/
	return 0;
}

//���㺣��
uint8 Get_SeaState(void)
{
	float Wind_Speed;
	Wind_Speed=0.0;

	Wind_Speed=(float)(USV_RM_MSG.RM_Meteorological_Msg.Wind_Msg.Wind_Speed*0.1);
	if((Wind_Speed>=60.0)||(Wind_Speed<0.0))
	{
#ifdef	debug_print
		printf("Get seastate error!\n");	
#endif
		return 10;
	}
	if((Wind_Speed<0.3)&&(Wind_Speed>=0.0))//����0��
		return 0;
	if((Wind_Speed<1.6)&&(Wind_Speed>=0.3))//����1��
		return 1;
	if((Wind_Speed<3.4)&&(Wind_Speed>=1.6))//����2��
		return 2;
	if((Wind_Speed<8.0)&&(Wind_Speed>=3.4))//����3��
		return 3;
	if((Wind_Speed<10.8)&&(Wind_Speed>=8.0))//����4��
		return 4;
	if((Wind_Speed<13.9)&&(Wind_Speed>=10.8))//����5��
		return 5;
	if((Wind_Speed<17.2)&&(Wind_Speed>=13.9))//����6��
		return 6;
	if((Wind_Speed<20.8)&&(Wind_Speed>=17.3))//����7��
		return 7;
	if((Wind_Speed<20.8)&&(Wind_Speed>=17.3))//����8��
		return 8;
	if((Wind_Speed<60.0)&&(Wind_Speed>=20.9))//����9��������
		return 9;	
	return 10;
}

//������ײ����
sint8 Calculate_Collision(void )
{	
	uint8 Sea_State;
	sint16 TCPA;
	uint32	Safe_DCPA,DCPA;
	Sea_State=0;
	Safe_DCPA=0;
	TCPA=0;
	DCPA=0;
	
	Sea_State=Get_SeaState();
	Safe_DCPA=Get_Safe_DCPA(Sea_State);
	Get_AIS_DCPA_TCPA(&DCPA,&TCPA);
	USV_Check_Collision[Collision_Num].Safe_DCPA=Safe_DCPA;
	USV_Check_Collision[Collision_Num].DCPA=DCPA;
	USV_Check_Collision[Collision_Num].TCPA=TCPA;
	if(DCPA>	Safe_DCPA)
	{
		memset((uint8 *)&USV_Check_Collision[Collision_Num].User_ID,0,sizeof(USV_Check_Collision));
		return -1;
	}
	if(TCPA<0)
	{
		memset((uint8 *)&USV_Check_Collision[Collision_Num].User_ID,0,sizeof(USV_Check_Collision));
		return -1;
	}
		return Collision_Num;
}
//����Safe_DCPA
uint32 Get_Safe_DCPA(uint8 Sea_State)
{
	uint32 safe_dcpa;
	safe_dcpa=0;
	if(USV_Check_Collision[Collision_Num].Ship_Lenght<100)
	{
		if(Sea_State<4)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(0.5*Nmile);
		else if(Sea_State<7)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(0.8*Nmile);
		else if(Sea_State<9)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(0.9*Nmile);
		else
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.1*Nmile);
	}
	else if(USV_Check_Collision[Collision_Num].Ship_Lenght<200)
	{
		if(Sea_State<4)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(0.6*Nmile);
		else if(Sea_State<7)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.1*Nmile);
		else if(Sea_State<9)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.2*Nmile);
		else
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.3*Nmile);
	}
	else if(USV_Check_Collision[Collision_Num].Ship_Lenght<300)
	{
		if(Sea_State<4)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(0.8*Nmile);
		else if(Sea_State<7)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.3*Nmile);
		else if(Sea_State<9)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.4*Nmile);
		else
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.5*Nmile);
	}
	else
	{
		if(Sea_State<4)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.0*Nmile);
		else if(Sea_State<7)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(1.5*Nmile);
		else if(Sea_State<9)
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(2.0*Nmile);
		else
			USV_Check_Collision[Collision_Num].Safe_DCPA=(uint32)(2.5*Nmile);
	}
	safe_dcpa=USV_Check_Collision[Collision_Num].Safe_DCPA;
	return  safe_dcpa;
}
//����AISĿ��DCPA/TCPA
void Get_AIS_DCPA_TCPA(uint32 *DCPA,sint16 *TCPA)
{
	double Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double Dst;
	
	Vu=(Smart_Navigation_St.USV_Speed*Nmile)/(3600*100);//USV���٣���λm/s
	Qu=Smart_Navigation_St.USV_Heading/100;//���򣬵�λ��
	Qu=Radian(Qu);//ת��Ϊ����
	VXu=Vu*sin(Qu);//x���ٶ�
	VYu=Vu*cos(Qu);//y���ٶ�
	Vt=USV_Check_Collision[Collision_Num].SOG;
	Qt=USV_Check_Collision[Collision_Num].COG;
	Qt=Radian(Qt);
	VXt=Vt*sin(Qt);
	VYt=Vt*cos(Qt);
	VXut=VXt-VXu;
	VYut=VYt-VYu;
	AQ=Get_AQ(VXut,VYut);
	
	Vut=sqrt(VXut*VXut+VYut*VYut);//����ٶ�
	Qut=atan(VXut/VYut)*180.0/Pi+AQ;//��Ժ���

	double latt=(USV_Check_Collision[Collision_Num].Latitude/600000);//������з�aa.bbbbbbb���ʽ
	double lngt=(USV_Check_Collision[Collision_Num].Longitude/600000);
	
	double Lau=Smart_Navigation_St.USV_Lat;
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		Lau=0-Lau;	//����Ϊ��
	double Lnu=Smart_Navigation_St.USV_Lng;
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		Lnu=0-Lnu;//��γΪ��
	Dst=Get_distance(latt, lngt, Lau,Lnu);//��λ��
	double AUT=Get_AUT(latt, lngt, Lau,Lnu)+AQ;
//	double ATU=Get_ATU(latt, lngt, Lau,Lnu)+AQ;

	*DCPA=(uint32)(fabs(Dst*sin(Radian(Qut)-AUT-Pi)));
	*TCPA=(int16)(Dst*cos(Radian(Qut)-AUT-Pi)/Vut);
	return ;
}
//��ò����Ƕ�
double Get_AQ(double vx,double vy)
{
	if((vx>=0)&&(vy>=0))
		return 0.0;
	if((vx<0)&&(vy<0))
		return 180.0;
	if((vx>=0)&&(vy<0))
		return 180.0;
	if((vx<0)&&(vy>=0))
		return 360.0;
	return 0.0;
}
//��ȡ1�����2�ķ�λ��
double Get_AUT(double lat1,double lng1,double lat2,double lng2)
{
	double A;
	A=0.0;
	if((lat1==lat2)&&(lng1==lng2))
	{
#ifdef	debug_print
		printf("USV location is error !\n");
#endif
		return A;		
	}
	else if((lng1==lng2)&&(lat1!=lat2))//ͬγ��
	{
		if(lat1<lat2)
		{
			if((lat1<-90.0)&&(lat2>90.0))//����������180�ȸ���
				A=Pi;
			else
				A=3*Pi;
		}
		else
		{
			if((lat1>90.0)&&(lat2<-90.0))//����������180�ȸ���
				A=3*Pi;
			else
				A=Pi;
		}
	}
	else if((lat1==lat2)&&(lng1!=lng2))//ͬ����
	{
		if(lng1>lng2)
			A=0.0;
		else 
			A=2*Pi;
	}
	else
	{
		//��λ�Ǿ�ȷ�㷨
		double CosC=cos(Pi/2-lng1)*cos(Pi/2-lng2)+sin(Pi/2-lng1)*sin(Pi/2-lng2)*cos(lat1-lat2);
		double SinC=sqrt(1-CosC*CosC);
		A=sin(Pi/2-lng1)*sin(lat1-lat2);
		A=A/SinC;
		A=asin(A);//Ŀ����Ա���λ�ڵ�һ����
		if(((lat1-lat2)<0.0)&&((lng1-lng2)>0.0))//Ŀ����Ա���λ�ڵڶ�����
			A=A+2*Pi;
		if(((lat1-lat2)<0.0)&&((lng1-lng2)<0.0))//Ŀ����Ա���λ�ڵ�������
			A=Pi-A;
		if(((lat1-lat2)>0.0)&&((lng1-lng2)<0.0))//Ŀ����Ա���λ�ڵ�������
			A=Pi-A;
	}
	return A;

}
//��ȡ2�����1�ķ�λ��
double Get_ATU(double lat1,double lng1,double lat2,double lng2)
{
	double A;
	A=0.0;
	
	if((lat1==lat2)&&(lng1==lng2))
	{
#ifdef	debug_print
		printf("USV location is error !\n");
#endif
		return A;		
	}
	else if((lng1==lng2)&&(lat1!=lat2))//ͬγ��
	{
		if(lat1<lat2)
		{
			if((lat1<-90.0)&&(lat2>90.0))//����������180�ȸ���
				A=3*Pi;
			else
				A=Pi;
		}
		else
		{
			if((lat1>90.0)&&(lat2<-90.0))//����������180�ȸ���
				A=Pi;
			else
				A=3*Pi;
		}
	}
	else if((lat1==lat2)&&(lng1!=lng2))//ͬ����
	{
		if(lng1>lng2)
			A=2*Pi;
		else 
			A=0.0;
	}
	else
	{
		A=atan((lat2-lat1)*cos(lng1)/(lng2-lng1));//��λ�Ǽ��㷨
		if(((lat2-lat1)<0.0)&&((lng2-lng1)>0.0))//���������Ŀ��λ�ڵڶ�����
			A=A+2*Pi;
		if(((lat2-lat1)<0.0)&&((lng2-lng1)<0.0))//���������Ŀ��λ�ڵ�������
			A=Pi-A;
		if(((lat2-lat1)>0.0)&&((lng2-lng1)<0.0))//���������Ŀ��λ�ڵ�������
			A=Pi-A;
	}
	return A;

}

//���Ƕȱ�ɻ���
double Radian(double d)
{
	return d*Pi/180.0;//�Ƕ�1��=pi/180

}
//����2��ľ��Ⱥ�ά�ȼ�������ľ��룬��λΪ ��
double Get_distance(double lat1,double lng1,double lat2,double lng2)
{
	double radLat1=Radian(lat1);
	double radLat2=Radian(lat2);
	double a=radLat1-radLat2;
	double b=Radian(lng1)-Radian(lng2);
	double dst=2*asin((sqrt(pow(sin(a/2),2)+cos(radLat1)*cos(radLat2)*pow(sin(b/2),2))));
	dst=dst*Earth_Radius;							//Earth_Radius--����뾶
//	dst=round(dst*10000)/10000;
	return dst;
}

//AIS�����㷨
double Get_Yaw_Dst()
{
	double lat1,lng1,lat2,lng2,lat3,lng3,Dst1,Dst2,Dst3,Heading1,Heading2,Heading_error,Yaw_Dst,Perimeter,Measure;
	Dst1=0.0;
	Dst2=0.0;
	Dst3=0.0;
	lat1=Smart_Navigation_St.USV_Lat;								//��ǰλ��
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//����Ϊ��
	lng1=Smart_Navigation_St.USV_Lng;
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;//��γΪ��
//Ŀ��λ��
	lng2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Degree;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Minute/60.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Second/3600.0;
	lng2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Latitude_Sign)
		lng2=0-lng2;//����Ϊ��
	lat2=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Degree;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Minute/60.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Second/3600.0;
	lat2+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old].Waypoint_Longitude_Sign)
		lat2=0-lat2;//��γΪ��
//��һ���Ŀ��λ��
	lng3=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Degree;
	lng3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Minute/60.0;
	lng3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Second/3600.0;
	lng3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Latitude_Sign)
		lng3=0-lng3;//����Ϊ��
	lat3=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Degree;
	lat3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Minute/60.0;
	lat3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Second/3600.0;
	lat3+=(double)USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Decimal/360000.0;
	if(1==USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[Sailing_Cnt_Old-1].Waypoint_Longitude_Sign)
		lat3=0-lat3;//��γΪ��
//	printf("lat1=%f,lng1=%f,lat2=%f,lng2=%f\n",lat1,lng1,lat2,lng2);	
	Dst1=Get_distance(lng1,lat1 , lng2, lat2);
	Dst2=Get_distance(lng1,lat1 , lng3, lat3);
	Dst3=Get_distance(lng3,lat3 , lng2, lat2);
	Heading1=Get_heading(lng2,lat2 , lng1, lat1);
	Heading2=Get_heading(lng2,lat2 , lng3, lat3);
	Perimeter=(Dst1+Dst2+Dst3)/2.0;
	Measure=sqrt(Perimeter*(Perimeter-Dst1)*(Perimeter-Dst2)*(Perimeter-Dst3));
	Yaw_Dst=2*Measure/Dst3;
	Heading_error=Heading1-Heading2;
	if(Heading_error>180.0)
		Heading_error=Heading_error-360.0;
	if(Heading_error<-180.0)
		Heading_error=360.0+Heading_error;
	if(Heading_error<0)//��ƫΪ������ƫΪ��
		Yaw_Dst=0.0-Yaw_Dst;
	return Yaw_Dst;
}

void Obstacle_acoidance(void)
{
	if(2!=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod)
		return;
	double QU=Smart_Navigation_St.USV_Heading/100;//�������򣬵�λ��
	double QA=USV_Check_Collision[Collision_Num].COG;//Ŀ�괬���򣬵�λ��
	double	 QUA=fabs(QA-QU);
	if(QUA>180)
		QUA=360-QUA;
	//Ŀ�괬��γ��
	double latt=(USV_Check_Collision[Collision_Num].Latitude/600000);//������з�aa.bbbbbbb���ʽ
	double lngt=(USV_Check_Collision[Collision_Num].Longitude/600000);
	//������γ��
	double Lau=Smart_Navigation_St.USV_Lat;
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		Lau=0-Lau;	//����Ϊ��
	double Lnu=Smart_Navigation_St.USV_Lng;
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		Lnu=0-Lnu;//��γΪ��
	//��Ծ���
	double Dst=Get_distance(latt, lngt, Lau,Lnu);
	//���Ƕȷ�
	if(QUA<=30)//׷��
	{//���Ͼ���
		if(Dst<2000)	Avoidance_Strategy(latt,lngt,QA,QU);
	}
	else if(QUA<=150)//����
	{
		if(Dst<3000)	Avoidance_Strategy(latt,lngt,QA,QU);
	}
	else if(QUA<=180)//����
	{
		if(Dst<5000)	Avoidance_Strategy(latt,lngt,QA,QU);
	}
	return ;
}

//���ò������
void Avoidance_Strategy(double lat1,double lng1,double COGA,double COGU)
{
	//Ŀ�괬�����ԭ��ķ�λ��С��USV�ĺ������ڴ���࣬��֮���Ҳ�
	double QV=Get_ATU(0.0,0.0,lat1,lng1);
	if(QV<=COGU)//�ϰ����ڴ����
	{
			Collision_Heading=Smart_Navigation_St.USV_Heading*0.01+Rudder_Con_Angle_Max;//��ת
			if((COGU-COGA)<=0.0)
				Collision_Speed=Smart_Navigation_St.USV_Speed*0.01*0.8;//�����ٶ�(1sÿ���ٶ�Ϊ�ϴε�0.8)
	}
	else//�ϰ����ڴ��Ҳ�
	{ 
			Collision_Heading=Smart_Navigation_St.USV_Heading*0.01-Rudder_Con_Angle_Max;//��ת
			if((COGU-COGA)>=0.0)
				Collision_Speed=Smart_Navigation_St.USV_Speed*0.01*0.8;//�����ٶ�(1sÿ���ٶ�Ϊ�ϴε�0.8)
	}
	//�߽�����
	if(Collision_Heading<0)
		Collision_Heading=0-Collision_Heading;
	if(Collision_Heading>=360.0)
		Collision_Heading-=360.0;
	if(Collision_Speed>=0.5*USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit)
		Collision_Speed=0.5*USV_Control.USV_Control_Message[Radio_Sign].Speed_Limit;
}
//�����״�Ŀ��DCPA/TCPA
void Get_Radar_DCPA_TCPA(double speed,double heading,uint8 OBS_Count, double *DCPA,double *TCPA)
{
	double Vu,Qu,Vt,Qt,VXu,VYu,VXt,VYt,VXut,VYut,Vut,Qut,AQ;
	double Dst,Dir;
	
	Vu=speed*Nmile/3600.0;//USV���٣���λm/s
	Qu=heading;//���򣬵�λ��
	Qu=Radian(Qu);//ת��Ϊ����
	VXu=Vu*sin(Qu);//x���ٶ�
	VYu=Vu*cos(Qu);//y���ٶ�
//	Vt=USV_RM_MSG.RM_radar_Msg[OBS_Count].OBS_Speed*Nmile/3600.0;//�ϰ��ﺽ�٣���λm/s
//	Qt=USV_RM_MSG.RM_radar_Msg[OBS_Count].OBS_Heading;//�ϰ��ﺽ�򣬵�λ��
	Vt=USV_RM_MSG.RM_radar_Msg[OBS_Count].obstacle_speed;
	Qt=USV_RM_MSG.RM_radar_Msg[OBS_Count].obstacle_direction;

	Qt=Radian(Qt);
	VXt=Vt*sin(Qt);
	VYt=Vt*cos(Qt);
	VXut=VXt-VXu;
	VYut=VYt-VYu;
	AQ=Get_AQ(VXut,VYut);
	
	Vut=sqrt(VXut*VXut+VYut*VYut);//����ٶ�
	Qut=atan(VXut/VYut)*180.0/Pi+AQ;//��Ժ���

//	Dst=USV_RM_MSG.RM_radar_Msg[OBS_Count].OBS_Distance;//��Ծ���
//	Dir=USV_RM_MSG.RM_radar_Msg[OBS_Count].OBS_Direction;//�ϰ����뱾������Է�λ


	double lat1,lng1,lat2,lng2;
	lat1=Smart_Navigation_St.USV_Lat;					//��ǰ����
	if(Smart_Navigation_St.Latitude_Sign_St==2)
		lat1=0-lat1;	//��γΪ��	
	lng1=Smart_Navigation_St.USV_Lng;					//��ǰά��
	if(Smart_Navigation_St.Longitude_Sign_St==2)
		lng1=0-lng1;	//����Ϊ��
		
	lat2 = USV_RM_MSG.RM_radar_Msg[OBS_Count].obstacle_locate.lat;
	lng2 = USV_RM_MSG.RM_radar_Msg[OBS_Count].obstacle_locate.lng;

	Dst = Get_distance(lat1, lng1, lat2,lng2);
	Qut = Get_heading(lng2,lat2 , lng1, lat1);

	*DCPA=fabs(Dst*sin(Radian(Qut)-Dir-Pi));
	*TCPA=Dst*cos(Radian(Qut)-Dir-Pi)/Vut;
	return ;
}
int cmp ( const void *a , const void *b)
{
        return *(int *)a - *(int *)b;
}
void Get_lat_lng(double lat,double lng,double dst,double heading,double *Lat,double *Lng)
{
//	double lat4=30.43393466;//��γΪ������γȡ-39.90744
//	double lng4=114.40480219;//����Ϊ��������ȡ-116.41615
//	double dst=580.9095;
//	double heading=293.4138;
	double c=dst/Earth_Radius;
	double a=acos(cos(Pi*(90.0-lat)/180.0)*cos(c)+sin(Pi*(90-lat)/180.0)*sin(c)*cos(Pi*(heading)/180.0));
	double C=asin(sin(c)*sin(Pi*(heading)/180.0)/sin(a));

	*Lat=90-a*180.0/Pi;
	*Lng=lng+C*180.0/Pi;
	return ;
}


//uint8 get_radar_collision_advance(void)
//{
//	
//}

