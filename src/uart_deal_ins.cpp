#include "stdafx.h"
#include <nanomsg/nn.h>
#include <nanomsg/pair.h>
#include "../include/usv_include.h"
#include "../include/uart_deal_ins.h"
#include <conf/conf.h>
#include <simulation/sim_boat.h>    
#include <user_time/user_time.h>
#include "uart_irtk.h"
#include "util/easylogging++.h"
#include <math/Location.h>
#include <conf/conf.h>
using namespace LOC;

#define INS_REC_TIMESLICE 10
#define UDP_INS_REC_PORT 8210
#define UDP_INS_SED_PORT 8211
/******************************  Local Variable  *****************************/
char insMsg[128];
uint8 ins_re_sign=0;					 //���ձ�־
uint8 ins_jco=0;
Smart_Navigation_Msg Smart_Navigation_St;//�ߵ�ϵͳ��ϸ��Ϣ
INS_DETELL	ins_msg	;					 //�ߵ�����double��
COMM_SIGN	ins_sign;					 //�ߵ�ͨѶ״̬
uint8	log_ins_sign;

uint8 wrValid_0 = 0;			//дϵͳʱ���ʶ
uint8 wrValid_1 = 0;

//�ߵ�����zmqת��
ZMQ_SOCKET ins_zmq_pub;
	

//�ߵ�ת��UDP�˿ڱ���
int16				ins_sockid	;
UDP_INF_STRUCT		ins_udp_inf	;

//�ߵ�ת��nano�˿� ����
int		ins_nanoSockit;
char	ins_nanoAddr[30];

//UN237 Ӳ��IO״̬
UN237_HARDWARE_STATE ins_hardState;
/******************************  Extern Variable  ****************************/
char ins_udp_addr[30];
uint32 ins_udp_port;
/******************************  Local Function   ****************************/
int8 uart_ins_init(void);
int8 uart_ins_rec_report(UART_TYPE uartid);
void ins_cmd_init(UART_TYPE uartid);
void INS_GPROT_Analytical(char  *buff);
void INS_GPHEV_Analytical(char  *buff);
void INS_PSAT_Analytical(char  *buff,int length);
void INS_GPRMC_Analytical(char  *buff,int length);
void INS_GPGGA_Analytical(char  *buff,int length);

void INS_GPGST_Analytical(char *buff, int length);
void insSocketInit(void);
void insUdpSend(int8* pBuf,int len);
int8 insUdpRecReport(int16 sockid);




void insNanoSocketInit(void);		//nanomsg ת���ߵ�ԭʼ���� ʹ��nano pair
void insNanoSend(char* pBuf, int len);
int8 insNanoRecReport(void);		

/******************************  Extern Function  ****************************/

/******************************    Code   ************************************/

void runins1(void *)
{
	printf("run ins1 \n");
}

void runins2(void *)
{
	printf("run ins2 \n");
}

void jugdeGPSState()
{
	uint64_t tnow = user_time::get_millis();
	if(last_rtk_time!=0&&(tnow-last_rtk_time)>2000)
	{
		irtk_msg.rtk_state.b1_diffSignalValid =0;
		irtk_msg.rtk_state.c_rmcValid = 'N';
	}

	

	if(irtk_msg.rtk_state.b1_diffSignalValid >= 2 && irtk_msg.rtk_state.c_rmcValid == 'A')
	{

		rtk_locating_fuc = 1;
	}
	else
	{

		rtk_locating_fuc = 0;
	}

}

static bool ekf_origin_is_set = false;
static uint16_t delay_time = 300;
Location ekf_origin_;

void *uart_deal_ins(void *aa)
{
	if(uart_ins_init()<0)
	{
		printf("Ins initialize failed!\n");
		return ((void*)0);
	}
	printf("=======Ins initialize successful!=====\n");
	memset((char*)&ins_msg,0,sizeof(ins_msg));					
	comm_time_return(&(ins_sign.timer),&(ins_sign.comm_sign));
	//	ins_cmd_init(UART5_Fd);
	insSocketInit();											
	//insNanoSocketInit();

	bool sitl_enable = AP::conf()->simualtion_enable;
	for(;;){
			jugdeGPSState();
		if(sitl_enable == true){
			double steering = jet_system.jetL.i16_Cmd_MotorRudderDeg/255.0;
			double throttle = jet_system.jetL.u8_Cmd_MotorOpenDeg/255.0;
			throttle = (jet_system.jetL.i16_Cmd_MotorGearDeg<=0) ?(-throttle):throttle;
			AP::sim_boat()->update(throttle,steering,INS_REC_TIMESLICE*0.001);
		}else{
			uart_ins_rec_report(UART5_Fd);
			insUdpRecReport(ins_sockid);
			//insNanoRecReport();
		}
		
		if(!ekf_origin_is_set && 
		   (ins_msg.insState.c_rmcValid == 'A' || irtk_msg.rtk_state.c_rmcValid == 'A')&&
		   (ins_msg.latitude >0.1 && ins_msg.longitude >0.1)){
			   ekf_origin_is_set = true;

			// get ekf_origin
         	   ekf_origin_.lat = ins_msg.latitude*1e7;
               ekf_origin_.lng = ins_msg.longitude*1e7;
               ekf_origin_.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
			   printf("EKF ORIGIN SET [lat = %ld,lng = %ld]\n", ekf_origin_.lat,ekf_origin_.lng);
		   }
	
		sleep_1(INS_REC_TIMESLICE);	//10ms
	}
	close_uart(UART5_Fd);
	
	return ((void *)0);
}


void INS_Init()
{
	char str1[] = "$JATT,TILTCAL\r\n";
	write_uart(UART5_Fd,str1,strlen(str1));
	sleep_1(2000);

	char str2[] = "$JATT,FLIPBRD,NO\r\n";
	write_uart(UART5_Fd,str2,strlen(str2));
	sleep_1(2000);
}

//惯导报文接收线程
int8 uart_ins_init( void )
{
	int8 iret = 0;
	UART5_Fd = open_port(COM5);
	#ifdef WINNT
	if(UART5_Fd == INVALID_HANDLE_VALUE)
	#else
	if (UART5_Fd <0)
	#endif
	{
		iret = -1;
	}
	if(AP::conf()->boat_conf_.ins_type==1) //ins type old
	{
		if (set_com_config(UART5_Fd, 115200, 8, 'N', 1)<0)
		{
		iret = -1;
		}
		INS_Init();
	}
	else  //ins type new
	{
		if (set_com_config(UART5_Fd, 460800, 8, 'N', 1)<0)
		{
		iret = -1;
		}
		printf("init com5 bate 460800\n");
	}
	
	return iret;
}
typedef struct __GPGGA__
{
	float	utc_time;		//UTC时分秒
	double	lat;			//纬度
	char	c_lat;			//纬度标志 N 北纬 S 南纬
	double	lon;			//经度
	char	c_lon;			//经度标志 E 东经 W 西经
	char	c_posType;		//GPS定位标识： 0 无定位 1 单点定位 2 差分定位 4 RTK定位 5 RTK浮点
	unsigned int satelliteNum;	//用于定位的卫星数目
	float	HDOP;			//HDOP = 0.0~9.9 水平位置精度因子
	double	height;			//天线海拔高度 saaaa.aa
	double	seaLvlSeparation;	//海平面分离度 ±xxxx.xx
	unsigned int diffCorrDelay;	//差分校正时延 sss
	unsigned int refStID;		//参考站ID		
}GPGGA;
GPGGA m_GPGGA;
// receive message from name0183
int8 uart_ins_rec_report( UART_TYPE uartid )
{
	int8	buff[MAX_UART_BUFF_LEN];
	int		ret;
	int		i;
	int		msg_len=0;
	memset(buff,0,MAX_UART_BUFF_LEN);
	
	static uint64_t last_time=0;
    uint64_t tnow = user_time::get_millis();
	if(last_time!=0&&(tnow-last_time)>1000)
	{
		ins_msg.insState.c_rmcValid='N';
	}

	ret = read_uart(uartid,buff,MAX_UART_BUFF_LEN);
	if(ret<=0){
	//s	printf("ins:: null ins data\n");
		return TRUE;
	}		
	//insNanoSend((int8*)buff, ret);
	for(i=0;i<ret;i++)
	{
		if((buff[i]=='$')||(1==ins_re_sign))
		{
			if(ins_jco>=127)
			{
				ins_jco=0;
				ins_re_sign=0;
				break;
			}
			else
			{
				insMsg[ins_jco++]=buff[i];
				ins_re_sign=1;
			}
		}
		if(buff[i]==0x0A)//LF
		{
			char* ptr=nullptr;
			if((insMsg[0]=='$')&&(insMsg[ins_jco-1]==10)&&(insMsg[ins_jco-2]==13))
			{
				
					msg_len = ins_jco-2;
					ins_jco = 0;
					if (GetCRC32(insMsg,msg_len))
					{
					//	printf("INS = %s\n",insMsg);

						if((insMsg[0]=='$')&&(insMsg[1]=='G')&&(insMsg[2]=='P')&&(insMsg[3]=='R')&&(insMsg[4]=='O')&&(insMsg[5]=='T'))
						{
							INS_GPROT_Analytical(insMsg);
						}
						if((insMsg[0]=='$')&&(insMsg[1]=='G')&&(insMsg[2]=='P')&&(insMsg[3]=='H')&&(insMsg[4]=='E')&&(insMsg[5]=='V'))
						{
							INS_GPHEV_Analytical(insMsg);
						}
						if((insMsg[0]=='$')&&(insMsg[1]=='P')&&(insMsg[2]=='S')&&(insMsg[3]=='A')&&(insMsg[4]=='T')&&(insMsg[5]==',')&&(insMsg[6]=='H')&&(insMsg[7]=='P')&&(insMsg[8]=='R'))
						{
							INS_PSAT_Analytical(insMsg,msg_len);//heading receive
						}
						if((insMsg[0]=='$')&&(insMsg[1]=='G')&&(insMsg[2]=='P')&&(insMsg[3]=='R')&&(insMsg[4]=='M')&&(insMsg[5]=='C'))
						{
							INS_GPRMC_Analytical(insMsg,msg_len);
						}
						if((insMsg[0]=='$')&&(insMsg[1]=='G')&&(insMsg[2]=='P')&&(insMsg[3]=='G')&&(insMsg[4]=='G')&&(insMsg[5]=='A'))
						{
							INS_GPGGA_Analytical(insMsg,msg_len);
						}
						if ((insMsg[0] == '$') && (insMsg[1] == 'G') && (insMsg[2] == 'P') && (insMsg[3] == 'G') && (insMsg[4] == 'S') && (insMsg[5] == 'T'))
						{
							INS_GPGST_Analytical(insMsg, msg_len);
						}
						if ((insMsg[0] == '$') && (insMsg[1] == 'G') && (insMsg[2] == 'P') && (insMsg[3] == 'H') && (insMsg[4] == 'D') && (insMsg[5] == 'T'))
						{

						}
						if(AP::conf()->boat_conf_.ins_type==2)
					   {

							if ((ptr = strstr(insMsg, "$PASHR")) != NULL)
							{
								float heading_T,roll,pitch,heaving;
								float tm;
								printf("get PASHR =%s\n",insMsg);
								sscanf(insMsg, "$PASHR,%f,%f,T,%f,%f,%f",&tm, &heading_T, &roll, &pitch,&heaving);
								Smart_Navigation_St.USV_Move_Heading = (uint16)(heading_T*100);	
								Smart_Navigation_St.USV_Roll = roll*100;
								Smart_Navigation_St.USV_Pitch = pitch*100;
								Smart_Navigation_St.USV_Heave= heaving*100;

								if(heading_T>360.0)
									heading_T -=360.0;
								if(heading_T<0.0)
									heading_T +=360.0;

								ins_msg.heading     = heading_T;
								ins_msg.u16_heading = (uint16)(heading_T*10);  

								ins_msg.i16_roll = roll*10;
								ins_msg.i16_pitch = pitch*10;
								ins_msg.i16_heaving = heaving*10;
								printf("ins_msg.heading=%lf,roll=%d,pitch=%d,heaving=%d\n",ins_msg.heading,ins_msg.i16_roll,ins_msg.i16_pitch,ins_msg.i16_heaving);
								
							}
							if ((ptr = strstr(insMsg, "$GPZDA")) != NULL)
							{

								float tm;
								int year,month,day;
								printf("get GPZDA =%s\n",insMsg);
								sscanf(insMsg, "$GPZDA,%f,%d,%d,%d",&tm, &day,&month,&year);
								Smart_Navigation_St.USV_Date =day;
								Smart_Navigation_St.USV_Month = month;
								Smart_Navigation_St.USV_Year = year-1970;

								ins_msg.u8_month	= Smart_Navigation_St.USV_Month;
								ins_msg.u8_date		= Smart_Navigation_St.USV_Date;
								ins_msg.u8_year		= Smart_Navigation_St.USV_Year;

								if(ins_msg.u8_year > 0)
								ins_msg.insState.b1_dateValid = 1;	
								printf("==========%d-%d-%d\n",ins_msg.u8_year,ins_msg.u8_month,ins_msg.u8_date);
							}
							if ((ptr = strstr(insMsg, "$GPVTG")) != NULL)
							{
								float heading_T,heading_M,speed_kn;
								printf("get GPVTG =%s\n",insMsg);
								sscanf(insMsg, "$GPVTG,%f,T,%f,M,%f,N", &heading_T, &heading_M, &speed_kn);

								Smart_Navigation_St.USV_Speed=(uint16)(speed_kn*100);				//�����ٶ�  000.0~999.9�� 1.852KM/h
								Smart_Navigation_St.USV_Move_Heading = (uint16)(heading_T*100);		//���溽��  0~36000   0.01��/bit
		
								ins_msg.speed =speed_kn;
								ins_msg.u16_speed   = (uint16)(speed_kn*10);
								
								if(heading_T>360.0)
									heading_T -=360.0;
								if(heading_T<0.0)
									heading_T +=360.0;
								ins_msg.motionDirection=heading_T;
								
								printf("%d-%d-%d,ins_msg.speed =%f,motionDirection=%d\n",ins_msg.u8_year,ins_msg.u8_month,ins_msg.u8_date,speed_kn,ins_msg.motionDirection);
							
							}
						}
						if (ins_sockid > 0){
							insUdpSend((int8*)insMsg, msg_len + 2);
						}
					}

				
				memset(insMsg,0,128);
				ins_re_sign=0;
				monitor_all_inf.monitor_comm_inf[MONITOR_COMM_INS_SN].rec_ok_number++;	//������ȷ����
				comm_time_return(&(ins_sign.timer),&(ins_sign.comm_sign));
				last_time = user_time::get_millis();
				
			}
		}
	}
}
//Ѱ�ұ�����ʼ��
void Get_beginend_num(char *buff,char *begin_num,char *end_num,char *begin_sign,char *end_sign,char *count_sign,char *Symbol,int length)
{
	unsigned char count=0,symbol_sign;
	int ico,jco;

	for(ico=0;ico<length;ico++)
	{
		if(buff[ico]==0x2c)								//\u9017\u53f7 2c--�ָ���
		{
			count++;
		}
		if((1==count)&&(0==begin_sign[count-1]))
		{
			begin_num[count-1]=ico+1;
			begin_sign[count-1]=1;
		}
		else if(count>1)
		{
			if(0==end_sign[count-2])
			{
				end_num[count-2]=ico;
				end_sign[count-2]=1;
			}
			if(0==begin_sign[count-1])
			{
				begin_num[count-1]=ico+1;
				begin_sign[count-1]=1;
			}
		}
	}
	for(jco=0;jco<20;jco++)
	{
		symbol_sign=begin_num[jco];
		if(buff[symbol_sign]=='-')//\u8d1f\u53f7
		{
			Symbol[jco]=1;
			begin_num[jco]++;
		}	
	}
}
//��þ���ֵ
void Get_value(char *buff,double * value,char *begin_num,char *end_num)
{
	int ico,jco;
	char min_num[20],count_sign[20];
	memset(min_num,0,sizeof(min_num));
	memset(count_sign,0,sizeof(count_sign));

	for(jco=0;jco<20;jco++)
	{
		if (0==end_num[jco])
			return;
		for(ico=begin_num[jco];ico<end_num[jco];ico++)
		{
			if(buff[ico]==0x2e)				//�ԣ�Ϊ�ָ���
			{
				min_num[jco]=1;				
				continue;
			}
			if(1!=min_num[jco])
			{
				value[jco]*=10.0;
				value[jco]+=AscToHex(buff[ico]);			//��ASCII��ת16������
			}
			else
			{
				count_sign[jco]++;
				value[jco]+=AscToHex(buff[ico])*(pow(0.1,count_sign[jco]));
			}
		}	
	}
}
//�ߵ�ROT��Ϣ����
//$GPROT,181.0,A*39<CR><LF>��ʾ181.0��/minת����1Hz
void INS_GPROT_Analytical(char  *buff)
{
	uint8 Symbol;
	float ROT;
	int i,count,begin_num=0,end_num=0,min_num=0,count_sign=0,begin_sign=0,end_sign=0;
	Symbol=0;
	ROT=0.0;
	count=0;
	for(i=0;i<32;i++)
	{
		if(buff[i]==0x2c)
		{
			count++;
		}
		if((1==count)&&(0==begin_sign))
		{
			begin_num=i+1;
			begin_sign=1;
		}
		if((2==count)&&(0==end_sign))
		{
			end_num=i;
			end_sign=1;
		}
	}
	if(buff[begin_num]=='-')//��ֵ
	{
		Symbol=1;
		begin_num++;
	}
	for(i=begin_num;i<end_num;i++)
	{
		if(buff[i]==0x2e)
		{
			min_num=1;
			continue;
		}
		if(1!=min_num)
		{
			ROT*=10.0;
			ROT+=AscToHex(buff[i]);
		}
		else
		{
			count_sign++;
			ROT=(float)(ROT+AscToHex(buff[i])*(pow(0.1,count_sign)));
		}
	}
//	printf("ROT=%f\n",ROT);
	ROT=(float)(ROT/60.0);//ÿ����ת����
	if(Symbol)
		Smart_Navigation_St.USV_ROT=(uint16)(10000-ROT*100);
	else
		Smart_Navigation_St.USV_ROT=(uint16)(10000+ROT*100);

	if (Symbol)
	{
		ins_msg.rotRate = -ROT;
		ins_msg.i16_rot = (int16)(-ROT * 10);//modify @foo 0.1��/s
	}
	else
	{
		ins_msg.rotRate = ROT;
		ins_msg.i16_rot = (int16)(ROT * 10);//modify @foo 0.1��/s
	}
	//printf("ins_msg.i16_rot=%d\n", ins_msg.i16_rot);
//	printf("ROT=%d\n",Smart_Navigation_St.USV_ROT);
	return  ;
}
//�ߵ���������
//$GPHEV,0.5,1*CC<><>��ʾ���0.5m				5Hz
void INS_GPHEV_Analytical(char  *buff)
{
	uint8 Symbol;
	int i,count=0,begin_num=0,end_num=0,min_num=0,count_sign=0,begin_sign=0,end_sign=0;
	float Heave;
	
	Symbol=0;
	Heave=0.0;
	for(i=0;i<32;i++)
	{
		if(buff[i]==0x2c)
		{
			count++;
		}
		if((1==count)&&(0==begin_sign))
		{
			begin_num=i+1;
			begin_sign=1;
		}
		if((2==count)&&(0==end_sign))
		{
			end_num=i;
			end_sign=1;
		}
	}
	if(buff[begin_num]=='-')//��ֵ
	{
		Symbol=1;
		begin_num++;
	}
	for(i=begin_num;i<end_num;i++)
	{
		if(buff[i]==0x2e)
		{
			min_num=1;
			continue;
		}
		if(1!=min_num)
		{
			Heave*=10.0;
			Heave+=AscToHex(buff[i]);
		}
		else
		{
			count_sign++;
			Heave+=(float)(AscToHex(buff[i])*(pow(0.1,count_sign)));
		}
	}
//	printf("Heave=%f!!!!!!!!!!!!!!!!!!!!!!\n",Heave);	
	Heave=Heave*100;
	if(Symbol)
		Smart_Navigation_St.USV_Heave=(uint16)(500-Heave);
	else
		Smart_Navigation_St.USV_Heave=(uint16)(Heave+500);

	if(Symbol)
		ins_msg.i16_heaving = (int16)(-Heave);
	else
		ins_msg.i16_heaving = (int16)(Heave);
//	printf("Heave=%d\n",Smart_Navigation_St.USV_Heave);
	return  ;
}
//�ߵ���̬��Ϣ
//$PSAT,HPR,173822.60,274.86,-12.9,3.85,N*0F			10Hz
/*
173822.60 UTCʱ��
274.86 ����
-12.9����
3.85���
N ����ģʽN GPS G����
*/
void INS_PSAT_Analytical(char  *buff,int length)
{
	double value[20];
	char begin_num[20],end_num[20],begin_sign[20],end_sign[20],count_sign[20],Symbol[20];
	int sys_state_num=0;
//	int ico;
	memset(Symbol,0,sizeof(Symbol));
	memset(begin_num,0,sizeof(begin_num));
	memset(end_num,0,sizeof(end_num));
	memset(begin_sign,0,sizeof(begin_sign));
	memset(end_sign,0,sizeof(end_sign));
	memset(count_sign,0,sizeof(count_sign));
	memset((char*)(&value),0,sizeof(value));
	Get_beginend_num(buff,begin_num,end_num,begin_sign,end_sign,count_sign,Symbol,length);
	Get_value(buff,value,begin_num,end_num);
/*	for(ico=0;ico<20;ico++)
		printf("value[%d]=%lf\n",ico,value[ico]);*/
	Smart_Navigation_St.USV_Hour=(uint8)(value[1]/10000);
	Smart_Navigation_St.USV_Minute=(uint8)(value[1]/100-100*Smart_Navigation_St.USV_Hour);
	Smart_Navigation_St.USV_Second=(uint8)(value[1]-10000*Smart_Navigation_St.USV_Hour-100*Smart_Navigation_St.USV_Minute);
	Smart_Navigation_St.USV_Second_2=(uint8)(100*(value[0]-(int)(value[0])));
//	Smart_Navigation_St.USV_Heading=(uint16)(value[2]*100);
	//Smart_Navigation_St.USV_Position_Heading=(uint16)(value[2]*100);
	
	//������Դ
	ins_msg.u8_hour    = (uint8)(value[1]/10000);
	ins_msg.u8_minute  = (uint8)(value[1]/100-100*ins_msg.u8_hour);
	ins_msg.u8_second  = (uint8)(value[1]-10000*ins_msg.u8_hour-100*ins_msg.u8_minute);
	ins_msg.u8_second_2=(uint8)(100*(value[1]-(int)(value[1])));

	ins_msg.insState.b1_timeValid = 1;

	double heading_postion =  value[2];
	heading_postion += INS_HEADING_COR;

	if(heading_postion>360.0)
		heading_postion -=360.0;
	if(heading_postion<0.0)
		heading_postion +=360.0;

	Smart_Navigation_St.USV_Heading=(uint16)(heading_postion*100);

	if(Symbol[2])
		Smart_Navigation_St.USV_Pitch=(int16)(-100*value[3]);
	else
		Smart_Navigation_St.USV_Pitch=(int16)(100*value[3]);

	if(Symbol[3])
		Smart_Navigation_St.USV_Roll=(int16)(-100*value[4]);
	else
		Smart_Navigation_St.USV_Roll=(int16)(100*value[4]);

	//������Դ
	ins_msg.heading     = heading_postion;
	ins_msg.u16_heading = (uint16)(heading_postion*10);  
	//printf("ins_msg.u16_heading == %d\n", ins_msg.u16_heading);
	
	if(Symbol[2])
		ins_msg.i16_pitch = (int16)(10*value[3]);
	else
		ins_msg.i16_pitch = (int16)(-10*value[3]);
	
	if(Symbol[3])
		ins_msg.i16_roll = (uint16)(10*value[4]);
	else
		ins_msg.i16_roll = (uint16)(-10*value[4]);

	sys_state_num=begin_num[5];
	if(buff[sys_state_num]=='N')//GPS����
	{
		Smart_Navigation_St.Sys_State_1=4;
		ins_msg.insState.u8_sysState1 = 4;
	}
	else if(buff[sys_state_num]=='G')//���ݶ���
	{
		Smart_Navigation_St.Sys_State_1=2;
		ins_msg.insState.u8_sysState1 = 2;
	}
	else 
	{
		ins_msg.insState.u8_sysState1 = 3;
	}
	//printf("u8_sysState1=%d\n",ins_msg.insState.u8_sysState1);
	if (ins_msg.insState.u8_sysState1_old != ins_msg.insState.u8_sysState1 && poweron_init)
	{
		if (ins_msg.insState.u8_sysState1 == 2)
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_GYRO_HEADING, WARN_ON);
		}
		
		if (ins_msg.insState.u8_sysState1 == 4)
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_GYRO_HEADING, WARN_OFF);
		}
	}

	ins_msg.insState.u8_sysState1_old = ins_msg.insState.u8_sysState1;
/*	printf("Smart_Navigation_St.USV_Hour=%d\n",Smart_Navigation_St.USV_Hour);
	printf("Smart_Navigation_St.USV_Minute=%d\n",Smart_Navigation_St.USV_Minute);
	printf("Smart_Navigation_St.USV_Second=%d\n",Smart_Navigation_St.USV_Second);
	printf("Smart_Navigation_St.USV_Second_2=%d\n",Smart_Navigation_St.USV_Second_2);
	printf("Smart_Navigation_St.USV_Heading=%d\n",Smart_Navigation_St.USV_Heading);
	printf("Smart_Navigation_St.USV_Pitch=%d\n",Smart_Navigation_St.USV_Pitch);
	printf("Smart_Navigation_St.USV_Roll=%d\n",Smart_Navigation_St.USV_Roll);
	printf("Smart_Navigation_St.Sys_State_1=%d\n",Smart_Navigation_St.Sys_State_1);*/
	return  ;
}
//�ӹߵ������л�ȡ ��λ��Ϣ��UTC ʱ�䡢UTC ���ڡ����ʡ�����������γ�ȡ���ƫ�Ǽ�����
//��λ��Ϣ����
//$GPRMC,HHMMSS.SS,A,DDMM.MMM,N,DDDMM.MMM,W,Z.Z,Y.Y,DDMMYY,D.D,V *CC<CR><LF>		10Hz
/*HHMMSS.SS  UTCʱ����
A ��Ч��λ��V��Ч��λ
ddmm.mmmmmά��d��ʾ�ȣ�m��ʾ��
N ��ʾ��γ��S��ʾ��γ
ddmm.mmmmm����d��ʾ�ȣ�m��ʾ��
E����W����
z.z�������ʽ�
y.y���溽���(�汱)
ddmmyy UTC����
d.d��ƫ��
V ��ƫ�Ƿ���E��W��
*/
void INS_GPRMC_Analytical(char  *buff,int length)
{
	double value[20],lng=0.0,lat=0.0;
	double temp_lat = 0.0, temp_lng = 0.0;
	char begin_num[20],end_num[20],begin_sign[20],end_sign[20],count_sign[20],Symbol[20];
	int sys_state_num=0;
	static double last_lat=0;
	static double last_long=0;
	static	uint8	u8_locationState=0;
	static uint16  u16_pseudoRangeError=0;
//	int ico;
	memset(Symbol,0,sizeof(Symbol));
	memset(begin_num,0,sizeof(begin_num));
	memset(end_num,0,sizeof(end_num));
	memset(begin_sign,0,sizeof(begin_sign));
	memset(end_sign,0,sizeof(end_sign));
	memset(count_sign,0,sizeof(count_sign));
	memset((char*)(&value),0,sizeof(value));
	Get_beginend_num(buff,begin_num,end_num,begin_sign,end_sign,count_sign,Symbol,length);	//��ȡ��ʼ�� ����ֹ��
	Get_value(buff,value,begin_num,end_num);				//��ø�������
/*	for(ico=0;ico<20;ico++)
		printf("value[%d]=%lf\n",ico,value[ico]);*/

	Smart_Navigation_St.USV_Longitude_Degree=(uint8)(value[2]/100);								//value[2]--����
	Smart_Navigation_St.USV_Longitude_Minute=(uint8)(value[2]-100*((int)(value[2]/100)));
	lng=(value[2]-(int)(value[2]))*60;
	Smart_Navigation_St.USV_Longitude_Second=(int)(lng);										//��
	Smart_Navigation_St.USV_Longitude_Decimal_2=(uint8)((lng-(int)(lng))*100);					//��С��λ1��2
	Smart_Navigation_St.USV_Longitude_Decimal_4=(uint8)(((lng-(int)(lng))*100-(int)((lng-(int)(lng))*100))*100);  //��С��λ3��4
	
	sys_state_num=begin_num[3];							
	if(buff[sys_state_num]=='N')						//������
		Smart_Navigation_St.Longitude_Sign_St=1;
	if(buff[sys_state_num]=='S')						//�ϰ���
		Smart_Navigation_St.Longitude_Sign_St=2;	
	
	Smart_Navigation_St.USV_Latitude_Degree=(uint8)(value[4]/100);							//����
	Smart_Navigation_St.USV_Latitude_Minute=(uint8)(value[4]-100*((int)(value[4]/100)));
	lat=(value[4]-(int)(value[4]))*60;
	Smart_Navigation_St.USV_Latitude_Second=(uint8)(lat);
	Smart_Navigation_St.USV_Latitude_Decimal_2=(uint8)((lat-(int)(lat))*100);
	Smart_Navigation_St.USV_Latitude_Decimal_4=(uint8)(((lat-(int)(lat))*100-(int)((lat-(int)(lat))*100))*100);
	
	sys_state_num=begin_num[5];
	if(buff[sys_state_num]=='E')
		Smart_Navigation_St.Latitude_Sign_St=1;
	if(buff[sys_state_num]=='W')
		Smart_Navigation_St.Latitude_Sign_St=2;	

	Smart_Navigation_St.USV_Lat=Smart_Navigation_St.USV_Latitude_Degree;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Minute))/60.0;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Second))/3600.0;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Decimal_2))/360000.0;
	Smart_Navigation_St.USV_Lng=Smart_Navigation_St.USV_Longitude_Degree;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Minute))/60.0;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Second))/3600.0;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Decimal_2))/360000.0;	
	
	Smart_Navigation_St.USV_Speed=(uint16)(value[6]*100);				//�����ٶ�  000.0~999.9�� 1.852KM/h
	Smart_Navigation_St.USV_Move_Heading = (uint16)(value[7]*100);		//���溽��  0~36000   0.01��/bit
	Smart_Navigation_St.USV_Date=(uint8)(value[8]/10000);				//UTC ddmmyy
	Smart_Navigation_St.USV_Month=(uint8)(value[8]/100-(100*(int)(value[8]/10000)));
	Smart_Navigation_St.USV_Year=(uint8)(value[8]-10000*(Smart_Navigation_St.USV_Date)-100*(Smart_Navigation_St.USV_Month));

	//�µ�����Դ
	// ins_msg.motionDirection = value[7];
	// double ground_seppd_kn = value[6];
	// double deltAng   = 	ins_msg.motionDirection  - ins_msg.heading;
	// ins_msg.speed    =  ground_seppd_kn * cos(Radian(deltAng));
   //�µ�����Դ
	ins_msg.speed = value[6];
	ins_msg.motionDirection = value[7];
	
	ins_msg.u16_speed   = (uint16)(value[6]*10);
	ins_msg.u16_volecityDir = (uint16)(value[7] * 10);
	ins_msg.u8_month	= (uint8)(value[8]/100-(100*(int)(value[8]/10000)));
	ins_msg.u8_date		= (uint8)(value[8]/10000);
	ins_msg.u8_year		= (uint8)(value[8]-10000*(ins_msg.u8_date)-100*(ins_msg.u8_month));
	//printf("ins_msg.u8_year ==%d \n",ins_msg.u8_year);
	if(ins_msg.u8_year > 0)
		ins_msg.insState.b1_dateValid = 1;		//������Ч��־

	//��λ��Ч
	sys_state_num = begin_num[1];
	ins_msg.insState.c_rmcValid = buff[sys_state_num];

	if (ins_msg.insState.c_rmcValid_old != ins_msg.insState.c_rmcValid && poweron_init)
	{
		if (ins_msg.insState.c_rmcValid == 'A')
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_LOC_INVALID, WARN_OFF);
		}
		
		if (ins_msg.insState.c_rmcValid == 'V')
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_LOC_INVALID, WARN_ON);
		}
	}

	ins_msg.insState.c_rmcValid_old = ins_msg.insState.c_rmcValid;

	uint8 temp_NS;
	uint8 temp_u8_latiDeg, temp_u8_latiMin, temp_u8_latiSec, temp_u8_latiSecDec, temp_u8_latiSecDec2; 

	uint8 temp_EW;
	uint8 temp_u8_longiDeg, temp_u8_longiMin, temp_u8_longiSec, temp_u8_longiSecDec, temp_u8_longiSecDec2; 


	if (rtk_locating_fuc){ 
		//echo app
		ins_msg.u8_latiSt = irtk_msg.u8_latiSt;
		ins_msg.u8_latiDeg = irtk_msg.u8_latiDeg;
		ins_msg.u8_latiMin = irtk_msg.u8_latiMin;
		ins_msg.u8_latiSec = irtk_msg.u8_latiSec;
		ins_msg.u8_latiSecDec = irtk_msg.u8_latiSecDec;
		ins_msg.u8_latiSecDec2 = irtk_msg.u8_latiSecDec2;

		ins_msg.u8_longiSt = irtk_msg.u8_longiSt;
		ins_msg.u8_longiDeg = irtk_msg.u8_longiDeg;
		ins_msg.u8_longiMin = irtk_msg.u8_longiMin;
		ins_msg.u8_longiSec = irtk_msg.u8_longiSec;
		ins_msg.u8_longiSecDec = irtk_msg.u8_longiSecDec;
		ins_msg.u8_longiSecDec2 = irtk_msg.u8_longiSecDec2;

		// ins_msg.u8_latiSecDec = irtk_msg.u8_latiSecDec;
		// ins_msg.u16_speed = irtk_msg.u16_speed;
		// ins_msg.u16_heading = irtk_msg.u16_heading;
		// ins_msg.u16_volecityDir = irtk_msg.u16_volecityDir;


		// ins_msg.i16_pitch = irtk_msg.i16_pitch;
		// ins_msg.i16_roll = irtk_msg.i16_roll;
		// ins_msg.i16_heaving	 = irtk_msg.i16_heaving;
		// ins_msg.i16_rot = irtk_msg.i16_rot;

		//control
		ins_msg.latitude = irtk_msg.latitude;
		ins_msg.longitude = irtk_msg.longitude;
		//ins_msg.speed = irtk_msg.speed;
		//ins_msg.heading = irtk_msg.heading;
		//ins_msg.rotRate = irtk_msg.rotRate;
		//printf(">>>LOCATION_RTK\n");
	}
	else if (ins_msg.insState.c_rmcValid == 'A')
	{
		//γ��
		temp_u8_latiDeg = (uint8)(value[2] / 100);								//value[2]--γ��
		temp_u8_latiMin = (uint8)(value[2] - 100 * ((int)(value[2] / 100)));
		lat = (value[2] - (int)(value[2])) * 60;
		temp_u8_latiSec = (int)(lat);										//��
		temp_u8_latiSecDec = (uint8)((lat - (int)(lat)) * 100);					//��С��λ1��2
		temp_u8_latiSecDec2 = (uint8)(((lat - (int)(lat)) * 100 - (int)((lat - (int)(lat)) * 100)) * 100);

		sys_state_num = begin_num[3];
		if (buff[sys_state_num] == 'N')						//������
			temp_NS = 1;
		if (buff[sys_state_num] == 'S')						//�ϰ���
			temp_NS = 0;

		//����
		temp_u8_longiDeg = (uint8)(value[4] / 100);							//����
		temp_u8_longiMin = (uint8)(value[4] - 100 * ((int)(value[4] / 100)));
		lng = (value[4] - (int)(value[4])) * 60;
		temp_u8_longiSec = (uint8)(lng);
		temp_u8_longiSecDec = (uint8)((lng - (int)(lng)) * 100);
		temp_u8_longiSecDec2 = (uint8)(((lng - (int)(lng)) * 100 - (int)((lng - (int)(lng)) * 100)) * 100);

		sys_state_num = begin_num[5];
		if (buff[sys_state_num] == 'E')						
			temp_EW = 1;
		if (buff[sys_state_num] == 'W')						
			temp_EW = 0;

	
		temp_lat = (temp_NS == 0 ? -1 : 1)*(ins_msg.u8_latiDeg + ((double)(ins_msg.u8_latiMin)) / 60.0 + \
												 ((double)(ins_msg.u8_latiSec)) / 3600.0+\
												 ((double)(ins_msg.u8_latiSecDec)) / 360000.0);
		temp_lng = (temp_EW == 0 ? -1 : 1)*(ins_msg.u8_longiDeg + ((double)(ins_msg.u8_longiMin)) / 60.0 + \
												 ((double)(ins_msg.u8_longiSec)) / 3600.0 + \
												 ((double)(ins_msg.u8_longiSecDec)) / 360000.0);
		ins_msg.u8_latiSt = temp_NS;
		ins_msg.u8_latiDeg = temp_u8_latiDeg;
		ins_msg.u8_latiMin = temp_u8_latiMin;
		ins_msg.u8_latiSec = temp_u8_latiSec;
		ins_msg.u8_latiSecDec = temp_u8_latiSecDec;
		ins_msg.u8_latiSecDec2 = temp_u8_latiSecDec2;

		ins_msg.u8_longiSt = temp_EW;
		ins_msg.u8_longiDeg = temp_u8_longiDeg;
		ins_msg.u8_longiMin = temp_u8_longiMin;
		ins_msg.u8_longiSec = temp_u8_longiSec;
		ins_msg.u8_longiSecDec = temp_u8_longiSecDec;
		ins_msg.u8_latiSecDec2 = temp_u8_longiSecDec2;

		ins_msg.latitude = temp_lat;
		ins_msg.longitude = temp_lng;
	}

	u16_pseudoRangeError = (uint16)((rtk_locating_fuc == 1 ? irtk_msg.pseudorRangeError : ins_msg.pseudorRangeError) * 100);//rtk / ins ��λ
    u8_locationState = rtk_locating_fuc == 1 ? irtk_msg.rtk_state.b1_diffSignalValid : ins_msg.insState.b1_diffSignalValid;

	if(last_lat>=1&&last_lat>=1)
	{
		double distance = fabsf(Get_distance(ins_msg.latitude,ins_msg.longitude,last_lat,last_long));
		if(distance >2)
		{
			LOG(ERROR)<<"ins_get::ins_msg jump from "<<last_lat<<" ,"<<last_long<<" to "
					  <<ins_msg.latitude<<" , "<<ins_msg.longitude
					  <<" u16_pseudoRangeError = "<<u16_pseudoRangeError
					  <<" u8_locationState = "<<u8_locationState
					  <<" distance = "<<distance
					  <<std::endl;
		}
	}
	last_lat = ins_msg.latitude;
	last_long = ins_msg.longitude;
	
}
//GPSλ����Ϣ
//$GPGGA,HHMMSS.SS,DDMM.MMMM,S,DDDMM.MMMM,S,N,QQ,PP.P,SAAAAA.AA,M,��XXXX.XX,M,SSS,AAAA*CC<CR><LF>	5Hz
/*HHMMSS.SS  UTCʱ����
A ��Ч��λ��V��Ч��λ
ddmm.mmmmmά��d��ʾ�ȣ�m��ʾ��
N ��ʾ��γ��S��ʾ��γ
ddmm.mmmmm����d��ʾ�ȣ�m��ʾ��
E����W����
N GPS��λ��־
qq���Ƕ�λ��Ŀ
p.p
SAAAAA.AA ����
M��
*/
void INS_GPGGA_Analytical(char  *buff,int length)
{

	double value[20],lng=0.0,lat=0.0;
	double temp_lat = 0.0, temp_lng = 0.0;
	int sys_state_num=0;
	static double last_lat=0;
	static double last_long=0;
	static	uint8	u8_locationState=0;
	static uint16  u16_pseudoRangeError=0;

	char begin_num[20],end_num[20],begin_sign[20],end_sign[20],count_sign[20],Symbol[20];
//	int ico;
	memset(Symbol,0,sizeof(Symbol));
	memset(begin_num,0,sizeof(begin_num));
	memset(end_num,0,sizeof(end_num));
	memset(begin_sign,0,sizeof(begin_sign));
	memset(end_sign,0,sizeof(end_sign));
	memset(count_sign,0,sizeof(count_sign));
	memset((char*)(&value),0,sizeof(value));
	Get_beginend_num(buff,begin_num,end_num,begin_sign,end_sign,count_sign,Symbol,length);
	Get_value(buff,value,begin_num,end_num);
/*	for(ico=0;ico<20;ico++)
		printf("value[%d]=%lf\n",ico,value[ico]);*/

	//lat lng


	if(2==value[5])
	{
		Smart_Navigation_St.Differerntial_Signal_Light=1;
		Smart_Navigation_St.Differential_Position_Light=1;
	//	ins_msg.insState.b1_diffSignalValid  = 1;
		ins_msg.insState.b1_diffPostionValid = 1;
	}
	else
	{
		Smart_Navigation_St.Differerntial_Signal_Light=0;
		Smart_Navigation_St.Differential_Position_Light=0;
	//	ins_msg.insState.b1_diffSignalValid  = 0;
		ins_msg.insState.b1_diffPostionValid = 0;
	}

	ins_msg.insState.b1_diffSignalValid = value[5];

	if (ins_msg.insState.b1_diffSignalValid != ins_msg.insState.b1_diffSignalValid_old && poweron_init){
		if (ins_msg.insState.b1_diffSignalValid >= 2)
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_DIF_INVALID, WARN_OFF);
		}
		if (ins_msg.insState.b1_diffSignalValid == 1)
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_DIF_INVALID, WARN_ON);
		}
			
	}

	ins_msg.insState.b1_diffSignalValid_old = ins_msg.insState.b1_diffSignalValid;

	Smart_Navigation_St.Satellite_Num_1=(int)value[6];
	ins_msg.insState.u8_sateliteNum1 = (int)value[6];		//����1�������Ǹ���
	if(Symbol[8])
		Smart_Navigation_St.USV_Height=(uint32)(-100*value[8]+200000);
	else
		Smart_Navigation_St.USV_Height=(uint32)(100*value[8]+200000);
/*	printf("Differerntial_Signal_Light=%d\n",Smart_Navigation_St.Differerntial_Signal_Light);
	printf("Differential_Position_Light=%d\n",Smart_Navigation_St.Differential_Position_Light);
	printf("Satellite_Num_1=%d\n",Smart_Navigation_St.Satellite_Num_1);
	printf("USV_Height=%d\n",Smart_Navigation_St.USV_Height);	*/
	if(AP::conf()->boat_conf_.ins_type==2)
	{

	Smart_Navigation_St.USV_Hour=(uint8)(value[0]/10000);
	Smart_Navigation_St.USV_Minute=(uint8)(value[0]/100-100*Smart_Navigation_St.USV_Hour);
	Smart_Navigation_St.USV_Second=(uint8)(value[0]-10000*Smart_Navigation_St.USV_Hour-100*Smart_Navigation_St.USV_Minute);
	Smart_Navigation_St.USV_Second_2=(uint8)(100*(value[0]-(int)(value[0])));
	
	ins_msg.u8_hour    = (uint8)(value[0]/10000);
	ins_msg.u8_minute  = (uint8)(value[0]/100-100*ins_msg.u8_hour);
	ins_msg.u8_second  = (uint8)(value[0]-10000*ins_msg.u8_hour-100*ins_msg.u8_minute);
	ins_msg.u8_second_2=(uint8)(100*(value[0]-(int)(value[0])));

	if(value[0]>0.0001)
		ins_msg.insState.b1_timeValid = 1;



	Smart_Navigation_St.USV_Longitude_Degree=(uint8)(value[1]/100);								//value[2]--����
	Smart_Navigation_St.USV_Longitude_Minute=(uint8)(value[1]-100*((int)(value[1]/100)));
	lng=(value[1]-(int)(value[1]))*60;
	Smart_Navigation_St.USV_Longitude_Second=(int)(lng);										//��
	Smart_Navigation_St.USV_Longitude_Decimal_2=(uint8)((lng-(int)(lng))*100);					//��С��λ1��2
	Smart_Navigation_St.USV_Longitude_Decimal_4=(uint8)(((lng-(int)(lng))*100-(int)((lng-(int)(lng))*100))*100);  //��С��λ3��4
	
	sys_state_num=begin_num[2];							
	if(buff[sys_state_num]=='N')						//������
		Smart_Navigation_St.Longitude_Sign_St=1;
	if(buff[sys_state_num]=='S')						//�ϰ���
		Smart_Navigation_St.Longitude_Sign_St=2;	
	
	Smart_Navigation_St.USV_Latitude_Degree=(uint8)(value[3]/100);							//����
	Smart_Navigation_St.USV_Latitude_Minute=(uint8)(value[3]-100*((int)(value[3]/100)));
	lat=(value[3]-(int)(value[3]))*60;
	Smart_Navigation_St.USV_Latitude_Second=(uint8)(lat);
	Smart_Navigation_St.USV_Latitude_Decimal_2=(uint8)((lat-(int)(lat))*100);
	Smart_Navigation_St.USV_Latitude_Decimal_4=(uint8)(((lat-(int)(lat))*100-(int)((lat-(int)(lat))*100))*100);
	
	sys_state_num=begin_num[4];
	if(buff[sys_state_num]=='E')
		Smart_Navigation_St.Latitude_Sign_St=1;
	if(buff[sys_state_num]=='W')
		Smart_Navigation_St.Latitude_Sign_St=2;	

	Smart_Navigation_St.USV_Lat=Smart_Navigation_St.USV_Latitude_Degree;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Minute))/60.0;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Second))/3600.0;
	Smart_Navigation_St.USV_Lat+=((double)(Smart_Navigation_St.USV_Latitude_Decimal_2))/360000.0;
	Smart_Navigation_St.USV_Lng=Smart_Navigation_St.USV_Longitude_Degree;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Minute))/60.0;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Second))/3600.0;
	Smart_Navigation_St.USV_Lng+=((double)(Smart_Navigation_St.USV_Longitude_Decimal_2))/360000.0;	
	


	if(ins_msg.insState.b1_diffSignalValid >= 1)
		ins_msg.insState.c_rmcValid='A';
	else
		ins_msg.insState.c_rmcValid='V';

	if (ins_msg.insState.c_rmcValid_old != ins_msg.insState.c_rmcValid && poweron_init)
	{
		if (ins_msg.insState.c_rmcValid == 'A')
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_LOC_INVALID, WARN_OFF);
		}
		
		if (ins_msg.insState.c_rmcValid == 'V')
		{
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_LOC_INVALID, WARN_ON);
		}
	}

	ins_msg.insState.c_rmcValid_old = ins_msg.insState.c_rmcValid;

	uint8 temp_NS;
	uint8 temp_u8_latiDeg, temp_u8_latiMin, temp_u8_latiSec, temp_u8_latiSecDec, temp_u8_latiSecDec2; 

	uint8 temp_EW;
	uint8 temp_u8_longiDeg, temp_u8_longiMin, temp_u8_longiSec, temp_u8_longiSecDec, temp_u8_longiSecDec2; 


	if (rtk_locating_fuc){ 
		//echo app
		ins_msg.u8_latiSt = irtk_msg.u8_latiSt;
		ins_msg.u8_latiDeg = irtk_msg.u8_latiDeg;
		ins_msg.u8_latiMin = irtk_msg.u8_latiMin;
		ins_msg.u8_latiSec = irtk_msg.u8_latiSec;
		ins_msg.u8_latiSecDec = irtk_msg.u8_latiSecDec;
		ins_msg.u8_latiSecDec2 = irtk_msg.u8_latiSecDec2;

		ins_msg.u8_longiSt = irtk_msg.u8_longiSt;
		ins_msg.u8_longiDeg = irtk_msg.u8_longiDeg;
		ins_msg.u8_longiMin = irtk_msg.u8_longiMin;
		ins_msg.u8_longiSec = irtk_msg.u8_longiSec;
		ins_msg.u8_longiSecDec = irtk_msg.u8_longiSecDec;
		ins_msg.u8_longiSecDec2 = irtk_msg.u8_longiSecDec2;

		// ins_msg.u8_latiSecDec = irtk_msg.u8_latiSecDec;
		// ins_msg.u16_speed = irtk_msg.u16_speed;
		// ins_msg.u16_heading = irtk_msg.u16_heading;
		// ins_msg.u16_volecityDir = irtk_msg.u16_volecityDir;


		// ins_msg.i16_pitch = irtk_msg.i16_pitch;
		// ins_msg.i16_roll = irtk_msg.i16_roll;
		// ins_msg.i16_heaving	 = irtk_msg.i16_heaving;
		// ins_msg.i16_rot = irtk_msg.i16_rot;

		//control
		ins_msg.latitude = irtk_msg.latitude;
		ins_msg.longitude = irtk_msg.longitude;
		//ins_msg.speed = irtk_msg.speed;
		//ins_msg.heading = irtk_msg.heading;
		//ins_msg.rotRate = irtk_msg.rotRate;
		//printf(">>>LOCATION_RTK\n");
	}
	else if (ins_msg.insState.c_rmcValid == 'A')
	{
		//γ��
		temp_u8_latiDeg = (uint8)(value[1] / 100);								//value[2]--γ��
		temp_u8_latiMin = (uint8)(value[1] - 100 * ((int)(value[1] / 100)));
		lat = (value[2] - (int)(value[1])) * 60;
		temp_u8_latiSec = (int)(lat);										//��
		temp_u8_latiSecDec = (uint8)((lat - (int)(lat)) * 100);					//��С��λ1��2
		temp_u8_latiSecDec2 = (uint8)(((lat - (int)(lat)) * 100 - (int)((lat - (int)(lat)) * 100)) * 100);

		sys_state_num = begin_num[2];
		if (buff[sys_state_num] == 'N')						//������
			temp_NS = 1;
		if (buff[sys_state_num] == 'S')						//�ϰ���
			temp_NS = 0;

		//����
		temp_u8_longiDeg = (uint8)(value[3] / 100);							//����
		temp_u8_longiMin = (uint8)(value[4] - 100 * ((int)(value[3] / 100)));
		lng = (value[3] - (int)(value[3])) * 60;
		temp_u8_longiSec = (uint8)(lng);
		temp_u8_longiSecDec = (uint8)((lng - (int)(lng)) * 100);
		temp_u8_longiSecDec2 = (uint8)(((lng - (int)(lng)) * 100 - (int)((lng - (int)(lng)) * 100)) * 100);

		sys_state_num = begin_num[4];
		if (buff[sys_state_num] == 'E')						
			temp_EW = 1;
		if (buff[sys_state_num] == 'W')						
			temp_EW = 0;

	
		temp_lat = (temp_NS == 0 ? -1 : 1)*(ins_msg.u8_latiDeg + ((double)(ins_msg.u8_latiMin)) / 60.0 + \
												 ((double)(ins_msg.u8_latiSec)) / 3600.0+\
												 ((double)(ins_msg.u8_latiSecDec)) / 360000.0);
		temp_lng = (temp_EW == 0 ? -1 : 1)*(ins_msg.u8_longiDeg + ((double)(ins_msg.u8_longiMin)) / 60.0 + \
												 ((double)(ins_msg.u8_longiSec)) / 3600.0 + \
												 ((double)(ins_msg.u8_longiSecDec)) / 360000.0);
		ins_msg.u8_latiSt = temp_NS;
		ins_msg.u8_latiDeg = temp_u8_latiDeg;
		ins_msg.u8_latiMin = temp_u8_latiMin;
		ins_msg.u8_latiSec = temp_u8_latiSec;
		ins_msg.u8_latiSecDec = temp_u8_latiSecDec;
		ins_msg.u8_latiSecDec2 = temp_u8_latiSecDec2;

		ins_msg.u8_longiSt = temp_EW;
		ins_msg.u8_longiDeg = temp_u8_longiDeg;
		ins_msg.u8_longiMin = temp_u8_longiMin;
		ins_msg.u8_longiSec = temp_u8_longiSec;
		ins_msg.u8_longiSecDec = temp_u8_longiSecDec;
		ins_msg.u8_latiSecDec2 = temp_u8_longiSecDec2;

		ins_msg.latitude = temp_lat;
		ins_msg.longitude = temp_lng;
	}

	printf("ins_msg.latitude=%f,ins_msg.longitude=%f\n",ins_msg.latitude,ins_msg.longitude);
	u16_pseudoRangeError = (uint16)((rtk_locating_fuc == 1 ? irtk_msg.pseudorRangeError : ins_msg.pseudorRangeError) * 100);//rtk / ins ��λ
    u8_locationState = rtk_locating_fuc == 1 ? irtk_msg.rtk_state.b1_diffSignalValid : ins_msg.insState.b1_diffSignalValid;

	if(last_lat>=1&&last_lat>=1)
	{
		double distance = fabsf(Get_distance(ins_msg.latitude,ins_msg.longitude,last_lat,last_long));
		if(distance >2)
		{
			LOG(ERROR)<<"ins_get::ins_msg jump from "<<last_lat<<" ,"<<last_long<<" to "
					  <<ins_msg.latitude<<" , "<<ins_msg.longitude
					  <<" u16_pseudoRangeError = "<<u16_pseudoRangeError
					  <<" u8_locationState = "<<u8_locationState
					  <<" distance = "<<distance
					  <<std::endl;
		}
	}
	last_lat = ins_msg.latitude;
	last_long = ins_msg.longitude;

	}
	return  ;
}


void INS_GPGST_Analytical(char *buff, int length)
{
	char *ptr = NULL;
	float tm;
	/*���buff�ַ����а���$GPROT ��GPRMC�ĵ�ַ��ֵ��ptr */
	if (NULL == (ptr = strstr(buff, "$GPGST")))
	{
		return;
	}

	//$GPROT
	sscanf(ptr, "$GPGST,%f,%f", &tm, &ins_msg.pseudorRangeError);

}

//�����˳�ʱ���ƺ����
void ins_cmd_init(UART_TYPE uartid)
{
	// int re;
	// //	float INS_Distance;
	// //	char MSEP[17]="$JATT,MSEP,1.20";
	// // char BAUD[14]="$JBAUD,115200";
	// // char SAVE[8]="$JSAVE";
	// // char OFF[7]="$JOFF";
	// // char GPHPR[15]="$JASC,GPHPR,20";
	// // char GPGGA[15]="$JASC,GPGGA,20";
	// // char GPRMC[15]="$JASC,GPRMC,20";
	// // char GPROT[15]="$JASC,GPROT,1";
	// // char GPHEV[15]="$JASC,GPHEV,1";
	// // char MOVEBASE[20]="$JATT,MOVEBASE,YES";
	// //	char CASP[12]="$JATT,CASP";
	// //	INS_Distance=Dradio_Config.Board_Cfg.INS_Antenna_Distance/100.0;
	// //	fprintf(MSEP,"$JATT,MSEP,%.2f",INS_Distance);

	// //Ϊ��ɨ�������Ӻ������
	// char GPHDT[15]="$JASC,GPHDT,1";
	// char GPVTG[15]="$JASC,GPVTG,1";
	
	// //�������
	// char HBIAS[19] = "$JATT,HBIAS,-90.00";
	// HBIAS[17] = 0x0D;
	// HBIAS[18] = 0x0A;


	// MOVEBASE[18]=0x0D;
	// MOVEBASE[19]=0x0A;
	// re=write_uart(uartid,MOVEBASE,sizeof(MOVEBASE));
	// sleep_1(2000); 	


	// BAUD[12]=0x0D;
	// BAUD[13]=0x0A;
	// SAVE[6]=0x0D;
	// SAVE[7]=0x0A;
	// OFF[5]=0x0D;
	// OFF[6]=0x0A;
	// GPHPR[13]=0x0D;
	// GPHPR[14]=0x0A;
	// GPGGA[13]=0x0D;
	// GPGGA[14]=0x0A;
	// GPRMC[13]=0x0D;
	// GPRMC[14]=0x0A;
	// GPROT[13]=0x0D;
	// GPROT[14]=0x0A;
	// GPHEV[13]=0x0D;
	// GPHEV[14]=0x0A;

	// GPHDT[13]=0x0D;
	// GPHDT[14]=0x0A;
	// GPVTG[13]=0x0D;
	// GPVTG[14]=0x0A;



	// //	MSEP[15]=0x0D;
	// //	MSEP[16]=0x0A;
	// //	re=write_uart(UART5_Fd,MSEP,sizeof(MSEP));
	// //	sleep_1(2000); 	
	// // re=write_uart(uartid,BAUD,sizeof(BAUD));
	// // sleep_1(2000); 	
	// // //	set_com_config(UART5_Fd, 38400, 8, 'N', 1);	
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,OFF,sizeof(OFF));
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,GPHPR,sizeof(GPHPR));
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,GPGGA,sizeof(GPGGA));
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,GPRMC,sizeof(GPRMC));
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,GPROT,sizeof(GPROT));
	// // sleep_1(2000); 	
	// // re=write_uart(uartid,GPHEV,sizeof(GPHEV));
	// // sleep_1(2000);
	// // re=write_uart(uartid,HBIAS,sizeof(HBIAS));
	// //re=write_uart(uartid,GPHDT,sizeof(GPHDT));
	// //sleep_1(2000); 	
	// //re=write_uart(UART5_Fd,GPVTG,sizeof(GPVTG));
	// //sleep_1(2000); 


	// //CASP[10]=0x0D;
	// //CASP[11]=0x0A;
	// //re=write_uart(UART5_Fd,CASP,sizeof(CASP));
	// //sleep_1(2000); 		

	// re=write_uart(uartid,SAVE,sizeof(SAVE));
	// sleep_1(2000); 	

}

void writeSysTime( void )
{
#ifndef WINNT

	StruTime time;
	char cmd[100];

	
	wrValid_0 = ins_msg.insState.b1_dateValid && ins_msg.insState.b1_timeValid;

	if(wrValid_0 == 1 && wrValid_1 == 0)
	{
		memset(cmd,0,sizeof(cmd));
		getINS();
		time.year		=	state_signal.time.u8_year+2000	;
		time.month		=	state_signal.time.u8_month	;
		time.day		=	state_signal.time.u8_date	;
		time.hour		=	state_signal.time.u8_hour	;
		time.minute		=	state_signal.time.u8_minute	;
		time.second		=	state_signal.time.u8_second	;
		time.m_second	=	0;
		
		if(time.year <= 2000)	//ʱ���������������ʱ��
		{
			printf("ins time error!\n");
		}
		else
		{
			//����ʱ��
			sprintf_usv(cmd,"date  %02d%02d%02d%02d%04d",time.month,time.day,time.hour,time.minute,time.year);/*(����ʱ����)*/
			printf("%s\n",cmd);
			system(cmd);

			sprintf_usv(cmd,"date -s %02d:%02d:%02d",time.hour,time.minute,time.second);/*(ʱ:��:��)*/
			printf("%s\n",cmd);
			system(cmd);
		
			sprintf_usv(cmd,"hwclock --systohc");
			printf("%s\n",cmd);
			system(cmd);
		}
	}
		
	wrValid_1 = wrValid_0;
	//printf("wrValid0=%d	wrValid1=%d\n",wrValid_0,wrValid_1);
#endif
}

void insSocketInit( void )
{
	ins_sockid = -1;
	ins_sockid = CreateUDPSocket((uint16)UDP_INS_REC_PORT);
}

void insUdpSend( int8* pBuf,int len )
{
	int dest_port;
	struct sockaddr_in server;

	dest_port = ins_udp_port; //UDP_INS_SED_PORT
	server.sin_family = AF_INET;
	server.sin_port = htons(dest_port);
	server.sin_addr.s_addr = inet_addr(ins_udp_addr);

	memset(server.sin_zero,0,8);
	sendto(ins_sockid,pBuf,len,0,(struct sockaddr*)&server, sizeof(server));
	//printf("%s", pBuf);
}

int8 insUdpRecReport( int16 sockid )
{
	fd_set	set					;
	int16	ret					;
	int16	iLen				;
	struct timeval timeout		;
	socklen_t	sockaddrlen		;
	int8		net_rec_buff[200]	;

	memset(net_rec_buff, 0, sizeof(net_rec_buff));
	
	FD_ZERO(&set);
	FD_SET(sockid,&set);
	timeout.tv_sec	=	0;
	timeout.tv_usec	=	5;
	ret = select(sockid+1,&set,NULL,NULL,&timeout);
	if(ret<0){
		return 1;
	}
	if(FD_ISSET(sockid,&set)){		//UDP����
		sockaddrlen = sizeof(sockaddr_in);
		iLen = recvfrom(sockid,net_rec_buff,sizeof(net_rec_buff),0,(struct sockaddr *)&ins_udp_inf.from_upd_ip, &sockaddrlen);
		if(iLen>=1){
			printf("%s\n",net_rec_buff);
			write_uart(UART5_Fd,(int8*)net_rec_buff,iLen);
		}
	}
	return 1;
}

void ins_zmq_init( void )
{
	ins_zmq_pub.zmq_context = zmq_ctx_new();
	ins_zmq_pub.zmq_publisher = zmq_socket(ins_zmq_pub.zmq_context,ZMQ_PUB);
	zmq_bind(ins_zmq_pub.zmq_publisher,ins_zmq_pub.zmq_cfg);
}

void ins_zmq_publish( void )
{
	INS_PUB_MSG	ins_pub_msg;
	ins_pub_msg.insValid = ins_sign.comm_sign==0? 1:0;
	ins_pub_msg.latitude = ins_msg.latitude;
	ins_pub_msg.longitude= ins_msg.longitude;
	ins_pub_msg.heading  = ins_msg.heading;

	zmq_msg_t msg_z;
	int rc = zmq_msg_init_size(&msg_z,sizeof(ins_pub_msg));
	memcpy(zmq_msg_data(&msg_z),(char*)&ins_pub_msg,sizeof(ins_pub_msg));

	s_sendmore(ins_zmq_pub.zmq_publisher,"insMsg");
	rc = zmq_msg_send(&msg_z,ins_zmq_pub.zmq_publisher,0);
}

void ins_zmq_close( void )
{
	zmq_close(ins_zmq_pub.zmq_publisher);
	zmq_ctx_destroy(ins_zmq_pub.zmq_context);
}

void * nano_deal_ins(void *aa)
{ 
	#if 0
	//
	usv::localization::gnss::Ins_un237 ins_un237;

	//��ʼ��nanomsg ģ��
	int sock = nn_socket(AF_SP, NN_SUB);
	if (sock < 0){
		printf("nano ins sock error\n");
	}
	if (nn_setsockopt(sock, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0){
		printf("nano ins failed to set socket opts\n");
	}
	int nanoInsTimeout = 2000;
	nn_setsockopt(sock, NN_SOL_SOCKET, NN_RCVTIMEO, &nanoInsTimeout, sizeof(nanoInsTimeout));

	if (nn_connect(sock, "tcp://127.0.0.1:5566") < 0){
		printf("nano ins failed to connect to url\n");
	}
	int recvBuf = 1024;
	nn_setsockopt(sock, NN_SOL_SOCKET, NN_RCVBUF, &recvBuf, sizeof(recvBuf));
	//��ʼ��nanomsg ģ��

	for (;;){
		if (sock >= 0){
			char * buf;
			int bytes = nn_recv(sock, &buf, NN_MSG, 0);

			if (bytes > 0){
				ins_un237.ParseFromArray(buf, bytes);
				//printf("frame id = %d\n", ins_un237.header().sequence_num());
				printf("[%02d.%02d.%02d-%02d:%02d:%02d.%02d]: %d\n", ins_un237.instime().year(), ins_un237.instime().month(), \
					ins_un237.instime().date(), ins_un237.instime().hour(), ins_un237.instime().minute(), \
					ins_un237.instime().second(), ins_un237.instime().second_decade(),ins_un237.header().sequence_num());
				nn_freemsg(buf);
			}
			else{
				if (nn_errno() == ETIMEDOUT){
					printf("nano ins recv timeout!!!\n");
				}
					
			}
		}
		sleep_1(10);
	}
	#endif
	return NULL;
}

int initUn237Gpio(void)
{
#ifndef WINNT
	char dirIn[] = "in";
	char index_PRIY[] = "205";
	char index_SECY[] = "202";
	char index_DIFF[] = "200";
	char index_DGPS[] = "204";
	char index_RTCE[] = "201";

	if (set_direction(index_PRIY, dirIn) < 0)
	{
		printf("PRIY set dirction error\n");
		return -1;
	}

	if (set_direction(index_SECY, dirIn) < 0)
	{
		printf("SECY set dirction error\n");
		return -1;
	}

	if (set_direction(index_DIFF, dirIn) < 0)
	{
		printf("DIFF set dirction error\n");
		return -1;
	}

	if (set_direction(index_DGPS, dirIn) < 0)
	{
		printf("DGPS set dirction error\n");
		return -1;
	}

	if (set_direction(index_RTCE, dirIn) < 0)
	{
		printf("RTCE set dirction error\n");
		return -1;
	}
#endif
	memset((char *)&ins_hardState.b_primary, 0, sizeof(ins_hardState));
	return 1;
}

void getHardwareState(void)
{
#ifndef WINNT
	char rdValue[] = "0";
	char index_PRIY[] = "205";
	char index_SECY[] = "202";
	char index_DIFF[] = "200";
	char index_DGPS[] = "204";
	char index_RTCE[] = "201";
	static uint8 count = 0;
	count++;
	if (count > 4)
	{
		count = 0;
	}
		
	switch (count)
	{
	case 0:
		if (get_value(index_PRIY, rdValue) < 0)
		{
			printf("get value error!\n");
		}
		else
		{
			ins_hardState.b_primary = ini_str2hex(rdValue);
			
		}
		break;
	case 1:
		if (get_value(index_SECY, rdValue) < 0)
		{
			printf("get value error!\n");
		}
		else
		{
			ins_hardState.b_secondary = ini_str2hex(rdValue);
			
		}
		break;
	case 2:
		if (get_value(index_DIFF, rdValue) < 0)
		{
			printf("get value error!\n");
		}
		else
		{
			ins_hardState.b_diff = ini_str2hex(rdValue);
			
		}
		break;
	case 3:
		if (get_value(index_DGPS, rdValue) < 0)
		{
			printf("get value error!\n");
		}
		else
		{
			ins_hardState.b_dGps = ini_str2hex(rdValue);
			
		}
		break;
	case 4:
		if (get_value(index_RTCE, rdValue) < 0)
		{
			printf("get value error!\n");
		}
		else
		{
			ins_hardState.b_rtcErr = ini_str2hex(rdValue);
			
		}
		break;
	default:
		break;
	}
	
	//printf("PRIMARY STATE : %d\n", ins_hardState.b_primary);
	//printf("SECONDARY STATE : %d\n", ins_hardState.b_secondary);
	//printf("DIFF STATE : %d\n", ins_hardState.b_diff);
	//printf("DGPS STATE : %d\n", ins_hardState.b_dGps);
	//printf("RTCERR STATE : %d\n\n\n", ins_hardState.b_rtcErr);

#endif
}

void ins_CommCal(void *)
{
	comm_time_cal(INS_COMM_TIMEMAX, &(ins_sign.timer), &(ins_sign.comm_sign));
	if (log_ins_sign != ins_sign.comm_sign && poweron_init && poweron_init)
	{
		switch (ins_sign.comm_sign)
		{
		case COMM_CONNECT_OK:	//SysLogMsgPost("�ߵ�ͨѶ�ָ�");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_RCV_TIMEOUT, 0);
			break;
		case COMM_CONNECT_FAIL:	//SysLogMsgPost("�ߵ�ͨѶ�ж�");
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_INS_RCV_TIMEOUT, 1);
			break;
		default:
			break;
		}
	}
	log_ins_sign = ins_sign.comm_sign;
}

void insCommCalInit(void)
{
	addTask(4, ins_CommCal, (void*)0);
}

void insNanoSocketInit(void)
{
	ins_nanoSockit = nn_socket(AF_SP, NN_PAIR);
	if (ins_nanoSockit < 0)
	{
		printf("ins Nano transmit init error\n");
	}

	int to = 100;
	if (nn_setsockopt(ins_nanoSockit, NN_SOL_SOCKET, NN_RCVTIMEO, &to, sizeof(to)) < 0)
	{
		printf("ins nano setsocket error\n");
	}

	
	//memcpy(ins_nanoAddr, "tcp://*:7777", strlen("tcp://*:7777"));
	if (nn_bind(ins_nanoSockit, ins_nanoAddr)<0){
		printf("ins nano bind error\n");
	}else{
		printf("ins nano bind success\n");
	}
}

void insNanoSend(char* pBuf, int len)
{
	nn_send(ins_nanoSockit, pBuf, len, NN_DONTWAIT);
}

int8 insNanoRecReport(void)
{
	char *buf = NULL;
	char trBuf[200];
	memset(trBuf, 0, sizeof(trBuf));
	int result = nn_recv(ins_nanoSockit, &buf, NN_MSG, NN_DONTWAIT);
	if (result > 0)
	{
		memcpy(trBuf, buf, result);
		printf("nanoRecv:%s\n", trBuf);
		write_uart(UART5_Fd, (int8*)buf, result);
		nn_freemsg(buf);
	}
	
	return 1;
}
