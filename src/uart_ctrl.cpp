/*
* uart_ctl.c --串口线程
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#include "stdafx.h"
#include "../include/usv_include.h"



void monitor_ins_data_restored(void);


uint8 fireOnflag_L;
uint8 fireOffflag_L;
int32	fireOnTimebuf_L;
int32	fireOffTimebuf_L;

uint8 fireOnflag_R;
uint8 fireOffflag_R;
int32	fireOnTimebuf_R;
int32	fireOffTimebuf_R;

#define FIREON	0x01
#define FIREOFF 0x02
#define FIREDEFAULT 0x00
#define FIRE_CMD_TIME	10

DRADIO_FRAME_STATIC dradio_frame_static;

//北斗发送点火命令后延时一秒返回无操作
void return_relay(uint8 *returnflag, uint8 delayflag,uint8 dalaysignal,int32 *timebuf,int32 timeset)
{
	if(delayflag == dalaysignal)
	{
		*timebuf = *timebuf + 1;
		if(*timebuf>=timeset)
		{
			*timebuf =  timeset;
		}
	}
	else
	{
		*timebuf = 0;
	}

	if(*timebuf>0 && *timebuf < timeset)
		*returnflag = 1;
	else    *returnflag = 0;
}


//DSP心跳监测，超过2s收不到心跳报文则紧急停机并切断油路
//发动机DSP
void *Uart0_Heartbeat(void *aa)  
{
	int re=0,ico=0,jco=0,re_sign=0,count=0;
//	int kco,dsp_count=0;//报文编号
	uint8	buff[MAX_UART_BUFF_LEN],Heatbeat[DSP_Heartbeat_Buffer],DSP_State[DSP_State_Buffer];
	
	memset(Heatbeat, 0, DSP_Heartbeat_Buffer);
	memset(DSP_State, 0, DSP_State_Buffer);
	
	UART0_Fd = open_port(COM0); //以读写方式打开COM0
	if(UART0_Fd< 0) 
	{
		perror("open_com0_port");
		return ((void *)0);
	}
	if(set_com_config(UART0_Fd, 38400, 8, 'N', 1) < 0) //配置COM0
	{
		perror("set_com0_config");
		return ((void *)0);
	}
	
	while(1)
	{
		memset(buff, 0, MAX_UART_BUFF_LEN);
		re=read_uart(UART0_Fd, (int8 *)&buff[0], MAX_UART_BUFF_LEN);
/*		printf("\n");				
		for(kco=0;kco<50;kco++)
			printf("%2x ",buff[kco]);*/
		if ( re> 0)
		{
//			printf("re=%d\n", re);
			for(ico=0;ico<re;ico++)
			{
				if((buff[ico]=='V')&&(buff[ico+1]=='E')&&(buff[ico+2]=='R')&&(buff[ico+3]=='S')&&(buff[ico+4]=='I')&&(buff[ico+5]=='O')&&(buff[ico+6]=='N'))
				{
					sprintf_usv(Version[2],"DSP_V0.%d_R%d_%04XH",buff[ico+7]*256+buff[ico+8],buff[ico+9]*256+buff[ico+10],buff[11]*256+buff[12]);
					DSPVersion_sign=1;
				}

				if((buff[ico]=='D')||(re_sign==2))
				{
					re_sign=2;
					DSP_State[jco++]=buff[ico];
					if(jco>=DSP_State_Buffer)
					{
						jco=0;
						re_sign=0;//re is end
						//DSP状态报文解析
						if(CheckCRC( 1,DSP_State,sizeof(DSP_State))&&(DSP_State[1]=='R'))
						{
							count++;
//							printf("%d rev uart0 is ok!\n",count);
							DSP_State_Analytical(&DSP_State_Msg,DSP_State);
							task_time.uart0_task_2s=0;
							task_time.uart0_task_50ms=0;
							DSP_State_Msg.DSP_Heatbeat=1;
						}
						memset(DSP_State, 0, DSP_State_Buffer);
					}
				}
			}
		}
#ifdef  debug_print 
		else
			printf("read uart0 error!\n");
#endif
		sleep_1(10);            //10ms
	}
	
	close_uart(UART0_Fd);
	return ((void *)0);
}

//AIS报文接收线程
void *Uart1_AIS_Receive(void *aa)  
{
	uint16	re=0;
	uint8   re_sign=0,ico=0,jco=0,Msg_len=0,count,AIS_sign;
	char	buff[MAX_UART_BUFF_LEN],AIS_head_M[]="!AIVDM";
	UART1_Fd = open_port(COM1);//以读写方式打开COM1
	if(UART1_Fd< 0) 
	{
		perror("open_com1_port");
		return ((void *)0);
	}
	if(set_com_config(UART1_Fd, 38400, 8, 'N', 1) < 0) //配置COM1
	{
		perror("set_com1_config");
		return ((void *)0);
	}
	while(1)
	{
		memset(buff, 0, MAX_UART_BUFF_LEN);
		memset(buffer_Data,0,sizeof(buffer_Data));
		memset(isASCII_code,0,sizeof(isASCII_code));
		memset(Collision_Alternative,0,sizeof(Collision_Alternative));
		Collision_Num=0;
		AIS_sign=0;

		re=read_uart(UART1_Fd, buff, MAX_UART_BUFF_LEN);
		if ( re> 0)
		{
			printf("The received words are : %s", buff);
			for(ico=0;ico<re;ico++)
			{
				if((buff[ico]=='!')||(re_sign==1))
				/*!AIVDM表示接收到的报文，!AVIDO表示本设备发送的AIS报文,定位信息用惯导，不用此消息*/
				{
					re_sign=1;
					AIS_Msg_St.AIS_Message[jco++]=buff[ico];//给到图传电台上送AIS报文结构体然后上送
					if(jco>=AIS_Message_Buffer)
					{
						jco=0;
						re_sign=0;
						continue;
					}
					if((AIS_Msg_St.AIS_Message[jco-1]==10)&&(AIS_Msg_St.AIS_Message[jco-2]==13))//换行
					{
						for(count=0;count<6;count++)
						{
							if(AIS_Msg_St.AIS_Message[count]!=AIS_head_M[count])
							{
								AIS_sign=0;
								break;
							}
							else 
							{
								if(count==5)
								{
									if (AIS_Msg_St.AIS_Message[5]==AIS_head_M[5])
										AIS_sign=1;
									if(AIS_Msg_St.AIS_Message[5]==AIS_head_M[5])
										AIS_sign=1;
								}
							}
						}
						Msg_len=jco-2;
						jco=0;
						re_sign=0;
					//	AIS_Msg_St.AIS_Message[jco-2]=0;//去掉空格符
					//	AIS_Msg_St.AIS_Message[jco-1]=0;//去掉换行符
						if(GetCRC32(AIS_Msg_St.AIS_Message,Msg_len)&&(1==AIS_sign))//校验通过
						{
							task_time.uart1_task_10s=0;
							AIS_Msg_St.AIS_Heatbeat=1;
							AIS_Send_Sign=1;//AIS报文宽带电台上送标志
							//if(Check_Intelligent_Mod()==2)
							//{
							AIS_Decode(AIS_Msg_St.AIS_Message);//报文解析
							Collision_Sign=Calculate_Collision();//计算碰撞可能性
							//}
						}
						if(2==AIS_sign)
						{
						//	if(GetCRC32(buff))//校验通过
						//	{
								AIS_Msg_St.AIS_Heatbeat=1;
								task_time.uart1_task_10s=0;
						//	}
						}
					}
				}
			}
		}
#ifdef  debug_print 
		else
			printf("read uart1 error!\n");
#endif
		sleep_1(10);            //10ms
	}
	
	close_uart(UART1_Fd);
	return ((void *)0);
}





//数字电台接收指令线程
void *Uart2_Dradio_Rec(void *aa)  
{
	uint8	buff[MAX_UART_BUFF_LEN],Dradio_Con[Dradio_Con_Msg],Dradio_Sal[Dradio_Sal_Msg];
	char Dradio_Con_head[]="$USV";
	int re=0,ico=0,jco=0,count,re_sign=0,Sail_len=0,Dradio_Con_sign;
	int loop_i;
//	int Dradio_Sal_sign=0;
//	char Dradio_Sal_head[]="#USV";
	uint16 whole_frame_num=0;
	uint16 right_frame_num=0;
	uint16 wrong_frame_num=0;

	uint16 state_num =0;
	uint16 send_cycle = 0;

	memset(Dradio_Con,0,Dradio_Con_Msg);
	memset(Dradio_Sal,0,Dradio_Sal_Msg);
	UART2_Fd = open_port(COM2);
	if(UART2_Fd< 0) //以读写方式打开COM2
	{
		perror("open_com2_port");
		return ((void *)0);
	}
	if(set_com_config(UART2_Fd, 19200, 8, 'N', 1) < 0) //配置COM2
	{
		perror("set_com2_config");
		return ((void *)0);
	}
	
	while(1)
	{
		memset(buff, 0, MAX_UART_BUFF_LEN);
		Dradio_Con_sign=0;
//		printf("sign=%d,250ms=%d,1s=%d\n",Dradio_Com1_Sign,uart2_task_250ms,uart2_task_1s);
		re=read_uart(UART2_Fd, (int8 *)&buff[0], MAX_UART_BUFF_LEN);
		if (re> 0)
		{
//			printf("re=%d\n",re);
			for(ico=0;ico<re;ico++)
			{
//				printf(" %d-0x%.2x",jco,buff[ico]);
				if(((buff[ico]=='$')||(1==re_sign))&&(re_sign!=2))
				{						
//					printf("$%d",jco);
					re_sign=1;//re is not end
					Dradio_Con[jco++]=buff[ico];
//					printf("jco=%d,%d ",jco,buff[ico]);
					if(jco>=Dradio_Con_Msg)//数字电台控制指令头标志
					{
//						printf("& ");
						jco=0;
						re_sign=0;//re is end
						for(count=0;count<4;count++)
						{
							if(Dradio_Con[count]!=Dradio_Con_head[count])
							{
								Dradio_Con_sign=0;
								break;
							}
							else
								Dradio_Con_sign=1;
								
						}

						if(Dradio_Con_sign==1)
						{
							whole_frame_num ++;
						}
						
//						printf("Dradio_Con_sign=%d\n",Dradio_Con_sign);
						//数字电台控制指令报文解析
						if(CheckCRC( 1,Dradio_Con,Dradio_Con_Msg)&&(1==Dradio_Con_sign)&&(Dradio_Con[Dradio_Con_Msg-2]==0x2A))
						{
//							printf("rev uart2 is ok!\n");
							USV_Control.Radio_Nummber=0;//数字电台控制指令标志
							 Dradio_Con_Analytical(0,Dradio_Con);
						//监控模块数据转存
							if(Dradio_Con[5]==USV_NUM)//是本船指令
							{
							for(loop_i = 0;loop_i<Dradio_Con_Msg;loop_i++)
								monitor_all_inf.module_report_inf[MONITOR_DRADIO_MSG].report_detail[loop_i] = Dradio_Con[loop_i]; 	
							}

							 
//							 Dradio_Con_Printf(0);
							 Rudder_Detail_St.Rudder_Angle_Left_Con=USV_Control.USV_Control_Message[0].Dradio_USV_Drive.Rudder_Angle_Left;
							 Rudder_Detail_St.Rudder_Angle_Right_Con=USV_Control.USV_Control_Message[0].Dradio_USV_Drive.Rudder_Angle_Right;
							 Rudder_Detail_St.Speed_Limit=USV_Control.USV_Control_Message[0].Speed_Limit;
							task_time.uart2_task_1s=0;
							task_time.uart2_task_250ms=0;
							Dradio_Com1_Sign=1;
							right_frame_num++;	//校验正确报文数量

							//主从式发送数字电台反馈
							//send_cycle++;
							//if(send_cycle >= 1)
							//{
							//	send_cycle = 0;
							//	memset(USV_State_Current,0,State_Current_Num);
							//	Send_Dradio_State(state_num);	//数字电台状态回复，数据加密

							//}
							
						}
						else if(1==Dradio_Con_sign)
						{
							wrong_frame_num++;
						}
						else
							;
						memset(Dradio_Con,0,Dradio_Con_Msg);
					}
				}
				if(((buff[ico]=='#')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(2==re_sign))
				{
					if(re_sign<2)
						re_sign+=2;
					Dradio_Sal[jco++]=buff[ico];
					if(jco>7)
						Sail_len=Dradio_Sal[6]*256+Dradio_Sal[7];
					if(jco==Sail_len)//数字电台航行任务报文头标志
					{
						jco=0;
						re_sign=0;//re is end
					/*	for(count=0;count<4;count++)
						{
							if(Dradio_Sal[count]!=Dradio_Sal_head[count])
							{
								Dradio_Sal_sign=0;
								break;
							}
							else
								Dradio_Sal_sign=1;
						}*/
//						for(count=0;count<Sail_len;count++)
//							printf("%d ", Dradio_Sal[count]);
//						printf("\n");
						if(CheckCRC( 1,Dradio_Sal,Sail_len))
						{
							printf("Uart2 sail is ok\n");
							Sailing_Sign=0;						//数字电台--接管控制船体
							USV_Sailing.Radio_Nummber=0;
							Dradio_Sal_Analytical(Dradio_Sal);//B系统

						//转存监控数据
						if(Dradio_Sal[5]==USV_NUM)//是本船指令
						{
						for(loop_i=Dradio_Con_Msg;loop_i<((Dradio_Con_Msg+Dradio_Sal[8])>288?288:(Dradio_Con_Msg+Dradio_Sal[8]));loop_i++)
							monitor_all_inf.module_report_inf[MONITOR_DRADIO_MSG].report_detail[loop_i] = Dradio_Sal[loop_i]; 	
						}


							
							USV_State.Sailing_Nummber=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber;
							USV_State.USV_Sailing_Intel_Sign=1;//收到航行任务，待开启
							Get_Return_Point();//航行任务的起点为返航点
							Sailing_Cnt_Old=1;
							Get_Compensating_Dst();			//获得航线的误差范围
							
						}
						memset(Dradio_Sal,0,Dradio_Sal_Msg);

					}
				}

			}
		}
#ifdef  debug_print 
		else
			printf("read uart2 error!\n");
#endif
		
		if(whole_frame_num >= 1000)
		{
			dradio_frame_static.whole_frame_num = whole_frame_num;
			dradio_frame_static.right_frame_num = right_frame_num;
			dradio_frame_static.wrong_frame_num = wrong_frame_num;

			whole_frame_num = 0;
			right_frame_num = 0;
			wrong_frame_num = 0;
		}

		sleep_1(10);            //10ms
	}
	
	close_uart(UART2_Fd);
	return ((void *)0);
}

void fireOndelay(void)
{
	return_relay(&fireOnflag_L,BD_Engine_Run_L,FIREON,&fireOnTimebuf_L,FIRE_CMD_TIME);
	return_relay(&fireOnflag_R,BD_Engine_Run_R,FIREON,&fireOnTimebuf_R,FIRE_CMD_TIME);
	return_relay(&fireOffflag_L,BD_Engine_Run_L,FIREOFF,&fireOffTimebuf_L,FIRE_CMD_TIME);
	return_relay(&fireOffflag_R,BD_Engine_Run_R,FIREOFF,&fireOffTimebuf_R,FIRE_CMD_TIME);

	if(fireOnflag_L == 1)
		USV_Control.USV_Control_Message[3].Engine_Run_L = FIREON;
	else if(fireOffflag_L == 1)
		USV_Control.USV_Control_Message[3].Engine_Run_L = FIREOFF;
	else
		USV_Control.USV_Control_Message[3].Engine_Run_L = FIREDEFAULT;

	if(fireOnflag_R == 1)
		USV_Control.USV_Control_Message[3].Engine_Run_R = FIREON;
	else if(fireOffflag_R == 1)
		USV_Control.USV_Control_Message[3].Engine_Run_R = FIREOFF;
	else
		USV_Control.USV_Control_Message[3].Engine_Run_R = FIREDEFAULT;


	//	printf("%d\n",USV_Control.USV_Control_Message[3].Engine_Run_L);
	//printf("%d\n",USV_Control.USV_Control_Message[3].Engine_Run_R);
}



//北斗通讯线程
void *Uart3_BDS(void *aa)  
{
	
	int ret;
	int re=0,ico=0,jco=0,count,re_sign=0,Dradio_Con_sign;
	uint8	buff[MAX_UART_BUFF_LEN],Dradio_Con[Dradio_Con_Msg],Dradio_Sal[Dradio_Sal_Msg];
	uint8   rcv_buf[MAX_UART_BUFF_LEN];
	uint8	BD_buf[MAX_UART_BUFF_LEN];

	int i;


	UART3_Fd = open_port(COM3);//COM3
	if (UART3_Fd < 0) //以读写方式打开COM3 
	{
		perror("open_com3_port");
		return ((void *)0);
	}
	if (set_com_config(UART3_Fd, 19200, 8, 'N', 1) < 0) //配置COM3
	{
		perror("set_com3_config");
		return ((void *)0);
	}
	while (1)
	{
		//ret = Recv_Data(rcv_buf, MAX_UART_BUFF_LEN); //接收通信信息
		//BD_RecvPacket_Depart(rcv_buf, ret);//分包
		//memset(rcv_buf, 0, sizeof(rcv_buf));
		memset(buff,0,MAX_UART_BUFF_LEN);
	//	memset(BD_buf,0,MAX_UART_BUFF_LEN);
	//test
		//BDS_Con_Sign = 1;
		//BD_test();
	//test end

		if((re=read_uart(UART3_Fd,(int8 *)&buff[0],MAX_UART_BUFF_LEN))>0)
		{
			//debug 2017年9月16日 14:04:07
			//debug start
			printf("rev %d chars\n",re);
			for(i=0;i<re;i++)
			{
				printf("0x%x",buff[i]);
			}
			printf("\n");
			//debug end
			//debug end
			for(ico=0;ico<re;ico++)
			{
				if(((buff[ico]=='$')&&(buff[ico+1]=='T')&&(buff[ico+2]=='X')&&(buff[ico+3]=='X')&&(buff[ico+4]=='X'))||(1==re_sign))
				{
					re_sign=1;//re is not end
					BD_buf[jco++]=buff[ico];
					if(jco>=/*计算后填入*/55)
					{
						jco=0;
						re_sign=0;
						if(CheckCRC(1,BD_buf,55))//计算北斗报文CRC
						{
							BDS_Con_Sign = 1;
							//Sailing_Sign = 3;		//航行任务
							BD_Msg_Analytical(BD_buf);
						}
					}
				}
			}
		}


		task_time.uart3_task_100s++;
#ifdef  debug_print 
	else
		printf("read uart3 error!\n");
#endif
	sleep_1(10);            //100ms
	}
	memset(buff, 0, MAX_UART_BUFF_LEN);
	close_uart(UART3_Fd);
	return ((void *)0);

}

//冗余数字电台接收线程
void *Uart4_SpareDradio_Rec(void *aa)  
{

	uint16 Connect_count,Sail_len;
	uint8	buff[MAX_UART_BUFF_LEN],Dradio_Con[Dradio_Con_Msg],Dradio_Sal[Dradio_Sal_Msg],Dradio_Cfg[Dradio_Cfg_Msg];
	char	Dradio_Con_head[]="&USV";
	uint16	Dradio_Con_len = 0;
	int re=0,ico=0,jco=0,count,re_sign=0,Dradio_Con_sign;
	int loop_i;
//	int Dradio_Sal_sign;
//	char Dradio_Sal_head[]="#USV";
	Connect_count=0;
	memset(Dradio_Con,0,Dradio_Con_Msg);
	memset(Dradio_Sal,0,Dradio_Sal_Msg);
	memset(Dradio_Cfg,0,Dradio_Cfg_Msg);
	
	UART4_Fd = open_port(COM4);
	if(UART4_Fd < 0) //以读写方式打开COM4
	{
		perror("open_com4_port");
		return ((void *)0);
	}
	if(set_com_config(UART4_Fd, 19200, 8, 'N', 1) < 0) //配置COM4
	{
		perror("set_com4_config");
		return ((void *)0);
	}
	
	while(1)
	{
		memset(buff, 0, MAX_UART_BUFF_LEN);
		Dradio_Con_sign=0;
		if ((re=read_uart(UART4_Fd, (int8 *)&buff[0], MAX_UART_BUFF_LEN)) > 0)
		{
			Sail_len=0;
			for(ico=0;ico<re;ico++)
			{
				if(((buff[ico]=='#')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(1==re_sign)){
					re_sign = 1;
					Dradio_Sal[jco++] = buff[ico];
					if(jco>7)
						Sail_len=Dradio_Sal[6]*256+Dradio_Sal[7];
					if(jco==Sail_len){
						jco=0;
						re_sign=0;
						if(CheckCRC( 1,Dradio_Sal,Sail_len)){
							getSailMsgWaterQuality(Dradio_Sal);
						}
						memset(Dradio_Sal,0,Dradio_Sal_Msg);
					}
				}



//
////				printf("%d-0x%.2x ",jco,buff[ico]);
////				if(((buff[ico]=='&')||(1==re_sign))&&(re_sign!=2)&&(re_sign!=3))			//收到包头为 &
////				{
////					re_sign=1;//re is not end
////					Dradio_Con[jco++]=buff[ico];											//填缓存区					
////				//	printf("%d ", Dradio_Con[jco-1]);
////					if(jco>=Dradio_Con_Msg)//数字电台控制指令头标志							//接收完毕
////					{
////						jco=0;
////						re_sign=0;
////						for(count=0;count<4;count++)
////						{
////							if(Dradio_Con[count]!=Dradio_Con_head[count])
////							{
////								Dradio_Con_sign=0;
////								break;
////							}
////							else
////								Dradio_Con_sign=1;
////						}
////					/*	for(count=0;count<Dradio_Con_Msg;count++)
////							printf("%d ", Dradio_Con[count]);
////						printf("\n");*/
////						Dradio_Con_len = Dradio_Con[9];		//报文长度
////						if(CheckCRC( 1,Dradio_Con,Dradio_Con_len)&&(1==Dradio_Con_sign)&&(Dradio_Con_len == Dradio_Con_Msg))
////						{
//////							printf("rev uart4 is ok!\n",jco);
////							monitor_all_inf.monitor_comm_inf[MONITOR_COMM_SRADIO_SN].rec_ok_number++;
////							USV_Control.Radio_Nummber=1;//冗余数字电台控制指令标志
////							 Dradio_Con_Analytical(1,Dradio_Con);
////
////						//监控模块数据转存
////						if(Dradio_Con[5]==USV_NUM)//是本船指令
////						for(loop_i = 0;loop_i<Dradio_Con_Msg;loop_i++)
////							monitor_all_inf.module_report_inf[MONITOR_SPARE_DRADIO_MSG].report_detail[loop_i] = Dradio_Con[loop_i]; 	
////							 
////					//		 Dradio_Con_Printf(1);							 
////							if(Connect_count==10000)
////								Connect_count=1;
////							else
////								Connect_count++;
////							task_time.uart4_task_1s=0;
////							task_time.uart4_task_250ms=0;
////							SpareDradio_Com1_Sign=1;
////						}
////						memset(Dradio_Con,0,Dradio_Con_Msg);
////					}
////				}
//				if(((buff[ico]=='#')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(2==re_sign))
//				{
////					printf("re=%d,ico=%d,%d\n",re,ico,buff[ico]);
//					if(re_sign<2)
//						re_sign+=2;
//					Dradio_Sal[jco++]=buff[ico];
//					if(jco>7)
//						Sail_len=Dradio_Sal[6]*256+Dradio_Sal[7];
////					printf("re_sign=%d,Sail_len=%d\n",re_sign,Sail_len);
//					if(jco==Sail_len)//数字电台航行任务报文头标志
//					{
//						jco=0;
//						re_sign=0;
//					/*	for(count=0;count<4;count++)
//						{
//							if(Dradio_Sal[count]!=Dradio_Sal_head[count])
//							{
//								Dradio_Sal_sign=0;
//								break;
//							}
//							else
//								Dradio_Sal_sign=1;
//						}*/
//						
///*						for(count=0;count<Sail_len;count++)
//							printf("%x ", Dradio_Sal[count]);
//						printf("\n");*/
//						if(CheckCRC( 1,Dradio_Sal,Sail_len))
//						{
//							printf("UART4 sail is ok\n");
//							Sailing_Sign=1;							//冗余数字电台接管控制
//							USV_Sailing.Radio_Nummber=1;
//							Dradio_Sal_Analytical(Dradio_Sal);//A系统
//
//						//转存监控数据
//						if(Dradio_Sal[5]==USV_NUM)//是本船指令
//						for(loop_i=Dradio_Con_Msg;loop_i<((Dradio_Con_Msg+Dradio_Sal[8])>288?288:(Dradio_Con_Msg+Dradio_Sal[8]));loop_i++)
//						{
//							monitor_all_inf.module_report_inf[MONITOR_SPARE_DRADIO_MSG].report_detail[loop_i] = Dradio_Sal[loop_i]; 	
//						}
//
//							
//							USV_State.Sailing_Nummber=USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber;
//							USV_State.USV_Sailing_Intel_Sign=1;//收到航行任务，待开启
//							Get_Return_Point();//航行任务的起点为返航点
//							Sailing_Cnt_Old=1;	
//							Get_Compensating_Dst();
//						}
//						memset(Dradio_Sal,0,Dradio_Sal_Msg);
//					}
//				}
//				if(((buff[ico]=='%')&&(buff[ico+1]=='U')&&(buff[ico+2]=='S')&&(buff[ico+3]=='V'))||(3==re_sign))
//				{
//					re_sign=3;
//					Dradio_Cfg[jco++]=buff[ico];
//					if(jco>=Dradio_Cfg_Msg)//数字电台配置报文长度
//					{
//						jco=0;
//						re_sign=0;//re is end
////						for(count=0;count<Sail_len;count++)
////							printf("%d ", Dradio_Sal[count]);
////						printf("\n");
//						if(CheckCRC( 1,Dradio_Cfg,Dradio_Cfg_Msg))
//						{
//							printf("\n ");	
//							printf("Uart4 Cconfig is ok\n");
//							Dradio_Cfg_Analytical(&Dradio_Config,Dradio_Cfg);
//							USV_State.Config_Nummber=Dradio_Config.Config_Num;
//						}
//						memset(Dradio_Cfg,0,Dradio_Cfg_Msg);			//将配置清除
//					}
//				}
			}
		}
#ifdef  debug_print 
		else
			printf("read uart4 error!\n");
#endif
		sleep_1(10);            //10ms
	}
	
	close_uart(UART4_Fd);
	return ((void *)0);
}
//
////惯导报文接收线程
//void *Uart5_Ins_Rec(void *aa)  
//{
//	char buff[MAX_UART_BUFF_LEN],INS_Msg[128];
//	int re,ico=0,jco=0,re_sign=0,rev_sign=0,Msg_len=0;
//
//	int monitor_task_1s=0;
//	uint16 ins_recoder_task_100ms=0;
//	uint8  msg_record_cmd_old=0;
//	uint8  msg_record_cmd_real=0;
//	uint8  msg_record_time_minute;
//	uint8  msg_record_time_minute_old;
//	uint8  msg_record_time_minute_dif;
////	int icount;
//	re=0;
//	memset(INS_Msg, 0, 128);	
//	UART5_Fd = open_port(COM5);//以读写方式打开COM5
//	if(UART5_Fd< 0) 
//	{
//		printf("open uart5 error!\n");
//		return ((void *)0);
//	}
//	if(set_com_config(UART5_Fd, 38400, 8, 'N', 1) < 0) //配置COM5
//	{
//		printf("set uart5 error!\n");
//		return ((void *)0);
//	}
//	
//	while(1)
//	{
//		memset(buff, 0, MAX_UART_BUFF_LEN);
//		re=read_uart(UART5_Fd, buff, MAX_UART_BUFF_LEN);
//		if (re > 0)
//		{
////			printf("%s",buff);
////			printf("re=%d\n",re);
//			for(ico=0;ico<re;ico++)
//			{
////				printf("%x ",buff[ico]);
//				if((buff[ico]=='$')||(1==re_sign))
//				{
//					if(jco>=127)
//					{
//						jco=0;
//						re_sign=0;
//						break;
//					}
//					else
//					{
//						INS_Msg[jco++]=buff[ico];
//						re_sign=1;
//					}
//				}
//				if(buff[ico]==0x0A)//LF
//				{
//					if((INS_Msg[0]=='$')&&(INS_Msg[1]=='G')&&(INS_Msg[2]=='P')&&(INS_Msg[3]=='R')&&(INS_Msg[4]=='O')&&(INS_Msg[5]=='T'))
//					{
//						if((INS_Msg[jco-1]==10)&&(INS_Msg[jco-2]==13))
//						{
////							for(icount=0;icount<(jco-1);icount++)
////								printf("%c",buff[icount]);				
//							Msg_len=jco-2;
//							jco=0;
//							if(GetCRC32( INS_Msg,Msg_len))
//							{
////								printf("ROT is OK\n ",jco);
//								INS_GPROT_Analytical(INS_Msg);
//								rev_sign=1;
//								memset(INS_Msg, 0, 128);
//
//							//	if(msg_record_cmd[MONITOR_COMM_INS_SN])
//
//							}
//#ifdef  debug_print 
//							else
//								printf("ROT CRC is not OK\n ");
//#endif
//						}
//					}
//					if((INS_Msg[0]=='$')&&(INS_Msg[1]=='G')&&(INS_Msg[2]=='P')&&(INS_Msg[3]=='H')&&(INS_Msg[4]=='E')&&(INS_Msg[5]=='V'))
//					{
//						if((INS_Msg[jco-1]==10)&&(INS_Msg[jco-2]==13))
//						{
//							Msg_len=jco-2;
//							jco=0;
//							if(GetCRC32( INS_Msg,Msg_len))
//							{
////								printf("HEV CRC is OK\n ");
//								INS_GPHEV_Analytical(INS_Msg);
//								rev_sign=1;
//							}
//#ifdef  debug_print 
//							else
//								printf("HEV CRC is not OK\n ");
//#endif
//						}
//					}
//					if((INS_Msg[0]=='$')&&(INS_Msg[1]=='P')&&(INS_Msg[2]=='S')&&(INS_Msg[3]=='A')&&(INS_Msg[4]=='T')&&(INS_Msg[5]==',')&&(INS_Msg[6]=='H')&&(INS_Msg[7]=='P')&&(INS_Msg[8]=='R'))
//					{
//						if((INS_Msg[jco-1]==10)&&(INS_Msg[jco-2]==13))
//						{
//							Msg_len=jco-2;
//							jco=0;
//							if(GetCRC32( INS_Msg,Msg_len))
//							{
////								printf("HPR CRC is OK\n ");
//								INS_PSAT_Analytical(INS_Msg,Msg_len);
//								rev_sign=1;
//							}
//#ifdef  debug_print 
//							else
//								printf("HPR CRC is not OK\n ");
//#endif
//						}
//					}
//					if((INS_Msg[0]=='$')&&(INS_Msg[1]=='G')&&(INS_Msg[2]=='P')&&(INS_Msg[3]=='R')&&(INS_Msg[4]=='M')&&(INS_Msg[5]=='C'))
//					{
//						if((INS_Msg[jco-1]==10)&&(INS_Msg[jco-2]==13))
//						{
//							Msg_len=jco-2;
//							jco=0;
//							if(GetCRC32( INS_Msg,Msg_len))
//							{
////								printf("RMC CRC is OK\n ");
//								INS_GPRMC_Analytical(INS_Msg,Msg_len);
//								rev_sign=1;
//							}
//#ifdef  debug_print 
//							else
//								printf("RMC CRC is not OK\n ");
//#endif
//						}
//					}
//					if((INS_Msg[0]=='$')&&(INS_Msg[1]=='G')&&(INS_Msg[2]=='P')&&(INS_Msg[3]=='G')&&(INS_Msg[4]=='G')&&(INS_Msg[5]=='A'))
//					{
//						if((INS_Msg[jco-1]==10)&&(INS_Msg[jco-2]==13))
//						{
//							Msg_len=jco-2;
//							jco=0;
//							if(GetCRC32( INS_Msg,Msg_len))
//							{
////								printf("GGA CRC is OK\n ");
//								INS_GPGGA_Analytical(INS_Msg,Msg_len);		
//								rev_sign=1;
//							}
//#ifdef  debug_print 
//							else
//								printf("GGA CRC is not OK\n ");
//#endif
//						}
//					}	
//					memset(INS_Msg, 0, 128);
//					re_sign=0;
//					
//				}
//				if(1==rev_sign)
//				{
//					rev_sign=0;
//					task_time.uart5_task_1s=0;
//					Smart_Navigation_St.Power_Light=1;
//					Smart_Navigation_St.Serial_Light=1;
//					Smart_Navigation_St.Main_Antenna_Light=1;
//					Smart_Navigation_St.Smart_Navigation_Heatbeat=1;
//					monitor_all_inf.monitor_comm_inf[MONITOR_COMM_INS_SN].rec_ok_number++;
//					Update_Meter_Clock();		//更新里程表
//				}
//			}
//		}
//#ifdef  debug_print 
//		else
//			printf("read uart5 error!\n");
//#endif
//
//	monitor_task_1s ++;
//	if(monitor_task_1s >= 100)
//	{
//		monitor_task_1s = 0;
//		monitor_ins_data_restored();
//	}
//
//	ins_recoder_task_100ms++;
//
//	if(ins_recoder_task_100ms>=10)
//	{
//		ins_recoder_task_100ms=0;
//		if(msg_record_cmd_old== 0&& msg_record_cmd[MONITOR_COMM_INS_SN] ==1)
//		{
//			msg_record_time_minute_old = Smart_Navigation_St.USV_Minute;
//			msg_record_cmd_real = 1;
//			insRecordInit();
//		}
//		msg_record_cmd_old = msg_record_cmd[MONITOR_COMM_INS_SN];
//
//		if(msg_record_cmd_real == 1)
//		{
//			msg_record_time_minute = Smart_Navigation_St.USV_Minute;
//
//			if(msg_record_time_minute>= msg_record_time_minute_old)
//				msg_record_time_minute_dif = msg_record_time_minute - msg_record_time_minute_old;
//			else
//				msg_record_time_minute_dif = 60+msg_record_time_minute-msg_record_time_minute_old;
//
//			if(msg_record_time_minute_dif>15)	//持续15分钟
//			{
//				 msg_record_cmd[MONITOR_COMM_INS_SN] = 0;
//				 msg_record_cmd_real=0;
//				 insRecordClose();
//			}
//			else
//			{
//				insRecordSave();
//			}
//
//		}
//}
//
//		sleep_1(10);            //10ms
//	}
//	close_uart(UART5_Fd);
//	return ((void *)0);
//}

void monitor_ins_data_restored(void)
{
	int cntr_i;
	cntr_i  = 0;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Speed)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Speed)&0x00ff));
	
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Heading)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Heading)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Roll)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Roll)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Pitch)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Pitch)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Bow)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Bow)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Transverse)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Transverse)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Surge)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Surge)&0x00ff));

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Heave)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Heave)&0x00ff));	

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Yaw)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Yaw)&0x00ff));	

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_ROT)&0xff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_ROT)&0x00ff));		

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Height)&0xff000000)>>24);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Height)&0x00ff0000)>>16);		
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Height)&0x0000ff00)>>8);
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = (uint8)(((Smart_Navigation_St.USV_Height)&0x000000ff));	

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Latitude_Sign_St;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Longitude_Sign_St;
	
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Latitude_Degree;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Latitude_Minute;
	
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Latitude_Second;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Latitude_Decimal_2;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Latitude_Decimal_4;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Longitude_Degree;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Longitude_Minute;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Longitude_Second;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Longitude_Decimal_2;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Longitude_Decimal_4;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Year;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Month;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Date;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Hour;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Minute;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Second;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.USV_Second_2;
	
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Satellite_Num_1;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Satellite_Num_2;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Sys_State_1;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Sys_State_1;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Power_Light;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Serial_Light;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Main_Antenna_Light;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Minor_Antenna_Light;

	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Differerntial_Signal_Light;
	monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Differential_Position_Light;

		monitor_all_inf.module_report_inf[MONITOR_INS_MSG].report_detail[cntr_i++] = Smart_Navigation_St.Smart_Navigation_Heatbeat;	
		
}