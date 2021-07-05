/*
* can_ctrl.c --CAN线程
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#include "stdafx.h"

#include "../include/usv_include.h"



#ifdef WINNT
extern void usleep(int delay_ms);
#define snprintf _snprintf

#endif

static struct can_filter *filter = NULL;
static int filter_count = 0;
int add_filter(u_int32_t id, u_int32_t mask)
{
	filter = (can_filter *)realloc(filter, sizeof(struct can_filter) * (filter_count + 1));
	if(!filter)
		return -1;

	filter[filter_count].can_id = id;
	filter[filter_count].can_mask = mask;
	filter_count++;
#ifdef  debug_print 
	printf("id: 0x%08x mask: 0x%08x\n",id,mask);
#endif
	return 0;
}


static void do_show_state(char* name)
{
#ifndef WINNT 
	int state;

	if (can_get_state(name, &state) < 0) {
		fprintf(stderr, "%s: failed to get state \n", name);
		exit(EXIT_FAILURE);
	}

//	if ((state >= 0) && (state < CAN_STATE_MAX))
//		fprintf(stdout, "%s state: %s\n", name, can_states[state]);
//	else
//		fprintf(stderr, "%s: unknown state\n", name);
#endif
}

static void do_start(char* name)
{
#ifndef WINNT 
	if (can_do_start(name) < 0) {
		fprintf(stderr, "%s: failed to start\n", name);
		exit(EXIT_FAILURE);
	} else {
		do_show_state(name);
	}
#endif
}


static void do_stop(char* name)
{
#ifndef WINNT
	if (can_do_stop(name) < 0) {
		fprintf(stderr, "%s: failed to stop\n", name);
		exit(EXIT_FAILURE);
	} else {
		do_show_state(name);
	}
#endif
}


/*static void do_show_restart_ms(char *name)
{
	__u32 restart_ms;

	if (can_get_restart_ms(name, &restart_ms) < 0) {
		fprintf(stderr, "%s: failed to get restart_ms\n", name);
		exit(EXIT_FAILURE);
	} 
	else
		fprintf(stdout,"%s restart-ms: %u\n", name, restart_ms);
}*/



void *CAN0_ARM_Rec(void *aa)  
{
#ifndef	WINNT
	uint8 count;
	char name[] = CAN;//argv=can0,family=29,type=3,proto=1
       //jinxin added for reveive 
    struct can_frame frame = {
		.can_id = CAN_ARM_ADD,//设定全局变量或在不同函数设置不同变量，绑定PGN
	};

	struct ifreq ifr;
	struct sockaddr_can addr;
//	struct can_filter rfilter[10];
       int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
	int n = 0, err;
	int nbytes, i;
//	int opt, optdaemon = 0;
//       uint8 id, mask;
//       int verbose = 0;
   	char buf[255];
//    FILE* out =  stdout;
//	char *optout = NULL;
    	//end
	memset(FAULT_CODE_Last,0,sizeof(FAULT_CODE_Last));
	memset((uint8 *)&CAN_Msg.CAN_add,0,sizeof(CAN_Struct));

	
	CAN_Init();
	frame.can_id =CAN_ARM_ADD;
	Sealight_Count=0;	
	Sealight_Sign=0;
	//set default bitrate
	do_stop(name);//设置波特率之前需要关闭can
	err = can_set_bitrate(name, CAN0_bitrate);//设置波特率
	if (err < 0)
	{
#ifdef  debug_print 
		printf( "failed to set bitrate of can0\n" );
#endif
		exit(EXIT_FAILURE);
	}

       //start the can device	 
       do_start(name);    
   
       CAN_0 = socket(family, type, proto);       //建立套接字
	if (CAN_0< 0) 
	{
#ifdef  debug_print 
		perror("socket");
#endif
		do_stop(name);
		goto out;
	}
	strcpy(ifr.ifr_name, name);
	if (ioctl(CAN_0, SIOCGIFINDEX, &ifr)) 		//指定CAN 设备
	{
#ifdef  debug_print 
		perror("ioctl");
#endif
		goto out;
	}
	addr.can_family = family;
	addr.can_ifindex = ifr.ifr_ifindex;
       if (bind(CAN_0, (struct sockaddr *)&addr, sizeof(addr)) < 0) 	//绑定套接字与CAN设备
	{
#ifdef  debug_print 
		perror("bind");
#endif
		goto out;
	}
	   
/*       //设置过滤，只接受id设定的报文
       id = 0x1;mask=0x01;//.1 is filter, and 0 is ignore
       add_filter(id,mask);
       if (filter) 
	{
		if (setsockopt(CAN_0, SOL_CAN_RAW, CAN_RAW_FILTER, filter,
			       filter_count * sizeof(struct can_filter)) != 0) 
		{
			perror("setsockopt");
			exit(1);
		}
	}
*/
//设置回环功能，可以接收自己发送的报文
/*int ro = 1; // 0 表示关闭( 默认), 1 表示开启 
setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &ro, sizeof(ro)); 
*/

#else
	uint8 count;
	char name[] = CAN;//argv=can0,family=29,type=3,proto=1
	//jinxin added for reveive 
	struct can_frame frame ;

	int n = 0, err;
	int nbytes, i;

	char buf[255];

	memset(FAULT_CODE_Last,0,sizeof(FAULT_CODE_Last));
	memset((uint8 *)&CAN_Msg.CAN_add,0,sizeof(CAN_Struct));


	frame.can_id =CAN_ARM_ADD;
	CAN_Init();
	Sealight_Count=0;	
	Sealight_Sign=0;

	init_can_win();



#endif
       //deal receive
       while (1) 
	{
		nbytes = read_can(CAN_0, &frame, sizeof(struct can_frame));
		if(nbytes<0)
			perror("read");
		else 
		{
/*			if(frame.can_id & CAN_ERR_FLAG)//错误帧
				printf("CAN数据帧错误!\n");
			if (frame.can_id & CAN_EFF_FLAG)//扩展帧标志
				n = snprintf(buf, 255, "<0x%08x> ", frame.can_id & CAN_EFF_MASK);
			else
				n = snprintf(buf, 255, "<0x%03x> ", frame.can_id & CAN_SFF_MASK);

			n += snprintf(buf + n, 255 - n, "[%d] ", frame.can_dlc);
			for (i = 0; i < frame.can_dlc; i++)
			{
				n += snprintf(buf + n, 255 - n, "%02x ", frame.data[i]);
			}
			printf("%s\n",buf); */
			CAN_Msg.CAN_add=(frame.can_id)&0x000000ff;
			CAN_Msg.CAN_PGN=((frame.can_id)&0x00ffff00)>>8;
			CAN_Msg.CAN_Priority=((frame.can_id)&0x1c000000)>>26;
			for(count=0;count<8;count++)
				CAN_Msg.CAN_Data[count]=frame.data[count];
			CAN_Analyzing(&CAN_Msg);
/*			if (frame.can_id & CAN_RTR_FLAG)//远程帧
				n += snprintf(buf + n, 255 - n, "remote request");
			
			fprintf(out, "%s\n", buf);*/
			
		}
		usleep(400);//0.2ms
	}
    
	#ifndef WINNT
	close(CAN_0);
        do_stop(name);
#ifdef  debug_print 
	printf("success to quit...\n");
#endif
	exit(EXIT_SUCCESS);
out:
	
	close(CAN_0);
        do_stop(name);
	exit(EXIT_FAILURE);
	
	#else
	   close_can_win();
	#endif
	
	return ((void *)0);
}

//发送CAN控制指令
void Send_CAN_Control(void)
{
	Get_CAN_SR_Con();				//填入CAN智能舵控制报文
	usleep(500);//0.5ms	
	Get_CAN_SR_Smart_Con();			//填入航向航速CAN报文
	usleep(500);//0.5ms	
	Get_CAN_UAV_Con();
	usleep(500);//0.5ms	
	Get_CAN_POW_Con();
	usleep(500);//0.5ms	
	Get_CAN_ST_PL_Con();
	usleep(500);//0.5ms		
	Get_CAN_Energy_Con();
	usleep(500);//0.5ms	
	return	;
}
//PID参数Kp的计算，注释掉，没用
void Accelerator_PID(uint16 Accelerator)
{
	aPID.Proportion = (float)0.8;             	//设定初始P值
	aPID.Integral   = (float)0.0;            	 	//设定初始I值
	aPID.Derivative = (float)1.0;             	//设定初始D值

//	float rot;
	aPID.SetPoint =Accelerator/((float)8.0);                 	//根据实际情况设定
	aPID.Actual=USV_State.Dradio_USV_Drive_State.Accelerator_Right_St/((float)8.0);               		//得到当前速度值
	aPID.Error =aPID.SetPoint- aPID.Actual;   	//与设定值比较，得到误差值
	if((aPID.Error <=20.0)&&(aPID.Error>=-20.0))
		return;
	aPID.Ec=aPID.Error-aPID.LastError;
	aPID.PreError=aPID.LastError;
	aPID.LastError=aPID.Error;
	aPID.Integral_error+=aPID.Error;
	Accelerator_R=(int)(Accelerator_R+(aPID.Proportion*aPID.Error*8+aPID.Integral*aPID.Integral_error*8+aPID.Derivative*aPID.Ec*8));
//	printf("accelerator_L=%f-%f-%f-%d",vPID.SetPoint ,vPID.Actual,vPID.Proportion,Accelerator_L/8);
	if(Accelerator_R<((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)<<3);
	if(Accelerator_R>((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3))
		Accelerator_R=((SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30)<<3);
}

//填入CAN智能舵控制报文
void Get_CAN_SR_Con()
{

//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
//	uint16 Accelerator_Left_St;

	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	Rudder_Zero_Angle=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L;
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Con_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
//	Accelerator_Left_St=USV_State.Dradio_USV_Drive_State.Accelerator_Left_St;
//	Accelerator_PID(Accelerator_Left_St);
	frame.data[0]=(Accelerator_L&0x00ff);
	frame.data[1]=((Accelerator_L&0xff00)>>8);
	frame.data[2]=(Accelerator_R&0x00ff);
	frame.data[3]=((Accelerator_R&0xff00)>>8);	
	frame.data[4]=Gear_L;
	frame.data[5]=Gear_R;
	frame.data[6]=(uint8)(Rudder_L+Rudder_Zero_Angle*2.5);	
	frame.data[7]=(uint8)(Rudder_R+Rudder_Zero_Angle*2.5);
//	printf("Speed_EXP=%d-%d-%d-%d-%d-%d\n",Accelerator_L/8,Accelerator_R/8,Gear_L,Gear_R,Rudder_L,Rudder_R);

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
/*	printf("\n");	
	for(count=0;count<8;count++)
		printf("CAN=0x%.2x",frame.data[count]);
	printf("\n");	
*/	
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65280 error!\n");

	return	;
}

//填入航向航速CAN报文
void Get_CAN_SR_Smart_Con()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Smart_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Sailing_Mod==2)//智能模式
	{
		//航行任务算法计算结果USV_Speed_Max
		frame.data[0]=(((uint16)(Speed_EXP*100))&0x00ff);
		frame.data[1]=((((uint16)(Speed_EXP*100))&0xff00)>>8);
		
		frame.data[2]=(((uint16)(Heading_EXP*100))&0x00ff);
		frame.data[3]=((((uint16)(Heading_EXP*100))&0xff00)>>8);
	}
	else//非智能模式
	{
		if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Speed_Constant_Mod==1)//定速模式
		{
			if(1==Collision_Sign)//AIS避障
			{
				frame.data[0]=(((uint16)(Collision_Speed*100))&0x00ff);
				frame.data[1]=((((uint16)(Collision_Speed*100))&0xff00)>>8);
			}
			else
			{
				frame.data[0]=((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0x00ff);
				frame.data[1]=(((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0xff00)>>8);//cxy
			}
		}
		else//非定速模式
		{
			frame.data[0]=0;
			frame.data[1]=0;
		}
		if(USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Model.Direction_Constant_Mod==1)//定向模式
		{
			if(1==Collision_Sign)//AIS避障
			{
				frame.data[2]=(((uint16)(Collision_Heading*100))&0x00ff);
				frame.data[3]=((((uint16)(Collision_Heading*100))&0xff00)>>8);
			}
			else
			{
				frame.data[2]=((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0x00ff);
				frame.data[3]=(((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0xff00)>>8);//cxy
			}
			
			if(1==radar_obstacle_sign)//RADAR避障
			{
				frame.data[2]=(((uint16)(Radar_Collision_Heading*100))&0x00ff);
				frame.data[3]=((((uint16)(Radar_Collision_Heading*100))&0xff00)>>8);
			}
			else
			{
				frame.data[2]=((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0x00ff);
				frame.data[3]=(((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0xff00)>>8);
				
			}

			
		}
		else//非定向模式
		{
			frame.data[2]=0;
			frame.data[3]=0;
		}
	}
	frame.data[4]=USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_L;
	frame.data[4]+=(USV_Control.USV_Control_Message[Radio_Sign].Engine_Run_R)<<2;
	
	frame.data[5]=USV_Control.USV_Control_Message[Radio_Sign].Outboard_Engine_L;
	frame.data[5]+=((USV_Control.USV_Control_Message[Radio_Sign].Outboard_Engine_R)<<2);
	frame.data[5]+=E_Stop<<4;
//	USV_State.Engine_run=3;
	frame.data[5]+=(USV_State.Engine_run<<5);
//	printf("Engine_run=%d\n",USV_State.Engine_run);

//	printf("ESTOP=%d-0x%.2x\n",	USV_Control.USV_Control_Message[Radio_Sign].Emergency_Stop,frame.data[5]);
	//各子系统状态1异常0正常消防系统暂无状态
	frame.data[6]=(DSP_State_Msg.DSP_Heatbeat^0x01);
	frame.data[6]+=((USV_RM_MSG.MPC_Heatbeat^0x01)<<1);
	frame.data[6]+=(((USV_State.Engine_power&0x01)^0x01)<<2);
	frame.data[6]+=(((USV_State.Engine_power&0x02)^0x01)<<3);
	frame.data[6]+=((Rudder_Detail_St.Heatbeat_Rudder^0x01)<<4);
	frame.data[6]+=((Energy_Control_St.Power_Control_Heatbeat^0x01)<<5);
	frame.data[6]+=((POW_CAN_State.System_Heatbeat^0x01)<<6);
	frame.data[6]+=((UAV_Detail_St.UAV_Heatbeat^0x01)<<7);
	frame.data[7]=(SP_CAN_Model_Con.System_Heatbeat^0x01);
		
//	Speed_EXP=0.0;//发送之后清零，在智能算法中重新计算。避免停留时间仍发送速度和航向
//	Heading_EXP=0.0;//cxy?
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	printf("OUT=0x%.2x\n",frame.data[5]);
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65284 error!\n");
	
	return	;
}
//填入发动机配置报文
void Get_CAN_SR_Engine_Config()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Spd_Limit_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=SR_Config_Msg.Motor_MAX_Speed_L_spn520220;
	frame.data[1]=SR_Config_Msg.Motor_Idling_Speed_L_spn520221;
	frame.data[2]=SR_Config_Msg.Motor_MAX_Speed_R_spn520222;
	frame.data[3]=SR_Config_Msg.Motor_Idling_Speed_R_spn520223;
	frame.data[4]=SR_Config_Msg.Motor_Travel_L_spn520224;
	frame.data[5]=SR_Config_Msg.Motor_Travel_R_spn520225;
	frame.data[6]=SR_Config_Msg.Motor_Fire_Time_spn520226;
	frame.data[6]+=(SR_Config_Msg.Motor_Direction_L_spn520227)<<3;
	frame.data[6]+=(SR_Config_Msg.Motor_Direction_R_spn520228)<<4;
	
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65281 error!\n");
	
	return	;
}
//填入档位配置报文
void Get_CAN_SR_Gear_Config()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Gear_Limit_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=SR_Config_Msg.Gear_Forward_Travel_L_spn520240;
	frame.data[1]=SR_Config_Msg.Gear_Forward_Travel_R_spn520241;
	frame.data[2]=SR_Config_Msg.Gear_Back_Travel_L_spn520242;
	frame.data[3]=SR_Config_Msg.Gear_Back_Travel_R_spn520243;
	frame.data[4]=SR_Config_Msg.Gear_Forward_Angle_L_spn520244;
	frame.data[5]=SR_Config_Msg.Gear_Forward_Angle_R_spn520245;
	frame.data[6]=SR_Config_Msg.Gear_Back_Angle_L_spn520246;
	frame.data[7]=SR_Config_Msg.Gear_Back_Angle_R_spn520247;
			
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65282 error!\n");
	
	return	;
}
//填入档位配置报文
void Get_CAN_SR_Gear2_Config()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Gear_Rudder_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	
	frame.data[0]=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
	frame.data[1]=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;
	frame.data[2]=SR_Config_Msg.Gear_Direction_L_spn520250;
	frame.data[2]+=(SR_Config_Msg.Gear_Direction_R_spn520251)<<1;
	frame.data[2]+=(SR_Config_Msg.Drive_Nummber_spn520252)<<2;

/*	printf("Rudder_Direction_L_spn520272=%d\n",SR_Config_Msg.Rudder_Direction_L_spn520276);	
	printf("Gear_Direction_L_spn520273=%d\n",SR_Config_Msg.Gear_Direction_L_spn520273);	
	printf("Motor_Direction_L_spn520274=%d\n",SR_Config_Msg.Motor_Direction_L_spn520274);	
	printf("Rudder_Direction_R_spn520275=%d\n",SR_Config_Msg.Rudder_Direction_R_spn520277);	
	printf("Gear_Direction_R_spn520276=%d\n",SR_Config_Msg.Gear_Direction_R_spn520276);	
	printf("Motor_Direction_R_spn520277=%d\n",SR_Config_Msg.Motor_Direction_R_spn520277);	
	printf("Drive_Nummber_spn520278=%d\n",SR_Config_Msg.Drive_Nummber_spn520278);	
	printf("Motor_Direction_L=%d-0x%.2x\n",SR_Config_Msg.Motor_Direction_L_spn520274,frame.data[6]);	*/

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	printf("\n");
//	for(count=0;count<8;count++)
//		printf("rudder=0x%2x",frame.data[count]);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65284 error!\n");
	
	return	;
}
//填入舵角配置报文
void Get_CAN_SR_Rudder_Config()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Rudder_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=SR_Config_Msg.Rudder_Angle_Max_L_spn520260;
	frame.data[1]=SR_Config_Msg.Rudder_Angle_Max_R_spn520261;
	frame.data[2]=SR_Config_Msg.Rudder_Left_Limit_Resistance_L_spn520262;
	frame.data[3]=SR_Config_Msg.Rudder_Left_Limit_Resistance_R_spn520263;
	frame.data[4]=SR_Config_Msg.Rudder_Middle_Limit_Resistance_L_spn520264;
	frame.data[5]=SR_Config_Msg.Rudder_Middle_Limit_Resistance_R_spn520265;
	frame.data[6]=SR_Config_Msg.Rudder_Right_Limit_Resistance_L_spn520266;
	frame.data[7]=SR_Config_Msg.Rudder_Right_Limit_Resistance_R_spn520267;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
//	for(count=0;count<8;count++)
//		printf("can-0x%.2x",frame.data[count]);
//	printf("\n");
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65285 error!\n");
	
	return	;
}

//填入舵角配置报文
void Get_CAN_SR_Rudder2_Config()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Rudder2_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度

	frame.data[0]=SR_Config_Msg.Rudder_Control_Angle_Max_L_spn520268;
	frame.data[1]=SR_Config_Msg.Rudder_Control_Angle_Max_R_spn520269;		
	frame.data[2]=SR_Config_Msg.Rudder_Control_Angle_Min_L_spn520270;
	frame.data[3]=SR_Config_Msg.Rudder_Control_Angle_Min_R_spn520271;
	frame.data[4]=SR_Config_Msg.Rudder_Left_Travel_L_spn520272;
	frame.data[5]=SR_Config_Msg.Rudder_Left_Travel_R_spn520273;
	frame.data[6]=SR_Config_Msg.Rudder_Right_Travel_L_spn520274;
	frame.data[7]=SR_Config_Msg.Rudder_Right_Travel_R_spn520275;

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
//	for(count=0;count<8;count++)
//		printf("can-0x%.2x",frame.data[count]);
//	printf("\n");
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65285 error!\n");
	
	return	;
}
//填入舵角配置报文
void Get_CAN_SR_Rudder3_Config()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_SR_Rudder3_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度

	frame.data[0]=SR_Config_Msg.Rudder_Control_Accuracy_L_spn520276;
	frame.data[0]+=(SR_Config_Msg.Rudder_Control_Accuracy_R_spn520277)<<4;		
	frame.data[1]=SR_Config_Msg.Rudder_Direction_L_spn520278;
	frame.data[1]+=(SR_Config_Msg.Rudder_Direction_R_spn520279)<<1;

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
//	for(count=0;count<8;count++)
//		printf("can-0x%.2x",frame.data[count]);
//	printf("\n");
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65285 error!\n");
	
	return	;
}

//填入无人机平台CAN报文
void Get_CAN_UAV_Con()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_UAV_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_UAV_Control.Platform_Hatch;
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_UAV_Control.Platform_Lift)<<2);
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_UAV_Control.Platform_Open)<<4);
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_UAV_Control.UAV_Charging)<<6);
	//各子系统状态1异常0正常消防系统暂无状态
	frame.data[1]=(DSP_State_Msg.DSP_Heatbeat^0x01);
	frame.data[1]+=((USV_RM_MSG.MPC_Heatbeat^0x01)<<1);
	frame.data[1]+=(((USV_State.Engine_power&0x01)^0x01)<<2);
	frame.data[1]+=(((USV_State.Engine_power&0x02)^0x01)<<3);
	frame.data[1]+=((Rudder_Detail_St.Heatbeat_Rudder^0x01)<<4);
	frame.data[1]+=((Energy_Control_St.Power_Control_Heatbeat^0x01)<<5);
	frame.data[1]+=((POW_CAN_State.System_Heatbeat^0x01)<<6);
	frame.data[1]+=((UAV_Detail_St.UAV_Heatbeat^0x01)<<7);
	frame.data[2]=(SP_CAN_Model_Con.System_Heatbeat^0x01);

	write_can(CAN_0,&frame,sizeof(frame));
//	printf("frame.data[0]=0x%.2x!\n",frame.data[0]);
//	sleep_1(1);
	
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65290 error!\n");
	
	return	;
}
//填入电源管理CAN报文
void Get_CAN_POW_Con()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_POW_PGN;
	frame.can_id=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
		
	frame.data[0]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Stable_Platform_Power;
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.UAV_Power)<<2);
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Horn_Power)<<4);
	frame.data[0]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Navigationlight_Power)<<6);
	frame.data[1]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.D_radar_Power;
	frame.data[1]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Camera_main_Power)<<2);
	frame.data[1]+=((USV_Control.USV_Control_Message[Radio_Sign].Application_24)<<4);
	frame.data[1]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Searchlight_Power)<<6);
	frame.data[2]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Camera_ahead_Power;
	frame.data[2]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Camera_lesser_Power)<<2);
	frame.data[2]+=((USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Device_Power.Camera_tail_Power)<<4);
	frame.data[2]+=((USV_Control.USV_Control_Message[Radio_Sign].Application_12)<<6);
	frame.data[3]=((USV_Control.USV_Control_Message[Radio_Sign].Sealight_Speed)<<4);

	frame.data[3]+=((USV_Control.USV_Control_Message[Radio_Sign].Sealight_Direction)<<5);

	//各子系统状态1异常0正常消防系统暂无状态
	frame.data[4]=(DSP_State_Msg.DSP_Heatbeat^0x01);
	frame.data[4]+=((USV_RM_MSG.MPC_Heatbeat^0x01)<<1);
	frame.data[4]+=(((USV_State.Engine_power&0x01)^0x01)<<2);
	frame.data[4]+=(((USV_State.Engine_power&0x02)^0x01)<<3);
	frame.data[4]+=((Rudder_Detail_St.Heatbeat_Rudder^0x01)<<4);
	frame.data[4]+=((Energy_Control_St.Power_Control_Heatbeat^0x01)<<5);
	frame.data[4]+=((POW_CAN_State.System_Heatbeat^0x01)<<6);
	frame.data[4]+=((UAV_Detail_St.UAV_Heatbeat^0x01)<<7);
	frame.data[5]=(SP_CAN_Model_Con.System_Heatbeat^0x01);
/*	for(count=0;count<8;count++)
		printf("0x%.2x ",frame.data[count]);
	printf("\n");
*/	
/*	
	//已打开或关闭电源，则无操作
	if((USV_Control.USV_Control_Message[Radio_Sign].Engine_Power&0x01)!=(USV_State.Engine_power&0x01))
		frame.data[3]+=(USV_Control.USV_Control_Message[Radio_Sign].Engine_Power&0x01);
	if((USV_Control.USV_Control_Message[Radio_Sign].Engine_Power&0x02)!=(USV_State.Engine_power&0x02))
		frame.data[3]+=((USV_Control.USV_Control_Message[Radio_Sign].Engine_Power&0x02)<<2);*/
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65291 error!\n");
	
	return	;
}
//填入稳定平台CAN报文
void Get_CAN_ST_PL_Con()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_ST_PL_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Stable_Platform.Stable_Platform_RL;
	frame.data[1]=USV_Control.USV_Control_Message[Radio_Sign].Dradio_USV_Stable_Platform.Stable_Platform_AT;
//	printf("Stable_Platform_AT=%d-%d\n",frame.data[0],frame.data[1]);
	//各子系统状态1异常0正常消防系统暂无状态
	frame.data[2]=(DSP_State_Msg.DSP_Heatbeat^0x01);
	frame.data[2]+=((USV_RM_MSG.MPC_Heatbeat^0x01)<<1);
	frame.data[2]+=(((USV_State.Engine_power&0x01)^0x01)<<2);
	frame.data[2]+=(((USV_State.Engine_power&0x02)^0x01)<<3);
	frame.data[2]+=((Rudder_Detail_St.Heatbeat_Rudder^0x01)<<4);
	frame.data[2]+=((Energy_Control_St.Power_Control_Heatbeat^0x01)<<5);
	frame.data[2]+=((POW_CAN_State.System_Heatbeat^0x01)<<6);
	frame.data[2]+=((UAV_Detail_St.UAV_Heatbeat^0x01)<<7);
	frame.data[3]=(SP_CAN_Model_Con.System_Heatbeat^0x01);

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65292 error!\n");
	return	;
}
void Get_CAN_Energy_Con()
{
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0x00,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//控制指令优先级
	CAN_PGN=CAN_Energy_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	
	//各子系统状态1异常0正常消防系统暂无状态
	frame.data[0]=(DSP_State_Msg.DSP_Heatbeat^0x01);
	frame.data[0]+=((USV_RM_MSG.MPC_Heatbeat^0x01)<<1);
	frame.data[0]+=(((USV_State.Engine_power&0x01)^0x01)<<2);
	frame.data[0]+=(((USV_State.Engine_power&0x02)^0x01)<<3);
	frame.data[0]+=((Rudder_Detail_St.Heatbeat_Rudder^0x01)<<4);
	frame.data[0]+=((Energy_Control_St.Power_Control_Heatbeat^0x01)<<5);
	frame.data[0]+=((POW_CAN_State.System_Heatbeat^0x01)<<6);
	frame.data[0]+=((UAV_Detail_St.UAV_Heatbeat^0x01)<<7);
	frame.data[1]=(SP_CAN_Model_Con.System_Heatbeat^0x01);
//	printf("Energy_Send\n");

	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);
	
//	if(ibytes!=sizeof(frame))
//		printf("send PGN65292 error!\n");

	return	;

}
void Send_CAN_State(void)//发送CAN状态信息
{
	Send_CAN_65305();
	usleep(500);//0.5ms	
	Send_CAN_65306();
	usleep(500);//0.5ms	
	Send_CAN_65307();
	usleep(500);//0.5ms	
	Send_CAN_65308();
	usleep(500);//0.5ms	
	Send_CAN_65309();
	usleep(500);//0.5ms	
	Send_CAN_65310();
	usleep(500);//0.5ms		
	Send_CAN_65311();
	usleep(500);//0.5ms	
	return	;
}
void Send_CAN_65305()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);

	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST1_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0x00ff);
	frame.data[1]=(((USV_State.Dradio_USV_Sailing_State.USV_Speed)&0xff00)>>8);
	frame.data[2]=((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0x00ff);
	frame.data[3]=(((USV_State.Dradio_USV_Sailing_State.USV_Heading)&0xff00)>>8);
	frame.data[4]=((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0x00ff);
	frame.data[5]=(((USV_State.Dradio_USV_Sailing_State.USV_Pitch)&0xff00)>>8);
	frame.data[6]=((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0x00ff);
	frame.data[7]=(((USV_State.Dradio_USV_Sailing_State.USV_Roll)&0xff00)>>8);
//	for(count=0;count<8;count++)
//		frame.data[count]=count;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65300 error!\n");
	return	;

}
void Send_CAN_65306()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST2_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=(uint8)(USV_State.Dradio_USV_Sailing_State.USV_Heave);
	frame.data[1]=USV_State.Dradio_USV_Model_State.Latitude_Sign_St;
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Longitude_Sign_St)<<1);
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Sailing_Model_St)<<2);
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Differential_Model_St)<<4);
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Set_Return_Point_Model_St)<<5);
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Speed_Constant_Model_St)<<6);
	frame.data[1]+=((USV_State.Dradio_USV_Model_State.Direction_Constant_Model_St)<<7);
	frame.data[2]=USV_State.Dradio_USV_Location.USV_Latitude_Degree;
	frame.data[3]=USV_State.Dradio_USV_Location.USV_Latitude_Minute;
	frame.data[4]=USV_State.Dradio_USV_Location.USV_Latitude_Second;
	frame.data[5]=USV_State.Dradio_USV_Location.USV_Latitude_Decimal;
	frame.data[6]=USV_State.Dradio_USV_Location.USV_Longitude_Degree;
	frame.data[7]=USV_State.Dradio_USV_Location.USV_Longitude_Minute;
	
//	for(count=0;count<8;count++)
//		frame.data[count]=2*count;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65301 error!\n");
	
	return	;
}
void Send_CAN_65307()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST3_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_State.Dradio_USV_Location.USV_Longitude_Second;
	frame.data[1]=USV_State.Dradio_USV_Location.USV_Longitude_Decimal;
	frame.data[2]=((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0x00ff);
	frame.data[3]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Left_St)&0xff00)>>8);
	frame.data[4]=((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0x00ff);
	frame.data[5]=(((USV_State.Dradio_USV_Drive_State.Accelerator_Right_St)&0xff00)>>8);
	frame.data[6]=USV_State.Dradio_USV_Drive_State.Gear_Left_St;
	frame.data[7]=USV_State.Dradio_USV_Drive_State.Gear_Right_St;
	
//	for(count=0;count<8;count++)
//		frame.data[count]=3*count;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65302 error!\n");
	
	return	;
}
void Send_CAN_65308()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST4_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St;
	frame.data[1]=USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St;
	frame.data[2]=USV_State.USV_Oil;
	frame.data[3]=USV_State.Dradio_USV_Battery.Battery_Left;
	frame.data[4]=USV_State.Dradio_USV_Battery.Battery_Right;
	frame.data[5]=USV_State.Dradio_USV_Battery.Battery_TEMP_Left;
	frame.data[6]=USV_State.Dradio_USV_Battery.Battery_TEMP_Right;
	frame.data[7]=USV_State.USV_Sailing_Intel_Sign;
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.UAV_Power_St)<<2);
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.Stable_Platform_Power_St)<<3);
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.Camera_ahead_Power_St)<<4);
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.D_radar_Power_St)<<5);
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.Camera_main_Power_St)<<6);
	frame.data[7]+=((USV_State.Dradio_USV_Device_State.Horn_Power_St)<<7);
//	printf("data[3]=0x%2x\n",frame.data[3]);
//	printf("data[4]=0x%2x\n",frame.data[4]);
	
//	for(count=0;count<8;count++)
//		frame.data[count]=4*count;
//		printf("data[count]=0x%2x\n",frame.data[count]);
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65303 error!\n");
	
	return	;

}
void Send_CAN_65309()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST5_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_State.Dradio_USV_Device_State.Searchlight_Power_St;
	frame.data[0]+=((USV_State.Dradio_USV_Device_State.Camera_lesser_Power_St)<<1);
	frame.data[0]+=((USV_State.Dradio_USV_Device_State.Navigationlight_Power_St)<<2);
	frame.data[0]+=((USV_State.Dradio_USV_Device_State.Camera_tail_Power_St)<<3);
	frame.data[0]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Left_Charging)<<4);
	frame.data[0]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_Right_Charging)<<5);
	frame.data[0]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_L)<<6);
	frame.data[0]+=((USV_State.Dradio_USV_Battery_Alarm.Battery_TEMP_Alarm_R)<<7);
	frame.data[1]=USV_State.Dradio_USV_Fire.Outfire_St;
	frame.data[1]+=((USV_State.Dradio_USV_Fire.Water_Level_St)<<1);
	frame.data[1]+=((USV_State.Dradio_USV_Fire.Ventilation_St)<<2);
	frame.data[1]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Left)<<3);
	frame.data[1]+=((USV_State.Dradio_USV_Engine_Alarm.SuperLoad_Alarm_Right)<<4);
	frame.data[1]+=((USV_State.Dradio_USV_Engine_Alarm.InletPressure_Alarm_Left)<<5);
	frame.data[1]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right)<<6);
	frame.data[1]+=((USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Left)<<7);
	frame.data[2]=USV_State.Dradio_USV_Engine_Alarm.InletTEMP_Alarm_Right;
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Left)<<1);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_TEMP_Alarm_Right)<<2);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Left)<<3);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.Cooling_Level_Alarm_Right)<<4);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Left)<<5);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.OilPressure_Alarm_Right)<<6);
	frame.data[2]+=((USV_State.Dradio_USV_Engine_Alarm.Fuel_Moisture_Alarm)<<7);
	frame.data[3]=USV_State.Hull_Fan_st;
	frame.data[3]+=((USV_State.Hull_Pump_st)<<2);
	frame.data[3]+=((USV_State.Dradio_USV_Model_State.Panel_Control_Sign_ST)<<6);
	frame.data[3]+=((USV_State.Dradio_USV_Model_State.Emergency_St)<<7);
	frame.data[4]=USV_State.APP_12V;
	frame.data[4]+=((USV_State.APP_24V)<<1);	
	frame.data[4]+=((USV_State.Outboard_Engine_st_L)<<2);	
	frame.data[4]+=((USV_State.Outboard_Engine_st_R)<<4);	
	frame.data[4]+=(E_Stop<<6);		
	frame.data[5]=((USV_State.Dradio_USV_Time.Date_Inside)&0x0000ff);
	frame.data[6]=(((USV_State.Dradio_USV_Time.Date_Inside)&0x00ff00)>>8);
	frame.data[7]=(((USV_State.Dradio_USV_Time.Date_Inside)&0xff0000)>>16);
//	printf("frame.data[3]=0x%.2x-frame.data[4]=0x%.2x\n",frame.data[3],frame.data[4]);
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65302 error!\n");

}
void Send_CAN_65310()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST6_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=((USV_State.Dradio_USV_Time.Time_Inside)&0x0000ff);
	frame.data[1]=(((USV_State.Dradio_USV_Time.Time_Inside)&0x00ff00)>>8);
	frame.data[2]=(((USV_State.Dradio_USV_Time.Time_Inside)&0xff0000)>>16);
	frame.data[3]=USV_State.Dradio_USV_Engine_State.Ldle_Time_Left;
	frame.data[4]=USV_State.Dradio_USV_Engine_State.Ldle_Time_Right;
	frame.data[5]=USV_State.Dradio_USV_Engine_State.Fuel_ConRate;
	frame.data[6]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_RL_St;
	frame.data[7]=USV_State.Dradio_USV_Stable_Platform_St.Stable_Platform_AT_St;
	
//	for(count=0;count<8;count++)
//		frame.data[count]=3*count;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65302 error!\n");
	
}

void Send_CAN_65311()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=6;//状态指令优先级
	CAN_PGN=SP_ST7_PGN;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]=USV_State.Dradio_USV_UAV_State.Platform_Hatch_St;
	frame.data[0]+=((USV_State.Dradio_USV_UAV_State.Platform_Lift_St)<<3);
	frame.data[0]+=((USV_State.Dradio_USV_UAV_State.Platform_Open_St)<<6);
	frame.data[1]=USV_State.Dradio_USV_UAV_State.UAV_Charging_St;
	frame.data[1]+=USV_State.USV_Current_Sailing;
	frame.data[1]+=((USV_State.Sailing_Nummber)<<4);
	frame.data[2]=USV_State.Engine_power;
	frame.data[2]+=((USV_State.Engine_run)<<2);
//	printf("Engine_run=%d\n",USV_State.Engine_run);
	frame.data[2]+=((USV_State.USV_oil_sign)<<4);
	frame.data[3]=USV_State.USV_Speed_limit;
	frame.data[4]=USV_State.Conrtol_System_Msg.Intelligent_rudder;
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Stabilized_platform)<<1;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Energy_management)<<2;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Power_management)<<3;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.UAV_platform)<<4;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Fire_fighting_system)<<5;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Engine_L)<<6;		
	frame.data[4]+=(USV_State.Conrtol_System_Msg.Engine_R)<<7;	
	frame.data[5]=SR_Config_Msg.Motor_Fire_Time_spn520226;
//	printf("Motor_Fire_Time_spn520226=%d\n",SR_Config_Msg.Motor_Fire_Time_spn520226);
/*	printf("Platform_Hatch_St=%d\n",USV_State.Dradio_USV_UAV_State.Platform_Hatch_St);
	printf("Platform_Lift_St=%d\n",USV_State.Dradio_USV_UAV_State.Platform_Lift_St);
	printf("Platform_Open_St=%d\n",USV_State.Dradio_USV_UAV_State.Platform_Open_St);
	printf("UAV_Charging_St=%d\n",USV_State.Dradio_USV_UAV_State.UAV_Charging_St);*/
	
	//2017-04-24 14:02:01 添加了平移旋转，仅转发报文
	frame.data[6]=SP_CAN_Model_Con.translation_mode;
	frame.data[6]+=SP_CAN_Model_Con.translation_command <<2;
	frame.data[6]+=SP_CAN_Model_Con.rotation_mode <<4;
	frame.data[6]+=SP_CAN_Model_Con.rotation_command <<6;

	//2017年8月13日 09:52:25 添加通讯连接状态
	frame.data[7] =	(~Dradio_Com1_Sign)&0x01;
	frame.data[7]+= ((~Bradio_Con_Sign)&0x01)<<1;
	frame.data[7]+= ((~BDS_Con_Sign)&0x01)<<2;	

//	for(count=0;count<8;count++)
//		frame.data[count]=3*count;
	write_can(CAN_0,&frame,sizeof(frame));
//	sleep_1(1);

//	if(ibytes!=sizeof(frame))
//		printf("send PGN65302 error!\n");

}
void Send_CAN_CRC()
{
//	uint8 count;
	uint8 CAN_add,CAN_Priority;
	uint32 CAN_PGN;
	struct can_frame frame;
	memset((uint8*)&frame.can_id,0,sizeof(frame));
	memset((uint8*)&frame.data[0],0xff,8);
	CAN_add=CAN_ARM_ADD;
	CAN_Priority=3;//状态指令优先级
	CAN_PGN=65319;
	frame.can_id+=CAN_add;
	frame.can_id+=(CAN_PGN<<8);
	frame.can_id+=(CAN_Priority<<26);
	frame.can_id=(CAN_EFF_FLAG|frame.can_id);//扩展帧
	frame.can_dlc=8;//数据长度
	frame.data[0]='V';
	frame.data[1]='E';
	frame.data[2]='R';
	frame.data[3]='S';
	frame.data[4]='I';
	frame.data[5]='O';	
	frame.data[6]='N';	

	write_can(CAN_0,&frame,sizeof(frame));

	return	;
}

//
void Send_CAN_Cfg()
{
	uint8 sign=0;
	if(Dradio_Config.Config_Direction=='T')				//配置读写标志
	{
		SR_Config_Msg.Drive_Nummber_spn520252=Dradio_Config.Basic_Cfg.Drive_Num;
		if(Dradio_Config.Throttle_Cfg.Motor_Sign==1)			//油门参数变化标志，现为配置文件出错置 1
		{
			SR_Config_Msg.Motor_MAX_Speed_L_spn520220=Dradio_Config.Throttle_Cfg.Motor_Speed_Max_L;				//
			SR_Config_Msg.Motor_Idling_Speed_L_spn520221=Dradio_Config.Throttle_Cfg.Motor_Speed_Idle_L;
			SR_Config_Msg.Motor_MAX_Speed_R_spn520222=Dradio_Config.Throttle_Cfg.Motor_Speed_Max_R;
			SR_Config_Msg.Motor_Idling_Speed_R_spn520223=Dradio_Config.Throttle_Cfg.Motor_Speed_Idle_R;
			SR_Config_Msg.Motor_Travel_L_spn520224=Dradio_Config.Throttle_Cfg.Motor_Trip_L;
			SR_Config_Msg.Motor_Travel_R_spn520225=Dradio_Config.Throttle_Cfg.Motor_Trip_R;
			SR_Config_Msg.Motor_Direction_L_spn520227=Dradio_Config.Throttle_Cfg.Motor_Direction_L;					     
			SR_Config_Msg.Motor_Direction_R_spn520228=Dradio_Config.Throttle_Cfg.Motor_Direction_R;	
			SR_Config_Msg.Motor_Fire_Time_spn520226=Dradio_Config.Throttle_Cfg.Motor_Fire_Time;	
//发动机左发动机油门系数			
			Accelerator_Coefficient_L=(SR_Config_Msg.Motor_MAX_Speed_L_spn520220*30-SR_Config_Msg.Motor_Idling_Speed_L_spn520221*15)/100.0;		//速度从配置文件获取
			Accelerator_Coefficient_R=(SR_Config_Msg.Motor_MAX_Speed_R_spn520222*30-SR_Config_Msg.Motor_Idling_Speed_R_spn520223*15)/100.0;
			sign=1;
		}
		if(Dradio_Config.Gear_Cfg.Gear_Sign==1)					//现为配置文件出错置 1
		{
			SR_Config_Msg.Gear_Forward_Travel_L_spn520240=Dradio_Config.Gear_Cfg.Gear_Forward_Travel_L;	   
			SR_Config_Msg.Gear_Forward_Travel_R_spn520241=Dradio_Config.Gear_Cfg.Gear_Forward_Travel_R;	   
			SR_Config_Msg.Gear_Back_Travel_L_spn520242=Dradio_Config.Gear_Cfg.Gear_Back_Travel_L;			   
			SR_Config_Msg.Gear_Back_Travel_R_spn520243=Dradio_Config.Gear_Cfg.Gear_Back_Travel_R;			   
			SR_Config_Msg.Gear_Forward_Angle_L_spn520244=Dradio_Config.Gear_Cfg.Gear_Forward_Angle_L;		   
			SR_Config_Msg.Gear_Forward_Angle_R_spn520245=Dradio_Config.Gear_Cfg.Gear_Forward_Angle_R;		   
			SR_Config_Msg.Gear_Back_Angle_L_spn520246=Dradio_Config.Gear_Cfg.Gear_Back_Angle_L;			   
			SR_Config_Msg.Gear_Back_Angle_R_spn520247=Dradio_Config.Gear_Cfg.Gear_Back_Angle_R;			   
			SR_Config_Msg.Gear_Neutral_Angle_L_spn520248=Dradio_Config.Gear_Cfg.Gear_Neutral_Angle_L;		   
			SR_Config_Msg.Gear_Neutral_Angle_R_spn520249=Dradio_Config.Gear_Cfg.Gear_Neutral_Angle_R;		   
			SR_Config_Msg.Gear_Direction_L_spn520250=Dradio_Config.Gear_Cfg.Gear_Direction_L;			
			SR_Config_Msg.Gear_Direction_R_spn520251=Dradio_Config.Gear_Cfg.Gear_Direction_R;		
			
			Gear_Coefficient_L_F=SR_Config_Msg.Gear_Forward_Angle_L_spn520244/25.0;
			Gear_Coefficient_L_B=SR_Config_Msg.Gear_Back_Angle_L_spn520246/25.0;
			Gear_Coefficient_R_F=SR_Config_Msg.Gear_Forward_Angle_R_spn520245/25.0;
			Gear_Coefficient_R_B=SR_Config_Msg.Gear_Back_Angle_R_spn520247/25.0;
			sign=1;
		}
		if(Dradio_Config.Rudder_Cfg.Rudder_Sign==1)					//现为配置文件出错置 1
		{
			SR_Config_Msg.Rudder_Angle_Max_L_spn520260=Dradio_Config.Rudder_Cfg.Rudder_Angle_Max_L;					   
			SR_Config_Msg.Rudder_Angle_Max_R_spn520261=Dradio_Config.Rudder_Cfg.Rudder_Angle_Max_R;					   
			SR_Config_Msg.Rudder_Left_Limit_Resistance_L_spn520262=Dradio_Config.Rudder_Cfg.Rudder_Left_Limit_Resistance_L;
			SR_Config_Msg.Rudder_Left_Limit_Resistance_R_spn520263=Dradio_Config.Rudder_Cfg.Rudder_Left_Limit_Resistance_R;
			SR_Config_Msg.Rudder_Middle_Limit_Resistance_L_spn520264=Dradio_Config.Rudder_Cfg.Rudder_Middle_Limit_Resistance_L;
			SR_Config_Msg.Rudder_Middle_Limit_Resistance_R_spn520265=Dradio_Config.Rudder_Cfg.Rudder_Middle_Limit_Resistance_R;
			SR_Config_Msg.Rudder_Right_Limit_Resistance_L_spn520266=Dradio_Config.Rudder_Cfg.Rudder_Right_Limit_Resistance_L;
			SR_Config_Msg.Rudder_Right_Limit_Resistance_R_spn520267=Dradio_Config.Rudder_Cfg.Rudder_Right_Limit_Resistance_R;
			SR_Config_Msg.Rudder_Control_Angle_Max_L_spn520268=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L;			
			SR_Config_Msg.Rudder_Control_Angle_Max_R_spn520269=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R;		 
			SR_Config_Msg.Rudder_Control_Angle_Min_L_spn520270=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L;		 
			SR_Config_Msg.Rudder_Control_Angle_Min_R_spn520271=Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R;		
			SR_Config_Msg.Rudder_Left_Travel_L_spn520272=Dradio_Config.Rudder_Cfg.Rudder_Left_Travel_L;		
			SR_Config_Msg.Rudder_Left_Travel_R_spn520273=Dradio_Config.Rudder_Cfg.Rudder_Left_Travel_R;		
			SR_Config_Msg.Rudder_Right_Travel_L_spn520274=Dradio_Config.Rudder_Cfg.Rudder_Right_Travel_L;		
			SR_Config_Msg.Rudder_Right_Travel_R_spn520275=Dradio_Config.Rudder_Cfg.Rudder_Right_Travel_R;					
			SR_Config_Msg.Rudder_Control_Accuracy_L_spn520276=Dradio_Config.Rudder_Cfg.Rudder_Control_Accuracy_L;
			SR_Config_Msg.Rudder_Control_Accuracy_R_spn520277=Dradio_Config.Rudder_Cfg.Rudder_Control_Accuracy_R;
			SR_Config_Msg.Rudder_Direction_L_spn520278=Dradio_Config.Rudder_Cfg.Rudder_Direction_L;						   
			SR_Config_Msg.Rudder_Direction_R_spn520279=Dradio_Config.Rudder_Cfg.Rudder_Direction_R;			

			Rudder_Coefficient_L=SR_Config_Msg.Rudder_Angle_Max_L_spn520260/100.0;
			Rudder_Coefficient_R=SR_Config_Msg.Rudder_Angle_Max_R_spn520261/100.0;
			sign=1;
		}
		if((Dradio_Config.Algorithm_Cfg[0].Algorithm_Sign==1)||(Dradio_Config.Algorithm_Cfg[1].Algorithm_Sign==1)||(Dradio_Config.Environment_Cfg.Environment_Sign==1))
			sign=1;
		if(sign==1)
			Write_Config();
		else 
			USV_State.Config_Nummber=0x07;//配置成功
	}

	if(SR_CAN_State.Config_Answer_spn520778==0)
	{
		if(Dradio_Config.Throttle_Cfg.Motor_Sign==1)
			Get_CAN_SR_Engine_Config();
		if(Dradio_Config.Gear_Cfg.Gear_Sign==1)
		{
			Get_CAN_SR_Gear_Config();
			usleep(500);//0.5ms	
			Get_CAN_SR_Gear2_Config();
		}
		if(Dradio_Config.Rudder_Cfg.Rudder_Sign==1)
		{
			Get_CAN_SR_Rudder_Config();
			usleep(500);//0.5ms	
			Get_CAN_SR_Rudder2_Config();
			usleep(500);//0.5ms	
			Get_CAN_SR_Rudder3_Config();
		}
//		printf("write can config\n");
	}
	else if(SR_CAN_State.Config_Answer_spn520778==1)
	{
		USV_State.Config_Nummber=0x07;//配置成功
		Dradio_Config.Throttle_Cfg.Motor_Sign=0;
		Dradio_Config.Gear_Cfg.Gear_Sign=0;
		Dradio_Config.Rudder_Cfg.Rudder_Sign=0;
		Dradio_Config.Algorithm_Cfg[0].Algorithm_Sign=0;
		Dradio_Config.Algorithm_Cfg[1].Algorithm_Sign=0;
		Dradio_Config.Environment_Cfg.Environment_Sign=0;
//		printf("write can config is ok\n");
	}
}

//根据宽台下发的配置更新配置文件
void Write_Config()
{
	char chan[8][50]={"**********Basic_Cfg**********","**********Board_Cfg**********","**********Throttle_Cfg**********","**********Gear_Cfg**********",
							"**********Rudder_Cfg**********","**********Algorithm_Cfg0**********","**********Algorithm_Cfg1**********","**********Environment_Cfg**********"};
	FILE *pFile;
#ifdef WINNT
	pFile=fopen("../../media/USVconfig.cfg", "w+");
#else
	pFile=fopen("media/USVconfig.cfg", "w+");
#endif
	if(pFile == NULL)
	{
		printf("open USVconfig.cfg error\n");
		return ;
	}
	fprintf(pFile,"%s\n",chan[0]);
	fprintf(pFile,"Config_Num=%d\n",Dradio_Config.Config_Num);
	fprintf(pFile,"Hull_Type=%d\n",Dradio_Config.Basic_Cfg.Hull_Type);
	fprintf(pFile,"Drive_Type=%d\n",Dradio_Config.Basic_Cfg.Drive_Type);
	fprintf(pFile,"Propeller_Type=%d\n",Dradio_Config.Basic_Cfg.Propeller_Type);
	fprintf(pFile,"Motor_Type=%d\n",Dradio_Config.Basic_Cfg.Motor_Type);
	fprintf(pFile,"Drive_Num=%d\n",Dradio_Config.Basic_Cfg.Drive_Num);

	fprintf(pFile,"%s\n",chan[1]);
	fprintf(pFile,"MMSI=%d\n",Dradio_Config.Board_Cfg.MMSI);
	fprintf(pFile,"Board_length=%d\n",Dradio_Config.Board_Cfg.Board_length);
	fprintf(pFile,"Board_Wide=%d\n",Dradio_Config.Board_Cfg.Board_Wide);
	fprintf(pFile,"Board_Deep=%d\n",Dradio_Config.Board_Cfg.Board_Deep);
	fprintf(pFile,"Max_Static_Draft=%d\n",Dradio_Config.Board_Cfg.Max_Static_Draft);
	fprintf(pFile,"Max_Speed=%d\n",Dradio_Config.Board_Cfg.Max_Speed);
	fprintf(pFile,"Max_Voyage=%d\n",Dradio_Config.Board_Cfg.Max_Voyage);
	fprintf(pFile,"INS_Antenna_Distance=%d\n",Dradio_Config.Board_Cfg.INS_Antenna_Distance);
	
	fprintf(pFile,"%s\n",chan[2]);
	fprintf(pFile,"Motor_Speed_Max_L=%d\n",Dradio_Config.Throttle_Cfg.Motor_Speed_Max_L);
	fprintf(pFile,"Motor_Speed_Idle_L=%d\n",Dradio_Config.Throttle_Cfg.Motor_Speed_Idle_L);
	fprintf(pFile,"Motor_Speed_Max_R=%d\n",Dradio_Config.Throttle_Cfg.Motor_Speed_Max_R);
	fprintf(pFile,"Motor_Speed_Idle_R=%d\n",Dradio_Config.Throttle_Cfg.Motor_Speed_Idle_R);
	fprintf(pFile,"Motor_Trip_L=%d\n",Dradio_Config.Throttle_Cfg.Motor_Trip_L);
	fprintf(pFile,"Motor_Trip_R=%d\n",Dradio_Config.Throttle_Cfg.Motor_Trip_R);
	fprintf(pFile,"Motor_Address_L=%d\n",Dradio_Config.Throttle_Cfg.Motor_Address_L);
	fprintf(pFile,"Motor_Address_R=%d\n",Dradio_Config.Throttle_Cfg.Motor_Address_R);
	fprintf(pFile,"Motor_Direction_L=%d\n",Dradio_Config.Throttle_Cfg.Motor_Direction_L);
	fprintf(pFile,"Motor_Direction_R=%d\n",Dradio_Config.Throttle_Cfg.Motor_Direction_R);
	fprintf(pFile,"Motor_Fire_Time=%d\n",Dradio_Config.Throttle_Cfg.Motor_Fire_Time);
	
	fprintf(pFile,"%s\n",chan[3]);
	fprintf(pFile,"Gear_Forward_Travel_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Forward_Travel_L);
	fprintf(pFile,"Gear_Forward_Travel_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Forward_Travel_R);
	fprintf(pFile,"Gear_Back_Travel_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Back_Travel_L);
	fprintf(pFile,"Gear_Back_Travel_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Back_Travel_R);
	fprintf(pFile,"Gear_Forward_Angle_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Forward_Angle_L);
	fprintf(pFile,"Gear_Forward_Angle_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Forward_Angle_R);
	fprintf(pFile,"Gear_Back_Angle_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Back_Angle_L);
	fprintf(pFile,"Gear_Back_Angle_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Back_Angle_R);
	fprintf(pFile,"Gear_Neutral_Angle_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Neutral_Angle_L);
	fprintf(pFile,"Gear_Neutral_Angle_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Neutral_Angle_R);
	fprintf(pFile,"Gear_Direction_L=%d\n",Dradio_Config.Gear_Cfg.Gear_Direction_L);
	fprintf(pFile,"Gear_Direction_R=%d\n",Dradio_Config.Gear_Cfg.Gear_Direction_R);

	fprintf(pFile,"%s\n",chan[4]);
	fprintf(pFile,"Rudder_Angle_Max_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Angle_Max_L);
	fprintf(pFile,"Rudder_Angle_Max_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Angle_Max_R);
	fprintf(pFile,"Rudder_Left_Limit_Resistance_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Left_Limit_Resistance_L);
	fprintf(pFile,"Rudder_Left_Limit_Resistance_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Left_Limit_Resistance_R);
	fprintf(pFile,"Rudder_Middle_Limit_Resistance_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Middle_Limit_Resistance_L);
	fprintf(pFile,"Rudder_Middle_Limit_Resistance_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Middle_Limit_Resistance_R);
	fprintf(pFile,"Rudder_Right_Limit_Resistance_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Right_Limit_Resistance_L);
	fprintf(pFile,"Rudder_Right_Limit_Resistance_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Right_Limit_Resistance_R);
	fprintf(pFile,"Rudder_Control_Angle_Max_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_L);
	fprintf(pFile,"Rudder_Control_Angle_Max_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Max_R);
	fprintf(pFile,"Rudder_Control_Angle_Min_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_L);
	fprintf(pFile,"Rudder_Control_Angle_Min_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Angle_Min_R);
	fprintf(pFile,"Rudder_Left_Travel_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Left_Travel_L);
	fprintf(pFile,"Rudder_Left_Travel_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Left_Travel_R);
	fprintf(pFile,"Rudder_Right_Travel_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Right_Travel_L);
	fprintf(pFile,"Rudder_Right_Travel_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Right_Travel_R);		
	fprintf(pFile,"Rudder_Control_Accuracy_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Accuracy_L);
	fprintf(pFile,"Rudder_Control_Accuracy_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Control_Accuracy_R);
	fprintf(pFile,"Rudder_Direction_L=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Direction_L);
	fprintf(pFile,"Rudder_Direction_R=%d\n",Dradio_Config.Rudder_Cfg.Rudder_Direction_R);

	fprintf(pFile,"%s\n",chan[5]);
	fprintf(pFile,"Control_Type=%d\n",Dradio_Config.Algorithm_Cfg[0].Control_Type);
	fprintf(pFile,"PID_P=%d\n",Dradio_Config.Algorithm_Cfg[0].PID_P);
	fprintf(pFile,"PID_I=%d\n",Dradio_Config.Algorithm_Cfg[0].PID_I);
	fprintf(pFile,"PID_D=%d\n",Dradio_Config.Algorithm_Cfg[0].PID_D);
	fprintf(pFile,"S_K1=%d\n",Dradio_Config.Algorithm_Cfg[0].S_K1);
	fprintf(pFile,"S_K2=%d\n",Dradio_Config.Algorithm_Cfg[0].S_K2);
	fprintf(pFile,"S_K3=%d\n",Dradio_Config.Algorithm_Cfg[0].S_K3);
	fprintf(pFile,"Filter_A=%d\n",Dradio_Config.Algorithm_Cfg[0].Filter_A);
	fprintf(pFile,"Filter_B=%d\n",Dradio_Config.Algorithm_Cfg[0].Filter_B);
	fprintf(pFile,"Filter_C=%d\n",Dradio_Config.Algorithm_Cfg[0].Filter_C);

	fprintf(pFile,"%s\n",chan[6]);
	fprintf(pFile,"Control_Type=%d\n",Dradio_Config.Algorithm_Cfg[1].Control_Type);
	fprintf(pFile,"PID_P=%d\n",Dradio_Config.Algorithm_Cfg[1].PID_P);
	fprintf(pFile,"PID_I=%d\n",Dradio_Config.Algorithm_Cfg[1].PID_I);
	fprintf(pFile,"PID_D=%d\n",Dradio_Config.Algorithm_Cfg[1].PID_D);
	fprintf(pFile,"S_K1=%d\n",Dradio_Config.Algorithm_Cfg[1].S_K1);
	fprintf(pFile,"S_K2=%d\n",Dradio_Config.Algorithm_Cfg[1].S_K2);
	fprintf(pFile,"S_K3=%d\n",Dradio_Config.Algorithm_Cfg[1].S_K3);
	fprintf(pFile,"Filter_A=%d\n",Dradio_Config.Algorithm_Cfg[1].Filter_A);
	fprintf(pFile,"Filter_B=%d\n",Dradio_Config.Algorithm_Cfg[1].Filter_B);
	fprintf(pFile,"Filter_C=%d\n",Dradio_Config.Algorithm_Cfg[1].Filter_C);

	fprintf(pFile,"%s\n",chan[7]);
	fprintf(pFile,"Wind_Speed=%d\n",Dradio_Config.Environment_Cfg.Wind_Speed);
	fprintf(pFile,"Wave_Hight=%d\n",Dradio_Config.Environment_Cfg.Wave_Hight);
	fprintf(pFile,"Flow_Speed=%d\n",Dradio_Config.Environment_Cfg.Flow_Speed);
	
	fclose(pFile);
	return;
}

