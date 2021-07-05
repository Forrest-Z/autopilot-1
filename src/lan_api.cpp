/*
* lan_ctrl.c --网口线程
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#include "stdafx.h"
#include "../include/usv_include.h"

//MPC发送DSP状态报文解析
void LAN0_RM_Analytical(LAN0_RM_Message *USV_RM_MSG,uint8 *buff,uint8 *Obs_num)
{
	uint8 count,dcount;
	uint8 *p_uint8_dst;
	uint8 *p_uint8_src;
	int  Dis;
	int i;
	USV_RM_MSG->Msg_Num=buff[5]*256+buff[6];
	USV_RM_MSG->Obstacles_Num=buff[10];
	//雷达信息
	p_uint8_src = (uint8 *)&buff[11];
	p_uint8_dst = (uint8 *)&USV_RM_MSG[0].RM_radar_Msg[0].obstacle_locate.lng;

	//test
	//uint8 buff_test[254];
	//memcpy((uint8 *)buff_test,(uint8 *)buff,254);


	memcpy((uint8 *)p_uint8_dst,(uint8 *)p_uint8_src,200);
	
	//监控数据缓存 用于调试软件召唤及显示
	for(i=0;i<MONITOR_OBSTACLE_MAX_NUMBER;i++)
	{
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[i].obstacle_locate.lng = USV_RM_MSG->RM_radar_Msg[i].obstacle_locate.lng;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[i].obstacle_locate.lat = USV_RM_MSG->RM_radar_Msg[i].obstacle_locate.lat;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[i].obstacle_direction = USV_RM_MSG->RM_radar_Msg[i].obstacle_direction;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[i].obstacle_radius = USV_RM_MSG->RM_radar_Msg[i].obstacle_radius;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[i].obstacle_speed = USV_RM_MSG->RM_radar_Msg[i].obstacle_speed;
	}

/*	for(count=1;count<(buff[10]+1);count++)
	{
		dcount=10*count;
		USV_RM_MSG->RM_radar_Msg[count-1].OBS_Distance=buff[dcount+1]*65536+buff[dcount+2]*256+buff[dcount+3];
		USV_RM_MSG->RM_radar_Msg[count-1].OBS_Direction=buff[dcount+4]*256+buff[dcount+5];
		USV_RM_MSG->RM_radar_Msg[count-1].OBS_Speed=buff[dcount+6];
		USV_RM_MSG->RM_radar_Msg[count-1].OBS_Heading=buff[dcount+7]*256+buff[dcount+8];		
		USV_RM_MSG->RM_radar_Msg[count-1].OBS_Length=buff[dcount+9]*256+buff[dcount+10];
		Dis=USV_RM_MSG->RM_radar_Msg[count-1].OBS_Distance;
		Dis-=USV_RM_MSG->RM_radar_Msg[count-1].OBS_Length;
		if(Dis<=USV_Safe_Distance*2)//障碍物进入计算范围，开始计算tcpa和dcpa
			Obs_num[count-1]=1;
*/
/*		printf("[%d]OBS_Distance=%d\n",count-1,USV_RM_MSG->RM_radar_Msg[count-1].OBS_Distance);
		printf("[%d]OBS_Direction=%d\n",count-1,USV_RM_MSG->RM_radar_Msg[count-1].OBS_Direction);
		printf("[%d]OBS_Speed=%d\n",count-1,USV_RM_MSG->RM_radar_Msg[count-1].OBS_Speed);
		printf("[%d]OBS_Heading=%d\n",count-1,USV_RM_MSG->RM_radar_Msg[count-1].OBS_Heading);		
		printf("[%d]OBS_Length=%d\n",count-1,USV_RM_MSG->RM_radar_Msg[count-1].OBS_Length);		

		
	}*/
	//风信息
	USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Direction=buff[211]*256+buff[212];
	USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Speed=buff[213]*256+buff[214];
	USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Update=buff[215];
/*	printf("Wind_Direction=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Direction);	
	printf("Wind_Speed=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Speed);	
	printf("Wind_Update=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wind_Msg.Wind_Update);	*/
	
	//气温气压湿度
	USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Temperature=buff[216]*256+buff[217];
	USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Atmospheric=buff[218]*256+buff[219];
	USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Humidity=buff[220]*256+buff[221];
	USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Wheather_Updata=buff[222];
/*	printf("Temperature=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Temperature);	
	printf("Atmospheric=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Atmospheric);	
	printf("Humidity=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Humidity);	
	printf("Wheather_Updata=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Wheather_Msg.Wheather_Updata);	*/
	
	//降水
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall=buff[223]*65536+buff[224]*256+buff[225];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall_time=buff[226]*256+buff[227];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall_Intensity=buff[228]*256+buff[229];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail=buff[230]*256+buff[231];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail_Time=buff[232]*256+buff[233];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail_Intensity=buff[234]*256+buff[235];
	USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Precipitation_Updata=buff[236];
/*	printf("Rainfall=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall);	
	printf("Rainfall_time=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall_time);	
	printf("Rainfall_Intensity=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Rainfall_Intensity);	
	printf("Hail=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail);	
	printf("Hail_Time=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail_Time);	
	printf("Hail_Intensity=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Hail_Intensity);	
	printf("Precipitation_Updata=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Precipitation_Msg.Precipitation_Updata);	*/
	
	//系统信息
	USV_RM_MSG->RM_Meteorological_Msg.Sensor_Add=buff[237];
	USV_RM_MSG->RM_Meteorological_Msg.Heating_TEMP=buff[238]*256+buff[239];
	USV_RM_MSG->RM_Meteorological_Msg.Heating_VOL=buff[240]*256+buff[241];
/*	printf("Sensor_Add=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Sensor_Add);	
	printf("Heating_TEMP=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Heating_TEMP);	
	printf("Heating_VOL=%d\n",USV_RM_MSG->RM_Meteorological_Msg.Heating_VOL);	*/
	//MPC资源
	USV_RM_MSG->MPC_Msg_St.MPC_Disk=buff[242];
	USV_RM_MSG->MPC_Msg_St.MPC_Memory=buff[243];
	USV_RM_MSG->MPC_Msg_St.MCP_CPU=buff[244];
/*	printf("MPC_Disk=%d\n",USV_RM_MSG->MPC_Msg_St.MPC_Disk);	
	printf("MPC_Memory=%d\n",USV_RM_MSG->MPC_Msg_St.MPC_Memory);	
	printf("MCP_CPU=%d\n",USV_RM_MSG->MPC_Msg_St.MCP_CPU);	*/
	return  ;
}
