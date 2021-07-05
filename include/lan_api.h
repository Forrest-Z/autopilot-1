/*
* lan_api.h --网口初始化
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2016-02-24，池晓阳，编写
*/
#ifndef	LAN_API_H
#define     LAN_API_H

#include <stdio.h>    
#include <stdlib.h>    
#include <string.h>    
 
#include <sys/types.h>    
  
 
#include <errno.h>    
   
#include <sys/stat.h>
#include <fcntl.h>

#ifndef WINNT
#include <sys/time.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <unistd.h> 
#else
#include<ws2tcpip.h>
#endif

#include "USV_const.h"


extern int GetCRC32(char* buff,uint32 length);
extern int CheckCRC(uint8 count,uint8* buff,uint32 length);
extern void Dradio_Con_Analytical(uint8 count,uint8  *Dradio_Con_str);
extern void Dradio_Con_Printf(uint8 count);
extern void GenHex2Str(uint8 *strbuf,uint8 *hexbuf, uint16 len);

void LAN0_RM_Analytical(LAN0_RM_Message *USV_RM_MSG,uint8 *buff,uint8 *Obs_num);
void Send_Motor_Detail_State(void);
void Update_Motor_Detail_State(void);
void Send_Rudder_Detail_State(void);
void Update_Rudder_Detail_State(void);
void Send_Stable_Platform_Detail_State(void);
void Update_Stable_Platform_Detail_State(void);
void Send_UAV_Detail_State(void);
void Update_UAV_Detail_State(void);
void Send_MCU_State(void);
void Update_MCU_State(void);
void Send_Hull_State(void);
void Update_Hull_State(void);
void Send_Smart_Navigation_Msg(void);
void Update_Smart_Navigation_Msg(void);
void Send_Panel_Control_Msg(void);
void Update_Panel_Control_Msg(void);
void Send_Energy_Control_Msg(void);
void Update_Energy_Control_Msg(void);
void Send_Power_Control_Msg(void);
void Update_Power_Control_Msg(void);
void Fill_Msg_Header(uint8  *msg_buff,uint8  msg_sign, uint8  msg_num );
uint32 Get_Mileage(void);
void Send_MPC_State(uint16 Heartbeat_Num);

//void LAN0_RM_Analytical(LAN0_RM_Message *USV_RM_MSG,uint8 *buff,uint8 Obs_num);

#endif 
