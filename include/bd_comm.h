#ifndef _BD_COMM_
#define _BD_COMM_



void BD_test(void);

void BD_Msg_Analytical(uint8 *BD_Com_Msg);	//北斗数据解析
void Send_BD_State(void);	//通过北斗通道发送报文

#endif