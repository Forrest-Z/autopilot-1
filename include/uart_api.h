/*
* uart_api.h --���ڳ�ʼ��
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#ifndef	UART_API_H
#define     UART_API_H

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <math.h>

#ifndef WINNT
#include <unistd.h>
#include <termios.h>
#endif

#include "USV_const.h"

#ifdef WINNT
//ʵ�����
//#define	COM0		"COM0"						//COM0
//#define	COM1		"COM1"						//COM1
//#define	COM2		"COM9"						//COM2
//#define	COM3		"COM7"						//COM3
//#define	COM4		"COM11"						//COM4
//#define	COM5		"COM2"						//COM5

//�������
#define	COM0		"COM4"						//COM0		DSP��������
#define	COM1		"\\.\COM30"					//COM1		AIS����
#define	COM2		"COM3"						//COM2		���ֵ�̨
#define	COM3		"COM7"						//COM3		����ͨѶ
#define	COM4		"COM9"					//COM4		���ֵ�̨(��̨)
#define	COM5		"COM2"						//COM5		�ߵ�
//#define	COM5		"COM8"						//COM5		��ӹߵ�



#else
	#ifdef MCU_V1
	#define	COM0		"/dev/ttyO2"						//COM0	DSP��������
	#define	COM1		"/dev/ttyO0"						//COM1	AIS����
	#define	COM2		"/dev/ttyO1"						//COM2	���ֵ�̨
	#define	COM3		"/dev/ttyO6"						//COM3	����ͨѶ
	#define	COM4		"/dev/ttyO3"						//COM4	���������̨(��̨)
	#define	COM5		"/dev/ttyO4"						//COM5	�ߵ�
	#endif
	#ifdef MCU_V2
	#define	COM0		"/dev/ttyO4"						//COM0��DSP�������ġ�		
	#define	COM1		"/dev/ttyO0"						//COM1	AIS����
	#define	COM2		"/dev/ttyO1"						//COM2	���ֵ�̨			�ⲿ˿ӡ COM1
	#define	COM3		"/dev/ttyO3"						//COM3	����ͨѶ			�ⲿ˿ӡ BDS
	#define	COM4		"/dev/ttyUSB1"						//COM4	���������̨(��̨)	�ⲿ˿ӡ COM2
	#define	COM5		"/dev/ttyO2"						//COM5	�ߵ�				�ⲿ˿ӡ GNSS	
	#endif    
	#ifdef MCU_V3
	#define	COM0		"/dev/ttyO4"						//COM0��DSP�������ġ�		
	#define	COM1		"/dev/ttyO0"						//COM1	AIS����
	#define	COM2		"/dev/ttyO1"						//COM2	���ֵ�̨			�ⲿ˿ӡ COM1
	#define	COM3		"/dev/ttyO6"						//COM3	����ͨѶ			�ⲿ˿ӡ BDS
	#define	COM4		"/dev/tty02"						//COM4	���������̨(��̨)	�ⲿ˿ӡ COM2
	#define	COM5		"/dev/ttyO5"						//COM5	�ߵ�				�ⲿ˿ӡ GNSS	
	#endif 
#endif

#define	MAX_UART_BUFF_LEN				1024  // 

extern uint8 AIS_Send_Sign;
extern uint8	Collision_Sign;	//��ײ��־

#ifdef WINNT
HANDLE open_port(char* com_port);
int set_com_config(HANDLE fd,int baud_rate, int data_bits, char parity, int stop_bits);
uint16 read_uart(HANDLE hCom,int8 *buff,uint16 len);
int8 write_uart(HANDLE hCom,int8 *buff,uint16 len);
void close_uart(HANDLE hCom);
#else
int open_port(char* com_port);
int set_com_config(int fd,int baud_rate, int data_bits, char parity, int stop_bits);
uint16 read_uart(int hCom,int8 *buff,uint16 len);
int8 write_uart(int hCom,int8 *buff,uint16 len);
void close_uart(int hCom);
#endif

extern uint8 Collision_Alternative[AIS_Collision_Num];//��ʱ��ȫ�Ĵ���ͳ�ƣ������
extern int8 isASCII_code[AIS_Buff_Num];
extern uint8 buffer_Data[10][AIS_Buff_Num];



int GetCRC32(char* buff,uint32 length);
int AscToHex(uint8 aHex);
int CheckCRC(uint8 count,uint8* buff,uint32 length);
void DSP_State_Analytical(DSP_State *DSP_State_Msg_str,uint8 *DSP_State_str);
void Dradio_Con_Analytical(uint8 count,uint8  *Dradio_Con_str);
void Dradio_Con_Printf(uint8 count);
void Dradio_Cfg_Analytical(Dradio_Config_Message*USV_Config_Str,uint8  *Dradio_Cfg_Str);
void Dradio_Sal_Analytical(uint8  *Dradio_Sal_Str);
void AIS_Decode(char* cAIS_Data);
uint8 ASCII_6bitASCII(BYTE ASCII_code);
void Position_Analytical_A(char* isASCII_code);//��Ϣ1/2/3
void UTC_Analytical(char* isASCII_code);//��Ϣ4/11
void Static_Analytical(uint8* isASCII_code_5);//��Ϣ5
void Position_Analytical_B(char* isASCII_code);//��Ϣ18
void Position_Analytical_B_Extended(char* isASCII_code);//��Ϣ19
void Static_Report_Analytical(char* isASCII_code_5);//��Ϣ24
void Collect_Safe_Ship_Num(void);
void Send_Config();
void monitor_ins_data_restored(void); //ת���������

extern sint8 Calculate_Collision(void );
extern void Obstacle_acoidance(void);
extern void Get_Compensating_Dst();


#endif /* UART_API_H */
