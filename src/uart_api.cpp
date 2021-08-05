/*
* uart_api.c --���ڳ�ʼ��
* �ķ��̱�(  �人)  �������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"
#include "../include/usv_include.h"
#ifdef WINNT
#include "../../win_prj/win_prj/win_drive.h"
#endif
uint8 Collision_Alternative[AIS_Collision_Num];//��ʱ��ȫ�Ĵ���ͳ�ƣ������
int8 isASCII_code[AIS_Buff_Num];
uint8 buffer_Data[10][AIS_Buff_Num];

#ifdef WINNT
HANDLE open_port(char* com_port)
{
	return uart_init_win(com_port);
}


uint16 read_uart(HANDLE hCom,int8 *buff,uint16 len)
{
	if(hCom ==INVALID_HANDLE_VALUE){
		return 0;
	}
	return uart_read_win(hCom,(uint8 *)buff,len);
}

int8 write_uart(HANDLE hCom,int8 *lpOutBuffer,uint16 write_len)
{
	if(hCom ==INVALID_HANDLE_VALUE){
		return 0;
	}
	return (int8)(uart_write_win(hCom,(uint8 *)lpOutBuffer,write_len));

}

void close_uart(HANDLE hCom)
{
	if(hCom ==INVALID_HANDLE_VALUE){
		return;
	}
	uart_close_win(hCom);
}

#else
int open_port(char* com_port)
{
    int fd;

    fd = open(com_port, O_RDWR|O_NOCTTY|O_NDELAY);

    if (fd < 0){
        perror("open serial port");
        return(-1);
    }

    if (fcntl(fd, F_SETFL, 0) < 0){
        perror("fcntl F_SETFL\n");
    }

    if (isatty(STDIN_FILENO) == 0){
        perror("standard input is not a terminal device");
    }

    return fd;
}


int set_com_config(int fd,int baud_rate, int data_bits, char parity, int stop_bits)
{
    struct termios new_cfg,old_cfg;
    int speed;

	if(fd<0){
		return -1;
	}
    if  (tcgetattr(fd, &old_cfg)  !=  0){
        perror("tcgetattr");
        return -1;
    }


    new_cfg = old_cfg;
    cfmakeraw(&new_cfg);
    new_cfg.c_cflag &= ~CSIZE;

    switch (baud_rate){
    case 2400:
        speed = B2400;
        break;

    case 4800:
        speed = B4800;
        break;

    case 9600:
        speed = B9600;
        break;

    case 19200:
        speed = B19200;
        break;

    case 38400:
        speed = B38400;
        break;

    default:
    case 115200:
        speed = B115200;
        break;
	case 460800:
        speed = B460800;
        break;
    }
    cfsetispeed(&new_cfg, speed);
    cfsetospeed(&new_cfg, speed);

    switch (data_bits){
    case 7:
        new_cfg.c_cflag |= CS7;
        break;

    default:
    case 8:
        new_cfg.c_cflag |= CS8;
        break;
    }

    switch (parity){
    default:
    case 'n':
    case 'N':
        new_cfg.c_cflag &= ~PARENB;
        new_cfg.c_iflag &= ~INPCK;
        break;

    case 'o':
    case 'O':
        new_cfg.c_cflag |= (PARODD | PARENB);
        new_cfg.c_iflag |= INPCK;
        break;

    case 'e':
    case 'E':
        new_cfg.c_cflag |= PARENB;
        new_cfg.c_cflag &= ~PARODD;
        new_cfg.c_iflag |= INPCK;
        break;

    case 's':  /*as no parity*/
    case 'S':
        new_cfg.c_cflag &= ~PARENB;
        new_cfg.c_cflag &= ~CSTOPB;
        break;
    }

    switch (stop_bits){
    default:
    case 1:
        new_cfg.c_cflag &=  ~CSTOPB;
        break;

    case 2:
        new_cfg.c_cflag |= CSTOPB;
    }

    new_cfg.c_cc[VTIME]  = 0;
    new_cfg.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);

    if((tcsetattr(fd, TCSANOW, &new_cfg)) != 0){
        perror("tcsetattr");
        return -1;
    }

    return 0;
}


uint16 read_uart(int hCom,int8 *buff,uint16 len)
{
	if(hCom<0){
		return 0;
	}
	return read(hCom,buff,len);
}

int8 write_uart(int hCom,int8 *lpOutBuffer,uint16 write_len)
{
	if(hCom<0){
		return 0;
	}
	return write(hCom,lpOutBuffer,write_len);
}

void close_uart(int hCom)
{
	if(hCom<0){
		return;
	}
	close(hCom);
}
#endif

/*************************************************
�� �� ����AscToHex()
������������ASCIIת��Ϊ����
**************************************************/
int AscToHex(uint8 aHex)
{
	if((aHex>=65)&&(aHex<=70))//A-F
		aHex -= 55;
	else if((aHex>=97)&&(aHex<=102))//a-f
		aHex -= 87;
	else if ((aHex>=48)&&(aHex<=57))//0-9
		aHex -= 48;
	else 
		aHex = 0xff;
	return aHex;
}
/*************************************************
��������:GetCRC32
��������:��ȡAIS�ַ���CRC32У��ֵ,�������ж�
��������:
***************************************************/
int GetCRC32(char* buff,uint32 length)
{

	int i,crc,CRC,CRC_1,CRC_2;
	crc=0;CRC=0;CRC_1=0;CRC_2=0;
	int len=length;
	crc=buff[1];
	for (i= 2; i < len-3; i++)
	{  
		crc =buff[i] ^crc;  
//		printf("crc=%d",crc);
	}  
	CRC_1=AscToHex(buff[len-2]);
	CRC_2=AscToHex(buff[len-1]);
	CRC=CRC_1*16+CRC_2;//У��ֵ
//	printf("CRC1=%d,CRC2=%d",CRC_1,CRC_2);
	if (crc==CRC)
		return 1;
	else
		return 0;
}
/*************************************************
��������:GetCRC
��������:�ַ���CRCУ��ֵ,�������ж�
��������:
***************************************************/
int CheckCRC(uint8 count,uint8* buff,uint32 length)
{

	int i,crc;
	crc=0;
	int len=length;
	crc=buff[count-1];
	for ( i = count; i < len-2; i++)//���������Ϊcrc����Ҫ���
	{  
		crc =buff[i] ^crc;  
	}  
//	printf("crc=0x%.2x\n",crc);
	if (crc==buff[len-1])
		return 1;
	else
		return 0;
}
//DSP״̬����
void DSP_State_Analytical(DSP_State *DSP_State_Msg_str,uint8 *DSP_State_str)
{
	DSP_State_Msg_str->Engine_power_st=DSP_State_str[10]&0x03;
	DSP_State_Msg_str->Engine_run_st=(DSP_State_str[10]&0x0C)>>2;
	DSP_State_Msg_str->USV_oil_st=(DSP_State_str[10]&0x10)>>4;
	DSP_State_Msg_str->Hull_Fan_st=(DSP_State_str[10]&0x60)>>5;
	DSP_State_Msg_str->Outfire_St=(DSP_State_str[10]&0x80)>>7;
	DSP_State_Msg_str->Hull_Pump_st=DSP_State_str[11]&0x0f;
	
	//���ƽ̨����ת��
	monitor_all_inf.module_report_inf[MONITOR_DSP_MSG].report_detail[0] = DSP_State_str[10];
	monitor_all_inf.module_report_inf[MONITOR_DSP_MSG].report_detail[1] = DSP_State_str[11];



/*	printf("Engine_power_st=%d\n",DSP_State_Msg_str->Engine_power_st);
	printf("Engine_run_st=%d\n",DSP_State_Msg_str->Engine_run_st);
	printf("USV_oil_st=%d\n",DSP_State_Msg_str->USV_oil_st);
	printf("Hull_Fan_st=%d\n",DSP_State_Msg_str->Hull_Fan_st);
	printf("Outfire_St=%d\n",DSP_State_Msg_str->Outfire_St);
	printf("Hull_Pump_st=%d\n",DSP_State_Msg_str->Hull_Pump_st);*/

}

void GenHex2Str(uint8 *strbuf,uint8 *hexbuf, uint16 len)
{
	uint16 i;
	uint8 h,l;

	for (i=0;i<len;i++)
	{
		h=(*(hexbuf+i)&0xf0)>>4;
		l=(*(hexbuf+i)&0x0f);
		if ((h>=0)&&(h<=9))
			*(strbuf++)='0'+h;
		else
			*(strbuf++)='A'-10+h;
		if ((l>=0)&&(l<=9))
			*(strbuf++)='0'+l;
		else
			*(strbuf++)='A'-10+l;
		*(strbuf++)=0x20;
	}
	*(--strbuf)='\0';
}



void printf_msg(uint8  *Dradio_Con_str)
{
	uint8 msg_buff[96];
	GenHex2Str(msg_buff,Dradio_Con_str,32);
//	SysLogMsgPost("���ձ���:%s",msg_buff);
}



//������̨����ָ�����
//count --��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ
void Dradio_Con_Analytical(uint8 count,uint8  *Dradio_Con_str)
{
	if(Dradio_Con_str[5]==USV_NUM)//�Ǳ���ָ��
	{
		USV_Control.USV_Control_Message[count].USV_Num=Dradio_Con_str[5];
		USV_Control.USV_Control_Message[count].Msg_Num=Dradio_Con_str[6]*256+Dradio_Con_str[7];
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.UAV_Power=(Dradio_Con_str[9]&0x03);
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Stable_Platform_Power=(Dradio_Con_str[9]&0x0c)>>2;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_ahead_Power=(Dradio_Con_str[9]&0x30)>>4;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.D_radar_Power=Dradio_Con_str[9]>>6;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_main_Power=(Dradio_Con_str[10]&0x03);
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Horn_Power=(Dradio_Con_str[10]&0x0c)>>2;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Searchlight_Power=(Dradio_Con_str[10]&0x30)>>4;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_lesser_Power=Dradio_Con_str[10]>>6;
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Navigationlight_Power=(Dradio_Con_str[11]&0x03);
		USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_tail_Power=(Dradio_Con_str[11]&0x0c)>>2;
		USV_Control.USV_Control_Message[count].Navigation_Tsak_sign=(Dradio_Con_str[11]&0xf0)>>4;
		USV_Control.USV_Control_Message[count].Engine_Run_L=(Dradio_Con_str[12]&0x03);
		USV_Control.USV_Control_Message[count].Engine_Run_R=(Dradio_Con_str[12]&0x0C)>>2;
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Sailing_Mod=(Dradio_Con_str[12]&0x30)>>4;
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Differential_Mod=(Dradio_Con_str[12]&0xc0)>>6;
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Set_Return_Point_Mod=(Dradio_Con_str[13]&0x03);
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Speed_Constant_Mod=(Dradio_Con_str[13]&0x0c)>>2;
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Direction_Constant_Mod=(Dradio_Con_str[13]&0x30)>>4;
		USV_Control.USV_Control_Message[count].Dradio_USV_Model.Return_Mod=(Dradio_Con_str[13]&0xc0)>>6;
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Gear_Left=Dradio_Con_str[14];
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Gear_Right=Dradio_Con_str[15];
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Accelerator_Left=Dradio_Con_str[16];			//�󷢶�������
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Accelerator_Right=Dradio_Con_str[17];
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Rudder_Angle_Left=Dradio_Con_str[18];
		USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Rudder_Angle_Right=Dradio_Con_str[19];
		USV_Control.USV_Control_Message[count].Speed_Limit=Dradio_Con_str[20];
		USV_Control.USV_Control_Message[count].Dradio_USV_Stable_Platform.Stable_Platform_RL=Dradio_Con_str[21];
		USV_Control.USV_Control_Message[count].Dradio_USV_Stable_Platform.Stable_Platform_AT=Dradio_Con_str[22];
		USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Hatch=(Dradio_Con_str[23]&0x03);
		USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Lift=(Dradio_Con_str[23]&0x0c)>>2;
		USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Open=(Dradio_Con_str[23]&0x30)>>4;
		USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.UAV_Charging=(Dradio_Con_str[23]&0xc0)>>6;
		USV_Control.USV_Control_Message[count].Hull_Fan_Con=Dradio_Con_str[24]&0x03;
		USV_Control.USV_Control_Message[count].Hull_Pump_Con=(Dradio_Con_str[24]&0x3C)>>2;
		USV_Control.USV_Control_Message[count].Oil_Con=(Dradio_Con_str[24]&0xC0)>>6;
//		USV_Control.USV_Control_Message[count].Sealight_LR_Con=Dradio_Con_str[25];
//		USV_Control.USV_Control_Message[count].Sealignt_HT_Con=Dradio_Con_str[26];
		USV_Control.USV_Control_Message[count].Application_24=(Dradio_Con_str[25]&0x03);
		USV_Control.USV_Control_Message[count].Application_12=((Dradio_Con_str[25]&0x0C)>>2);
		USV_Control.USV_Control_Message[count].Sealight_Speed=((Dradio_Con_str[25]&0x10)>>4);
		USV_Control.USV_Control_Message[count].Sealight_Direction=((Dradio_Con_str[25]&0xe0)>>5);
		USV_Control.USV_Control_Message[count].Emergency_Stop=Dradio_Con_str[26]&0x01;
		USV_Control.USV_Control_Message[count].Outboard_Engine_L=((Dradio_Con_str[26]&0x06)>>1);
		USV_Control.USV_Control_Message[count].Outboard_Engine_R=((Dradio_Con_str[26]&0x18)>>3);

		USV_Control.USV_Control_Message[count].trans_rot_cmd.translation_mode = Dradio_Con_str[28]&0x03;
		USV_Control.USV_Control_Message[count].trans_rot_cmd.translation_command = ((Dradio_Con_str[28]&0x0c)>>2);
		USV_Control.USV_Control_Message[count].trans_rot_cmd.rotation_mode = ((Dradio_Con_str[28]&0x30)>>4);
		USV_Control.USV_Control_Message[count].trans_rot_cmd.rotation_command = ((Dradio_Con_str[28]&0xc0)>>6);
	
	
		//����д��־
		if(USV_Control.USV_Control_Message[count].Engine_Run_L == 0x01)
		{
		//	SysLogMsgPost("�󷢶������,�����Դ%d ��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ",count);
				printf_msg(Dradio_Con_str);
		}
		if(USV_Control.USV_Control_Message[count].Engine_Run_L == 0x01)
		{
		//	SysLogMsgPost("�ҷ��������,�����Դ%d ��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ",count);
				printf_msg(Dradio_Con_str);
		}
		if(USV_Control.USV_Control_Message[count].Engine_Run_L == 0x02)
		{

		//	SysLogMsgPost("�󷢶���Ϩ��,Ϩ����Դ%d ��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ",count);
				printf_msg(Dradio_Con_str);
		}
		if(USV_Control.USV_Control_Message[count].Engine_Run_L == 0x02)
		{
		//	SysLogMsgPost("�ҷ�����Ϩ��,Ϩ����Դ%d ��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ",count);
				printf_msg(Dradio_Con_str);
		}

		if(USV_Control.USV_Control_Message[count].Emergency_Stop == 0x01)	//��ͣ��¼����
		{
		//	SysLogMsgPost("��ͣ %d ��Դ 0--���ֵ�̨ 1--�������ֵ�̨ 2--������̨ 3--����ͨѶ",count);
			printf_msg(Dradio_Con_str);
		}
	}
	return  ;
}
void Dradio_Con_Printf(uint8 count)
{
	printf("USV_Num=%d\n",USV_Control.USV_Control_Message[count].USV_Num);
	printf("Msg_Num=%d\n",USV_Control.USV_Control_Message[count].Msg_Num);
	printf("UAV_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.UAV_Power);
	printf("Stable_Platform_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Stable_Platform_Power);
	printf("Camera_ahead_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_ahead_Power);
	printf("D_radar_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.D_radar_Power);
	printf("Camera_main_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_main_Power);
	printf("Horn_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Horn_Power);
	printf("Searchlight_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Searchlight_Power);
	printf("Camera_lesser_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_lesser_Power);
	printf("Navigationlight_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Navigationlight_Power);
	printf("Camera_tail_Power=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Device_Power.Camera_tail_Power);
	printf("Navigation_Tsak_sign=%d\n",USV_Control.USV_Control_Message[count].Navigation_Tsak_sign);
	printf("Engine_Power=%d\n",USV_Control.USV_Control_Message[count].Engine_Run_L);
	printf("Engine_Run=%d\n",USV_Control.USV_Control_Message[count].Engine_Run_R);
	printf("Sailing_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Sailing_Mod);
	printf("Differential_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Differential_Mod);
	printf("Set_Return_Point_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Set_Return_Point_Mod);
	printf("Speed_Constant_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Speed_Constant_Mod);
	printf("Direction_Constant_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Direction_Constant_Mod);
	printf("Return_Mod=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Model.Return_Mod);
	printf("Gear_Left=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Gear_Left);
	printf("Gear_Right=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Gear_Right);
	printf("Accelerator_Left=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Accelerator_Left);
	printf("Accelerator_Right=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Accelerator_Right);
	printf("Rudder_Angle_Left=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Rudder_Angle_Left);
	printf("Rudder_Angle_Right=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Drive.Rudder_Angle_Right);
	printf("Speed_Limit=%d\n",USV_Control.USV_Control_Message[count].Speed_Limit);
	printf("Stable_Platform_AT=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Stable_Platform.Stable_Platform_AT);
	printf("Stable_Platform_RL=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_Stable_Platform.Stable_Platform_RL);
	printf("Platform_Hatch=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Hatch);
	printf("Platform_Lift=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Lift);
	printf("Platform_Open=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.Platform_Open);
	printf("UAV_Charging=%d\n",USV_Control.USV_Control_Message[count].Dradio_USV_UAV_Control.UAV_Charging);
	printf("Hull_Fan_Con=%d\n",USV_Control.USV_Control_Message[count].Hull_Fan_Con);
	printf("Hull_Pump_Con=%d\n",USV_Control.USV_Control_Message[count].Hull_Pump_Con);
	printf("Oil_Con=%d\n",USV_Control.USV_Control_Message[count].Oil_Con);
//	printf("Sealight_LR_Con=%d\n",USV_Control.USV_Control_Message[count].Sealight_LR_Con);
//	printf("Sealignt_HT_Con=%d\n",USV_Control.USV_Control_Message[count].Sealignt_HT_Con);
	printf("Application_24=%d\n",USV_Control.USV_Control_Message[count].Application_24);
	printf("Application_12=%d\n",USV_Control.USV_Control_Message[count].Application_12);
	printf("Sealight_Speed=%d\n",USV_Control.USV_Control_Message[count].Sealight_Speed);
	printf("Sealight_Direction=%d\n",USV_Control.USV_Control_Message[count].Sealight_Direction);
	printf("Emergency_Stop=%d\n",USV_Control.USV_Control_Message[count].Emergency_Stop);
	printf("Outboard_Engine_L=%d\n",USV_Control.USV_Control_Message[count].Outboard_Engine_L);
	printf("Outboard_Engine_R=%d\n",USV_Control.USV_Control_Message[count].Outboard_Engine_R);	
	
}

//����������Ϣ����
void Dradio_Sal_Analytical(uint8  *Dradio_Sal_Str)
{
	uint8 sailing_count,end_count,count_i,ico;
//	floatchange	utemp;
	
	sailing_count=0;
	end_count=0;
//	utemp.ftemp=0.0;

	if(Dradio_Sal_Str[5]==USV_NUM)//�Ǳ���ָ��
	{
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].USV_Num=Dradio_Sal_Str[5];
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber=Dradio_Sal_Str[8];
		sailing_count=Dradio_Sal_Str[8];
		for(count_i=0;count_i<sailing_count;count_i++)
		{
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Latitude_Sign=(Dradio_Sal_Str[count_i*13+9]&0x01);
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Longitude_Sign=(Dradio_Sal_Str[count_i*13+9]&0x02)>>1;

			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Latitude_Degree=Dradio_Sal_Str[count_i*13+10];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Latitude_Minute=Dradio_Sal_Str[count_i*13+11];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Latitude_Second=Dradio_Sal_Str[count_i*13+12];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Latitude_Decimal=Dradio_Sal_Str[count_i*13+13];		
			
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Longitude_Degree=Dradio_Sal_Str[count_i*13+14];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Longitude_Minute=Dradio_Sal_Str[count_i*13+15];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Longitude_Second=Dradio_Sal_Str[count_i*13+16];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Longitude_Decimal=Dradio_Sal_Str[count_i*13+17];		
			
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_speed=Dradio_Sal_Str[count_i*13+18]*256+Dradio_Sal_Str[count_i*13+19];
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[count_i].Waypoint_Stop_Time=Dradio_Sal_Str[count_i*13+20]*256+Dradio_Sal_Str[count_i*13+21];
		}
		end_count=9+(Dradio_Sal_Str[8])*13;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber=Dradio_Sal_Str[end_count];
		for(ico=0;ico<(Dradio_Sal_Str[8]);ico++)
		{
			printf("%d-Waypoint_Latitude_Sign=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Latitude_Sign);
			printf("%d-Waypoint_Longitude_Sign=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Longitude_Sign);
			printf("%d-Waypoint_Latitude_Degree=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Latitude_Degree);
			printf("%d-Waypoint_Latitude_Minute=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Latitude_Minute);
			printf("%d-Waypoint_Latitude_Second=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Latitude_Second);
			printf("%d-Waypoint_Latitude_Decimal=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Latitude_Decimal);
			printf("%d-Waypoint_Longitude_Degree=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Longitude_Degree);
			printf("%d-Waypoint_Longitude_Minute=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Longitude_Minute);
			printf("%d-Waypoint_Longitude_Second=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Longitude_Second);
			printf("%d-Waypoint_Longitude_Decimal=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Longitude_Decimal);
			printf("%d-Waypoint_speed=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_speed);
			printf("%d-Waypoint_Stop_Time=%d\n",ico,USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[ico].Waypoint_Stop_Time);		
		}
		printf("Sailing_Nummber=%d\n",USV_Sailing.USV_Sailing_Message[Sailing_Sign].Sailing_Nummber);
		

		//ת������

	}
	return  ;
}

//������̨�·������ý���
//Dradio_Cfg_Str --���ջ�����
//USV_Config_Str --���û�����
void Dradio_Cfg_Analytical(Dradio_Config_Message*USV_Config_Str,uint8  *Dradio_Cfg_Str)
{
	if(Dradio_Cfg_Str[5]==USV_NUM)//�Ǳ���ָ��
	{
		USV_Config_Str->Config_Direction=Dradio_Cfg_Str[4];					// ���䷽�� T R
		if(USV_Config_Str->Config_Direction=='R')							//�ٻ�����
		{
			USV_State.Config_Nummber=((Dradio_Cfg_Str[11]&0x1C)>>2);		//USV���ñ��ı��
			Send_Config();												//��������
		}
		if(USV_Config_Str->Config_Direction=='T')						//�·�����
		{
			//��������
			USV_Config_Str->Basic_Cfg.Hull_Type=Dradio_Cfg_Str[7];
			USV_Config_Str->Basic_Cfg.Drive_Type=Dradio_Cfg_Str[8];
			USV_Config_Str->Basic_Cfg.Propeller_Type=Dradio_Cfg_Str[9];
			USV_Config_Str->Basic_Cfg.Motor_Type=Dradio_Cfg_Str[10];
			USV_Config_Str->Basic_Cfg.Drive_Num=(Dradio_Cfg_Str[11]&0x03);
			USV_Config_Str->Config_Num=((Dradio_Cfg_Str[11]&0x1C)>>2);
			USV_State.Config_Nummber=((Dradio_Cfg_Str[11]&0x1C)>>2);
			
			//��������
			USV_Config_Str->Board_Cfg.MMSI=((Dradio_Cfg_Str[12])<<24);
			USV_Config_Str->Board_Cfg.MMSI+=((Dradio_Cfg_Str[13])<<16);
			USV_Config_Str->Board_Cfg.MMSI+=((Dradio_Cfg_Str[14])<<8);
			USV_Config_Str->Board_Cfg.MMSI+=Dradio_Cfg_Str[15];
			USV_Config_Str->Board_Cfg.Board_length=((Dradio_Cfg_Str[16])<<8);
			USV_Config_Str->Board_Cfg.Board_length+=Dradio_Cfg_Str[17];
			USV_Config_Str->Board_Cfg.Board_Wide=((Dradio_Cfg_Str[18])<<8);
			USV_Config_Str->Board_Cfg.Board_Wide+=Dradio_Cfg_Str[19];
			USV_Config_Str->Board_Cfg.Board_Deep=((Dradio_Cfg_Str[20])<<8);
			USV_Config_Str->Board_Cfg.Board_Deep+=Dradio_Cfg_Str[21];
			USV_Config_Str->Board_Cfg.Max_Static_Draft=((Dradio_Cfg_Str[22])<<8);
			USV_Config_Str->Board_Cfg.Max_Static_Draft+=Dradio_Cfg_Str[23];
			USV_Config_Str->Board_Cfg.Max_Speed=Dradio_Cfg_Str[24];
			USV_Config_Str->Board_Cfg.Max_Voyage=((Dradio_Cfg_Str[25])<<8);
			USV_Config_Str->Board_Cfg.Max_Voyage+=Dradio_Cfg_Str[26];
			//��������
			USV_Config_Str->Throttle_Cfg.Motor_Speed_Max_L=Dradio_Cfg_Str[27];				//�ɿ�����̨�·�
			USV_Config_Str->Throttle_Cfg.Motor_Speed_Idle_L=Dradio_Cfg_Str[28];
			USV_Config_Str->Throttle_Cfg.Motor_Speed_Max_R=Dradio_Cfg_Str[29];
			USV_Config_Str->Throttle_Cfg.Motor_Speed_Idle_R=Dradio_Cfg_Str[30];
			USV_Config_Str->Throttle_Cfg.Motor_Trip_L=Dradio_Cfg_Str[31];
			USV_Config_Str->Throttle_Cfg.Motor_Trip_R=Dradio_Cfg_Str[32];
			USV_Config_Str->Throttle_Cfg.Motor_Address_L=Dradio_Cfg_Str[33];
			USV_Config_Str->Throttle_Cfg.Motor_Address_R=Dradio_Cfg_Str[34];
			USV_Config_Str->Throttle_Cfg.Motor_Direction_L=(Dradio_Cfg_Str[35]&0x01);
			USV_Config_Str->Throttle_Cfg.Motor_Direction_R=((Dradio_Cfg_Str[35]&0x02)>>1);
			USV_Config_Str->Throttle_Cfg.Motor_Sign=((Dradio_Cfg_Str[35]&0x04)>>2);
			USV_Config_Str->Throttle_Cfg.Motor_Fire_Time=((Dradio_Cfg_Str[35]&0x38)>>3);
			
			//��λ����
			USV_Config_Str->Gear_Cfg.Gear_Forward_Travel_L=Dradio_Cfg_Str[36];
			USV_Config_Str->Gear_Cfg.Gear_Forward_Travel_R=Dradio_Cfg_Str[37];
			USV_Config_Str->Gear_Cfg.Gear_Back_Travel_L=Dradio_Cfg_Str[38];
			USV_Config_Str->Gear_Cfg.Gear_Back_Travel_R=Dradio_Cfg_Str[39];
			USV_Config_Str->Gear_Cfg.Gear_Forward_Angle_L=Dradio_Cfg_Str[40];
			USV_Config_Str->Gear_Cfg.Gear_Forward_Angle_R=Dradio_Cfg_Str[41];
			USV_Config_Str->Gear_Cfg.Gear_Back_Angle_L=Dradio_Cfg_Str[42];
			USV_Config_Str->Gear_Cfg.Gear_Back_Angle_R=Dradio_Cfg_Str[43];
			USV_Config_Str->Gear_Cfg.Gear_Neutral_Angle_L=Dradio_Cfg_Str[44];
			USV_Config_Str->Gear_Cfg.Gear_Neutral_Angle_R=Dradio_Cfg_Str[45];
			USV_Config_Str->Gear_Cfg.Gear_Direction_L=(Dradio_Cfg_Str[46]&0x01);
			USV_Config_Str->Gear_Cfg.Gear_Direction_R=((Dradio_Cfg_Str[46]&0x02)>>1);
			USV_Config_Str->Gear_Cfg.Gear_Sign=((Dradio_Cfg_Str[46]&0x04)>>2);
			//��ǲ���
			USV_Config_Str->Rudder_Cfg.Rudder_Angle_Max_L=Dradio_Cfg_Str[47];
			USV_Config_Str->Rudder_Cfg.Rudder_Angle_Max_R=Dradio_Cfg_Str[48];
			USV_Config_Str->Rudder_Cfg.Rudder_Left_Limit_Resistance_L=Dradio_Cfg_Str[49];
			USV_Config_Str->Rudder_Cfg.Rudder_Left_Limit_Resistance_R=Dradio_Cfg_Str[50];
			USV_Config_Str->Rudder_Cfg.Rudder_Middle_Limit_Resistance_L=Dradio_Cfg_Str[51];
			USV_Config_Str->Rudder_Cfg.Rudder_Middle_Limit_Resistance_R=Dradio_Cfg_Str[52];
			USV_Config_Str->Rudder_Cfg.Rudder_Right_Limit_Resistance_L=Dradio_Cfg_Str[53];
			USV_Config_Str->Rudder_Cfg.Rudder_Right_Limit_Resistance_R=Dradio_Cfg_Str[54];
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Angle_Max_L=Dradio_Cfg_Str[55];
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Angle_Max_R=Dradio_Cfg_Str[56];
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Angle_Min_L=Dradio_Cfg_Str[57];
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Angle_Min_R=Dradio_Cfg_Str[58];
			USV_Config_Str->Rudder_Cfg.Rudder_Direction_L=(Dradio_Cfg_Str[59]&0x01);
			USV_Config_Str->Rudder_Cfg.Rudder_Direction_R=((Dradio_Cfg_Str[59]&0x02)>>1);
			USV_Config_Str->Rudder_Cfg.Rudder_Sign=((Dradio_Cfg_Str[59]&0x04)>>2);
			//�������
			USV_Config_Str->Algorithm_Cfg[0].Control_Type=(Dradio_Cfg_Str[60]&0x0f);
			USV_Config_Str->Algorithm_Cfg[0].PID_P=Dradio_Cfg_Str[61]<<16;
			USV_Config_Str->Algorithm_Cfg[0].PID_P+=Dradio_Cfg_Str[62]<<8;
			USV_Config_Str->Algorithm_Cfg[0].PID_P+=Dradio_Cfg_Str[63];
			USV_Config_Str->Algorithm_Cfg[0].PID_I=Dradio_Cfg_Str[64]<<16;
			USV_Config_Str->Algorithm_Cfg[0].PID_I+=Dradio_Cfg_Str[65]<<8;
			USV_Config_Str->Algorithm_Cfg[0].PID_I+=Dradio_Cfg_Str[66];
			USV_Config_Str->Algorithm_Cfg[0].PID_D=Dradio_Cfg_Str[67]<<16;
			USV_Config_Str->Algorithm_Cfg[0].PID_D+=Dradio_Cfg_Str[68]<<8;
			USV_Config_Str->Algorithm_Cfg[0].PID_D+=Dradio_Cfg_Str[69];
			USV_Config_Str->Algorithm_Cfg[0].S_K1=Dradio_Cfg_Str[70];
			USV_Config_Str->Algorithm_Cfg[0].S_K2=Dradio_Cfg_Str[71];
			USV_Config_Str->Algorithm_Cfg[0].S_K3=Dradio_Cfg_Str[72];
			USV_Config_Str->Algorithm_Cfg[0].Filter_A=Dradio_Cfg_Str[73];
			USV_Config_Str->Algorithm_Cfg[0].Filter_B=Dradio_Cfg_Str[74];
			USV_Config_Str->Algorithm_Cfg[0].Filter_C=Dradio_Cfg_Str[75];
			USV_Config_Str->Algorithm_Cfg[0].Algorithm_Sign=((Dradio_Cfg_Str[60]&0x10)>>4);
			//���ٲ���
			USV_Config_Str->Algorithm_Cfg[1].Control_Type=(Dradio_Cfg_Str[76]&0x0f);
			USV_Config_Str->Algorithm_Cfg[1].PID_P=Dradio_Cfg_Str[77]<<16;
			USV_Config_Str->Algorithm_Cfg[1].PID_P+=Dradio_Cfg_Str[78]<<8;
			USV_Config_Str->Algorithm_Cfg[1].PID_P+=Dradio_Cfg_Str[79];
			USV_Config_Str->Algorithm_Cfg[1].PID_I=Dradio_Cfg_Str[80]<<16;
			USV_Config_Str->Algorithm_Cfg[1].PID_I+=Dradio_Cfg_Str[81]<<8;
			USV_Config_Str->Algorithm_Cfg[1].PID_I+=Dradio_Cfg_Str[82];
			USV_Config_Str->Algorithm_Cfg[1].PID_D=Dradio_Cfg_Str[83]<<16;
			USV_Config_Str->Algorithm_Cfg[1].PID_D+=Dradio_Cfg_Str[84]<<8;
			USV_Config_Str->Algorithm_Cfg[1].PID_D+=Dradio_Cfg_Str[85];
			USV_Config_Str->Algorithm_Cfg[1].S_K1=Dradio_Cfg_Str[86];
			USV_Config_Str->Algorithm_Cfg[1].S_K2=Dradio_Cfg_Str[87];
			USV_Config_Str->Algorithm_Cfg[1].S_K3=Dradio_Cfg_Str[88];
			USV_Config_Str->Algorithm_Cfg[1].Filter_A=Dradio_Cfg_Str[89];
			USV_Config_Str->Algorithm_Cfg[1].Filter_B=Dradio_Cfg_Str[90];
			USV_Config_Str->Algorithm_Cfg[1].Filter_C=Dradio_Cfg_Str[91];
			USV_Config_Str->Algorithm_Cfg[1].Algorithm_Sign=((Dradio_Cfg_Str[76]&0x10)>>4);
			//��������
			USV_Config_Str->Environment_Cfg.Environment_Sign=Dradio_Cfg_Str[92]&0x01;
			USV_Config_Str->Environment_Cfg.Wind_Speed=Dradio_Cfg_Str[93];
			USV_Config_Str->Environment_Cfg.Wave_Hight=Dradio_Cfg_Str[94];
			USV_Config_Str->Environment_Cfg.Flow_Speed=Dradio_Cfg_Str[95];
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Accuracy_L=Dradio_Cfg_Str[96]&0x0f;
			USV_Config_Str->Rudder_Cfg.Rudder_Control_Accuracy_R=(Dradio_Cfg_Str[96]&0xf0)>>4;
			USV_Config_Str->Rudder_Cfg.Rudder_Left_Travel_L=Dradio_Cfg_Str[97];
			USV_Config_Str->Rudder_Cfg.Rudder_Left_Travel_R=Dradio_Cfg_Str[98];
			USV_Config_Str->Rudder_Cfg.Rudder_Right_Travel_L=Dradio_Cfg_Str[99];
			USV_Config_Str->Rudder_Cfg.Rudder_Right_Travel_R=Dradio_Cfg_Str[100];
			//�ߵ�����
			USV_Config_Str->Board_Cfg.INS_Antenna_Distance=(Dradio_Cfg_Str[101]<<8);
			USV_Config_Str->Board_Cfg.INS_Antenna_Distance+=Dradio_Cfg_Str[102];
			
		}
	}
	return  ;
}


void Send_Config()
{
	uint8 	USV_Config[Dradio_Cfg_Msg],CRC=0,count;
	memset(USV_Config,0,Dradio_Cfg_Msg);
	USV_Config[0]='%';
	USV_Config[1]='U';
	USV_Config[2]='S';
	USV_Config[3]='V';
	USV_Config[4]='R';
	USV_Config[5]=USV_NUM;
	USV_Config[6]=0xff;
	USV_Config[7]=Dradio_Config.Basic_Cfg.Hull_Type;
	USV_Config[8]=Dradio_Config.Basic_Cfg.Drive_Type;
	USV_Config[9]=Dradio_Config.Basic_Cfg.Propeller_Type;
	USV_Config[10]=Dradio_Config.Basic_Cfg.Motor_Type;
	USV_Config[11]=SR_Config_Msg.Drive_Nummber_spn520252;
	USV_Config[12]=(Dradio_Config.Board_Cfg.MMSI&0xff000000)>>24;
	USV_Config[13]=(Dradio_Config.Board_Cfg.MMSI&0x00ff0000)>>16;
	USV_Config[14]=(Dradio_Config.Board_Cfg.MMSI&0x0000ff00)>>8;
	USV_Config[15]=Dradio_Config.Board_Cfg.MMSI&0x000000ff;
	USV_Config[16]=(Dradio_Config.Board_Cfg.Board_length&0xff00)>>8;
	USV_Config[17]=Dradio_Config.Board_Cfg.Board_length&0x00ff;
	USV_Config[18]=(Dradio_Config.Board_Cfg.Board_Wide&0xff00)>>8;
	USV_Config[19]=Dradio_Config.Board_Cfg.Board_Wide&0x00ff;
	USV_Config[20]=(Dradio_Config.Board_Cfg.Board_Deep&0xff00)>>8;
	USV_Config[21]=Dradio_Config.Board_Cfg.Board_Deep&0x00ff;
	USV_Config[22]=(Dradio_Config.Board_Cfg.Max_Static_Draft&0xff00)>>8;
	USV_Config[23]=Dradio_Config.Board_Cfg.Max_Static_Draft&0x00ff;
	USV_Config[24]=Dradio_Config.Board_Cfg.Max_Speed;
	USV_Config[25]=(Dradio_Config.Board_Cfg.Max_Voyage&0xff00)>>8;
	USV_Config[26]=Dradio_Config.Board_Cfg.Max_Voyage&0x00ff;
	USV_Config[27]=SR_Config_Msg.Motor_MAX_Speed_L_spn520220;
	USV_Config[28]=SR_Config_Msg.Motor_Idling_Speed_L_spn520221;
	USV_Config[29]=SR_Config_Msg.Motor_MAX_Speed_R_spn520222;
	USV_Config[30]=SR_Config_Msg.Motor_Idling_Speed_R_spn520223;
	USV_Config[31]=SR_Config_Msg.Motor_Travel_L_spn520224;
	USV_Config[32]=SR_Config_Msg.Motor_Travel_R_spn520225;
	USV_Config[33]=Dradio_Config.Throttle_Cfg.Motor_Address_L;
	USV_Config[34]=Dradio_Config.Throttle_Cfg.Motor_Address_R;
	USV_Config[35]=SR_Config_Msg.Motor_Direction_L_spn520227;
	USV_Config[35]+=(SR_Config_Msg.Motor_Direction_R_spn520228)<<1;
	USV_Config[35]+=(SR_Config_Msg.Motor_Fire_Time_spn520226)<<3;
	USV_Config[36]=SR_Config_Msg.Gear_Forward_Travel_L_spn520240;
	USV_Config[37]=SR_Config_Msg.Gear_Forward_Travel_R_spn520241;
	USV_Config[38]=SR_Config_Msg.Gear_Back_Travel_L_spn520242;
	USV_Config[39]=SR_Config_Msg.Gear_Back_Travel_R_spn520243;
	USV_Config[40]=SR_Config_Msg.Gear_Forward_Angle_L_spn520244;
	USV_Config[41]=SR_Config_Msg.Gear_Forward_Angle_R_spn520245;
	USV_Config[42]=SR_Config_Msg.Gear_Back_Angle_L_spn520246;
	USV_Config[43]=SR_Config_Msg.Gear_Back_Angle_R_spn520247;
	USV_Config[44]=SR_Config_Msg.Gear_Neutral_Angle_L_spn520248;
	USV_Config[45]=SR_Config_Msg.Gear_Neutral_Angle_R_spn520249;
	USV_Config[46]=SR_Config_Msg.Gear_Direction_L_spn520250;
	USV_Config[46]+=(SR_Config_Msg.Gear_Direction_R_spn520251)<<1;
	USV_Config[47]=SR_Config_Msg.Rudder_Angle_Max_L_spn520260;
	USV_Config[48]=SR_Config_Msg.Rudder_Angle_Max_R_spn520261;
	USV_Config[49]=SR_Config_Msg.Rudder_Left_Limit_Resistance_L_spn520262;
	USV_Config[50]=SR_Config_Msg.Rudder_Left_Limit_Resistance_R_spn520263;
	USV_Config[51]=SR_Config_Msg.Rudder_Middle_Limit_Resistance_L_spn520264;
	USV_Config[52]=SR_Config_Msg.Rudder_Middle_Limit_Resistance_R_spn520265;
	USV_Config[53]=SR_Config_Msg.Rudder_Right_Limit_Resistance_L_spn520266;
	USV_Config[54]=SR_Config_Msg.Rudder_Right_Limit_Resistance_R_spn520267;
	USV_Config[55]=SR_Config_Msg.Rudder_Control_Angle_Max_L_spn520268;
	USV_Config[56]=SR_Config_Msg.Rudder_Control_Angle_Max_R_spn520269;
	USV_Config[57]=SR_Config_Msg.Rudder_Control_Angle_Min_L_spn520270;
	USV_Config[58]=SR_Config_Msg.Rudder_Control_Angle_Min_R_spn520271;
	USV_Config[59]=SR_Config_Msg.Rudder_Direction_L_spn520278;
	USV_Config[59]+=(SR_Config_Msg.Rudder_Direction_R_spn520279)<<1;
	USV_Config[60]=Dradio_Config.Algorithm_Cfg[0].Control_Type;
	USV_Config[61]=(Dradio_Config.Algorithm_Cfg[0].PID_P&0xff0000)>>16;
	USV_Config[62]=(Dradio_Config.Algorithm_Cfg[0].PID_P&0x00ff00)>>8;
	USV_Config[63]=Dradio_Config.Algorithm_Cfg[0].PID_P&0x0000ff;
	USV_Config[64]=(Dradio_Config.Algorithm_Cfg[0].PID_I&0xff0000)>>16;
	USV_Config[65]=(Dradio_Config.Algorithm_Cfg[0].PID_I&0x00ff00)>>8;
	USV_Config[66]=Dradio_Config.Algorithm_Cfg[0].PID_I&0x0000ff;
	USV_Config[67]=(Dradio_Config.Algorithm_Cfg[0].PID_D&0xff0000)>>16;
	USV_Config[68]=(Dradio_Config.Algorithm_Cfg[0].PID_D&0x00ff00)>>8;
	USV_Config[69]=Dradio_Config.Algorithm_Cfg[0].PID_D&0x0000ff;
	USV_Config[70]=Dradio_Config.Algorithm_Cfg[0].S_K1;
	USV_Config[71]=Dradio_Config.Algorithm_Cfg[0].S_K2;
	USV_Config[72]=Dradio_Config.Algorithm_Cfg[0].S_K3;
	USV_Config[73]=Dradio_Config.Algorithm_Cfg[0].Filter_A;
	USV_Config[74]=Dradio_Config.Algorithm_Cfg[0].Filter_B;
	USV_Config[75]=Dradio_Config.Algorithm_Cfg[0].Filter_C;
	USV_Config[76]=Dradio_Config.Algorithm_Cfg[1].Control_Type;
	USV_Config[77]=(Dradio_Config.Algorithm_Cfg[1].PID_P&0xff0000)>>16;
	USV_Config[78]=(Dradio_Config.Algorithm_Cfg[1].PID_P&0x00ff00)>>8;
	USV_Config[79]=Dradio_Config.Algorithm_Cfg[1].PID_P&0x0000ff;
	USV_Config[80]=(Dradio_Config.Algorithm_Cfg[1].PID_I&0xff0000)>>16;
	USV_Config[81]=(Dradio_Config.Algorithm_Cfg[1].PID_I&0x00ff00)>>8;
	USV_Config[82]=Dradio_Config.Algorithm_Cfg[1].PID_I&0x0000ff;
	USV_Config[83]=(Dradio_Config.Algorithm_Cfg[1].PID_D&0xff0000)>>16;
	USV_Config[84]=(Dradio_Config.Algorithm_Cfg[1].PID_D&0x00ff00)>>8;
	USV_Config[85]=Dradio_Config.Algorithm_Cfg[1].PID_D&0x0000ff;
	USV_Config[86]=Dradio_Config.Algorithm_Cfg[1].S_K1;
	USV_Config[87]=Dradio_Config.Algorithm_Cfg[1].S_K2;
	USV_Config[88]=Dradio_Config.Algorithm_Cfg[1].S_K3;
	USV_Config[89]=Dradio_Config.Algorithm_Cfg[1].Filter_A;
	USV_Config[90]=Dradio_Config.Algorithm_Cfg[1].Filter_B;
	USV_Config[91]=Dradio_Config.Algorithm_Cfg[1].Filter_C;
	USV_Config[93]=Dradio_Config.Environment_Cfg.Wind_Speed;
	USV_Config[94]=Dradio_Config.Environment_Cfg.Wave_Hight;
	USV_Config[95]=Dradio_Config.Environment_Cfg.Flow_Speed;
	USV_Config[96]=SR_Config_Msg.Rudder_Control_Accuracy_L_spn520276;
	USV_Config[96]+=(SR_Config_Msg.Rudder_Control_Accuracy_R_spn520277)<<4;
	USV_Config[97]=SR_Config_Msg.Rudder_Left_Travel_L_spn520272;
	USV_Config[98]=SR_Config_Msg.Rudder_Left_Travel_R_spn520273;
	USV_Config[99]=SR_Config_Msg.Rudder_Right_Travel_L_spn520274;
	USV_Config[100]=SR_Config_Msg.Rudder_Right_Travel_R_spn520275;
	USV_Config[101]=((Dradio_Config.Board_Cfg.INS_Antenna_Distance&0xff00)>>8);
	USV_Config[102]=(Dradio_Config.Board_Cfg.INS_Antenna_Distance&0x00ff);
	
	USV_Config[253]='*';
	GetCheck(&CRC,USV_Config,Dradio_Cfg_Msg);
	USV_Config[254]=CRC;
	printf("\n ");	
	for(count=0;count<255;count++)
		printf("%d-0x%.2x ",count,USV_Config[count]);
	printf("\n ");
	//write_uart(UART4_Fd, (int8 *)&USV_Config[0], Dradio_Cfg_Msg);//commented 2019-05-24 @foo
	USV_State.Config_Nummber=0x06;//���óɹ�
}

/*************************************************
��������:ASCII_6bitASCII
��������:��ASCII��ת��Ϊ6 BIT ��ASCII��
��������:����ֵ��00+6BIT ���ɵ�һ���ֽ�
***************************************************/
uint8 ASCII_6bitASCII(BYTE ASCII_code)
{ 
	uint8 value=0; 
	if(ASCII_code<0x30)
		return _FALSE;       //���ݴ��󣬻ָ�
	else if(ASCII_code>0x77)
		return _FALSE;
	else if(ASCII_code>0x57)
	{ 
		if(ASCII_code<0x60)
			return _FALSE;
		else
			value=ASCII_code+0x28;
	}
	else
		value=ASCII_code+0x28;
	if(value>0x80)
	{
		value=value+0x20;   //�����Ǵ���0x60��ASCII��
	}
	else
	{
		value=value+0x28; //С��0X57��ASCII��
	}

	value=(value&0x3f); //ȡ������λ��Ϊ6BIT_ASCII    
	return value;
}
/*************************************************
��������:AIS_Decode
��������:��AIS���н���
��������:
***************************************************/
void AIS_Decode(char* cAIS_Data)
{
	int i_sign=0,len=0,i;
	uint8 isASCII_code_5[AIS_Buff_Num];

	memset(isASCII_code_5,0,sizeof(isASCII_code));
	len=strlen(cAIS_Data);
	for(i_sign=0;i_sign<len;i_sign++)
	{
		isASCII_code[i_sign]=ASCII_6bitASCII(cAIS_Data[i_sign]);
	}
	if (isASCII_code[0]==0)//����
	{
		if (isASCII_code[7]==1)//��Ϣֻ��һ��
		{
			if ((isASCII_code[14]==1)||(isASCII_code[14]==2)||(isASCII_code[14]==3))//��Ϣ����1/2/3
			{
				Position_Analytical_A(&isASCII_code[14]);//����,ע���ʲô�ط���ʼ�����ݲ���
			}
			if ((isASCII_code[14]==4)||(isASCII_code[14]==11))//��Ϣ����4/11
			{
				UTC_Analytical(&isASCII_code[14]);//����,ע���ʲô�ط���ʼ�����ݲ���
			}
			if (isASCII_code[14]==18)//��Ϣ����18
			{
				Position_Analytical_B(&isASCII_code[14]);//����,ע���ʲô�ط���ʼ�����ݲ���
			}
			if (isASCII_code[14]==24)//��Ϣ����24
			{
				Static_Report_Analytical(&isASCII_code[14]);//����,ע���ʲô�ط���ʼ�����ݲ���
			}
		}
		else//��Ϣ�ж������
		{
			for( i=1;i<len-19;i++)//��ҪͷβֻҪ����
			{
				buffer_Data[isASCII_code[9]-1][0]=len-20;//���ݳ���
				buffer_Data[isASCII_code[9]-1][i]=cAIS_Data[i+14];//��һ�����ڵ�0��������			
			}
			if (cAIS_Data[7]==cAIS_Data[9])//��Ϣ�ϲ���ϣ�ǰ����Ϣ�Ĵ洢
			{
				//�ϲ�,����
				//isASCII_code[11]�������ĵĵھŸ���Ҫ��ͬ
				if(buffer_Data[0][1]==53)//��Ϣ����5
				{
					for ( i=0;i<buffer_Data[0][0];i++)
					{
						isASCII_code_5[i]=buffer_Data[0][i+1];
					}
					for ( i=0;i<buffer_Data[1][0];i++)
					{
						isASCII_code_5[i+buffer_Data[0][0]]=buffer_Data[1][i+1];
					}

					Static_Analytical(&isASCII_code_5[0]);//����
					memset(buffer_Data,0,sizeof(buffer_Data));
				}							
				else;//������Ϣ
			}
		}
	}
/*	if (cAIS_Data[0]=="$")//����GPS��������82�ֽ�
	{

	}*/
	return  ;
}

/*************************************************
��������:Position_Analytical_A
��������:A�ബλ����Ϣ��������Ϣ1/2/3
��������:
***************************************************/
void Position_Analytical_A(char* isASCII_code)
{
	uint8 j,count;
	Ship_A_position	Ship_A_position_Msg;
	
	count=0;
	memset((char *)&Ship_A_position_Msg.Msg_ID,0,sizeof(Ship_A_position_Msg));
	Ship_A_position_Msg.Msg_ID=isASCII_code[0]&0x3f;//��Ϣ1/2/3�ı�ʶ��

	Ship_A_position_Msg.Rep_ID=(isASCII_code[1]&0x30)>>4;//��ʾһ����Ϣ��ת���Ĵ���0~3��Ĭ��Ϊ0,3��ʾ��ת��

	Ship_A_position_Msg.User_ID+=((isASCII_code[1]&0x0f)<<26);
	Ship_A_position_Msg.User_ID+=((isASCII_code[2]&0x3f)<<20);
	Ship_A_position_Msg.User_ID+=((isASCII_code[3]&0x3f)<<14);
	Ship_A_position_Msg.User_ID+=((isASCII_code[4]&0x3f)<<8);
	Ship_A_position_Msg.User_ID+=((isASCII_code[5]&0x3f)<<2);
	Ship_A_position_Msg.User_ID+=((isASCII_code[6]&0x30)>>4);
	Ship_A_position_Msg.Navigation_status=isASCII_code[6]&0x0f;//0��ʾ�ں�
	Ship_A_position_Msg.ROT+=(isASCII_code[7]&0xff);
	Ship_A_position_Msg.ROT+=((isASCII_code[8]&0x30)>>4);
	Ship_A_position_Msg.SOG+=((isASCII_code[8]&0x0f)<<6);
	Ship_A_position_Msg.SOG+=(isASCII_code[9]&0x3f);
	Ship_A_position_Msg.Position_accuracy=(isASCII_code[10]&0x20)>>5;
	Ship_A_position_Msg.Longitude+=((isASCII_code[10]&0x1f)<<23);
	Ship_A_position_Msg.Longitude+=((isASCII_code[11]&0x3f)<<17);
	Ship_A_position_Msg.Longitude+=((isASCII_code[12]&0x3f)<<11);
	Ship_A_position_Msg.Longitude+=((isASCII_code[13]&0x3f)<<5);
	Ship_A_position_Msg.Longitude+=((isASCII_code[14]&0x3e)>>1);
	Ship_A_position_Msg.Latitude+=((isASCII_code[14]&0x01)<<26);
	Ship_A_position_Msg.Latitude+=((isASCII_code[15]&0x3f)<<20);
	Ship_A_position_Msg.Latitude+=((isASCII_code[16]&0x3f)<<14);
	Ship_A_position_Msg.Latitude+=((isASCII_code[17]&0x3f)<<8);
	Ship_A_position_Msg.Latitude+=((isASCII_code[18]&0x3f)<<2);
	Ship_A_position_Msg.Latitude+=((isASCII_code[19]&0x30)>>4);
	Ship_A_position_Msg.COG+=((isASCII_code[19]&0x0f)<<8);
	Ship_A_position_Msg.COG+=((isASCII_code[20]&0x3f)<<2);
	Ship_A_position_Msg.COG+=((isASCII_code[21]&0x30)>>4);
	Ship_A_position_Msg.True_Head+=((isASCII_code[21]&0x0f)<<5);
	Ship_A_position_Msg.True_Head+=((isASCII_code[22]&0x3e)>>1);
	Ship_A_position_Msg.Timer_stamp+=((isASCII_code[22]&0x01)<<5);
	Ship_A_position_Msg.Timer_stamp+=((isASCII_code[23]&0x3e)>>1);
	Ship_A_position_Msg.Regional_APP+=((isASCII_code[23]&0x01)<<3);
	Ship_A_position_Msg.Regional_APP+=((isASCII_code[24]&0x38)>>3);
	Ship_A_position_Msg.Spare=(isASCII_code[24]&0x04)>>2;
	Ship_A_position_Msg.RAIM_Flag=(isASCII_code[24]&0x02)>>1;
	Ship_A_position_Msg.UTC_direct+=((isASCII_code[24]&0x01)<<1);
	Ship_A_position_Msg.UTC_direct+=((isASCII_code[25]&0x10)>>1);
	Ship_A_position_Msg.Time_lost=(isASCII_code[25]&0x1c)>>2;
	Ship_A_position_Msg.Sub_information+=((isASCII_code[25]&0x03)<<12);
	Ship_A_position_Msg.Sub_information+=((isASCII_code[26]&0x3f)<<6);
	Ship_A_position_Msg.Sub_information+=(isASCII_code[27]&0x3f);
#ifdef  debug_print 
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_A_position_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_A_position_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_A_position_Msg.User_ID);
	printf("����״̬��%d\n",Ship_A_position_Msg.Navigation_status);
	printf("ת���ʣ�%d\n",Ship_A_position_Msg.ROT);
	printf("�Եغ��٣�%d\n",Ship_A_position_Msg.SOG);
	printf("��λ��ȷ�ȣ�%d\n",Ship_A_position_Msg.Position_accuracy);
	printf("���ȣ�%d\n",Ship_A_position_Msg.Longitude);
	printf("γ�ȣ�%d\n",Ship_A_position_Msg.Latitude);
	printf("�Եغ���%d\n",Ship_A_position_Msg.COG);
	printf("�溽��%d\n",Ship_A_position_Msg.True_Head);
	printf("ʱ���ǣ�%d\n",Ship_A_position_Msg.Timer_stamp);
	printf("������Ӧ�ñ�����%d\n",Ship_A_position_Msg.Regional_APP);
	printf("���ã�%d\n",Ship_A_position_Msg.Spare);
	printf("RAIM��־��%d\n",Ship_A_position_Msg.RAIM_Flag);
	printf("ͬ��״̬��%d\n",Ship_A_position_Msg.UTC_direct);
	printf("ʱ϶��ʱ��%d\n",Ship_A_position_Msg.Time_lost);
	printf("����Ϣ��%d\n",Ship_A_position_Msg.Sub_information);
#endif		
	//������ײ��Ϣ
	for(j=0;j<AIS_Collision_Num;j++)
	{
		if((USV_Check_Collision[j].User_ID==0)||(USV_Check_Collision[j].User_ID==Ship_A_position_Msg.User_ID))
		{
			USV_Check_Collision[j].User_ID=Ship_A_position_Msg.User_ID;
			if(Ship_A_position_Msg.SOG<1023)
				USV_Check_Collision[j].SOG=(float)(Ship_A_position_Msg.SOG*0.1);
#ifdef  debug_print 
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_A_position_Msg.User_ID);
#endif
			if(Ship_A_position_Msg.COG<3600)
				USV_Check_Collision[j].COG=(float)(Ship_A_position_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_A_position_Msg.User_ID);
#endif
			USV_Check_Collision[j].Longitude_old=USV_Check_Collision[j].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[j].Latitude_old=USV_Check_Collision[j].Latitude;
			USV_Check_Collision[j].UTC_Time_old=USV_Check_Collision[j].UTC_Time;
			USV_Check_Collision[j].UTC_Time_old_2=USV_Check_Collision[j].UTC_Time_2;
			
			USV_Check_Collision[j].Longitude=Ship_A_position_Msg.Longitude;
			USV_Check_Collision[j].Latitude=Ship_A_position_Msg.Latitude;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[j].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			if(USV_Check_Collision[j].Ship_Lenght==0)
				USV_Check_Collision[j].Ship_Lenght=50;//���δ��ȡ��������̬��Ϣ����Ĭ��50�׼���
			Collision_Num=j;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
			break;
		}
		if(j==AIS_Collision_Num)//û�п���λ�÷����µĴ�����Ϣ
		{
			Collect_Safe_Ship_Num();
			count=Collision_Alternative[0];
			memset((uint8 *)&USV_Check_Collision[count].User_ID,0,sizeof(USV_Check_Collision));
			USV_Check_Collision[count].User_ID=Ship_A_position_Msg.User_ID;
			if(Ship_A_position_Msg.SOG<1023)
				USV_Check_Collision[count].SOG=(float)(Ship_A_position_Msg.SOG*0.1);
#ifdef  debug_print 
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_A_position_Msg.User_ID);
#endif
			if(Ship_A_position_Msg.COG<3600)
				USV_Check_Collision[count].COG=(float)(Ship_A_position_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_A_position_Msg.User_ID);
#endif
			USV_Check_Collision[count].Longitude_old=USV_Check_Collision[count].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[count].Latitude_old=USV_Check_Collision[count].Latitude;
			USV_Check_Collision[count].UTC_Time_old=USV_Check_Collision[count].UTC_Time;
			USV_Check_Collision[count].UTC_Time_old_2=USV_Check_Collision[count].UTC_Time_2;
			
			USV_Check_Collision[count].Longitude=Ship_A_position_Msg.Longitude;
			USV_Check_Collision[count].Latitude=Ship_A_position_Msg.Latitude;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[count].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			if(USV_Check_Collision[count].Ship_Lenght==0)
				USV_Check_Collision[count].Ship_Lenght=AIS_ShipLength_A;//���δ��ȡ��������̬��Ϣ����Ĭ��AIS_ShipLength_A����
			Collision_Num=count;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
		}
	}
	return  ;
}
/*************************************************
��������:UTC_Analytical
��������:����̨���棬��Ϣ4��UTC�����ڻ�Ӧ����Ϣ11.
��������:
***************************************************/
void UTC_Analytical(char* isASCII_code)
{
	Ship_UTC	Ship_UTC_Msg;
	memset((char *)&Ship_UTC_Msg.Msg_ID,0,sizeof(Ship_UTC_Msg));
	Ship_UTC_Msg.Msg_ID=isASCII_code[0]&0x3f;//��Ϣ1/2/3�ı�ʶ��
	Ship_UTC_Msg.Rep_ID=(isASCII_code[1]&0x30)>>4;//��ʾһ����Ϣ��ת���Ĵ���0~3��Ĭ��Ϊ0,3��ʾ��ת��
	Ship_UTC_Msg.User_ID+=((isASCII_code[1]&0x0f)<<26);
	Ship_UTC_Msg.User_ID+=((isASCII_code[2]&0x3f)<<20);
	Ship_UTC_Msg.User_ID+=((isASCII_code[3]&0x3f)<<14);
	Ship_UTC_Msg.User_ID+=((isASCII_code[4]&0x3f)<<8);
	Ship_UTC_Msg.User_ID+=((isASCII_code[5]&0x3f)<<2);
	Ship_UTC_Msg.User_ID+=((isASCII_code[6]&0x30)>>4);
	Ship_UTC_Msg.UTC_year+=((isASCII_code[6]&0x0f)<<10);
	Ship_UTC_Msg.UTC_year+=((isASCII_code[7]&0x3f)<<4);
	Ship_UTC_Msg.UTC_year+=((isASCII_code[8]&0x3c)>>2);
	Ship_UTC_Msg.UTC_month+=((isASCII_code[8]&0x03)<<2);
	Ship_UTC_Msg.UTC_month+=((isASCII_code[9]&0x30)>>4);
	Ship_UTC_Msg.UTC_day+=((isASCII_code[9]&0x0f)<<1);
	Ship_UTC_Msg.UTC_day+=((isASCII_code[10]&0x20)>>5);
	Ship_UTC_Msg.UTC_hour=isASCII_code[10]&0x1f;
	Ship_UTC_Msg.UTC_minute=isASCII_code[11]&0x3f;
	Ship_UTC_Msg.UTC_second=isASCII_code[12]&0x3f;
	Ship_UTC_Msg.Position_accuracy=((isASCII_code[13]&0x20)>>5);
	Ship_UTC_Msg.Longitude+=((isASCII_code[13]&0x1f)<<23);
	Ship_UTC_Msg.Longitude+=((isASCII_code[14]&0x3f)<<17);
	Ship_UTC_Msg.Longitude+=((isASCII_code[15]&0x3f)<<11);
	Ship_UTC_Msg.Longitude+=((isASCII_code[16]&0x3f)<<5);
	Ship_UTC_Msg.Longitude+=((isASCII_code[17]&0x3d)>>1);
	Ship_UTC_Msg.Latitude+=((isASCII_code[17]&0x01)<<26);
	Ship_UTC_Msg.Latitude+=((isASCII_code[18]&0x3f)<<20);
	Ship_UTC_Msg.Latitude+=((isASCII_code[19]&0x3f)<<14);
	Ship_UTC_Msg.Latitude+=((isASCII_code[20]&0x3f)<<8);
	Ship_UTC_Msg.Latitude+=((isASCII_code[21]&0x3f)<<2);
	Ship_UTC_Msg.Latitude+=((isASCII_code[22]&0x30)>>4);
	Ship_UTC_Msg.Device_type=isASCII_code[22]&0x0f;
	Ship_UTC_Msg.Spare+=((isASCII_code[23]&0x3f)<<4);
	Ship_UTC_Msg.Spare+=((isASCII_code[24]&0x3c)>>2);
	Ship_UTC_Msg.RAIM_Flag=(isASCII_code[24]&0x02)>>1;
	Ship_UTC_Msg.UTC_direct+=((isASCII_code[24]&0x01)<<1);
	Ship_UTC_Msg.UTC_direct+=((isASCII_code[25]&0x10)>>1);
	Ship_UTC_Msg.Time_lost=(isASCII_code[25]&0x1c)>>2;
	Ship_UTC_Msg.Sub_information+=((isASCII_code[25]&0x03)<<12);
	Ship_UTC_Msg.Sub_information+=((isASCII_code[26]&0x3f)<<6);
	Ship_UTC_Msg.Sub_information+=(isASCII_code[27]&0x3f);
#ifdef  debug_print 
	printf("����Ϣ��%d\n",Ship_UTC_Msg.Sub_information);
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_UTC_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_UTC_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_UTC_Msg.User_ID);
	printf("UTC��ݣ�%d\n",Ship_UTC_Msg.UTC_year);
	printf("UTC�·ݣ�%d\n",Ship_UTC_Msg.UTC_month);
	printf("UTC���ڣ�%d\n",Ship_UTC_Msg.UTC_day);
	printf("UTCСʱ��%d\n",Ship_UTC_Msg.UTC_hour);
	printf("UTC���ӣ�%d\n",Ship_UTC_Msg.UTC_minute);
	printf("UTC�룺%d\n",Ship_UTC_Msg.UTC_second);
	printf("��λ��ȷ�ȣ�%d\n",Ship_UTC_Msg.Position_accuracy);
	printf("���ȣ�%d\n",Ship_UTC_Msg.Longitude);
	printf("γ�ȣ�%d\n",Ship_UTC_Msg.Latitude);
	printf("���Ӷ�λװ�����ͣ�%d\n",Ship_UTC_Msg.Device_type);
	printf("���ã�%d\n",Ship_UTC_Msg.Spare);
	printf("RAIM��־��%d\n",Ship_UTC_Msg.RAIM_Flag);
	printf("ͬ��״̬��%d\n",Ship_UTC_Msg.UTC_direct);
	printf("ʱ϶��ʱ��%d\n",Ship_UTC_Msg.Time_lost);
	printf("����Ϣ��%d\n",Ship_UTC_Msg.Sub_information);	
#endif
	return  ;
}
/*************************************************
��������:Static_Analytical
��������:��̬������Ϣ��������Ϣ5
��������:
***************************************************/
void Static_Analytical(uint8* isASCII_code_5)
{	
	Ship_static_information Ship_Static_Msg;
//	int i;
	int j,count,length_1,length_2;
	count=0;
	length_1=0;
	length_2=0;
	memset((char *)&Ship_Static_Msg.Msg_ID,0,sizeof(Ship_Static_Msg));
	
	Ship_Static_Msg.Msg_ID=isASCII_code_5[0]&0x3f;
	Ship_Static_Msg.Rep_ID=((isASCII_code_5[1]&0x30)>>4);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[1]&0x0f)<<26);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[2]&0x3f)<<20);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[3]&0x3f)<<14);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[4]&0x3f)<<8);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[5]&0x3f)<<2);
	Ship_Static_Msg.User_ID+=((isASCII_code_5[6]&0x30)>>4);
	Ship_Static_Msg.AIS_version=((isASCII_code_5[6]&0xc0)>>2);
	Ship_Static_Msg.IMO+=((isASCII_code_5[6]&0x03)<<28);
	Ship_Static_Msg.IMO+=((isASCII_code_5[7]&0x3f)<<22);
	Ship_Static_Msg.IMO+=((isASCII_code_5[8]&0x3f)<<16);
	Ship_Static_Msg.IMO+=((isASCII_code_5[9]&0x3f)<<10);
	Ship_Static_Msg.IMO+=((isASCII_code_5[10]&0x3f)<<4);
	Ship_Static_Msg.IMO+=((isASCII_code_5[11]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[6]+=((isASCII_code_5[11]&0x03)<<2);
	Ship_Static_Msg.Call_sign[6]+=((isASCII_code_5[12]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[5]+=((isASCII_code_5[12]&0x03)<<2);
	Ship_Static_Msg.Call_sign[5]+=((isASCII_code_5[13]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[4]+=((isASCII_code_5[13]&0x03)<<2);
	Ship_Static_Msg.Call_sign[4]+=((isASCII_code_5[14]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[3]+=((isASCII_code_5[14]&0x03)<<2);
	Ship_Static_Msg.Call_sign[3]+=((isASCII_code_5[15]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[2]+=((isASCII_code_5[15]&0x03)<<2);
	Ship_Static_Msg.Call_sign[2]+=((isASCII_code_5[16]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[1]+=((isASCII_code_5[16]&0x03)<<2);
	Ship_Static_Msg.Call_sign[1]+=((isASCII_code_5[17]&0x3c)>>2);
	Ship_Static_Msg.Call_sign[0]+=((isASCII_code_5[17]&0x03)<<2);
	Ship_Static_Msg.Call_sign[0]+=((isASCII_code_5[18]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[19]+=((isASCII_code_5[18]&0x03)<<2);
	Ship_Static_Msg.Ship_name[19]+=((isASCII_code_5[19]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[18]+=((isASCII_code_5[19]&0x03)<<2);
	Ship_Static_Msg.Ship_name[18]+=((isASCII_code_5[20]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[17]+=((isASCII_code_5[20]&0x03)<<2);
	Ship_Static_Msg.Ship_name[17]+=((isASCII_code_5[21]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[16]+=((isASCII_code_5[21]&0x03)<<2);
	Ship_Static_Msg.Ship_name[16]+=((isASCII_code_5[22]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[15]+=((isASCII_code_5[22]&0x03)<<2);
	Ship_Static_Msg.Ship_name[15]+=((isASCII_code_5[23]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[14]+=((isASCII_code_5[23]&0x03)<<2);
	Ship_Static_Msg.Ship_name[14]+=((isASCII_code_5[24]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[13]+=((isASCII_code_5[24]&0x03)<<2);
	Ship_Static_Msg.Ship_name[13]+=((isASCII_code_5[25]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[12]+=((isASCII_code_5[25]&0x03)<<2);
	Ship_Static_Msg.Ship_name[12]+=((isASCII_code_5[26]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[11]+=((isASCII_code_5[26]&0x03)<<2);
	Ship_Static_Msg.Ship_name[11]+=((isASCII_code_5[27]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[10]+=((isASCII_code_5[27]&0x03)<<2);
	Ship_Static_Msg.Ship_name[10]+=((isASCII_code_5[28]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[9]+=((isASCII_code_5[28]&0x03)<<2);
	Ship_Static_Msg.Ship_name[9]+=((isASCII_code_5[29]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[8]+=((isASCII_code_5[29]&0x03)<<2);
	Ship_Static_Msg.Ship_name[8]+=((isASCII_code_5[30]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[7]+=((isASCII_code_5[30]&0x03)<<2);
	Ship_Static_Msg.Ship_name[7]+=((isASCII_code_5[31]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[6]+=((isASCII_code_5[31]&0x03)<<2);
	Ship_Static_Msg.Ship_name[6]+=((isASCII_code_5[32]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[5]+=((isASCII_code_5[32]&0x03)<<2);
	Ship_Static_Msg.Ship_name[5]+=((isASCII_code_5[33]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[4]+=((isASCII_code_5[33]&0x03)<<2);
	Ship_Static_Msg.Ship_name[4]+=((isASCII_code_5[34]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[3]+=((isASCII_code_5[34]&0x03)<<2);
	Ship_Static_Msg.Ship_name[3]+=((isASCII_code_5[35]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[2]+=((isASCII_code_5[35]&0x03)<<2);
	Ship_Static_Msg.Ship_name[2]+=((isASCII_code_5[36]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[1]+=((isASCII_code_5[36]&0x03)<<2);
	Ship_Static_Msg.Ship_name[1]+=((isASCII_code_5[37]&0x3c)>>2);
	Ship_Static_Msg.Ship_name[0]+=((isASCII_code_5[37]&0x03)<<2);
	Ship_Static_Msg.Ship_name[0]+=((isASCII_code_5[38]&0x3c)>>2);
	Ship_Static_Msg.Cargo_type+=((isASCII_code_5[38]&0x03)<<6);
	Ship_Static_Msg.Cargo_type+=(isASCII_code_5[39]&0x3f);
	Ship_Static_Msg.Dimension+=((isASCII_code_5[40]&0x3f)<<24);
	Ship_Static_Msg.Dimension+=((isASCII_code_5[41]&0x3f)<<18);
	Ship_Static_Msg.Dimension+=((isASCII_code_5[42]&0x3f)<<12);
	Ship_Static_Msg.Dimension+=((isASCII_code_5[43]&0x3f)<<6);
	Ship_Static_Msg.Dimension+=(isASCII_code_5[44]&0x3f);
	Ship_Static_Msg.Device_type=((isASCII_code_5[45]&0x3c)>>2);
	Ship_Static_Msg.EAT+=((isASCII_code_5[45]&0x03)<<18);
	Ship_Static_Msg.EAT+=((isASCII_code_5[46]&0x3f)<<12);
	Ship_Static_Msg.EAT+=((isASCII_code_5[47]&0x3f)<<6);
	Ship_Static_Msg.EAT+=(isASCII_code_5[48]&0x3f);
	Ship_Static_Msg.Darught_Max+=((isASCII_code_5[49]&0x3f)<<2);
	Ship_Static_Msg.Darught_Max+=((isASCII_code_5[49]&0x30)>>4);
	Ship_Static_Msg.Destination[19]+=((isASCII_code_5[50]&0x0f)<<2);
	Ship_Static_Msg.Destination[19]+=((isASCII_code_5[51]&0x3c)>>2);
	Ship_Static_Msg.Destination[18]+=((isASCII_code_5[51]&0x0f)<<2);
	Ship_Static_Msg.Destination[18]+=((isASCII_code_5[52]&0x3c)>>2);
	Ship_Static_Msg.Destination[17]+=((isASCII_code_5[52]&0x0f)<<2);
	Ship_Static_Msg.Destination[17]+=((isASCII_code_5[53]&0x3c)>>2);
	Ship_Static_Msg.Destination[16]+=((isASCII_code_5[53]&0x0f)<<2);
	Ship_Static_Msg.Destination[16]+=((isASCII_code_5[54]&0x3c)>>2);
	Ship_Static_Msg.Destination[15]+=((isASCII_code_5[54]&0x0f)<<2);
	Ship_Static_Msg.Destination[15]+=((isASCII_code_5[55]&0x3c)>>2);
	Ship_Static_Msg.Destination[14]+=((isASCII_code_5[55]&0x0f)<<2);
	Ship_Static_Msg.Destination[14]+=((isASCII_code_5[56]&0x3c)>>2);
	Ship_Static_Msg.Destination[13]+=((isASCII_code_5[56]&0x0f)<<2);
	Ship_Static_Msg.Destination[13]+=((isASCII_code_5[57]&0x3c)>>2);
	Ship_Static_Msg.Destination[12]+=((isASCII_code_5[57]&0x0f)<<2);
	Ship_Static_Msg.Destination[12]+=((isASCII_code_5[58]&0x3c)>>2);
	Ship_Static_Msg.Destination[11]+=((isASCII_code_5[58]&0x0f)<<2);
	Ship_Static_Msg.Destination[11]+=((isASCII_code_5[59]&0x3c)>>2);
	Ship_Static_Msg.Destination[10]+=((isASCII_code_5[59]&0x0f)<<2);
	Ship_Static_Msg.Destination[10]+=((isASCII_code_5[60]&0x3c)>>2);
	Ship_Static_Msg.Destination[9]+=((isASCII_code_5[60]&0x0f)<<2);
	Ship_Static_Msg.Destination[9]+=((isASCII_code_5[61]&0x3c)>>2);
	Ship_Static_Msg.Destination[8]+=((isASCII_code_5[61]&0x0f)<<2);
	Ship_Static_Msg.Destination[8]+=((isASCII_code_5[62]&0x3c)>>2);
	Ship_Static_Msg.Destination[7]+=((isASCII_code_5[62]&0x0f)<<2);
	Ship_Static_Msg.Destination[7]+=((isASCII_code_5[63]&0x3c)>>2);
	Ship_Static_Msg.Destination[6]+=((isASCII_code_5[63]&0x0f)<<2);
	Ship_Static_Msg.Destination[6]+=((isASCII_code_5[64]&0x3c)>>2);
	Ship_Static_Msg.Destination[5]+=((isASCII_code_5[64]&0x0f)<<2);
	Ship_Static_Msg.Destination[5]+=((isASCII_code_5[65]&0x3c)>>2);
	Ship_Static_Msg.Destination[4]+=((isASCII_code_5[65]&0x0f)<<2);
	Ship_Static_Msg.Destination[4]+=((isASCII_code_5[66]&0x3c)>>2);
	Ship_Static_Msg.Destination[3]+=((isASCII_code_5[66]&0x0f)<<2);
	Ship_Static_Msg.Destination[3]+=((isASCII_code_5[67]&0x3c)>>2);
	Ship_Static_Msg.Destination[2]+=((isASCII_code_5[67]&0x0f)<<2);
	Ship_Static_Msg.Destination[2]+=((isASCII_code_5[68]&0x3c)>>2);
	Ship_Static_Msg.Destination[1]+=((isASCII_code_5[68]&0x0f)<<2);
	Ship_Static_Msg.Destination[1]+=((isASCII_code_5[69]&0x3c)>>2);
	Ship_Static_Msg.Destination[0]+=((isASCII_code_5[69]&0x0f)<<2);
	Ship_Static_Msg.Destination[0]+=((isASCII_code_5[70]&0x3c)>>2);
	Ship_Static_Msg.DET=((isASCII_code_5[70]&0x08)>>3);
	Ship_Static_Msg.Spare=((isASCII_code_5[70]&0x04)>>2);
/*#ifdef  debug_print 	
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_Static_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_Static_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_Static_Msg.User_ID);
	printf("AIS�汾��%d\n",Ship_Static_Msg.AIS_version);
	printf("IMO���룺%d\n",Ship_Static_Msg.IMO);
	printf("���ţ�");	
	for (i=0;i<7;i++)
	{
		printf("%d ",Ship_Static_Msg.Call_sign[i]);
	}
	printf("\n"); 
	printf("������");
	for (i=0;i<20;i++)
	{
		printf("%d ",Ship_Static_Msg.Ship_name[i]);
	}
	printf("\n"); 
	printf("�������ػ����ͣ�%d\n",Ship_Static_Msg.Cargo_type);
	printf("�߶�/λ�òο���%d\n",Ship_Static_Msg.Dimension);//�ɻ�ô����ʹ���
	printf("���Ӷ�λװ�����ͣ�%d\n",Ship_Static_Msg.Device_type);
	printf("Ԥ�Ƶ���ʱ�䣺%d\n",Ship_Static_Msg.EAT);
	printf("��ǰ���̬��ˮ����%d\n",Ship_Static_Msg.Darught_Max);
	printf("Ŀ�ĵأ�");
	for (i=0;i<20;i++)
	{
		printf("%d ",Ship_Static_Msg.Destination[i]);
	}
	printf("\n"); 
	printf("�����ն��豸ָʾ����%d\n",Ship_Static_Msg.DET);
	printf("���ã�%d\n",Ship_Static_Msg.Spare);
#endif*/
	//������ײ��Ϣ
	for(j=0;j<AIS_Collision_Num;j++)
	{
		if((USV_Check_Collision[j].User_ID==0)||(USV_Check_Collision[j].User_ID==Ship_Static_Msg.User_ID))
		{
			USV_Check_Collision[j].User_ID=Ship_Static_Msg.User_ID;
			length_1=(Ship_Static_Msg.Dimension)&0x000001ff;
			length_2=(Ship_Static_Msg.Dimension)&0x0002fe00;
			USV_Check_Collision[j].Ship_Lenght=length_1+length_2;
			Collision_Num=j;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
			break;
		}
		if(j==AIS_Collision_Num)//û�п���λ�÷����µĴ�����Ϣ
		{
			Collect_Safe_Ship_Num();
			count=Collision_Alternative[0];
			memset((uint8 *)&USV_Check_Collision[count].User_ID,0,sizeof(USV_Check_Collision));
			USV_Check_Collision[count].User_ID=Ship_Static_Msg.User_ID;
			length_1=(Ship_Static_Msg.Dimension)&0x000001ff;
			length_2=(Ship_Static_Msg.Dimension)&0x0002fe00;
			USV_Check_Collision[count].Ship_Lenght=length_1+length_2;
			Collision_Num=count;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
		}
	}
	return  ;
}
/*************************************************
��������:Position_Analytical_B
��������:B�ബλ����Ϣ��������Ϣ18
��������:
***************************************************/
void Position_Analytical_B(char* isASCII_code)
{
	uint8 j,count;
	count=0;
	Ship_B_position	Ship_B_position_Msg;
	memset((char *)&Ship_B_position_Msg.Msg_ID,0,sizeof(Ship_B_position_Msg));
	Ship_B_position_Msg.Msg_ID=isASCII_code[0]&0x3f;//��Ϣ1/2/3�ı�ʶ��
	Ship_B_position_Msg.Rep_ID=(isASCII_code[1]&0x30)>>4;//��ʾһ����Ϣ��ת���Ĵ���0~3��Ĭ��Ϊ0,3��ʾ��ת��

	Ship_B_position_Msg.User_ID+=((isASCII_code[1]&0x0f)<<26);
	Ship_B_position_Msg.User_ID+=((isASCII_code[2]&0x3f)<<20);
	Ship_B_position_Msg.User_ID+=((isASCII_code[3]&0x3f)<<14);
	Ship_B_position_Msg.User_ID+=((isASCII_code[4]&0x3f)<<8);
	Ship_B_position_Msg.User_ID+=((isASCII_code[5]&0x3f)<<2);
	Ship_B_position_Msg.User_ID+=((isASCII_code[6]&0x30)>>4);

	Ship_B_position_Msg.Regional_APP_1+=((isASCII_code[7]&0x03)<<8);
	Ship_B_position_Msg.Regional_APP_1+=((isASCII_code[8]&0x3f)<<2);
	Ship_B_position_Msg.Regional_APP_1+=((isASCII_code[9]&0x30)>>4);
	Ship_B_position_Msg.SOG+=((isASCII_code[8]&0x0f)<<6);
	Ship_B_position_Msg.SOG+=(isASCII_code[9]&0x3f);

	Ship_B_position_Msg.Position_accuracy=(isASCII_code[9]&0x08)>>3;

	Ship_B_position_Msg.Longitude+=((isASCII_code[9]&0x07)<<25);
	Ship_B_position_Msg.Longitude+=((isASCII_code[10]&0x3f)<<19);
	Ship_B_position_Msg.Longitude+=((isASCII_code[11]&0x3f)<<13);
	Ship_B_position_Msg.Longitude+=((isASCII_code[12]&0x3f)<<7);
	Ship_B_position_Msg.Longitude+=((isASCII_code[13]&0x3f)<<1);
	Ship_B_position_Msg.Longitude+=((isASCII_code[14]&0x20)>>5);

	Ship_B_position_Msg.Latitude+=((isASCII_code[14]&0x1f)<<22);
	Ship_B_position_Msg.Latitude+=((isASCII_code[15]&0x3f)<<16);
	Ship_B_position_Msg.Latitude+=((isASCII_code[16]&0x3f)<<10);
	Ship_B_position_Msg.Latitude+=((isASCII_code[17]&0x3f)<<4);
	Ship_B_position_Msg.Latitude+=((isASCII_code[18]&0x3c)>>2);

	Ship_B_position_Msg.COG+=((isASCII_code[18]&0x03)<<10);
	Ship_B_position_Msg.COG+=((isASCII_code[19]&0x3f)<<4);
	Ship_B_position_Msg.COG+=((isASCII_code[20]&0x3c)>>2);

	Ship_B_position_Msg.True_Head+=((isASCII_code[20]&0x03)<<7);
	Ship_B_position_Msg.True_Head+=((isASCII_code[21]&0x3f)<<1);
	Ship_B_position_Msg.True_Head+=((isASCII_code[22]&0x20)>>5);

	Ship_B_position_Msg.Timer_stamp+=((isASCII_code[22]&0x1f)<<1);
	Ship_B_position_Msg.Timer_stamp+=((isASCII_code[23]&0x20)>>5);

	Ship_B_position_Msg.Regional_APP_2=((isASCII_code[23]&0x1d)>>1);

	Ship_B_position_Msg.Spare+=((isASCII_code[23]&0x01)<<3);
	Ship_B_position_Msg.Spare+=((isASCII_code[24]&0x38)>>3);

	Ship_B_position_Msg.RAIM_Flag=(isASCII_code[24]&0x04)>>2;

	Ship_B_position_Msg.Communication_Sign=(isASCII_code[24]&0x02)>>1;

	Ship_B_position_Msg.UTC_direct+=((isASCII_code[24]&0x01)<<1);
	Ship_B_position_Msg.UTC_direct+=((isASCII_code[25]&0x10)>>1);

	Ship_B_position_Msg.Time_lost=(isASCII_code[25]&0x1c)>>2;

	Ship_B_position_Msg.Sub_information+=((isASCII_code[25]&0x03)<<12);
	Ship_B_position_Msg.Sub_information+=((isASCII_code[26]&0x3f)<<6);
	Ship_B_position_Msg.Sub_information+=(isASCII_code[27]&0x3f);
#ifdef  debug_print 
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_B_position_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_B_position_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_B_position_Msg.User_ID);
	printf("����Ӧ�ñ���1��%d\n",Ship_B_position_Msg.Regional_APP_1);
	printf("�Եغ��٣�%d\n",Ship_B_position_Msg.SOG);
	printf("��λ��ȷ�ȣ�%d\n",Ship_B_position_Msg.Position_accuracy);
	printf("���ȣ�%d\n",Ship_B_position_Msg.Longitude);
	printf("γ�ȣ�%d\n",Ship_B_position_Msg.Latitude);
	printf("�Եغ���%d\n",Ship_B_position_Msg.COG);
	printf("�溽��%d\n",Ship_B_position_Msg.True_Head);
	printf("ʱ���ǣ�%d\n",Ship_B_position_Msg.Timer_stamp);
	printf("����Ӧ�ñ���2��%d\n",Ship_B_position_Msg.Regional_APP_2);
	printf("���ã�%d\n",Ship_B_position_Msg.Spare);
	printf("RAIM��־��%d\n",Ship_B_position_Msg.RAIM_Flag);
	printf("ͨ��״̬ѡ���־��%d\n",Ship_B_position_Msg.Communication_Sign);
	printf("ͬ��״̬��%d\n",Ship_B_position_Msg.UTC_direct);
	printf("ʱ϶��ʱ��%d\n",Ship_B_position_Msg.Time_lost);
	printf("����Ϣ��%d\n",Ship_B_position_Msg.Sub_information);
#endif
	//������ײ��Ϣ
	for(j=0;j<AIS_Collision_Num;j++)
	{
		if((USV_Check_Collision[j].User_ID==0)||(USV_Check_Collision[j].User_ID==Ship_B_position_Msg.User_ID))
		{
			USV_Check_Collision[j].User_ID=Ship_B_position_Msg.User_ID;
			if(Ship_B_position_Msg.SOG<1023)
				USV_Check_Collision[j].SOG=(float)(Ship_B_position_Msg.SOG*0.1);
#ifdef  debug_print 
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_B_position_Msg.User_ID);
#endif			
			if(Ship_B_position_Msg.COG<3600)
				USV_Check_Collision[j].COG=(float)(Ship_B_position_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_B_position_Msg.User_ID);
#endif			
			USV_Check_Collision[j].Longitude_old=USV_Check_Collision[j].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[j].Latitude_old=USV_Check_Collision[j].Latitude;
			USV_Check_Collision[j].UTC_Time_old=USV_Check_Collision[j].UTC_Time;
			USV_Check_Collision[j].UTC_Time_old_2=USV_Check_Collision[j].UTC_Time_2;
			
			USV_Check_Collision[j].Longitude=Ship_B_position_Msg.Longitude;
			USV_Check_Collision[j].Latitude=Ship_B_position_Msg.Latitude;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[j].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			if(USV_Check_Collision[j].Ship_Lenght==0)
				USV_Check_Collision[j].Ship_Lenght=50;//���δ��ȡ��������̬��Ϣ����Ĭ��50�׼���
			Collision_Num=j;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
			break;
		}
		if(j==AIS_Collision_Num)//û�п���λ�÷����µĴ�����Ϣ
		{
			Collect_Safe_Ship_Num();
			count=Collision_Alternative[0];
			memset((uint8 *)&USV_Check_Collision[count].User_ID,0,sizeof(USV_Check_Collision));
			USV_Check_Collision[count].User_ID=Ship_B_position_Msg.User_ID;
			if(Ship_B_position_Msg.SOG<1023)
				USV_Check_Collision[count].SOG=(float)(Ship_B_position_Msg.SOG*0.1);
#ifdef  debug_print 
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_B_position_Msg.User_ID);
#endif			
			if(Ship_B_position_Msg.COG<3600)
				USV_Check_Collision[count].COG=(float)(Ship_B_position_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_B_position_Msg.User_ID);
#endif			
			USV_Check_Collision[count].Longitude_old=USV_Check_Collision[count].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[count].Latitude_old=USV_Check_Collision[count].Latitude;
			USV_Check_Collision[count].UTC_Time_old=USV_Check_Collision[count].UTC_Time;
			USV_Check_Collision[count].UTC_Time_old_2=USV_Check_Collision[count].UTC_Time_2;
			
			USV_Check_Collision[count].Longitude=Ship_B_position_Msg.Longitude;
			USV_Check_Collision[count].Latitude=Ship_B_position_Msg.Latitude;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[count].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			if(USV_Check_Collision[count].Ship_Lenght==0)
				USV_Check_Collision[count].Ship_Lenght=AIS_ShipLength_B;//���δ��ȡ��������̬��Ϣ����Ĭ��AIS_ShipLength_B����
			Collision_Num=count;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
		}
	}
	return  ;
}
/*************************************************
��������:Position_Analytical_B_Extended
��������:B�ബ��չλ����Ϣ��������Ϣ19
��������:
***************************************************/
void Position_Analytical_B_Extended(char* isASCII_code)
{	
	Ship_B_position_Extended	Ship_B_position_Extended_Msg;
//	int i;
	int j,count,length_1,length_2;
	count=0;
	length_1=0;
	length_2=0;
	memset((char *)&Ship_B_position_Extended_Msg.Msg_ID,0,sizeof(Ship_B_position_Extended_Msg));
	Ship_B_position_Extended_Msg.Msg_ID=isASCII_code[0]&0x3f;//��Ϣ1/2/3�ı�ʶ��

	Ship_B_position_Extended_Msg.Rep_ID=(isASCII_code[1]&0x30)>>4;//��ʾһ����Ϣ��ת���Ĵ���0~3��Ĭ��Ϊ0,3��ʾ��ת��

	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[1]&0x0f)<<26);
	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[2]&0x3f)<<20);
	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[3]&0x3f)<<14);
	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[4]&0x3f)<<8);
	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[5]&0x3f)<<2);
	Ship_B_position_Extended_Msg.User_ID+=((isASCII_code[6]&0x30)>>4);

	Ship_B_position_Extended_Msg.Regional_APP_1+=((isASCII_code[7]&0x03)<<8);
	Ship_B_position_Extended_Msg.Regional_APP_1+=((isASCII_code[8]&0x3f)<<2);
	Ship_B_position_Extended_Msg.Regional_APP_1+=((isASCII_code[9]&0x30)>>4);

	Ship_B_position_Extended_Msg.SOG+=((isASCII_code[8]&0x0f)<<6);
	Ship_B_position_Extended_Msg.SOG+=(isASCII_code[9]&0x3f);

	Ship_B_position_Extended_Msg.Position_accuracy=(isASCII_code[9]&0x08)>>3;

	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[9]&0x07)<<25);
	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[10]&0x3f)<<19);
	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[11]&0x3f)<<13);
	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[12]&0x3f)<<7);
	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[13]&0x3f)<<1);
	Ship_B_position_Extended_Msg.Longitude+=((isASCII_code[14]&0x20)>>5);

	Ship_B_position_Extended_Msg.Latitude+=((isASCII_code[14]&0x1f)<<21);
	Ship_B_position_Extended_Msg.Latitude+=((isASCII_code[15]&0x3f)<<15);
	Ship_B_position_Extended_Msg.Latitude+=((isASCII_code[16]&0x3f)<<9);
	Ship_B_position_Extended_Msg.Latitude+=((isASCII_code[17]&0x3f)<<3);
	Ship_B_position_Extended_Msg.Latitude+=((isASCII_code[18]&0x3c)>>2);

	Ship_B_position_Extended_Msg.COG+=((isASCII_code[18]&0x03)<<10);
	Ship_B_position_Extended_Msg.COG+=((isASCII_code[19]&0x3f)<<4);
	Ship_B_position_Extended_Msg.COG+=((isASCII_code[20]&0x3c)>>2);

	Ship_B_position_Extended_Msg.True_Head+=((isASCII_code[20]&0x03)<<7);
	Ship_B_position_Extended_Msg.True_Head+=((isASCII_code[21]&0x3f)<<1);
	Ship_B_position_Extended_Msg.True_Head+=((isASCII_code[22]&0x20)>>5);

	Ship_B_position_Extended_Msg.Timer_stamp+=((isASCII_code[22]&0x1f)<<1);
	Ship_B_position_Extended_Msg.Timer_stamp+=((isASCII_code[23]&0x20)>>5);

	Ship_B_position_Extended_Msg.Regional_APP_2=((isASCII_code[23]&0x1d)>>1);

	Ship_B_position_Extended_Msg.Ship_name[19]+=((isASCII_code[23]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[19]+=((isASCII_code[24]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[18]+=((isASCII_code[24]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[18]+=((isASCII_code[25]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[17]+=((isASCII_code[25]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[17]+=((isASCII_code[26]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[16]+=((isASCII_code[26]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[16]+=((isASCII_code[27]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[15]+=((isASCII_code[27]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[15]+=((isASCII_code[28]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[14]+=((isASCII_code[28]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[14]+=((isASCII_code[29]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[13]+=((isASCII_code[29]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[13]+=((isASCII_code[30]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[12]+=((isASCII_code[30]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[12]+=((isASCII_code[31]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[11]+=((isASCII_code[31]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[11]+=((isASCII_code[32]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[10]+=((isASCII_code[32]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[10]+=((isASCII_code[33]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[9]+=((isASCII_code[33]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[9]+=((isASCII_code[34]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[8]+=((isASCII_code[34]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[8]+=((isASCII_code[35]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[7]+=((isASCII_code[35]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[7]+=((isASCII_code[36]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[6]+=((isASCII_code[36]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[6]+=((isASCII_code[37]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[5]+=((isASCII_code[37]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[5]+=((isASCII_code[38]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[4]+=((isASCII_code[38]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[4]+=((isASCII_code[39]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[3]+=((isASCII_code[39]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[3]+=((isASCII_code[40]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[2]+=((isASCII_code[40]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[2]+=((isASCII_code[41]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[1]+=((isASCII_code[41]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[1]+=((isASCII_code[42]&0x3d)>>1);
	Ship_B_position_Extended_Msg.Ship_name[0]+=((isASCII_code[42]&0x01)<<5);
	Ship_B_position_Extended_Msg.Ship_name[0]+=((isASCII_code[43]&0x3d)>>1);

	Ship_B_position_Extended_Msg.Cargo_type+=((isASCII_code[43]&0x01)<<7);
	Ship_B_position_Extended_Msg.Cargo_type+=((isASCII_code[44]&0x3f)<<1);
	Ship_B_position_Extended_Msg.Cargo_type+=((isASCII_code[45]&0x20)>>5);

	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[45]&0x1f)<<25);
	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[46]&0x3f)<<19);
	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[47]&0x3f)<<13);
	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[48]&0x3f)<<7);
	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[49]&0x3f)<<1);
	Ship_B_position_Extended_Msg.Dimension+=((isASCII_code[50]&0x20)>>5);

	Ship_B_position_Extended_Msg.Device_type=((isASCII_code[50]&0x1d)>>1);

	Ship_B_position_Extended_Msg.RAIM_Flag=isASCII_code[50]&0x01;

	Ship_B_position_Extended_Msg.Data_terminal=((isASCII_code[51]&0x20)>>5);

	Ship_B_position_Extended_Msg.Spare=isASCII_code[51]&0x1f;
/*#ifdef  debug_print 
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_B_position_Extended_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_B_position_Extended_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_B_position_Extended_Msg.User_ID);
	printf("����Ӧ�ñ���1��%d\n",Ship_B_position_Extended_Msg.Regional_APP_1);
	printf("�Եغ��٣�%d\n",Ship_B_position_Extended_Msg.SOG);
	printf("��λ��ȷ�ȣ�%d\n",Ship_B_position_Extended_Msg.Position_accuracy);
	printf("���ȣ�%d\n",Ship_B_position_Extended_Msg.Longitude);
	printf("γ�ȣ�%d\n",Ship_B_position_Extended_Msg.Latitude);
	printf("�Եغ���%d\n",Ship_B_position_Extended_Msg.COG);
	printf("�溽��%d\n",Ship_B_position_Extended_Msg.True_Head);
	printf("ʱ���ǣ�%d\n",Ship_B_position_Extended_Msg.Timer_stamp);
	printf("����Ӧ�ñ���2��%d\n",Ship_B_position_Extended_Msg.Regional_APP_2);
	printf("������");
	for (i=0;i<20;i++)
	{
		printf("%d ",Ship_B_position_Extended_Msg.Ship_name[i]);
	}
	printf("\n"); 
	printf("�������ػ����ͣ�%d\n",Ship_B_position_Extended_Msg.Cargo_type);
	printf("�����߶�/λ�òο���%d\n",Ship_B_position_Extended_Msg.Dimension);
	printf("���Ӷ�λװ�����ͣ�%d\n",Ship_B_position_Extended_Msg.Device_type);
	printf("RAIM��־��%d\n",Ship_B_position_Extended_Msg.RAIM_Flag);
	printf("�����նˣ�%d\n",Ship_B_position_Extended_Msg.Data_terminal);
	printf("���ã�%d\n",Ship_B_position_Extended_Msg.Spare);
#endif*/
	//������ײ��Ϣ
	for(j=0;j<AIS_Collision_Num;j++)
	{
		if((USV_Check_Collision[j].User_ID==0)||(USV_Check_Collision[j].User_ID==Ship_B_position_Extended_Msg.User_ID))
		{
			USV_Check_Collision[j].User_ID=Ship_B_position_Extended_Msg.User_ID;
			if(Ship_B_position_Extended_Msg.SOG<1023)
				USV_Check_Collision[j].SOG=(float)(Ship_B_position_Extended_Msg.SOG*0.1);
#ifdef  debug_print 
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_B_position_Extended_Msg.User_ID);
#endif			
			if(Ship_B_position_Extended_Msg.COG<3600)
				USV_Check_Collision[j].COG=(float)(Ship_B_position_Extended_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_B_position_Extended_Msg.User_ID);
#endif			
			USV_Check_Collision[j].Longitude_old=USV_Check_Collision[j].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[j].Latitude_old=USV_Check_Collision[j].Latitude;
			USV_Check_Collision[j].UTC_Time_old=USV_Check_Collision[j].UTC_Time;
			USV_Check_Collision[j].UTC_Time_old_2=USV_Check_Collision[j].UTC_Time_2;
			
			USV_Check_Collision[j].Longitude=Ship_B_position_Extended_Msg.Longitude;
			USV_Check_Collision[j].Latitude=Ship_B_position_Extended_Msg.Latitude;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[j].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[j].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			length_1=(Ship_B_position_Extended_Msg.Dimension)&0x000001ff;
			length_2=(Ship_B_position_Extended_Msg.Dimension)&0x0002fe00;
			USV_Check_Collision[j].Ship_Lenght=length_1+length_2;
			Collision_Num=j;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
			break;
		}
		if(j==AIS_Collision_Num)//û�п���λ�÷����µĴ�����Ϣ
		{
			Collect_Safe_Ship_Num();
			count=Collision_Alternative[0];
			memset((uint8 *)&USV_Check_Collision[count].User_ID,0,sizeof(USV_Check_Collision));
			USV_Check_Collision[count].User_ID=Ship_B_position_Extended_Msg.User_ID;
			if(Ship_B_position_Extended_Msg.SOG<1023)
				USV_Check_Collision[count].SOG=(float)(Ship_B_position_Extended_Msg.SOG*0.1);
#ifdef  debug_print 			
			else
				printf("SOG��Ϣ������MMSI��%d\n",Ship_B_position_Extended_Msg.User_ID);
#endif			
			if(Ship_B_position_Extended_Msg.COG<3600)
				USV_Check_Collision[count].COG=(float)(Ship_B_position_Extended_Msg.COG*0.1);
#ifdef  debug_print 
			else
				printf("COG��Ϣ������MMSI��%d\n",Ship_B_position_Extended_Msg.User_ID);
#endif			
			USV_Check_Collision[count].Longitude_old=USV_Check_Collision[count].Longitude;//�������Ϊ0��ʾ��һ���յ���Ϣ����COG����TCPA
			USV_Check_Collision[count].Latitude_old=USV_Check_Collision[count].Latitude;
			USV_Check_Collision[count].UTC_Time_old=USV_Check_Collision[count].UTC_Time;
			USV_Check_Collision[count].UTC_Time_old_2=USV_Check_Collision[count].UTC_Time_2;
			
			USV_Check_Collision[count].Longitude=Ship_B_position_Extended_Msg.Longitude;
			USV_Check_Collision[count].Latitude=Ship_B_position_Extended_Msg.Latitude;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Hour*3600;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Minute*60;
			USV_Check_Collision[count].UTC_Time+=Smart_Navigation_St.USV_Second;
			USV_Check_Collision[count].UTC_Time_2=Smart_Navigation_St.USV_Second_2;
			
			length_1=(Ship_B_position_Extended_Msg.Dimension)&0x000001ff;
			length_2=(Ship_B_position_Extended_Msg.Dimension)&0x0002fe00;
			USV_Check_Collision[count].Ship_Lenght=length_1+length_2;
			Collision_Num=count;//���յ��µ�AIS���ݣ�������µ�DCPA��TCPA
		}
	}
	return  ;
}
/*************************************************
��������:Static_Report_Analytical
��������:��̬�����������Ϣ24
��������:
***************************************************/
void Static_Report_Analytical(char* isASCII_code)
{	
	Ship_Static_A Ship_Static_A_Msg;
	Ship_Static_B Ship_Static_B_Msg;
//	int i;
	memset((char *)&Ship_Static_A_Msg.Msg_ID,0,sizeof(Ship_Static_A_Msg));
	memset((char *)&Ship_Static_B_Msg.Msg_ID,0,sizeof(Ship_Static_B_Msg));

	Ship_Static_A_Msg.Msg_ID=isASCII_code[0]&0x3f;//��Ϣ1/2/3�ı�ʶ��
	Ship_Static_B_Msg.Msg_ID=Ship_Static_A_Msg.Msg_ID;

	Ship_Static_A_Msg.Rep_ID=(isASCII_code[1]&0x30)>>4;//��ʾһ����Ϣ��ת���Ĵ���0~3��Ĭ��Ϊ0,3��ʾ��ת��
	Ship_Static_B_Msg.Rep_ID=Ship_Static_A_Msg.Rep_ID;

	Ship_Static_A_Msg.User_ID+=((isASCII_code[1]&0x0f)<<26);
	Ship_Static_A_Msg.User_ID+=((isASCII_code[2]&0x3f)<<20);
	Ship_Static_A_Msg.User_ID+=((isASCII_code[3]&0x3f)<<14);
	Ship_Static_A_Msg.User_ID+=((isASCII_code[4]&0x3f)<<8);
	Ship_Static_A_Msg.User_ID+=((isASCII_code[5]&0x3f)<<2);
	Ship_Static_A_Msg.User_ID+=((isASCII_code[6]&0x30)>>4);
	Ship_Static_B_Msg.User_ID=Ship_Static_A_Msg.User_ID;

	Ship_Static_A_Msg.Part_Number=((isASCII_code[6]&0x0c)>>2);

	if (Ship_Static_A_Msg.Part_Number==0)
	{
		memset((char *)&Ship_Static_B_Msg.Msg_ID,0,sizeof(Ship_Static_B_Msg));
		Ship_Static_A_Msg.Ship_name[19]+=((isASCII_code[6]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[19]+=((isASCII_code[7]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[18]+=((isASCII_code[7]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[18]+=((isASCII_code[8]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[17]+=((isASCII_code[8]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[17]+=((isASCII_code[9]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[16]+=((isASCII_code[9]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[16]+=((isASCII_code[10]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[15]+=((isASCII_code[10]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[15]+=((isASCII_code[11]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[14]+=((isASCII_code[11]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[14]+=((isASCII_code[12]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[13]+=((isASCII_code[12]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[13]+=((isASCII_code[13]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[12]+=((isASCII_code[13]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[12]+=((isASCII_code[14]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[11]+=((isASCII_code[14]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[11]+=((isASCII_code[15]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[10]+=((isASCII_code[15]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[10]+=((isASCII_code[16]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[9]+=((isASCII_code[16]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[9]+=((isASCII_code[17]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[8]+=((isASCII_code[17]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[8]+=((isASCII_code[18]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[7]+=((isASCII_code[18]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[7]+=((isASCII_code[19]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[6]+=((isASCII_code[19]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[6]+=((isASCII_code[20]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[5]+=((isASCII_code[20]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[5]+=((isASCII_code[21]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[4]+=((isASCII_code[21]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[4]+=((isASCII_code[22]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[3]+=((isASCII_code[22]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[3]+=((isASCII_code[23]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[2]+=((isASCII_code[23]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[2]+=((isASCII_code[24]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[1]+=((isASCII_code[24]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[1]+=((isASCII_code[25]&0x3c)>>2);
		Ship_Static_A_Msg.Ship_name[0]+=((isASCII_code[25]&0x03)<<4);
		Ship_Static_A_Msg.Ship_name[0]+=((isASCII_code[26]&0x3c)>>2);

	}
	else if (Ship_Static_A_Msg.Part_Number==1)
	{
		memset((char *)&Ship_Static_A_Msg.Msg_ID,0,sizeof(Ship_Static_A_Msg));
		Ship_Static_B_Msg.Part_Number=1;
		Ship_Static_B_Msg.Cargo_type+=((isASCII_code[6]&0x03)<<6);
		Ship_Static_B_Msg.Cargo_type+=isASCII_code[7]&0x3f;

		Ship_Static_B_Msg.Vendor_ID[6]=isASCII_code[8]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[5]=isASCII_code[9]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[4]=isASCII_code[10]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[3]=isASCII_code[11]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[2]=isASCII_code[12]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[1]=isASCII_code[13]&0x3f;
		Ship_Static_B_Msg.Vendor_ID[0]=isASCII_code[14]&0x3f;
	
		Ship_Static_B_Msg.Call_Sign[6]=isASCII_code[15]&0x3f;
		Ship_Static_B_Msg.Call_Sign[5]=isASCII_code[16]&0x3f;
		Ship_Static_B_Msg.Call_Sign[4]=isASCII_code[17]&0x3f;
		Ship_Static_B_Msg.Call_Sign[3]=isASCII_code[18]&0x3f;
		Ship_Static_B_Msg.Call_Sign[2]=isASCII_code[19]&0x3f;
		Ship_Static_B_Msg.Call_Sign[1]=isASCII_code[20]&0x3f;
		Ship_Static_B_Msg.Call_Sign[0]=isASCII_code[21]&0x3f;
		Ship_Static_B_Msg.Dimension+=((isASCII_code[22]&0x3f)<<24);
		Ship_Static_B_Msg.Dimension+=((isASCII_code[23]&0x3f)<<18);
		Ship_Static_B_Msg.Dimension+=((isASCII_code[24]&0x3f)<<12);
		Ship_Static_B_Msg.Dimension+=((isASCII_code[25]&0x3f)<<6);
		Ship_Static_B_Msg.Dimension+=isASCII_code[26]&0x3f;

		Ship_Static_B_Msg.Spare=isASCII_code[27]&0x3f;
	}
/*#ifdef  debug_print 
	printf("\n"); 
	printf("��Ϣʶ���룺%d\n",Ship_Static_A_Msg.Msg_ID);
	printf("ת��ָʾ����%d\n",Ship_Static_A_Msg.Rep_ID);
	printf("�û�ʶ���룺%d\n",Ship_Static_A_Msg.User_ID);
	printf("��Ϣ����ű�ʶ��%d\n",Ship_Static_A_Msg.Part_Number);
	printf("������");
	for (i=0;i<20;i++)
	{
		printf("%d ",Ship_Static_A_Msg.Ship_name[i]);
	}
	printf("\n"); 
	printf("�������������ͣ�%d\n",Ship_Static_B_Msg.Cargo_type);
	printf("��Ӧ�̣�");
	for (i=0;i<7;i++)
	{
		printf("%d ",Ship_Static_B_Msg.Vendor_ID[i]);
	}
	printf("\n"); 
	printf("���ţ�");
	for (i=0;i<7;i++)
	{
		printf("%d ",Ship_Static_B_Msg.Call_Sign[i]);
	}
	printf("\n"); 
	printf("�����ߴ缰��λ�豸λ�ã�%d\n",Ship_Static_B_Msg.Dimension);
	printf("���ã�%d\n",Ship_Static_B_Msg.Spare);
#endif*/
	return  ;
}

//�Ѽ���ȫ������Ϣ
void Collect_Safe_Ship_Num(void)
{
	uint8  k,m,n,count;
	count=0;

	for(k=0;k<AIS_Collision_Num;k++)
	{
		if(USV_Check_Collision[k].TCPA>AIS_TCPA_1)
		{
			Collision_Alternative[count]=k;
			count++;
		}
		if(count>0)
			break;
	}	
	if(count==0)
	{
		for(m=0;m<AIS_Collision_Num;m++)
		{
			if(USV_Check_Collision[m].TCPA>AIS_TCPA_2)
			{
				Collision_Alternative[count]=m;
				count++;
			}
			if(count>0)
				break;
		}
	}
	if(count==0)
	{
		for(n=0;n<AIS_Collision_Num;n++)
		{
			if(USV_Check_Collision[n].TCPA>AIS_TCPA_3)
			{
				Collision_Alternative[count]=n;
				count++;
			}
			if(count>0)
				break;
#ifdef  debug_print 
			else
				printf("USV��ǰλ��Σ�գ����ֶ�����!\n");
#endif
		}
	}
	return;
}
