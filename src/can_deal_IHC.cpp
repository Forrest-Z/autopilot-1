/*==========================================================*
 * ģ��˵��: can_deal_IHC.cpp                               *
 * �ļ��汾: v1.00 (˵�����ļ��İ汾��Ϣ)                   *
 * ������Ա:                                                *
 * ����ʱ��: 2018��2��2��14:45:00                           *
 * Copyright(c) sf-auto.ltd									*
 *==========================================================*
 * �����޸ļ�¼(���µķ�����ǰ��):                          *
 *  <�޸�����>, <�޸���Ա>: <�޸Ĺ��ܸ���>                  *
 *==========================================================*
 *=========================================================*/


/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/can_deal_IHC.h"
#include "../include/usv_include.h"
#include "../include/scu_io.h"
/******************************  Local Variable  *****************************/
IHC_REV_MSG		IHC_rev_msg		;
IHC_REV_CONFIG	IHC_rev_config	;
IHC_SEND_MSG	IHC_send_msg	;
IHC_ERR_CODE	IHC_err_code	;
COMM_SIGN		IHC_comm_sign = {0,0};
ERROR_MSG		IHC_err_msg		;		//������ԭʼ����
FAULT_CODE		IHC_fault_code	;		//������
struct scu_io_send_msg_t scu_io_send_msg;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void IHC_rvMsg_Init(void)
{
    memset(&IHC_rev_msg,0,sizeof(IHC_REV_MSG));
	memset(&IHC_err_msg, 0, sizeof(IHC_err_msg));
	memset(&IHC_fault_code, 0, sizeof(IHC_fault_code));
}

void IHC_config_Init(void)
{
    memset(&IHC_rev_config,0,sizeof(IHC_REV_CONFIG));
}

void IHC_sdMsg_Init(void)
{
    memset(&IHC_send_msg,0,sizeof(IHC_SEND_MSG));
	memset(&jet_system,0,sizeof(JET_SYSTEM));
	IHC_send_msg.b2_Cmd_Motor1OnOff			= & jet_system.jetL.b2_Cmd_MotorOnOff ;
	IHC_send_msg.b1_Cmd_Motor1EmergencyStop = & jet_system.b1_cmd_emergencyStop;
	IHC_send_msg.u8_Cmd_Motor1OpenDeg		= & jet_system.jetL.u8_Cmd_MotorOpenDeg;
	IHC_send_msg.i16_Cmd_Motor1GearDeg		= & jet_system.jetL.i16_Cmd_MotorGearDeg;
	IHC_send_msg.i16_Cmd_Motor1RudderDeg    = & jet_system.jetL.i16_Cmd_MotorRudderDeg;
	

	IHC_send_msg.b2_Cmd_Motor2OnOff			= & jet_system.jetR.b2_Cmd_MotorOnOff ;
	IHC_send_msg.b1_Cmd_Motor2EmergencyStop = & jet_system.b1_cmd_emergencyStop;
	IHC_send_msg.u8_Cmd_Motor2OpenDeg		= & jet_system.jetR.u8_Cmd_MotorOpenDeg;
	IHC_send_msg.i16_Cmd_Motor2GearDeg		= & jet_system.jetR.i16_Cmd_MotorGearDeg;
	IHC_send_msg.i16_Cmd_Motor2RudderDeg    = & jet_system.jetR.i16_Cmd_MotorRudderDeg;

	IHC_send_msg.b1_Cmd_elecWindless1OnOff	= & command_signal.equpment_cmd.b1_elecWindlass1OnOff	;
	IHC_send_msg.b1_Cmd_elecWindless1UpDown	= & command_signal.equpment_cmd.b1_elecWindlass1UpDown	;
	IHC_send_msg.b1_Cmd_elecWindless2OnOff	= & command_signal.equpment_cmd.b1_elecWindlass2OnOff	;
	IHC_send_msg.b1_Cmd_elecWindless2UpDown	= & command_signal.equpment_cmd.b1_elecWindlass2UpDown	;
	IHC_send_msg.b1_Cmd_sprayStrip1OnOff	= & command_signal.equpment_cmd.b1_sprayStrip1OnOff		;
	IHC_send_msg.b1_Cmd_sprayStrip1UpDown	= & command_signal.equpment_cmd.b1_sprayStrip1UpDown	;
	IHC_send_msg.b1_Cmd_sprayStrip2OnOff	= & command_signal.equpment_cmd.b1_sprayStrip2OnOff		;
	IHC_send_msg.b1_Cmd_sprayStrip2UpDown	= & command_signal.equpment_cmd.b1_sprayStrip2UpDown	;

}

void IHC_Err_Init( void )
{
	memset(&IHC_err_code,0,sizeof(IHC_ERR_CODE));
}


void IHC_Init(void)
{
    IHC_rvMsg_Init();
    IHC_config_Init();
    IHC_sdMsg_Init();
	IHC_Err_Init();
    comm_time_return(&(IHC_comm_sign.timer),&(IHC_comm_sign.comm_sign));
}

//ͳ��ͨѶ��ʱ
void IHC_CommCal(void)
{
    comm_time_cal(CAN_IHC_DISCONNECT_MAX,&(IHC_comm_sign.timer),&(IHC_comm_sign.comm_sign));
}

//
extern void IHC_recv( uint8 psID,uint8* data )
{
	float calc_tmp;
	if(0 == psID)
	{
		IHC_rev_msg.b1_St_Motor1OnOff         = (data[0]&0x01)			;
		IHC_rev_msg.b1_St_Motor1EmergencyStop = (data[0]&0x02)>>1		;
		IHC_rev_msg.u16_St_Motor1Rpm          =  u8tou16(&data[1])		;
		IHC_rev_msg.i16_St_Motor1Gear         =	 u8toi16(&data[3])		;
		IHC_rev_msg.i16_St_Motor1Rudder       =	 u8toi16(&data[5])		;
		IHC_rev_msg.b1_water_level            = data[7];

		//����ת��
		//IHC_rev_msg.u16_St_Motor1Rpm >>=2;	
		//�����ƶ���
		//IHC_rev_msg.i16_St_Motor1Gear >>=4;
		//�������
		calc_tmp = IHC_rev_msg.i16_St_Motor1Rudder;
		IHC_rev_msg.i16_St_Motor1Rudder = (int16)(calc_tmp * 10);
	}
	else if(1 == psID)
	{
		IHC_rev_msg.b1_St_Motor2OnOff         = (data[0]&0x01)			;
		IHC_rev_msg.b1_St_Motor2EmergencyStop = (data[0]&0x02)>>1		;
		IHC_rev_msg.u16_St_Motor2Rpm          =  u8tou16(&data[1])		;
		IHC_rev_msg.i16_St_Motor2Gear         =	 u8toi16(&data[3])		;
		IHC_rev_msg.i16_St_Motor2Rudder       =	 u8toi16(&data[5])		;

		//����ת��
	//	IHC_rev_msg.u16_St_Motor2Rpm >>=2;	
		//�����ƶ���
	//	IHC_rev_msg.i16_St_Motor2Gear >>=4;
		//�������
		calc_tmp = IHC_rev_msg.i16_St_Motor2Rudder;
		IHC_rev_msg.i16_St_Motor2Rudder = (int16)(calc_tmp * 10);

#ifdef OUTBOARD_SINGLE_ENGINE
		IHC_rev_msg.mid_st.b1_St_MotorOn = IHC_rev_msg.b1_St_Motor1OnOff ;
#elif defined  TIANJI_SIGLE_ENGINE
	IHC_rev_msg.mid_st.b1_St_MotorOn = IHC_rev_msg.b1_St_Motor1OnOff ;
#else
		IHC_rev_msg.mid_st.b1_St_MotorOn = IHC_rev_msg.b1_St_Motor2OnOff & IHC_rev_msg.b1_St_Motor1OnOff	;	//�����������ɹ�
#endif
	}
	else if(2 == psID)
	{
		IHC_rev_msg.b1_St_elecWindlass1OnOff =  (data[0]&0x01)			;
		IHC_rev_msg.b1_St_elecWindlass2OnOff =  (data[0]&0x02)>>1		;
		IHC_rev_msg.b1_St_sprayStrip1OnOff	 =  (data[0]&0x04)>>2		;
		IHC_rev_msg.b1_St_sprayStrip2OnOff	 =  (data[0]&0x08)>>3		;

		//IHC_rev_msg.b1_Wn_ClassIWarn	 =  (data[1]&0x01)			;
		//IHC_rev_msg.b1_Wn_mainBoardPower =  (data[1]&0x02)>>1		;
		//IHC_rev_msg.b1_Wn_mainBoardTempe =  (data[1]&0x04)>>2		;
		//IHC_rev_msg.b1_Wn_SPI			 =  (data[1]&0x08)>>3		;
		//IHC_rev_msg.b1_Wn_TL16C554_1Comm =  (data[1]&0x10)>>4		;
		//IHC_rev_msg.b1_Wn_TL16C554_2Comm =  (data[1]&0x20)>>5		;
		//IHC_rev_msg.b1_Wn_TL16C554_3Comm =  (data[1]&0x40)>>6		;
		//IHC_rev_msg.b1_Wn_TL16C554_4Comm =  (data[1]&0x80)>>7		;

		//IHC_rev_msg.b1_Wn_ClassIIWarn	 =  (data[2]&0x01)			;
		//IHC_rev_msg.b1_Wn_PORTMotorWarn	 =  (data[2]&0x02)>>1		;
		//IHC_rev_msg.b1_Wn_STBDMotorWarn	 =  (data[2]&0x04)>>2		;
		//IHC_rev_msg.b1_Wn_PORTGearWarn	 =  (data[2]&0x08)>>3		;
		//IHC_rev_msg.b1_Wn_PORTRudderWarn =  (data[2]&0x10)>>4		;
		//IHC_rev_msg.b1_Wn_STBDGearWarn	 =  (data[2]&0x20)>>5		;
		//IHC_rev_msg.b1_Wn_STBDRudderWarn =  (data[2]&0x40)>>6		;

		

		//IHC_rev_msg.u8_St_PORTBatLvl = data[7];			
		//IHC_rev_msg.u8_St_STBDBatLvl = data[7];			


		//IHC_rev_msg.mid_st.b1_Wn_CommonWarn = IHC_rev_msg.b1_Wn_ClassIWarn | IHC_rev_msg.b1_Wn_ClassIIWarn	;
	}
	else if (3 == psID)
	{
		IHC_rev_msg.u16_St_BatteryVoltage	  = u8tou16(&data[1]);
		IHC_rev_msg.u16_St_BatteryCurrent	  = u8tou16(&data[3]);
		IHC_rev_msg.u8_St_BatteryLevel		  = data[0];
		IHC_rev_msg.u8_St_BatteryRemainTime_H = data[5];
		IHC_rev_msg.u8_St_BatteryRemainTime_M = data[6];
		IHC_rev_msg.u8_St_BatteryRemainTime_S = data[7];

	}
	else if(18 == psID)
	{
		IHC_rev_msg.mid_st.b1_St_Authority = data[0];
	}
	else if(128 == psID)
	{
		IHC_rev_config.u16_version	=	u8tou16(&(data[0]));
		IHC_rev_config.u16_svn		=	u8tou16(&(data[2]));
		IHC_rev_config.u16_crc		=	u8tou16(&(data[4]));
	}
	else if(129 == psID)
	{
		IHC_rev_config.u8_confState = (data[0]&0x01)		;
		IHC_rev_config.u8_initState = (data[1]&0x02)>>1		;
	}
	else if(130 == psID)
	{
		switch (data[0])
		{
		case 0:
			IHC_err_code.u32_Wn_driver1ErrCode = u8tou32(&(data[4]));
			break;
		case 1:
			IHC_err_code.u32_Wn_driver2ErrCode = u8tou32(&(data[4]));
			break;
		case 2:
			IHC_err_code.u32_Wn_driver3ErrCode = u8tou32(&(data[4]));
			break;
		case 3:
			IHC_err_code.u32_Wn_driver4ErrCode = u8tou32(&(data[4]));
			break;
		default:
			break;
		}		
	}
	else if(131 == psID)
	{
		switch (data[0])
		{
		case 0:
			IHC_err_code.u32_Wn_Motor1ErrCode = u8tou32(&(data[4]));
			break;
		case 1:
			IHC_err_code.u32_Wn_Motor2ErrCode = u8tou32(&(data[4]));
			break;
		default:
			break;
		}
	}
	else if(144 == psID){ 
		uint8	u8_faultSrc = data[0];
		uint8	u8_faultLvl = data[1];
		uint8	u8_faultSel = data[2];
	//	SysLogMsgPost("[IHC������] ��Դ:%d  ����:%d  ����:%d",u8_faultSrc,u8_faultLvl,u8_faultSel);
	//	SysPubMsgPost("[IHC������] ��Դ:%d  ����:%d  ����:%d",u8_faultSrc,u8_faultLvl,u8_faultSel);

		memcpy((char *)&IHC_err_msg.e_data[0], (char *)&data[0], 8);
		IHC_err_msg.e_flag = 1;

		IHC_fault_code.u16_errorSq = u8tou16(&(data[0]));
		IHC_fault_code.u8_errorStat = data[2];

		if (IHC_fault_code.u16_errorSq == 17)
		{
			//IHC_rev_msg.mid_st.b1_St_Authority = IHC_fault_code.u8_errorStat;
		}
		if (poweron_init)
		{
			WarnMsgQueuePut(WARN_SRC_IHC, IHC_fault_code.u16_errorSq, IHC_fault_code.u8_errorStat);
		}

	}
	//printf("IHC_rev_msg.u16_St_Motor2Rpm == %d\n", IHC_rev_msg.u16_St_Motor2Rpm);
	comm_time_return(&(IHC_comm_sign.timer),&(IHC_comm_sign.comm_sign));
}

void IHC_sendMsg( void )
{
	uint8 pData[8];
	#if 0
	int16 i16_rudder1,i16_rudder2,i16_gear1,i16_gear2;		//����Ҫ��ͬ�����˴����ӻ���
	uint8 u8_openDeg1,u8_openDeg2;
	i16_rudder1 = *IHC_send_msg.i16_Cmd_Motor1RudderDeg;
	i16_rudder2 = *IHC_send_msg.i16_Cmd_Motor2RudderDeg;
	i16_gear1   = *IHC_send_msg.i16_Cmd_Motor1GearDeg;
	i16_gear2	= *IHC_send_msg.i16_Cmd_Motor2GearDeg;
	u8_openDeg1 = *IHC_send_msg.u8_Cmd_Motor1OpenDeg;
	u8_openDeg2 = *IHC_send_msg.u8_Cmd_Motor2OpenDeg;

	//PS10
	memset(pData,0,8);
	pData[0]  = (*IHC_send_msg.b2_Cmd_Motor1OnOff			& 0x03)		;
	pData[0] |= (*IHC_send_msg.b1_Cmd_Motor1EmergencyStop	& 0x01)<<2	;
	pData[1]  =  u8_openDeg1							;
	memcpy(&(pData[2]),(uint8 *)(&i16_gear1),2);
	memcpy(&(pData[4]),(uint8 *)(&i16_rudder1),2);


	sendCanMsg(can_0,255,10,128,pData);

	usleep(500);//0.5ms	
	//PS11
	memset(pData,0,8);
	pData[0]  = (*IHC_send_msg.b2_Cmd_Motor2OnOff			& 0x03)		;
	pData[0] |= (*IHC_send_msg.b1_Cmd_Motor2EmergencyStop	& 0x01)<<2	;
	pData[1]  =  u8_openDeg2											;
	memcpy(&(pData[2]),(uint8 *)(&i16_gear2),2);
	memcpy(&(pData[4]),(uint8 *)(&i16_rudder2),2);

	
	sendCanMsg(can_0,255,11,128,pData);

	usleep(500);
	//PS12
	memset(pData,0,8);
	pData[0]  = (*IHC_send_msg.b1_Cmd_elecWindless1OnOff	  &0x01  )      ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_elecWindless1UpDown	  &0x01  )<<1   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_elecWindless2OnOff	  &0x01  )<<2   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_elecWindless2UpDown	  &0x01  )<<3   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_sprayStrip1OnOff	  &0x01  )<<4   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_sprayStrip1UpDown	  &0x01  )<<5   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_sprayStrip2OnOff	  &0x01  )<<6   ;
	pData[0] |= (*IHC_send_msg.b1_Cmd_sprayStrip2UpDown	  &0x01  )<<7   ;

	sendCanMsg(can_0,255,12,128,pData);
	usleep(500);//0.5ms	

	//PS13
	//����can��������
	memset(pData, 0, 8);
	memcpy(&pData[0], &docking_control_cmd, sizeof(docking_control_cmd));
	if (ins_msg.insState.c_rmcValid == 'A') //��λ��Ч
	{
		sendCanMsg(can_0, 255, 13, 128, pData);
		usleep(500);//0.5ms	
		//printf("can_cmd     == %d\n", docking_control_cmd.cmd_state);
		//printf("can_vx      == %d\n", docking_control_cmd.vx);
		//printf("can_heading == %d\n", docking_control_cmd.heading);
		//printf("can_wr	  == %d\n", docking_control_cmd.wr);
	}
#endif

#if 1
	//PS14
	scu_io_send_msg._armed = (*IHC_send_msg.b2_Cmd_Motor1OnOff			& 0x03)	;
	scu_io_send_msg._ems   = (*IHC_send_msg.b1_Cmd_Motor1EmergencyStop	& 0x01);
	scu_io_send_msg._forward = (*IHC_send_msg.u8_Cmd_Motor1OpenDeg)/255.0f * 1000;
	scu_io_send_msg._forward = constrain_value(scu_io_send_msg._forward,-1000,+1000);

	int16 ret  = ((*IHC_send_msg.i16_Cmd_Motor1GearDeg) < 0)?(-1):(1);
	if(*IHC_send_msg.i16_Cmd_Motor1GearDeg == 0) ret = 0;
	scu_io_send_msg._forward  = ret * scu_io_send_msg._forward;
	scu_io_send_msg._lateral = 0;
	scu_io_send_msg._yaw = (*IHC_send_msg.i16_Cmd_Motor1RudderDeg)/255.0f * 1000;

	memset(pData,0,8);


	// if(docking_control_cmd.cmd_state == 1)
	// {
	// 	int16 tmp_vx;
	// 	int16 tmp_heading;
	// 	pData[0] = scu_io_send_msg._armed | scu_io_send_msg._ems << 2 | CTL_ATT << 3 | MIXER_DIFF_SPEED << 6;
	// 	tmp_vx = docking_control_cmd.vx*1000;
	// 	tmp_heading = wrap_PI(deg2rad(docking_control_cmd.heading))/Pi*1000; // -1 1
	// 	memcpy(&(pData[1]),(uint8 *)(&tmp_vx),2);
	// 	memcpy(&(pData[5]),(uint8 *)(&tmp_heading),2);

	// }
	//else
	//{
		pData[0] = scu_io_send_msg._armed | scu_io_send_msg._ems << 2 | CTL_MUAL << 3 | MIXER_DIFF_SPEED << 6;
		memcpy(&(pData[1]),(uint8 *)(&scu_io_send_msg._forward),2);
		memcpy(&(pData[3]),(uint8 *)(&scu_io_send_msg._lateral),2);
		memcpy(&(pData[5]),(uint8 *)(&scu_io_send_msg._yaw),2);
	//}
	
	pData[7] = 0;

	sendCanMsg(can_0,255,14,128,pData);
	//usleep(500);//0.5ms	
	

	// PS15 
	int32 speed     =   (ins_msg.speed * (double)(1000000));      // kn
	int32 gama = (ins_msg.motionDirection * (double)(1000000));   // deg
	memset(pData,0,8);
	memcpy(&(pData[0]),(uint8 *)(&speed),4);
	memcpy(&(pData[4]),(uint8 *)(&gama),4);
	sendCanMsg(can_0,255,15,128,pData);
	//printf("speed = %d\n",speed);
	//printf("ins_speed = %f\n",ins_msg.speed);


	// PS16
	int32 heading   =   (ins_msg.heading * (double)(1000000));    // deg
	int32 heading_rate = (ins_msg.rotRate * (double)(1000000));   // deg/s
	memset(pData,0,8);
	memcpy(&(pData[0]),(uint8 *)(&heading),4);
	memcpy(&(pData[4]),(uint8 *)(&heading_rate),4);

	//printf("heading = %d\n",heading);
	//printf("ins_heading = %f\n",ins_msg.heading);

	sendCanMsg(can_0,255,16,128,pData);
#endif


	//PGN FF85
	//�ٻ����ϴ���

	//PS144
	//������ظ�
	if (IHC_err_msg.e_flag == 1)
	{
		IHC_err_msg.e_flag = 0;
		sendCanMsg(can_0, 255, 144, 128, IHC_err_msg.e_data);
		usleep(500);//0.5ms	
	}

	//���ͨѶ�жϺ�������֡
	if(IHC_comm_sign.comm_sign == COMM_CONNECT_FAIL){
		callAllStart();
		//printf("send IHC can start frame\n");
	}
}

void IHC_call_driverErrCode( uint8 driverNum )
{
	uint8 pData[8];
	if(driverNum>3)
	{
		return;
	}
	memset(pData,0,8);
	pData[0] = 1;
	pData[1] = driverNum;
	sendCanMsg(can_0,255,133,128,pData);
}

void IHC_call_motorErrCode( uint8 motorNum)
{
	uint8 pData[8];
	if(motorNum>1)
	{
		return;
	}
	memset(pData,0,8);
	pData[2] = 1;
	pData[3] = motorNum;
	sendCanMsg(can_0,255,133,128,pData);
}

void initIHCsendTask( void )
{
	IHC_sdMsg_Init();
	addTask(2,runIHCsendTask,(void*)0);
}

void runIHCsendTask(void *)
{
	IHC_sendMsg();
	monitor_all_inf.monitor_comm_inf[MONITOR_COMM_IHC_SN].send_ok_number++;	//������ȷ����
}









