/*
* can_ctrl.c --CAN�߳�
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-02-24������������д
*/
#include "stdafx.h"

#include "../include/usv_include.h"

MONITOR_ALL_INF_STRUCT			monitor_all_inf;		//����ͨѶ��ȫ����Ϣ
SHIP_SIMULATION_PARAM_STRUCT	ship_simulate_param;	//���洬�Ĳ���
uint8	rec_report_buff[310];							//���ջ�������������3֡��ֵ	

//���ر�����
const uint8 UP_SETTING_PID_ERR1[]={"����PID����֡��ų���"};
const uint8 UP_SETTING_PID_ERR2[]={"����PID����У��ͳ���"};
const uint8 UP_SETTING_PID_ERR3[]={"����PID����д�ļ�����"};
const uint8 UP_SETTING_PID_OK[]={"����PID������ȷ��"};

const uint8 UP_SETTING_S_FACE_ERR1[]={"����S�����֡��ų���"};
const uint8 UP_SETTING_S_FACE_ERR2[]={"����S�����У��ͳ���"};
const uint8 UP_SETTING_S_FACE_ERR3[]={"����S�����д�ļ�����"};
const uint8 UP_SETTING_S_FACE_OK[]={"����S�������ȷ��"};

const uint8 UP_SETTING_TRANS_ERR1[]={"����ƽ�Ʋ���֡��ų���"};
const uint8 UP_SETTING_TRANS_ERR2[]={"����ƽ�Ʋ���У��ͳ���"};
const uint8 UP_SETTING_TRANS_ERR3[]={"����ƽ�Ʋ���д�ļ�����"};
const uint8 UP_SETTING_TRANS_OK[]={"����ƽ�Ʋ�����ȷ��"};

const uint8 UP_SETTING_ROTATE_ERR1[]={"������ת����֡��ų���"};
const uint8 UP_SETTING_ROTATE_ERR2[]={"������ת����У��ͳ���"};
const uint8 UP_SETTING_ROTATE_ERR3[]={"������ת����д�ļ�����"};
const uint8 UP_SETTING_ROTATE_OK[]={"������ת������ȷ��"};

const uint8 UP_SETTING_SHIP_ROUTE_ERR1[]={"���غ���֡��ų���"};
const uint8 UP_SETTING_SHIP_ROUTE_ERR2[]={"���غ���У��ͳ���"};
const uint8 UP_SETTING_SHIP_ROUTE_ERR3[]={"���غ���д�ļ�����"};
const uint8 UP_SETTING_SHIP_ROUTE_OK[]={"���غ�����ȷ��"};

const uint8 UP_SETTING_OBSTACLE_ERR1[]={"���ر��ϵ�֡��ų���"};
const uint8 UP_SETTING_OBSTACLE_ERR2[]={"���ر��ϵ�У��ͳ���"};
const uint8 UP_SETTING_OBSTACLE_ERR3[]={"���ر��ϵ�д�ļ�����"};
const uint8 UP_SETTING_OBSTACLE_OK[]={"���ر��ϵ���ȷ��"};

const uint8 UP_MMI_ERR1[]={"����MMI��������"};


	

// ########################################################
//			��߷��ͳ�����
// ########################################################
//��д����������
void input_ask_report(uint8 *content,uint8 error_flag,uint8 report_type,uint8 sub_type,uint8 func)
{
uint8	report_len;

	monitor_ask_operation.send_flag =SWITCH_ON;			//�÷��ͱ�־
	if(error_flag==SWITCH_ON){
		monitor_ask_operation.error_flag=SWITCH_ON;
	}
	else{
		monitor_ask_operation.error_flag=0;
	}
	monitor_ask_operation.report_type=report_type;
	monitor_ask_operation.sub_type=sub_type;
	monitor_ask_operation.func=func;
	report_len=strlen((char *)content);
	if(report_len>=50){
		report_len =49; 
	}
	monitor_ask_operation.buff_len=report_len;
	memcpy((uint8*) &monitor_ask_operation.buff[0],content,50);
	monitor_ask_operation.buff[49]=0;

}

//���Ͳ������ر���
uint16 send_monitor_ask(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=54+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;			//��������
	if(monitor_ask_operation.error_flag==SWITCH_ON){						//������ȷ
		p_udp_monitor_report->func =DOWN_FUNC_OPERATE_ERR;					//������
	}
	else{						//������ȷ
		p_udp_monitor_report->func =DOWN_FUNC_OPERATE_OK;					//������
	}
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	

	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��д������Ϣ
	*p_uint8 =monitor_ask_operation.report_type;
	p_uint8++;
	*p_uint8 =monitor_ask_operation.sub_type;
	p_uint8++;
	*p_uint8 =monitor_ask_operation.func;
	p_uint8++;
	*p_uint8 =monitor_ask_operation.buff_len;
	p_uint8++;
	memcpy(p_uint8,&monitor_ask_operation.buff[0],50);
	p_uint8 =p_uint8 +50;
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//��дͨѶ��Ϣ
uint16	monitor_send_input_comm(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint16	*p_uint16;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
uint8	loop_i;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_COMM_MONITOR;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��

	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	for(loop_i=0;loop_i<5;loop_i++){
		p_uint32 = (uint32 *)&monitor_all_inf.monitor_comm_inf[current_ID].send_ok_number ;
		memcpy_32((uint32*)p_uint8,p_uint32,4);			//ȡ4��32λ
		p_uint8=p_uint8 +4*4;
		p_uint16 =(uint16 *)&monitor_all_inf.monitor_comm_inf[current_ID].send_err_ID;
		memcpy_16((uint16*)p_uint8,p_uint16,2);			//ȡ2��16λ
		current_ID++;
		p_uint8=p_uint8 +2*2;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//��д�澯��Ϣ
uint16	monitor_send_input_alarm(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=32+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_ALARM;					//������
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��

	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.alm_state[0];
	memcpy_32((uint32*)p_uint8,p_uint32,MONITOR_USV_ALM_NUMBER);			//ȡ8��32λ
	p_uint8 =p_uint8 +4*MONITOR_USV_ALM_NUMBER;
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//��д������Ϣ
uint16	monitor_send_input_test_inf(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_TEST;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*25;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.tmp_data[current_ID] ;
	memcpy_32((uint32*)p_uint8,p_uint32,25);			//ȡ4��32λ
	p_uint8=p_uint8 +25*4;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}


//��д���ó�����Ϣ
uint16	monitor_send_input_cfg_error_inf(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	if(monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number>MONITOR_MAX_CFG_ERROR_ITEM+1){
		monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number=MONITOR_MAX_CFG_ERROR_ITEM+1;
	}
	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT)+6+2;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_CFG_ERR_INF;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq;					
	if((monitor_udp_check_send_report.send_report_seq+1)>=monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	

	if(monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number==0){		//û�г�����Ϣ
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
		p_udp_monitor_report->report_len=6+2;
		*p_uint8=0;		//������
		p_uint8++;
		*p_uint8=0;				//��ǰ����
		p_uint8++;
	}
	else{
		*p_uint8=monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number;		//������
		p_uint8++;
		*p_uint8=current_ID;				//��ǰ����
		p_uint8++;
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
		p_uint32 = (uint32 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[current_ID];
		memcpy(p_uint8,(uint8 *)p_uint32,sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT));
		p_uint8=p_uint8 +sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT);
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//��Ҫʵʱ��������
uint16	monitor_send_input_main_run_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_MAIN_PARAM;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*6;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_main_real_data[current_ID].expect_data ;
	memcpy_32((uint32*)p_uint8,p_uint32,24);			//ȡ4��32λ
	p_uint8=p_uint8 +6*sizeof(MONITOR_MAIN_REAL_DATA_STRUCT);

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//������������
uint16	monitor_send_input_other_run_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_OTHER_PARAM;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*25;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_sub_real_data[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			//ȡ4��32λ
	p_uint8=p_uint8 +25*sizeof(uint32);

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//����״̬����
uint16	monitor_send_input_run_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_RUN_STATE;					//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*100;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.usv_equ_state[current_ID];
	memcpy((uint8*)p_uint8,(uint8 *)p_uint32,100);			
	p_uint8=p_uint8 +100;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}




//����ʵʱ����
uint16	monitor_send_input_curve_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_CURVE;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_CURVE_REAL_DATA;					//������
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���100��byte
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_curve_real_data;
	memcpy_32((uint32*)p_uint8,p_uint32,25);					
	p_uint8=p_uint8 +100;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//��ģ�鱨�ļ���
//sub_equ_ID--�豸���
uint16	monitor_send_input_sub_equ_report(uint8 *report_buff,uint8 sub_equ_ID)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
uint8	report_NO;

	if(sub_equ_ID<DOWN_FUNC_STABLE_PLAT){
		clear_monitor_udp_life_time();
		return apdu_len;
	}
	if(sub_equ_ID>DOWN_FUNC_BRA){
		clear_monitor_udp_life_time();
		return apdu_len;
	}
	current_ID=sub_equ_ID-DOWN_FUNC_STABLE_PLAT;
	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_MODEL_REPORT;						//��������
	p_udp_monitor_report->func =sub_equ_ID;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	report_NO=monitor_udp_check_send_report.send_report_seq*100;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.module_report_inf[current_ID].report_detail[report_NO] ;
	memcpy((uint8*)p_uint8,(uint8 *)p_uint32,100);			
	p_uint8=p_uint8 +100;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}

//PID�����趨
//���Ͳ���
uint16	monitor_send_input_PID_param(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_PID_PARAM;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*8;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;

	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[0], 24*4);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}


//S������趨
//���Ͳ���
uint16	monitor_send_input_S_face_param(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_S_FACE_PARAM;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*8;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;

	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)p_udp_monitor_report->buff[0], 24*4);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//ƽ�Ʋ����趨
//���Ͳ���
 uint16 monitor_send_input_trans_param(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type = UP_TYPE_ARM_2_PC_REPORT;		//��������
	p_udp_monitor_report->report_len   = 96+6;
	p_udp_monitor_report->sub_type    = DOWN_SUB_ASK_SETTING_PARAM;		//��������
	p_udp_monitor_report->func          = DOWN_FUNC_TRANS_PARAM;			//������
	p_udp_monitor_report->socket_ID_l =  0;
	p_udp_monitor_report->socket_ID_h = 0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;				//ARMģ��
	p_uint8 = (uint8 *)&p_udp_monitor_report->buff[0];
	
	//��дʵʱ����
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);
	p_uint8 = p_uint8 + 96;

	//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;	
}

//��ת�����趨
//���Ͳ���
 uint16 monitor_send_input_rotate_param(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type = UP_TYPE_ARM_2_PC_REPORT;		//��������
	p_udp_monitor_report->report_len   = 96+6;
	p_udp_monitor_report->sub_type    = DOWN_SUB_ASK_SETTING_PARAM;		//��������
	p_udp_monitor_report->func          = DOWN_FUNC_ROTATE_PARAM;			//������
	p_udp_monitor_report->socket_ID_l =  0;
	p_udp_monitor_report->socket_ID_h = 0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;				//ARMģ��
	p_uint8 = (uint8 *)&p_udp_monitor_report->buff[0];

	//��дʵʱ����
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);
	p_uint8 = p_uint8 + 96;

	//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;	
	
}


//�����趨
//���Ͳ���
uint16	monitor_send_input_ship_route(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SHIP_ROUTE;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;			//ÿ֡5������
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=2){				//��֡Ϊ����֡ ����3֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//�����ܸ���
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;		//�ܺ�����
	p_uint8++;
	if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_number>=current_ID+5){
		*p_uint8=5;
	}
	else if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_number>current_ID){
		*p_uint8=monitor_all_inf.rec_monitor_all_set_param.ship_stop_number-current_ID;
	}
	else{
		*p_uint8 =0;
	}
	p_uint8++;
//��֡�������
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}

//�ϰ����趨
//���Ͳ���
uint16	monitor_send_input_obstacle(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_OBSTACLE_LOCATE;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡ 2
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//�����ܸ���
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.obstacle_number;		//�ܺ�����
	p_uint8++;
	if(monitor_all_inf.rec_monitor_all_set_param.obstacle_number>=current_ID+5){
		*p_uint8=5;
	}
	else if(monitor_all_inf.rec_monitor_all_set_param.obstacle_number>current_ID){
		*p_uint8=monitor_all_inf.rec_monitor_all_set_param.obstacle_number-current_ID;
	}
	else{
		*p_uint8 =0;
	}
	p_uint8++;
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}


//�����С��Ϣ
//���Ͳ���
uint16	monitor_send_input_lake_large_inf(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_LAKE_LARGE_INF;										//������
	p_udp_monitor_report->socket_ID_l=0;	
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
		
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//�����ܸ���
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.lake_point_number;		//�ܺ������
	p_uint8++;
	*p_uint8=12;				//��֡����
	p_uint8++;
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.lake_point[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//���洬�ĳ�ʼ״̬
uint16	monitor_send_input_ship_start_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=16*4+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SIM_SHIP_START_STATE;					//������
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
	p_uint8_src =(uint8 *)&ship_simulate_param.monitor_ship_run_inf.ship_init_state.Intellingent_Switch;
	memcpy(p_uint8,p_uint8_src,4);
	p_uint8 =p_uint8 +4;
	p_uint8_src =(uint8 *)&ship_simulate_param.monitor_ship_run_inf.ship_init_state.ship_locate;
	memcpy_32((uint32 *)p_uint8,(uint32 *)p_uint8_src,15);

	p_uint8=p_uint8 +15*4;
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//���洬�ĺ�������
uint16	monitor_send_input_simulate_route(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SIM_ROUTE;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;			//ÿ֡5������
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=2){				//��֡Ϊ����֡ ����3֡
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//�����ܸ���
	*p_uint8 =ship_simulate_param.monitor_ship_run_inf.stop_number;		//�ܺ�����
	p_uint8++;
	if(ship_simulate_param.monitor_ship_run_inf.stop_number>=current_ID+5){
		*p_uint8=5;
	}
	else if(ship_simulate_param.monitor_ship_run_inf.stop_number>current_ID){
		*p_uint8=ship_simulate_param.monitor_ship_run_inf.stop_number-current_ID;
	}
	else{
		*p_uint8 =0;
	}
	p_uint8++;
//��֡�������
//��дʵʱ���ݣ�ÿ�η���6���ṹ��
	p_uint32 = (uint32 *)&ship_simulate_param.monitor_ship_run_inf.ship_stop_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)p_udp_monitor_report->buff[2], 100);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}
//���洬���ϰ�������
uint16	monitor_send_input_simulate_obstacle(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;
static  uint8 all_sum=0;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SIM_OBSTACLE;										//������
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//��ʼ֡
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//��֡Ϊ����֡ 2
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//�����ܸ���
	*p_uint8 =ship_simulate_param.monitor_ship_run_inf.obstacle_number;		//�ܺ�����
	p_uint8++;
	if(ship_simulate_param.monitor_ship_run_inf.obstacle_number>=current_ID+5){
		*p_uint8=5;
	}
	else if(ship_simulate_param.monitor_ship_run_inf.obstacle_number>current_ID){
		*p_uint8=ship_simulate_param.monitor_ship_run_inf.obstacle_number-current_ID;
	}
	else{
		*p_uint8 =0;
	}
	p_uint8++;
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	p_uint32 = (uint32 *)&ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//���һ֡
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
	//	p_udp_monitor_report->report_len ++;
	}
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//���洬��ʵʱ����ֵ
uint16	monitor_send_input_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=20*4+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SIM_INPUT_REAL_DATA;					//������
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
	p_uint8_src =(uint8 *)&ship_simulate_param.ship_adj_output_param.Intellingent_Switch;
	memcpy(p_uint8,p_uint8_src,4);
	p_uint8 =p_uint8 +4;
	p_uint8_src =(uint8 *)&ship_simulate_param.ship_adj_output_param.Accelerator_L;
	memcpy_32((uint32 *)p_uint8,(uint32 *)p_uint8_src,19);

	p_uint8=p_uint8 +19*4;
//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//������嵱ǰ״̬
uint16	monitor_send_input_MMI_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//��������
	p_udp_monitor_report->report_len=33+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//��������
	p_udp_monitor_report->func =DOWN_FUNC_SHIP_ROUTE;										//������
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARMģ��	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//��дʵʱ���ݣ�ÿ�η���5���ṹ��
	*p_uint8=monitor_all_inf.rec_pc_simulate_mmi.mmi_state;
	p_uint8++;
	p_uint8_src = (uint8 *)&monitor_all_inf.rec_pc_simulate_mmi.mmi_can_report[0].rec_mmi_buff[0];
	memcpy(p_uint8,p_uint8_src,32);			
	p_uint8=p_uint8 +32;

//���㱨��У���
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}


// ########################################################
//			��߽��մ��������
// ########################################################
//PID�����趨
//���в���
void	monitor_rec_deal_PID_param(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint16	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

//��ʼ֡	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8 *)&UP_SETTING_PID_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
		return;
	}
	current_ID=p_udp_monitor_report->socket_ID_l *96;
	for(loop_i=0;loop_i<96;loop_i++){							//�������ݻ�����
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i];
	}
	p_uint8 =&p_udp_monitor_report->buff[96];
	if(p_udp_monitor_report->socket_ID_h==0x80){			//Ϊĩβ֡,�������ȷ����
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<96*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//У��Ͳ���
		//	input_ask_report((uint8 *)&UP_SETTING_PID_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
		//	return;
		//}
	}
    //	monitor_all_inf.rec_monitor_all_set_param.PID_number= 16;
	monitor_all_inf.rec_monitor_all_set_param.PID_number = MAX_PID_GROUP_NUMBER;
//�޸Ľṹ��
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,48);
	if(write_setting_file()==TRUE){
		input_ask_report((uint8 *)&UP_SETTING_PID_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_PID_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
	}
	//PID�����趨������ϵͳ��
	{
		Dradio_Config.Algorithm_Cfg[0].PID_P = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].P *100.0);
		Dradio_Config.Algorithm_Cfg[0].PID_I = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].I *100.0);
		Dradio_Config.Algorithm_Cfg[0].PID_D = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].D *100.0);

		Dradio_Config.Algorithm_Cfg[1].PID_P = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].P *100.0);
		Dradio_Config.Algorithm_Cfg[1].PID_I = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].I *100.0);
		Dradio_Config.Algorithm_Cfg[1].PID_D = int32(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].D *100.0);
	}

	pPidAutoSpeed->setPidCoff(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].P,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].D);

	pPidHeading->setPidCoff(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].P,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].D);

	pPidRot->setPidCoff(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].P,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].D);
}

//S������趨
void	monitor_rec_deal_S_face_param(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint16	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

//��ʼ֡	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8 *)&UP_SETTING_S_FACE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
		return;
	}
	current_ID=p_udp_monitor_report->socket_ID_l *96;
	for(loop_i=0;loop_i<96;loop_i++){							//�������ݻ�����
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i];
	}
	p_uint8 =&p_udp_monitor_report->buff[96];
	if(p_udp_monitor_report->socket_ID_h==0x80){			//Ϊĩβ֡,�������ȷ����
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<96*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		if(cal_sum!=report_sum){					//У��Ͳ���
			input_ask_report((uint8 *)&UP_SETTING_S_FACE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
			return;
		}
	}
//�޸Ľṹ��
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,48);
	if(write_setting_file()==TRUE){	
		input_ask_report((uint8 *)&UP_SETTING_S_FACE_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_S_FACE_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
	}
}


//ƽ�Ʋ����趨
void	monitor_rec_deal_trans_param(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint16	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

	//��֡����У���
		p_uint8 =&p_udp_monitor_report->buff[96];
		report_sum =*p_uint8;

		cal_sum	= sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->report_type,p_udp_monitor_report->report_len);

		if(cal_sum!=report_sum){					//У��Ͳ���
			input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
			return;
		}
	
	//�޸Ľṹ��
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0];
	p_uint32_src =(uint32 *)&p_udp_monitor_report->buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,24);
	
	//ƽ�Ʋ�����Χ�޶�
	{
		for(loop_i=0;loop_i<MAX_TRANSLATION_MOTION_NUMBER;loop_i++)
		{
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L>255)		//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L<0)			//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 0;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R>255)		//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R<0)			//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 0;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L>255)		//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L<-255)			//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = -255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R>255)		//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R<-255)			//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L>255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L<-255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R>255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R<-255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = -255;
		}
		
	}


	if(write_setting_file()==TRUE){	
		input_ask_report((uint8 *)&UP_SETTING_TRANS_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
	}

}


//��ת�����趨
void	monitor_rec_deal_rotate_param(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint16	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

	//��֡����У���
	p_uint8 =&p_udp_monitor_report->buff[96];
	report_sum =*p_uint8;

	cal_sum	= sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->report_type,p_udp_monitor_report->report_len);

	if(cal_sum!=report_sum){					//У��Ͳ���
		input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
		return;
	}

	//�޸Ľṹ��
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0];
	p_uint32_src =(uint32 *)&p_udp_monitor_report->buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,24);

	//��ת������Χ�޶�
	{
		for(loop_i=0;loop_i<MAX_TRANSLATION_MOTION_NUMBER;loop_i++)
		{
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L>255)		//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L<0)			//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 0;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R>255)		//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R<0)			//ת������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 0;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L>255)		//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L<-255)			//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = -255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R>255)		//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R<-255)			//��λ����
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L>255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L<-255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R>255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R<-255)		//�������
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = -255;
		}

	}


	//printf("rudder_l=%d,rudder_r=%d\n",monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_L,monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0].rudder_R);

	if(write_setting_file()==TRUE){	
		input_ask_report((uint8 *)&UP_SETTING_TRANS_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
	}
}



//�����趨
void	monitor_rec_deal_ship_route(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	current_ID;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint8	ship_stop_all_number;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

//��ʼ֡	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=3){
		input_ask_report((uint8*)&UP_SETTING_SHIP_ROUTE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
		return;
	}
	ship_stop_all_number =p_udp_monitor_report->buff[0];			//�ܵĺ�����
	current_ID=p_udp_monitor_report->socket_ID_l *100;
	for(loop_i=0;loop_i<100;loop_i++){							//�������ݻ�����
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i+2];
	}
	
	if(p_udp_monitor_report->socket_ID_h==0x80){			//Ϊĩβ֡,�������ȷ����
		p_uint8 =&p_udp_monitor_report->buff[102];
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<100*3;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//У��Ͳ���
		//	input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
		//	return;
		//}
	}
	monitor_all_inf.rec_monitor_all_set_param.ship_stop_number= ship_stop_all_number;
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,48);
//�޸Ľṹ��
	for(loop_i=0;loop_i<MONITOR_STOP_MAX_NUMBER;loop_i++){
		memcpy_32(p_uint32_dest,p_uint32_src,4);				//2������+2������
		p_uint32_src=p_uint32_src+4;
		p_uint32_dest=p_uint32_dest+4;
		memcpy_16((uint16 *)p_uint32_dest,(uint16 *)p_uint32_src,2);	//ͣ��ʱ��+����
		p_uint32_src=p_uint32_src++;
		p_uint32_dest=p_uint32_dest++;		
	}
	if(write_setting_file()==TRUE){
		input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
	}

	//�����趨�ɹ��󣬺�������״̬
	if(1)	//У��ͨ��
	{

		Sailing_Cnt_Old=1;
		USV_State.USV_Sailing_Intel_Sign=1;//�յ��������񣬴�����
		USV_State.Sailing_Nummber = monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0].bak_16_1+1;// monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;	   //�����������
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Waypoint_Nummber = monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0].bak_16_1+1;

		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Sign   = Smart_Navigation_St.Longitude_Sign_St    ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Degree = Smart_Navigation_St.USV_Longitude_Degree ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Minute = Smart_Navigation_St.USV_Longitude_Minute ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Second = Smart_Navigation_St.USV_Longitude_Second ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Latitude_Decimal= Smart_Navigation_St.USV_Longitude_Decimal_2;	

		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Sign   = Smart_Navigation_St.Latitude_Sign_St      ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Degree = Smart_Navigation_St.USV_Latitude_Degree   ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Minute = Smart_Navigation_St.USV_Latitude_Minute   ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Second = Smart_Navigation_St.USV_Latitude_Second   ;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[0].Waypoint_Longitude_Decimal= Smart_Navigation_St.USV_Latitude_Decimal_2;	
	


		for(loop_i=1;loop_i<monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;loop_i++)
		{
		//��������
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Type = (monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_1 & 0x000000ff);

		if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat>0)		//γ�ȱ�־
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Sign=0;
		else
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Sign=1;
		if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng>0)
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Sign=0;
		else
			USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Sign=1;

		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Degree=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat/360000;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Minute=(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat/6000)%60;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Second=(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat/100)%60;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Latitude_Decimal=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat%100;	

		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Degree=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng/360000;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Minute=(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng/6000)%60;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Second=(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng/100)%60;
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Longitude_Decimal=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng%100;	


		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_speed=  1000;//monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_16_1;	//bak_16_1 ����Ϊ�����ٶ�
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Stop_Time=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_time;
		}
		Get_Compensating_Dst();
	}


}


//�ϰ����趨
void	monitor_rec_deal_obstacle(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint32  *p_uint32_src;
uint32  *p_uint32_dest;
uint8	current_ID;
uint8	cal_sum;
uint8	report_sum;
uint16	loop_i;
uint8	obstacle_all_number;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;

//��ʼ֡	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8*)&UP_SETTING_OBSTACLE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
		return;
	}
	obstacle_all_number =p_udp_monitor_report->buff[0];		//�ܸ���
	current_ID=p_udp_monitor_report->socket_ID_l *100;
	for(loop_i=0;loop_i<100;loop_i++){							//�������ݻ�����
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i+2];
	}
	
	if(p_udp_monitor_report->socket_ID_h==0x80){			//Ϊĩβ֡,�������ȷ����
		p_uint8 =&p_udp_monitor_report->buff[102];
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<100*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//У��Ͳ���
		//	input_ask_report((uint8 *)&UP_SETTING_OBSTACLE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
		//	return;
		//}
	}
	monitor_all_inf.rec_monitor_all_set_param.obstacle_number= obstacle_all_number;
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
//�޸Ľṹ��
	memcpy_32(p_uint32_dest,p_uint32_src,MONITOR_OBSTACLE_MAX_NUMBER*5);				//ÿ���ϰ�����5��long	
	if(write_setting_file()==TRUE){
		input_ask_report((uint8 *)&UP_SETTING_OBSTACLE_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_OBSTACLE_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
	}
	if(1)
	{
		USV_RM_MSG.Obstacles_Num = obstacle_all_number;

		for(loop_i=0;loop_i<MONITOR_OBSTACLE_MAX_NUMBER;loop_i++)
		{
			USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lat	=	monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_locate.lat	;
			USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lng	=	monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_locate.lng	;
			USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_radius		=	monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_radius		;
			USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_speed		=	monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_speed		;
			USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_direction	=	monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_direction	;
		}

	}

}

//���ؿ�����������
void	monitor_rec_deal_mmi_code(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint8  *p_uint8_dest;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
//��ʼ֡	
	if((p_udp_monitor_report->socket_ID_h !=0x80) &&(p_udp_monitor_report->socket_ID_l !=0)){
		input_ask_report((uint8*)&UP_MMI_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SIMULATE_SET,DOWN_FUNC_MMI);
		return;
	}

	p_uint8 = &p_udp_monitor_report->buff[0];
	if((*p_uint8!=SWITCH_ON)&&(*p_uint8!=0x11)&&(*p_uint8 !=0x22)){
		input_ask_report((uint8*)&UP_MMI_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SIMULATE_SET,DOWN_FUNC_MMI);
		return;
			
	}
	monitor_all_inf.rec_pc_simulate_mmi.mmi_state=*p_uint8;
	p_uint8++;
	p_uint8_dest = (uint8 *)&monitor_all_inf.rec_pc_simulate_mmi.mmi_can_report[0].rec_mmi_buff[0];
	memcpy(p_uint8_dest,p_uint8,32);	
	
}

//д��ֵ����REC_MONITOR_ALL_SET_PARAM_STRUCT
int8 write_setting_file(void)
{
FILE *pFile;
int8 buff[200];
uint16	ret_len;
EACH_MONITOR_SET_PID_PARAM_STRUCT	*p_pid;
EACH_MONITOR_SET_S_FACE_PARAM_STRUCT *p_s_face;
EACH_MONITOR_SET_TRANS_PARAM_STRUCT *p_trans;
EACH_MONITOR_SET_ROTATE_PARAM_STRUCT *p_rotate;
SHIP_STOP_INF_STRUCT	*p_stop;
OBSTACLE_LOCATE_INF_STRUCT	*p_obs;
COORDINATE_STRUCT			*p_co;
uint16	loop_i;
int8	ret_val=TRUE;

	pFile=fopen(USV_SETTING_FILE_NAME, "w+");

	if(pFile == NULL){		
		return FALSE;
	}

	if(monitor_all_inf.rec_monitor_all_set_param.lake_point_number>MAX_LAKE_POINT_NUMBER){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.PID_number> MAX_PID_GROUP_NUMBER){
		ret_val =FALSE;
	}

	if(monitor_all_inf.rec_monitor_all_set_param.s_face_number> MAX_PID_GROUP_NUMBER){
		ret_val =FALSE;
	}

	if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_number> MONITOR_STOP_MAX_NUMBER){
		ret_val =FALSE;
	}

	if(monitor_all_inf.rec_monitor_all_set_param.obstacle_number> MONITOR_OBSTACLE_MAX_NUMBER){
		ret_val =FALSE;
	}
	if(ret_val==FALSE){
		fclose(pFile);
		printf("write setting ERROR \n");
		return FALSE;
	}

//д�����С
	ret_len=sprintf_usv((int8 *)&buff[0],";�����С\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Lake_Point_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Lake_point_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.lake_point_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.lake_point_number;loop_i++){
		p_co = &monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Lake_Point_%d=%d,%d   ;���ȣ�γ��\n",loop_i,p_co->lng,p_co->lat);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//дPID����
	ret_len=sprintf_usv((int8 *)&buff[0],";PID����\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[PID_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"PID_Setting_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.PID_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.PID_number;loop_i++){
		p_pid = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"PID_Param_%d=%8.6f,%8.6f,%8.6f    ;P,I,D\n",loop_i,p_pid->P,p_pid->I,p_pid->D);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}
//дS�����
	ret_len=sprintf_usv((int8 *)&buff[0],";S�����\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[S_Face_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"S_Face_Setting_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.s_face_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.s_face_number;loop_i++){
		p_s_face = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"S_Face_Param_%d=%4.3f,%4.3f,%4.3f    ;k1,k2,k3\n",loop_i,p_s_face->k1,p_s_face->k2,p_s_face->k3);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//дƽ�Ʋ���
	monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number=3;// ��ʱ��

	ret_len=sprintf_usv((int8 *)&buff[0],";ƽ�Ʋ��� ��1����ƽ�Ʋ��� ��2����ƽ�Ʋ��� ��3��ֹͣ����\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Translation_Motion_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Translation_Motion_Setting_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number);
	printf("trans_number = %d\n",monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
		for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number;loop_i++){
		p_trans = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Translation_Param_%d=%d,%d,%d,%d,%d,%d    ;���ǣ��Ҷ�ǣ���λ���ҵ�λ���󷢶������ҷ�����\n",loop_i,p_trans->rudder_L,p_trans->rudder_R,p_trans->gear_L,p_trans->gear_R,p_trans->accelerator_L,p_trans->accelerator_R);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//д��ת����
	monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number=4;//��ʱ
	ret_len=sprintf_usv((int8 *)&buff[0],";��ת���� ��1����ʱ����ת��������2��˳ʱ��ƽ�Ʋ��� ������ֹͣ����\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Rotation_Motion_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Rotation_Motion_Setting_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
		for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number;loop_i++){
		p_rotate = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Rotation_Param_%d=%d,%d,%d,%d,%d,%d    ;���ǣ��Ҷ�ǣ���λ���ҵ�λ���󷢶������ҷ�����\n",loop_i,p_rotate->rudder_L,p_rotate->rudder_R,p_rotate->gear_L,p_rotate->gear_R,p_rotate->accelerator_L,p_rotate->accelerator_R);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//д��������Ϣ
	ret_len=sprintf_usv((int8 *)&buff[0],";��������Ϣ\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Ship_Route_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Ship_Route_Number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.ship_stop_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;loop_i++){
		p_stop = &monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Ship_Route_%d=%d,%d,%d,%d,%d,%d    ;���ȣ�γ��,����1_32,����2_32, ͣ��(s),����0_16\n",loop_i,p_stop->stop_locate.lng,p_stop->stop_locate.lat,p_stop->bak_32_1,p_stop->bak_32_2,p_stop->stop_time,p_stop->bak_16_1);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//���ϰ�����Ϣ
	ret_len=sprintf_usv((int8 *)&buff[0],";���ϰ�����Ϣ\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Obstacle_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"obstacle_number = %d    ;����\n",monitor_all_inf.rec_monitor_all_set_param.obstacle_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.obstacle_number;loop_i++){
		p_obs = &monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Obstacle_%d=%d,%d,%4.3f,%4.3f,%4.3f    ;���ȣ�γ��, �ٶ�,����,��ȫ�뾶\n",loop_i,p_obs->obstacle_locate.lng,p_obs->obstacle_locate.lat,p_obs->obstacle_speed,p_obs->obstacle_direction,p_obs->obstacle_radius);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}
	
	fclose(pFile);
	printf("write setting ok \n");

	return TRUE;
}

//�������������
//���룺�ֶΣ���ţ�
//��� FALSE--���� ��TRUE--��ȷ
int8 read_sub_setting_df(int8* main_item, int8* sub_item, int8 id, uint64 *out_data, uint8 flag)
{
	int8 ret_val;
	int32	len;
	int8 *str;
	int8 * p;
	int8 split = ',';
	int8 space = ' ';
	int32 i;
	double	tmp_double;
	uint64	*tmp_int64;


	ret_val = TRUE;
	if (str = ini_GetVarStr(main_item, sub_item, (int32 *)&i))
	{
		p = ini_splitStr(str, split, id, (int32 *)&len);
		if (p) {
			if (flag == INT_TYPE){
				*out_data = ini_str2hex(p);
			}
			else{
				tmp_double = ini_str2double(p);
				tmp_int64 = (uint64 *)&tmp_double;
				*out_data = *tmp_int64;
			}
		}
		else{
			ret_val = FALSE;
			input_cfg_ini_err_sub(main_item, sub_item, id + 1);
		}
	}
	else{
		ret_val = FALSE;
		input_cfg_ini_err_sub(main_item, sub_item, id + 1);
	}
	return ret_val;
}

//�������������
//���룺�ֶΣ���ţ�
//��� FALSE--���� ��TRUE--��ȷ
int8 read_sub_setting(int8* main_item,int8* sub_item,int8 id, uint32 *out_data,uint8 flag)
{
int8 ret_val;
int32	len;
int8 *str;
int8 * p;
int8 split = ',';
int8 space =' ';
int32 i;
float	tmp_float;
uint32	*tmp_int32;

	
	ret_val =TRUE;
	if(str = ini_GetVarStr(main_item, sub_item, (int32 *)&i))
	{					
		p = ini_splitStr(str, split, id, (int32 *)&len); 
		if(p) {
			if(flag==INT_TYPE){
				*out_data=ini_str2hex(p);		
			}
			else{
				tmp_float=ini_str2float(p);
				tmp_int32 =(uint32 *)&tmp_float;
				*out_data =*tmp_int32;
			}
		}
		else{
			ret_val =FALSE;
			input_cfg_ini_err_sub(main_item,sub_item,id+1);
		}
	}
	else{
		ret_val =FALSE;
		input_cfg_ini_err_sub(main_item,sub_item,id+1);
	}
	return ret_val;
}

//��ȡ�ַ�������
int8 read_sub_setting_string(int8* main_item,int8*sub_item,int8 id, char *out_data)
{
	int8 ret_val;
	int32	len;
	int8 *str;
	int8 * p;
	int8 split = ',';
	int8 space =' ';
	int32 i;
	float	tmp_float;
	uint32	*tmp_int32;

	ret_val = TRUE;
	if(str = ini_GetVarStr(main_item, sub_item, (int32 *)&i))
	{
		p=ini_splitStr(str,split,id,(int32 *)&len);
		if(p)
		{
			memcpy(out_data,(char*)p,len);
		}
		else
		{
			ret_val = FALSE;
			input_cfg_ini_err_sub(main_item,sub_item,id+1);
		}
	}
	else
	{
		ret_val =FALSE;
		input_cfg_ini_err_sub(main_item,sub_item,id+1);
	}
	return ret_val;
}





//����ֵ��
int8 read_setting_file(void)
{
FILE *pFile;
int8 *p_file_memory;				//������
int32 *p_buffer;
uint32 lSize;
int32 result;
uint32 len;
int8 s1[32];
int8 s2[50];
EACH_MONITOR_SET_PID_PARAM_STRUCT	*p_pid;
EACH_MONITOR_SET_TRANS_PARAM_STRUCT *p_trans;
EACH_MONITOR_SET_ROTATE_PARAM_STRUCT *p_rotate;
EACH_MONITOR_SET_S_FACE_PARAM_STRUCT *p_s_face;
SHIP_STOP_INF_STRUCT	*p_stop;
OBSTACLE_LOCATE_INF_STRUCT	*p_obs;
COORDINATE_STRUCT			*p_co;
uint16	loop_i;
int8	ret_val=TRUE;

	pFile=fopen(USV_SETTING_FILE_NAME, "r+");

	if(pFile == NULL){
		printf("read setting error\n");
		sprintf_usv(s1,USV_SETTING_FILE_NAME);
		sprintf_usv(s2,"not fond");
		input_cfg_ini_err_sub(s1,s2,0);
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);				//16K
    if (NULL == p_file_memory)
    {
		sprintf_usv(s1,"USV setting file memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		fclose(pFile);
        return FALSE;
    }

	p_buffer = (int32 *)malloc(0x10000);				//64K
    if (NULL == p_buffer)
    {
		sprintf_usv(s1,"USV setting explain memory");
		sprintf_usv(s2,"not enough");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		fclose(pFile);
        return FALSE;
    }
	
// ��ȡ�ļ���С 
    fseek (pFile , 0 , SEEK_END);  
    lSize = ftell (pFile);  
    rewind (pFile);					//��ָ��ָ���ļ���ͷ
  
	if(lSize>=0xffff){
		sprintf_usv(s1,"USV setting read file");
		sprintf_usv(s2,"too large");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}
  
   
    result = fread (p_file_memory,1,lSize,pFile);			 // ���ļ�������buffer��   
  /*  if (result != lSize)  
    {  
        sprintf_usv(s1,"USV setting read file");
		sprintf_usv(s2,"not same");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
    }  
*/
    if(32768 < lSize) len = 2 * lSize;
    else len = 1280 + 2 * lSize;
	len=len * 2;
	result=ini_Initialize((char *)p_file_memory, p_buffer, len);

	if(result!=0){										//�ļ���ʼ������
		sprintf_usv(s1,"USV setting  memory");
		sprintf_usv(s2,"explain error");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

// ��ʼ���������ļ�
//���������С
	sprintf_usv(s1,"Lake_Point_Setting");
	sprintf_usv(s2,"Lake_point_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.lake_point_number,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.lake_point_number>MAX_LAKE_POINT_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val =FALSE;
		monitor_all_inf.rec_monitor_all_set_param.lake_point_number=0;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.lake_point_number;loop_i++){
		sprintf_usv(s2,"Lake_Point_%d",loop_i);
		p_co = &monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_co->lng,INT_TYPE)==FALSE){		//����
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_co->lat,INT_TYPE)==FALSE){		//γ��
			ret_val =FALSE;
		}
	}

//����PID����
	sprintf_usv(s1,"PID_Setting");
	sprintf_usv(s2,"PID_Setting_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.PID_number,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.PID_number>MAX_PID_GROUP_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val =FALSE;
		monitor_all_inf.rec_monitor_all_set_param.PID_number=0;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.PID_number;loop_i++){
		sprintf_usv(s2,"PID_Param_%d",loop_i);
		p_pid = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_pid->P,FLOAT_TYPE)==FALSE){		//P
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_pid->I,FLOAT_TYPE)==FALSE){		//I
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_pid->D,FLOAT_TYPE)==FALSE){		//D
			ret_val =FALSE;
		}
	}


//����ƽ�Ʋ���
	sprintf_usv(s1,"Translation_Motion_Setting");
	sprintf_usv(s2,"Translation_Motion_Setting_Number");
	if(read_sub_setting(s1,s2,0,(uint32 *)&monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number,INT_TYPE)==FALSE){
		ret_val = FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number>MAX_TRANSLATION_MOTION_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val = FALSE;
		monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number=3;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number;loop_i++){
		sprintf_usv(s2,"Translation_Param_%d",loop_i);
		p_trans = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_trans->rudder_L,INT_TYPE)==FALSE){		//����
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_trans->rudder_R,INT_TYPE)==FALSE){		//�Ҷ��
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_trans->gear_L,INT_TYPE)==FALSE){		//��λ
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_trans->gear_R,INT_TYPE)==FALSE){		//�ҵ�λ
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_trans->accelerator_L,INT_TYPE)==FALSE){		//�󷢶���ת��
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_trans->accelerator_R,INT_TYPE)==FALSE){		//�ҷ�����ת��
			ret_val = FALSE;
		}
	}

	//������ת����
	sprintf_usv(s1,"Rotation_Motion_Setting");
	sprintf_usv(s2,"Rotation_Motion_Setting_Number");
	if(read_sub_setting(s1,s2,0,(uint32 *)&monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number,INT_TYPE)==FALSE){
		ret_val = FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number>MAX_ROTATE_MOTION_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val = FALSE;
		monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number=3;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number;loop_i++){
		sprintf_usv(s2,"Rotation_Param_%d",loop_i);
		p_rotate = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_rotate->rudder_L,INT_TYPE)==FALSE){		//����
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_rotate->rudder_R,INT_TYPE)==FALSE){		//�Ҷ��
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_rotate->gear_L,INT_TYPE)==FALSE){		//��λ
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_rotate->gear_R,INT_TYPE)==FALSE){		//�ҵ�λ
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_rotate->accelerator_L,INT_TYPE)==FALSE){		//�ҵ�λ
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_rotate->accelerator_R,INT_TYPE)==FALSE){		//�ҵ�λ
			ret_val = FALSE;
		}
	}



//����S_FACE����
	sprintf_usv(s1,"S_Face_Setting");
	sprintf_usv(s2,"S_Face_Setting_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.s_face_number,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.s_face_number>MAX_PID_GROUP_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val =FALSE;
		monitor_all_inf.rec_monitor_all_set_param.s_face_number=0;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.s_face_number;loop_i++){
		sprintf_usv(s2,"S_Face_Param_%d",loop_i);
		p_s_face = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_s_face->k1,FLOAT_TYPE)==FALSE){		//P
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_s_face->k2,FLOAT_TYPE)==FALSE){		//I
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_s_face->k3,FLOAT_TYPE)==FALSE){		//D
			ret_val =FALSE;
		}
	}

//������������Ϣ
	sprintf_usv(s1,"Ship_Route_Setting");
	sprintf_usv(s2,"Ship_Route_Number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.ship_stop_number,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_number>MONITOR_STOP_MAX_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val =FALSE;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_number=0;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;loop_i++){
		sprintf_usv(s2,"Ship_Route_%d",loop_i);
		p_stop = &monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_stop->stop_locate.lng,INT_TYPE)==FALSE){		//����
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_stop->stop_locate.lat,INT_TYPE)==FALSE){		//γ��
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_stop->bak_32_1,INT_TYPE)==FALSE){		//32λ����1 ��������ģʽѡ��
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_stop->bak_32_2,INT_TYPE)==FALSE){		//32λ����2
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_stop->stop_time,INT_TYPE)==FALSE){		//ͣ��ʱ��
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_stop->bak_16_1,INT_TYPE)==FALSE){		//16λ����1 ������������
			ret_val =FALSE;
		}
	}


//�������ϰ�����Ϣ
	sprintf_usv(s1,"Obstacle_Setting");
	sprintf_usv(s2,"obstacle_number");
	if(read_sub_setting(s1,s2,0, (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.obstacle_number,INT_TYPE)==FALSE){
		ret_val =FALSE;
	}
	if(monitor_all_inf.rec_monitor_all_set_param.obstacle_number>MONITOR_OBSTACLE_MAX_NUMBER){
		input_cfg_ini_err_sub(s1,s2,0);
		ret_val =FALSE;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_number=0;
	}
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.obstacle_number;loop_i++){
		sprintf_usv(s2,"Obstacle_%d",loop_i);
		p_obs = &monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i];
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_obs->obstacle_locate.lng,INT_TYPE)==FALSE){		//����
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_obs->obstacle_locate.lat,INT_TYPE)==FALSE){		//γ��
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_obs->obstacle_speed,FLOAT_TYPE)==FALSE){		//�ٶ�
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_obs->obstacle_direction,FLOAT_TYPE)==FALSE){		//����
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_obs->obstacle_radius,FLOAT_TYPE)==FALSE){		//��ȫ�뾶
			ret_val =FALSE;
		}
	}

	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	printf("read setting ok\n");
	return ret_val;
}



void init_flash_setting_default(void)
{
uint8	loop_i;

	memset((uint8 *)&monitor_all_inf.rec_monitor_all_set_param,0,sizeof(REC_MONITOR_ALL_SET_PARAM_STRUCT));
//������Ϣ
	monitor_all_inf.rec_monitor_all_set_param.lake_point_number =4;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.lake_point_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i].lng=123456;
		monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i].lat=456789;
	}
//PID�����趨
	monitor_all_inf.rec_monitor_all_set_param.PID_number=16;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.PID_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].P=float(0.1+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].I=float(0.2+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].D=float(0.3+loop_i);
	}
//S�����
	monitor_all_inf.rec_monitor_all_set_param.s_face_number=4;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.s_face_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k1=float(0.4+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k2=float(0.5+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k3=float(0.6+loop_i);
	}
//ƽ�Ʋ���
	monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number=3;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].rudder_L=125;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].rudder_R=125;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].gear_L=110;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].gear_R=110;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].accelerator_L=40;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].accelerator_R=40;
	}
//��ת����
	monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number=3;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R=0;
	}


//��������Ϣ
	monitor_all_inf.rec_monitor_all_set_param.ship_stop_number=4;
	for(loop_i=0;loop_i<MONITOR_STOP_MAX_NUMBER;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng =111111+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat =222222+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_time =3333+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_1= 444444+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_2= 55555+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_16_1 =6666+loop_i;
	}
//�ϰ�����Ϣ
	monitor_all_inf.rec_monitor_all_set_param.obstacle_number=3;
	for(loop_i=0;loop_i<MONITOR_OBSTACLE_MAX_NUMBER;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_locate.lng = 1;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_locate.lat = 1;
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_speed      = 2.0;//float(3.2+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_direction  = 2.0;//float(4.3+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i].obstacle_radius     = 2.0;//float(5.1+loop_i);
	}
}


void monitor_data_fresh(void)
{
	int8 *p_int8;
	int loop_i;
	//��Ҫ��������
	////�󷢶���ת��
	//monitor_all_inf.monitor_main_real_data[0].expect_data=float(Accelerator_L/8.0);
	//monitor_all_inf.monitor_main_real_data[0].output_data=float(Accelerator_L/8.0);
	//monitor_all_inf.monitor_main_real_data[0].real_data= float(USV_State.Dradio_USV_Drive_State.Accelerator_Left_St);
	//monitor_all_inf.monitor_main_real_data[0].delta_data=float(monitor_all_inf.monitor_main_real_data[0].real_data-monitor_all_inf.monitor_main_real_data[0].expect_data);
	////�ҷ�����ת��
	//monitor_all_inf.monitor_main_real_data[1].expect_data=float(Accelerator_R/8.0);
	//monitor_all_inf.monitor_main_real_data[1].output_data=float(Accelerator_R/8.0);
	//monitor_all_inf.monitor_main_real_data[1].real_data= float(USV_State.Dradio_USV_Drive_State.Accelerator_Right_St);
	//monitor_all_inf.monitor_main_real_data[1].delta_data=float(monitor_all_inf.monitor_main_real_data[1].real_data-monitor_all_inf.monitor_main_real_data[1].expect_data);
	////���ת��
	//monitor_all_inf.monitor_main_real_data[2].expect_data=float(Rudder_L);
	//monitor_all_inf.monitor_main_real_data[2].output_data=float(Rudder_L);
	//monitor_all_inf.monitor_main_real_data[2].real_data  =float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St);
	//monitor_all_inf.monitor_main_real_data[2].delta_data =float(monitor_all_inf.monitor_main_real_data[2].real_data-monitor_all_inf.monitor_main_real_data[2].expect_data);
	////�Ҷ�ת��
	//monitor_all_inf.monitor_main_real_data[3].expect_data=float(Rudder_R);
	//monitor_all_inf.monitor_main_real_data[3].output_data=float(Rudder_R);
	//monitor_all_inf.monitor_main_real_data[3].real_data= float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St);
	//monitor_all_inf.monitor_main_real_data[3].delta_data=float(monitor_all_inf.monitor_main_real_data[3].real_data-monitor_all_inf.monitor_main_real_data[3].expect_data);
	////��λ
	//monitor_all_inf.monitor_main_real_data[4].expect_data=float(Gear_L);
	//monitor_all_inf.monitor_main_real_data[4].output_data=float(Gear_L);
	//monitor_all_inf.monitor_main_real_data[4].real_data  =float(USV_State.Dradio_USV_Drive_State.Gear_Left_St);
	//monitor_all_inf.monitor_main_real_data[4].delta_data=float(monitor_all_inf.monitor_main_real_data[4].real_data-monitor_all_inf.monitor_main_real_data[4].expect_data);
	////�ҵ�λ
	//monitor_all_inf.monitor_main_real_data[5].expect_data=float(Gear_R);
	//monitor_all_inf.monitor_main_real_data[5].output_data=float(Gear_R);
	//monitor_all_inf.monitor_main_real_data[5].real_data= float(USV_State.Dradio_USV_Drive_State.Gear_Right_St);
	//monitor_all_inf.monitor_main_real_data[5].delta_data=float(monitor_all_inf.monitor_main_real_data[5].real_data-monitor_all_inf.monitor_main_real_data[5].expect_data);
	//
	//�󷢶���
	monitor_all_inf.monitor_main_real_data[0].expect_data = float(jet_system.jetL.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[0].output_data = float(jet_system.jetL.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[0].real_data	  = float(IHC_rev_msg.u16_St_Motor1Rpm);
	monitor_all_inf.monitor_main_real_data[0].delta_data  = 0;

	//�ҷ�����
	monitor_all_inf.monitor_main_real_data[1].expect_data = float(jet_system.jetR.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[1].output_data = float(jet_system.jetR.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[1].real_data	  = float(IHC_rev_msg.u16_St_Motor2Rpm);
	monitor_all_inf.monitor_main_real_data[1].delta_data  = 0;
	
	//���
	monitor_all_inf.monitor_main_real_data[2].expect_data = float(jet_system.jetL.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[2].output_data = float(jet_system.jetL.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[2].real_data   = float(IHC_rev_msg.i16_St_Motor1Rudder);
	monitor_all_inf.monitor_main_real_data[2].delta_data  = 0;

	//�Ҷ�
	monitor_all_inf.monitor_main_real_data[3].expect_data = float(jet_system.jetR.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[3].output_data = float(jet_system.jetR.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[3].real_data   = float(IHC_rev_msg.i16_St_Motor2Rudder);
	monitor_all_inf.monitor_main_real_data[3].delta_data  = 0;

	//��λ
	monitor_all_inf.monitor_main_real_data[4].expect_data = float(jet_system.jetL.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[4].output_data = float(jet_system.jetL.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[4].real_data   = float(IHC_rev_msg.i16_St_Motor1Gear);
	monitor_all_inf.monitor_main_real_data[4].delta_data  = 0;

	//�ҵ�λ
	monitor_all_inf.monitor_main_real_data[5].expect_data = float(jet_system.jetR.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[5].output_data = float(jet_system.jetR.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[5].real_data   = float(IHC_rev_msg.i16_St_Motor2Gear);
	monitor_all_inf.monitor_main_real_data[5].delta_data  = 0;

	
	
	
	
	//����
	monitor_all_inf.monitor_main_real_data[6].expect_data=float(autoNaviSt.double_speed_exp);
	monitor_all_inf.monitor_main_real_data[6].output_data=float(autoNaviSt.double_speed_exp);
	monitor_all_inf.monitor_main_real_data[6].real_data= float(ins_msg.speed);
	monitor_all_inf.monitor_main_real_data[6].delta_data=float(monitor_all_inf.monitor_main_real_data[6].real_data-monitor_all_inf.monitor_main_real_data[6].expect_data);
	//����
	monitor_all_inf.monitor_main_real_data[7].expect_data=float(autoNaviSt.double_heading_exp);
	monitor_all_inf.monitor_main_real_data[7].output_data=float(autoNaviSt.double_heading_exp);
	monitor_all_inf.monitor_main_real_data[7].real_data= float(ins_msg.heading);
	monitor_all_inf.monitor_main_real_data[7].delta_data=float(monitor_all_inf.monitor_main_real_data[7].real_data-monitor_all_inf.monitor_main_real_data[7].expect_data);

			
	//��������״̬
	int32 memTemp;
	memTemp = (int32)(ins_msg.latitude *360000);
	memcpy_32((uint32*)&monitor_all_inf.monitor_sub_real_data[0],(uint32*)&memTemp,1);//����γ��
	memTemp = (int32)(ins_msg.longitude*360000);
	memcpy_32((uint32*)&monitor_all_inf.monitor_sub_real_data[1],(uint32*)&memTemp,1);//��������
	//monitor_all_inf.monitor_sub_real_data[0]	=		;	
	//monitor_all_inf.monitor_sub_real_data[1]	=	(ins_msg.longitude*360000)	;	
	monitor_all_inf.monitor_sub_real_data[2]	=	(float)(ins_msg.heading)			;	//��������
	monitor_all_inf.monitor_sub_real_data[3]	=	(float)(ins_msg.speed)				;	//��������
	monitor_all_inf.monitor_sub_real_data[4]	=	(float)(ins_msg.rotRate)		;	//ת����
	monitor_all_inf.monitor_sub_real_data[5]	=	(float)(ins_msg.i16_pitch/10.0)		;	//����
	monitor_all_inf.monitor_sub_real_data[6]	=	(float)(ins_msg.i16_roll/10.0)		;	//���
	monitor_all_inf.monitor_sub_real_data[7]	=	(float)(ins_msg.i16_heaving/10.0)	;	//����




	//�豸Ͷ��״̬
	monitor_all_inf.usv_equ_state[0]  =  ship_version	;			//1: �켫��		2: ˮ�ʴ�

	monitor_all_inf.usv_equ_state[1]  =  command_signal.sail_mode_cmd.u8_authority			;		//�ٿ�Ȩ��                                                                           
	monitor_all_inf.usv_equ_state[2]  =	 command_signal.sail_mode_cmd.b2_sailMode			;		//����ģʽ                                                                       
	monitor_all_inf.usv_equ_state[3]  =  command_signal.func_mode_cmd.b1_headingConstant	;		//������                                                                       
	monitor_all_inf.usv_equ_state[4]  =  command_signal.func_mode_cmd.b1_speedConstant		;		//���ٺ���                                                                       
	monitor_all_inf.usv_equ_state[5]  =  command_signal.func_mode_cmd.b1_dock_cmd			;		//������                                                                       
	monitor_all_inf.usv_equ_state[6]  =  command_signal.func_mode_cmd.b1_autoReturn			;		//�Զ�����                                                                       
	monitor_all_inf.usv_equ_state[7]  =  command_signal.func_mode_cmd.b1_setReturn			;		//���÷�����                                                                     
	monitor_all_inf.usv_equ_state[8]  =  sailTask.u8_St_sailMsgRev							;		//��������                                                                       
	monitor_all_inf.usv_equ_state[9]  =  command_signal.sail_mode_cmd.b2_sailTask			;		//�Զ���������                                                                   
	monitor_all_inf.usv_equ_state[10]  =  command_signal.sail_mode_cmd.b1_emergencyMode		;		//Ӧ��ģʽ                                                                       
	monitor_all_inf.usv_equ_state[11] =  jet_system.b1_cmd_emergencyStop					;		//��ͣ                                                                           

	monitor_all_inf.usv_equ_state[12] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type					;		//��������	0:��ͣ������ 1:������                        
	monitor_all_inf.usv_equ_state[13] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete		;		//������ɣ�0:����δ��� 1:�������                      
	monitor_all_inf.usv_equ_state[14] =  smpRep.sampleStatus		;		//���������·� 0=����δ�·�,1=���������ѽ���,2=������,3=�������,4=��������,5=����æ
	monitor_all_inf.usv_equ_state[15] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival				;		//���ߵ��0��δ���� 1���Ѿ��ﵽ                        

	monitor_all_inf.usv_equ_state[16] =  *(int8*)&autoNaviSt.b1_st_apf 			;		//���ߵ��0��δ���� 1���Ѿ��ﵽ                                                            

	monitor_all_inf.usv_equ_state[17] =  *(int8*)&obs_var.obsSenserValid	;                                                                                                        
	monitor_all_inf.usv_equ_state[18] =  *(int8*)&obs_var.obsInsValid		;         

	monitor_all_inf.usv_equ_state[19] = pAutoReturnInst->isAutoReturnCfgValid();	//�Զ����������ļ���Ч
	monitor_all_inf.usv_equ_state[20] = pAutoReturnInst->isAutoReturnRunning();		//�Զ������Ƿ�����
	monitor_all_inf.usv_equ_state[21] = pAutoReturnInst->getReturnState();			//�Զ�����״̬


	//monitor_all_inf.usv_equ_state[19] =  *IDU_send_msg.b1_Cmd_periPowK1		;                                                                                                        
	//monitor_all_inf.usv_equ_state[20] =  *IDU_send_msg.b1_Cmd_periPowK2		;                                                                                                        
	//monitor_all_inf.usv_equ_state[21] =  *IDU_send_msg.b1_Cmd_periPowK3		;                                                                                                        
	//monitor_all_inf.usv_equ_state[22] =  *IDU_send_msg.b1_Cmd_periPowK4		;                                                                                                        
	//monitor_all_inf.usv_equ_state[23] =  *IDU_send_msg.b1_Cmd_periPowK5		;                                                                                                        
	//monitor_all_inf.usv_equ_state[24] =  *IDU_send_msg.b1_Cmd_periPowK6		;                                                                                                        
	//monitor_all_inf.usv_equ_state[25] =  *IDU_send_msg.b1_Cmd_periPowK7		;                                                                                                        
	//monitor_all_inf.usv_equ_state[26] =  *IDU_send_msg.b1_Cmd_periPowK8		;                                                                                                        
	//monitor_all_inf.usv_equ_state[27] =  *IDU_send_msg.b1_Cmd_periPowK9		;                                                                                                        
	//monitor_all_inf.usv_equ_state[28] =  *IDU_send_msg.b1_Cmd_periPowK10	;                                                                                                        
	//monitor_all_inf.usv_equ_state[29] =  *IDU_send_msg.b1_Cmd_periPowK11	;                                                                                                        
	//monitor_all_inf.usv_equ_state[30] =  *IDU_send_msg.b1_Cmd_periPowK12	;                                                                                                        
	//monitor_all_inf.usv_equ_state[31] =  *IDU_send_msg.b1_Cmd_periPowK13	;                                                                                                        
	//monitor_all_inf.usv_equ_state[32] =  *IDU_send_msg.b1_Cmd_periPowK14	;                                                                                                        
	//monitor_all_inf.usv_equ_state[33] =  *IDU_send_msg.b1_Cmd_periPowK15	;                                                                                                        
	//monitor_all_inf.usv_equ_state[34] =  *IDU_send_msg.b1_Cmd_periPowK16	;                                                                                                        
	//monitor_all_inf.usv_equ_state[35] =  *IDU_send_msg.b1_Cmd_periPowK17	;                                                                                                        
	//monitor_all_inf.usv_equ_state[36] =  *IDU_send_msg.b1_Cmd_periPowK18	;                                                                                                        
	//monitor_all_inf.usv_equ_state[37] =  *IDU_send_msg.b1_Cmd_periPowK19	;                                                                                                        
	//monitor_all_inf.usv_equ_state[38] =  *IDU_send_msg.b1_Cmd_periPowK20	;                                                                                                        


	//monitor_all_inf.usv_equ_state[39] =	(IDU_rev_msg.b1_St_periPowK1)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[40] =	(IDU_rev_msg.b1_St_periPowK2)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[41] =	(IDU_rev_msg.b1_St_periPowK3)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[42] =	(IDU_rev_msg.b1_St_periPowK4)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[43] =	(IDU_rev_msg.b1_St_periPowK5)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[44] =	(IDU_rev_msg.b1_St_periPowK6)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[45] =	(IDU_rev_msg.b1_St_periPowK7)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[46] =	(IDU_rev_msg.b1_St_periPowK8)	;                                                                                                            
	//monitor_all_inf.usv_equ_state[47] =	(IDU_rev_msg.b1_St_periPowk9	)	;                                                                                                        
	//monitor_all_inf.usv_equ_state[48] =	(IDU_rev_msg.b1_St_periPowk10	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[49] =	(IDU_rev_msg.b1_St_periPowk11	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[50] =	(IDU_rev_msg.b1_St_periPowk12	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[51] =	(IDU_rev_msg.b1_St_periPowk13	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[52] =	(IDU_rev_msg.b1_St_periPowk14	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[53] =	(IDU_rev_msg.b1_St_periPowk15	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[54] =	(IDU_rev_msg.b1_St_periPowk16	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[55] =	(IDU_rev_msg.b1_St_periPowk17	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[56] =	(IDU_rev_msg.b1_St_periPowk18	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[57] =	(IDU_rev_msg.b1_St_periPowk19	)   ;                                                                                                        
	//monitor_all_inf.usv_equ_state[58] =	(IDU_rev_msg.b1_St_periPowk20	)   ;                                                                                                        

	//monitor_all_inf.usv_equ_state[59] = IDU_rev_msg.b1_St_remoteKey			;                                                                                                        
	//monitor_all_inf.usv_equ_state[60] = IDU_rev_msg.b1_St_PORTMotorCharge	;                                                                                                        
	//monitor_all_inf.usv_equ_state[61] = IDU_rev_msg.b1_St_STBDMotorCharge	;                                                                                                        
	//monitor_all_inf.usv_equ_state[62] = IDU_rev_msg.b1_St_shorePower		;                                                                                                        
	//monitor_all_inf.usv_equ_state[63] = IDU_rev_msg.b1_St_PORTShoreCharge	;                                                                                                        
	//monitor_all_inf.usv_equ_state[64] = IDU_rev_msg.b1_St_STBDShoreCharge	;                                                                                                        
	//monitor_all_inf.usv_equ_state[65] = IDU_rev_msg.b1_St_ShoreChargeEnd	;                                                                                                        
	//monitor_all_inf.usv_equ_state[66] = IDU_rev_msg.b1_St_SystemPowerOn		;                                                                                                        

	//monitor_all_inf.usv_equ_state[67] = IDU_rev_msg.b3_St_supplySource		;                                                                                                        

	//monitor_all_inf.usv_equ_state[68] = command_signal.equpment_cmd.b1_elecWindlass1OnOff | (command_signal.equpment_cmd.b1_elecWindlass1UpDown << 1);	//ê��1                      
	//monitor_all_inf.usv_equ_state[69] = command_signal.equpment_cmd.b1_elecWindlass2OnOff | (command_signal.equpment_cmd.b1_elecWindlass2UpDown << 1);	//ê��2                      
	//monitor_all_inf.usv_equ_state[70] = command_signal.equpment_cmd.b1_sprayStrip1OnOff | (command_signal.equpment_cmd.b1_sprayStrip1UpDown<<1);		//ѹ�˰�1                    
	//monitor_all_inf.usv_equ_state[71] = command_signal.equpment_cmd.b1_sprayStrip2OnOff | (command_signal.equpment_cmd.b1_sprayStrip2UpDown<<1);		//ѹ�˰�2       







	//������Ϣ
	monitor_all_inf.tmp_data[0]	= (uint32)(USV_Meter_Clock_Data.run_time_second);			//����ʱ��			��λ ��
	monitor_all_inf.tmp_data[1] = (uint32)(USV_Meter_Clock_Data.run_total_dist);			//�����			��λ ����*10
	monitor_all_inf.tmp_data[2] = (uint32)(USV_Meter_Clock_Data.run_total_dist_this_time);	//�����			��λ ����*10
	monitor_all_inf.tmp_data[3] = (uint32)(USV_Meter_Clock_Data.run_average_speed);			//ƽ������ ��λ	��*10
	monitor_all_inf.tmp_data[4] = (uint32)(USV_Meter_Clock_Data.run_time_second_this_time);	//��������ʱ��		��λ ��

	monitor_all_inf.tmp_data[5] = (uint32)(IDU_rev_msg.u8_St_PORTBatLvl);
	monitor_all_inf.tmp_data[6] = (uint32)(IDU_rev_msg.u8_St_STBDBatLvl);


	float tmp_float;
	tmp_float = IDU_rev_msg.u16_St_remainBatTime * 0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[7],(uint32*)&tmp_float,1);			
	//monitor_all_inf.tmp_data[7] = (float)&IDU_rev_msg.u16_St_remainBatTime);
	tmp_float = IDU_rev_msg.u16_St_PORTInstanVol * 0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[8],(uint32*)&tmp_float,1);	
	//monitor_all_inf.tmp_data[8] = (float)(IDU_rev_msg.u16_St_PORTInstanVol * 0.1);
	tmp_float = IDU_rev_msg.u16_St_PORTInstanCur * 0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[9],(uint32*)&tmp_float,1);	
	//monitor_all_inf.tmp_data[9] = (float)(IDU_rev_msg.u16_St_PORTInstanCur * 0.1);
	tmp_float = IDU_rev_msg.u16_St_PORTInstanPow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[10],(uint32*)&tmp_float,1);	
	//monitor_all_inf.tmp_data[10] = (float)(IDU_rev_msg.u16_St_PORTInstanPow);
	tmp_float = IDU_rev_msg.u16_St_STBDInstanVol * 0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[11],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[11] = (float)(IDU_rev_msg.u16_St_STBDInstanVol * 0.1);
	tmp_float = IDU_rev_msg.u16_St_STBDInstanCur * 0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[12],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[12] = (float)(IDU_rev_msg.u16_St_STBDInstanCur * 0.1);
	tmp_float = IDU_rev_msg.u16_St_STBDInstanPow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[13],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[13] = (float)(IDU_rev_msg.u16_St_STBDInstanPow);
	tmp_float = IDU_rev_msg.u16_St_ShoreInstanVol *0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[14],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[14] = (float)(IDU_rev_msg.u16_St_ShoreInstanVol *0.1);
	tmp_float = IDU_rev_msg.u16_St_ShoreInstanCur *0.1;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[15],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[15] = (float)(IDU_rev_msg.u16_St_ShoreInstanCur *0.1);
	tmp_float = IDU_rev_msg.u16_St_ShoreInstanPow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[16],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[16] = (float)(IDU_rev_msg.u16_St_ShoreInstanPow);
	monitor_all_inf.tmp_data[17] = (uint32)(IDU_rev_msg.u8_St_PortOilLvl);

	tmp_float = IDU_rev_msg.f32_St_PORTIntergFlow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[18],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[18] = (float)(IDU_rev_msg.f32_St_PORTIntergFlow);
	tmp_float = IDU_rev_msg.f32_St_PORTInstanFlow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[19],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[19] = (float)(IDU_rev_msg.f32_St_PORTInstanFlow);
	tmp_float = IDU_rev_msg.f32_St_PORTIntergTime;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[20],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[20] = (float)(IDU_rev_msg.f32_St_PORTIntergTime);
	monitor_all_inf.tmp_data[21] = (uint32)(IDU_rev_msg.u8_St_STBDOilLvl);
	
	tmp_float = IDU_rev_msg.f32_St_STBDIntergFlow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[22],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[22] = (float)(IDU_rev_msg.f32_St_STBDIntergFlow);
	tmp_float = IDU_rev_msg.f32_St_STBDInstanFlow;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[23],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[23] = (float)(IDU_rev_msg.f32_St_STBDInstanFlow);
	tmp_float = IDU_rev_msg.f32_St_STBDIntergTime;
	memcpy_32((uint32*)&monitor_all_inf.tmp_data[24],(uint32*)&tmp_float,1);
	//monitor_all_inf.tmp_data[24] = (float)(IDU_rev_msg.f32_St_STBDIntergTime);

	//�澯
	//DRIOP
	monitor_all_inf.alm_state[0]  = 0;	
	monitor_all_inf.alm_state[1]  = 0;	
	monitor_all_inf.alm_state[2]  = 0;

	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_warn					)   	;		//�ܸ澯
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_mainBoardPower &0x01	)<<1	;		//�����Դ�쳣
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_mainBoardTempe &0x01	)<<2	;		//�����¶��쳣
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_int			&0x01	)<<3	;		//�ж��쳣
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_CANComm		&0x01	)<<4	;		//CAN2ͨѶ��
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_SCIA			&0x01	)<<5	;		//SCIA	����
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_SCIB			&0x01	)<<6	;		//SCIB	����
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP1		&0x01	)<<7	;		//��չ����1
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP2		&0x01	)<<8	;		//��չ����2
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP3		&0x01	)<<9	;		//��չ����3
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP4		&0x01	)<<10	;		//��չ����4
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP5		&0x01	)<<11	;		//��չ����5
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP6		&0x01	)<<12	;		//��չ����6
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP7		&0x01	)<<13	;		//��չ����7
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP8		&0x01	)<<14	;		//��չ����8
	//IDU
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_warn				&0x01	)<<15	;		//�ܸ澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<16	;		//�����Դ�澯
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<17	;		//�����¶��쳣
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_int				&0x01	)<<18	;		//�ж��쳣
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_CANComm			&0x01	)<<19	;		//CAN2ͨѶ��
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_SCIA				&0x01	)<<20	;		//SCIA	����
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_SCIB				&0x01	)<<21	;		//SCIB	����
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_batLow			&0x01	)<<22	;		//�����͸澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_oilLow			&0x01	)<<23	;		//�����͸澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_autoOff			&0x01	)<<24	;		//�Զ��ѿ۸澯
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_oilLevelGauge	&0x01	)<<25	;		//��λ����Ч�澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreVolOver		&0x01	)<<26	;		//�����ѹ�澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreVolBelow	&0x01	)<<27	;		//����Ƿѹ�澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreCurOver		&0x01	)<<28	;		//��������澯
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_PORTVolOver		&0x01	)<<29	;		//PORT��ع�ѹ�澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_PORTVolBelow		&0x01	)<<30	;		//PORT���Ƿѹ�澯
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_PORTCurOver		&0x01	)<<31	;		//PORT��ع����澯
	
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDVolOver		&0x01	)<<0	;		//STBD��ع�ѹ�澯
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDVolBelow		&0x01	)<<1	;		//STBD���Ƿѹ�澯
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDCurOver		&0x01	)<<2	;		//STBD��ع����澯

	//IOP
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_warn				&0x01	)<<3   	 ;		//�ܸ澯
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<4	 ;		//�����Դ�쳣
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<5	 ;		//�����¶��쳣
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_int				&0x01	)<<6	 ;		//�ж��쳣
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_CANComm			&0x01	)<<7	 ;		//CAN2ͨѶ��
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_SCIA				&0x01	)<<8	 ;		//SCIA	����
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_SCIB				&0x01	)<<9	 ;		//SCIB	����
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP1		&0x01	)<<10	 ;		//��չ����1
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP2		&0x01	)<<11	 ;		//��չ����2
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP3		&0x01	)<<12	 ;		//��չ����3
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP4		&0x01	)<<13	 ;		//��չ����4
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP5		&0x01	)<<14	 ;		//��չ����5
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP6		&0x01	)<<15	 ;		//��չ����6
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP7		&0x01	)<<16	 ;		//��չ����7
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP8		&0x01	)<<17	 ;		//��չ����8
	//IHC
	monitor_all_inf.alm_state[1] +=	(IHC_rev_msg.b1_Wn_ClassIWarn		&0x01	)<<18	;		//I���ܸ澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<19	;		//�����Դ�쳣
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<20	;		//�����¶��쳣
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_SPI				&0x01	)<<21	;		//SPIͨѶ����
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_1Comm	&0x01	)<<22	;		//��չ����1�澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_2Comm	&0x01	)<<23	;		//��չ����2�澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_3Comm	&0x01	)<<24	;		//��չ����3�澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_4Comm	&0x01	)<<25	;		//��չ����4�澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_ClassIIWarn		&0x01	)<<26	;		//II���ܸ澯
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTMotorWarn	&0x01	)<<27	;		//PORT����������
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_STBDMotorWarn	&0x01	)<<28	;		//STBD����������
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTGearWarn		&0x01	)<<29	;		//PORT�ƶ�������
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_STBDGearWarn		&0x01	)<<30	;		//STBD�ƶ�������
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTRudderWarn	&0x01	)<<31	;		//PORTת��������
	monitor_all_inf.alm_state[2] += (IHC_rev_msg.b1_Wn_STBDRudderWarn	&0x01	)<<0	;		//STBDת��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK1		&0x01   )<<1	;		//K1��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK2		&0x01   )<<2	;		//K2��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK3		&0x01   )<<3	;		//K3��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK4		&0x01   )<<4	;		//K4��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK5		&0x01   )<<5	;		//K5��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK6		&0x01   )<<6	;		//K6��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK7		&0x01   )<<7	;		//K7��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK8		&0x01   )<<8	;		//K8��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK9 		&0x01   )<<9 	;		//K9��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK10		&0x01   )<<10	;		//K10��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK11		&0x01   )<<11	;		//K11��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK12		&0x01   )<<12	;		//K12��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK13		&0x01   )<<13	;		//K13��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK14		&0x01   )<<14	;		//K14��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK15		&0x01   )<<15	;		//K15��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK16		&0x01   )<<16	;		//K16��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK17		&0x01   )<<17	;		//K17��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK18		&0x01   )<<18	;		//K18��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK19		&0x01   )<<19	;		//K19��������
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK20		&0x01   )<<20	;		//K20��������

	monitor_all_inf.alm_state[2] += (ins_hardState.b_primary	&0x01) << 21;		//K16��������
	monitor_all_inf.alm_state[2] += (ins_hardState.b_secondary	&0x01) << 22;		//K17��������
	monitor_all_inf.alm_state[2] += (ins_hardState.b_diff		&0x01) << 23;		//K18��������
	monitor_all_inf.alm_state[2] += (ins_hardState.b_dGps		&0x01) << 24;		//K19��������
	monitor_all_inf.alm_state[2] += (ins_hardState.b_rtcErr		&0x01) << 25;		//K20��������


	//ͨѶ����
	//���뵽����ͨ��ģ���и���

	//���ó�����Ϣ����ʱ����
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number=0;
	
	//�ϰ�����Ϣ���� USV_RM_MSG.RM_radar_Msg[i].obstacle_locate.lat
	if (ship_version == TIANJI_VERSION)
	{
		ship_simulate_param.monitor_ship_run_inf.obstacle_number = USV_RM_MSG.Obstacles_Num;
		for (loop_i = 0; loop_i < MONITOR_OBSTACLE_MAX_NUMBER; loop_i++)
		{
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lat = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lat;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lng = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_locate.lng;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_radius = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_radius;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_speed = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_speed;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_direction = USV_RM_MSG.RM_radar_Msg[loop_i].obstacle_direction;
		}
	}


	//apf_obs �ϰ�����Ϣ����
	//ship_simulate_param.monitor_ship_run_inf.obstacle_number = apf_obs.size();
	//uint8 obs_num = (apf_obs.size() > MONITOR_OBSTACLE_MAX_NUMBER)?MONITOR_OBSTACLE_MAX_NUMBER:apf_obs.size();
	//for(loop_i=0;loop_i<obs_num;loop_i++)
	//{
	//	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lat = apf_obs[loop_i].lat*360000;
	//	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lng = apf_obs[loop_i].lng*360000;
	//	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_radius	 = apf_obs[loop_i].radius;
	//	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_speed		 = 0;
	//	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_direction  = 0;
	//}

	//zmq_obs �ϰ�����Ϣ����
	if (ship_version == WATER_QUALITY)
	{
		uint8 obs_num = (obs_var.obsNum > MONITOR_OBSTACLE_MAX_NUMBER) ? MONITOR_OBSTACLE_MAX_NUMBER : obs_var.obsNum;
		ship_simulate_param.monitor_ship_run_inf.obstacle_number = obs_num;
		for (loop_i = 0; loop_i < obs_num; loop_i++)
		{
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lat = obs_var.obsAttr[loop_i].lat * 360000;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lng = obs_var.obsAttr[loop_i].lng * 360000;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_radius = obs_var.obsAttr[loop_i].radius;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_speed = 0;
			ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_direction = 0;
		}
	}






	//���Ĺ켣
	monitor_all_inf.monitor_curve_real_data.current_locate.lat					=	(int32)(ins_msg.latitude  * 360000)			;	//(Smart_Navigation_St.USV_Lng * 360000);
	monitor_all_inf.monitor_curve_real_data.current_locate.lng					=	(int32)(ins_msg.longitude * 360000)			;	//(Smart_Navigation_St.USV_Lat * 360000);
	monitor_all_inf.monitor_curve_real_data.ship_speed.expect_data				=	(float)pPidAutoSpeed->getSetingValue();	
	monitor_all_inf.monitor_curve_real_data.ship_speed.output_data				=	(float)ins_msg.speed						;	//float(USV_State.Dradio_USV_Sailing_State.USV_Speed);
	monitor_all_inf.monitor_curve_real_data.ship_direction.expect_data			=	(float)pPidHeading->getSetingValue();;	//float(Heading_EXP);
	monitor_all_inf.monitor_curve_real_data.ship_direction.output_data			=	(float)ins_msg.heading						;	//float(USV_State.Dradio_USV_Sailing_State.USV_Heading*0.01);
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.expect_data	=	(float)jet_system.jetL.u8_Cmd_MotorOpenDeg	;	//float(Accelerator_L/8.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.output_data	=	(float)jet_system.jetL.u8_Cmd_MotorOpenDeg	/JOYSTICK_MOTOR_OPENDEG_COFF;	//float(Accelerator_L/8.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.real_data	=	(float)IHC_rev_msg.u16_St_Motor1Rpm			;	//float(USV_State.Dradio_USV_Drive_State.Accelerator_Left_St);
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.expect_data =	(float)jet_system.jetR.u8_Cmd_MotorOpenDeg	;	//float(Accelerator_L/8.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.output_data =	(float)jet_system.jetR.u8_Cmd_MotorOpenDeg	/JOYSTICK_MOTOR_OPENDEG_COFF;	//float(Accelerator_L/8.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.real_data   =	(float)IHC_rev_msg.u16_St_Motor2Rpm			;	//float(USV_State.Dradio_USV_Drive_State.Accelerator_Left_St);
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.expect_data		=	(float)jet_system.jetL.i16_Cmd_MotorRudderDeg	;	//float(Rudder_L);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.output_data		=	(float)pPidHeading->getOutputValue();	//float(Rudder_L);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.real_data			=	(float)ins_msg.rotRate;	//float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St);
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.expect_data		=	(float)jet_system.jetR.i16_Cmd_MotorRudderDeg	;	//float(Rudder_L);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.output_data		=	(float)pPidRot->getOutputValue();	//float(Rudder_L);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.real_data		=	(float)IHC_rev_msg.i16_St_Motor2Rudder			;	//float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St);
	//monitor_all_inf.monitor_curve_real_data.left_gear.expect_data  =  float(Gear_L);
	monitor_all_inf.monitor_curve_real_data.left_gear.output_data  =  (float)jet_system.jetL.i16_Cmd_MotorGearDeg/16.0;  
	monitor_all_inf.monitor_curve_real_data.left_gear.real_data  =  (float)IHC_rev_msg.i16_St_Motor1Rudder;
	
	monitor_all_inf.monitor_curve_real_data.right_gear.output_data =  (float)jet_system.jetR.i16_Cmd_MotorGearDeg/16.0;  
	monitor_all_inf.monitor_curve_real_data.right_gear.real_data  =  (float)IHC_rev_msg.i16_St_Motor2Rudder;

	monitor_all_inf.monitor_curve_real_data.track_error							=	track_control.track_error;
	monitor_all_inf.monitor_curve_real_data.collision_data.sail_mode            =   autoNaviSt.b1_st_apf;
	monitor_all_inf.monitor_curve_real_data.collision_data.dcpa					=	float(GLB_DCPA);
	monitor_all_inf.monitor_curve_real_data.collision_data.tcpa					=	float(GLB_TCPA);

	//�˹��Ƴ� ���Ϻ���
	if(apf_valid == 1)
	{
		monitor_all_inf.monitor_curve_real_data.exp_destination.lat = int32(apf_dstCalc.lat * 360000.0);
		monitor_all_inf.monitor_curve_real_data.exp_destination.lng = int32(apf_dstCalc.lng * 360000.0);
	}
	else if (pAutoReturnInst->isAutoReturnRunning()){	//�Զ���������
		monitor_all_inf.monitor_curve_real_data.exp_destination.lat = int32(pAutoReturnInst->getDestPoint().lat * 360000);
		monitor_all_inf.monitor_curve_real_data.exp_destination.lng = int32(pAutoReturnInst->getDestPoint().lng * 360000);
	}
	else{	//�������񺽵�
		monitor_all_inf.monitor_curve_real_data.exp_destination.lat = int32(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_latitude * 360000);
		monitor_all_inf.monitor_curve_real_data.exp_destination.lng = int32(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_longitude * 360000);

	}


}


// #########################################################
void monitor_rec_deal_record_cmd(uint8 func)
{
	msg_record_cmd[func- 0x31] = 1;
}



// #########################################################
void test_monitor(void)
{
uint16	loop_i;
uint16  loop_j;
static uint16	test_start=0;
int8 *p_int8;
uint32 *p_uint32;
float  tmp_float;

//ͨѶ��Ϣ
//	test_start =test_start+10;
	for(loop_i=0;loop_i<ALL_MONITOR_COMM_NUMBER;loop_i++){		
		monitor_all_inf.monitor_comm_inf[loop_i].send_ok_number =test_start+loop_i+1;		//������ȷ֡��
		monitor_all_inf.monitor_comm_inf[loop_i].send_error_number =test_start+loop_i+2;	//���ͳ���֡��
		monitor_all_inf.monitor_comm_inf[loop_i].rec_ok_number =test_start+loop_i+3;		//������ȷ֡��
		monitor_all_inf.monitor_comm_inf[loop_i].rec_error_number =test_start+loop_i+4;	//���ճ���֡��
		monitor_all_inf.monitor_comm_inf[loop_i].send_err_ID =test_start+loop_i+5;			//���ͳ������
		monitor_all_inf.monitor_comm_inf[loop_i].rec_err_ID =test_start+loop_i+6;			//���ճ������						
	}
//��Ҫ���Ӳ���
	for(loop_i=0;loop_i<MONITOR_REAL_DATA_NUMBER;loop_i++){
	//	monitor_all_inf.monitor_main_real_data[loop_i].expect_data = (float)2.0; //(test_start+loop_i+0.01);
		monitor_all_inf.monitor_main_real_data[loop_i].output_data = (float)2.0; //(test_start+loop_i+0.02);
		monitor_all_inf.monitor_main_real_data[loop_i].real_data   = (float)2.0; //(test_start+loop_i+0.03);
		monitor_all_inf.monitor_main_real_data[loop_i].delta_data  = (float)2.0; //(test_start+loop_i+0.4);
	}
//��Ҫ���Ӳ���
	for(loop_i=0;loop_i<MONITOR_SUB_REAL_DATA_NUMBER;loop_i++){
		monitor_all_inf.monitor_sub_real_data[loop_i]=(float)(test_start+loop_i+0.1);
	}

//����ʵʱ����
	////��ǰ����
	//monitor_all_inf.monitor_curve_real_data.current_locate.lng =test_start*10000+1;	//ά��
	//monitor_all_inf.monitor_curve_real_data.current_locate.lat =test_start*10000+2;	//����
	////����
	//monitor_all_inf.monitor_curve_real_data.ship_speed.expect_data =(float)(test_start+1+0.01);
	//monitor_all_inf.monitor_curve_real_data.ship_speed.output_data =(float)(test_start+2+0.01);
	////����
	//monitor_all_inf.monitor_curve_real_data.ship_direction.expect_data =(float)(test_start+3+0.01);
	//monitor_all_inf.monitor_curve_real_data.ship_direction.output_data =(float)(test_start+4+0.01);
	////�󷢶���
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.expect_data =(float)(test_start+5+0.01);
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.output_data =(float)(test_start+6+0.02);
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.real_data =(float)(test_start+7+0.03);
	////�ҷ�����		
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.expect_data =(float)(test_start+8+0.04);
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.output_data =(float)(test_start+9+0.05);
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.real_data =(float)(test_start+10+0.06);
	////���
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.expect_data =(float)(test_start+11+0.07);
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.output_data =(float)(test_start+12+0.08);
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.real_data =(float)(test_start+13+0.09);
	////�Ҷ�
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.expect_data =(float)(test_start+14+0.10);
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.output_data =(float)(test_start+15+0.11);
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.real_data =(float)(test_start+16+0.12);
	////��λ
	//monitor_all_inf.monitor_curve_real_data.left_gear.expect_data =(float)(test_start+17+0.13);
	//monitor_all_inf.monitor_curve_real_data.left_gear.output_data =(float)(test_start+18+0.14);

	////�ҵ�λ
	//monitor_all_inf.monitor_curve_real_data.right_gear.expect_data =(float)(test_start+20+0.15);
	//monitor_all_inf.monitor_curve_real_data.right_gear.output_data =(float)(test_start+21+0.16);
		
	//��ǰ����
	monitor_all_inf.monitor_curve_real_data.current_locate.lng += 2	;	//ά��
	monitor_all_inf.monitor_curve_real_data.current_locate.lat += 1	;	//����
	//����
	monitor_all_inf.monitor_curve_real_data.ship_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.ship_speed.output_data =(float)(2.0);
	//����
	monitor_all_inf.monitor_curve_real_data.ship_direction.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.ship_direction.output_data =(float)(2.0);
	
	monitor_all_inf.monitor_curve_real_data.ship_direction.output_data = (monitor_all_inf.monitor_curve_real_data.ship_direction.output_data+1)>360? 0:(monitor_all_inf.monitor_curve_real_data.ship_direction.output_data+1);


	//�󷢶���
//	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.real_data =(float)(2.0);
	//�ҷ�����		
//	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.real_data =(float)(2.0);
	//���
//	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.real_data =(float)(2.0);
	//�Ҷ�
//	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.real_data =(float)(2.0);
	////��λ
	//monitor_all_inf.monitor_curve_real_data.left_gear.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_gear.output_data =(float)(2.0);

	////�ҵ�λ
	//monitor_all_inf.monitor_curve_real_data.right_gear.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_gear.output_data =(float)(2.0);

	//Ŀ�꺽��
	monitor_all_inf.monitor_curve_real_data.exp_destination.lng = 20;
	monitor_all_inf.monitor_curve_real_data.exp_destination.lat = 10;

	//����ƫ��
	monitor_all_inf.monitor_curve_real_data.track_error = 0.1;


//���˴�����״̬
	for(loop_i=0;loop_i<MONITOR_USV_EQU_STATE_NUMBER;loop_i++){
		monitor_all_inf.usv_equ_state[loop_i]=0x1;
	}
	monitor_all_inf.usv_equ_state[0]=0;
	monitor_all_inf.usv_equ_state[1]=1;
	monitor_all_inf.usv_equ_state[2]=2;
	monitor_all_inf.usv_equ_state[3]=3;
	monitor_all_inf.usv_equ_state[4]=4;
	monitor_all_inf.usv_equ_state[5]=5;
//�澯״̬
	for(loop_i=0;loop_i<MONITOR_USV_ALM_NUMBER;loop_i++){
		monitor_all_inf.alm_state[loop_i]=0x00000000;
	}
//������Ϣ
	for(loop_i=0;loop_i<MONITOR_TMP_DATA_NUMBER;loop_i++){
		monitor_all_inf.tmp_data[loop_i]=test_start+1000+loop_i;
	}
	monitor_all_inf.tmp_data[0]=100;
	monitor_all_inf.tmp_data[1]=0x200;
	tmp_float =(float)12.34;
	p_uint32 =(uint32 *)&tmp_float;
	monitor_all_inf.tmp_data[2]=*p_uint32;
	monitor_all_inf.tmp_data[3]=((10*60+20)*60+30)*100+45;

//���ó�����Ϣ
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number=MONITOR_MAX_CFG_ERROR_ITEM;
	for(loop_i=0;loop_i<MONITOR_MAX_CFG_ERROR_ITEM;loop_i++){
		p_int8=(int8 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].string_title[0];
		sprintf_usv(p_int8,"aaaaaa%d",loop_i);
		p_int8=(int8 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].string_line[0];
		sprintf_usv(p_int8,"bbbbbb%d",loop_i);
		monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].param1=loop_i+1;
	}
//��ģ�鱨��
	for(loop_i=0;loop_i<ALL_MONITOR_COMM_NUMBER;loop_i++){
		for(loop_j=0;loop_j<400;loop_j++){
			monitor_all_inf.module_report_inf[loop_i].report_detail[loop_j]=(uint8)(loop_j+loop_i);
		}
	}
//���洬�ϰ���
	ship_simulate_param.monitor_ship_run_inf.obstacle_number = 10;
	for(loop_i=0;loop_i<MONITOR_OBSTACLE_MAX_NUMBER;loop_i++){
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lat = loop_i*150;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lng = loop_i*100;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_direction  = loop_i+2;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_radius     = loop_i+3;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_speed      = loop_i+4;
	}

//PID��������
	init_flash_setting_default();
//PC�������
	monitor_all_inf.rec_pc_simulate_mmi.mmi_state=SWITCH_ON;
	for(loop_i=0;loop_i<4;loop_i++){
		for(loop_j=0;loop_j<8;loop_j++){
			monitor_all_inf.rec_pc_simulate_mmi.mmi_can_report[loop_i].rec_mmi_buff[loop_j]=loop_i*0x10+loop_j;
		}
	}
}

