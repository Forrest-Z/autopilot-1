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

MONITOR_ALL_INF_STRUCT			monitor_all_inf;		//与监控通讯的全部信息
SHIP_SIMULATION_PARAM_STRUCT	ship_simulate_param;	//仿真船的参数
uint8	rec_report_buff[310];							//接收缓存区，最多接收3帧定值	

//返回报文区
const uint8 UP_SETTING_PID_ERR1[]={"下载PID参数帧序号出错！"};
const uint8 UP_SETTING_PID_ERR2[]={"下载PID参数校验和出错！"};
const uint8 UP_SETTING_PID_ERR3[]={"下载PID参数写文件出错！"};
const uint8 UP_SETTING_PID_OK[]={"下载PID参数正确！"};

const uint8 UP_SETTING_S_FACE_ERR1[]={"下载S面参数帧序号出错！"};
const uint8 UP_SETTING_S_FACE_ERR2[]={"下载S面参数校验和出错！"};
const uint8 UP_SETTING_S_FACE_ERR3[]={"下载S面参数写文件出错！"};
const uint8 UP_SETTING_S_FACE_OK[]={"下载S面参数正确！"};

const uint8 UP_SETTING_TRANS_ERR1[]={"下载平移参数帧序号出错！"};
const uint8 UP_SETTING_TRANS_ERR2[]={"下载平移参数校验和出错！"};
const uint8 UP_SETTING_TRANS_ERR3[]={"下载平移参数写文件出错！"};
const uint8 UP_SETTING_TRANS_OK[]={"下载平移参数正确！"};

const uint8 UP_SETTING_ROTATE_ERR1[]={"下载旋转参数帧序号出错！"};
const uint8 UP_SETTING_ROTATE_ERR2[]={"下载旋转参数校验和出错！"};
const uint8 UP_SETTING_ROTATE_ERR3[]={"下载旋转参数写文件出错！"};
const uint8 UP_SETTING_ROTATE_OK[]={"下载旋转参数正确！"};

const uint8 UP_SETTING_SHIP_ROUTE_ERR1[]={"下载航线帧序号出错！"};
const uint8 UP_SETTING_SHIP_ROUTE_ERR2[]={"下载航线校验和出错！"};
const uint8 UP_SETTING_SHIP_ROUTE_ERR3[]={"下载航线写文件出错！"};
const uint8 UP_SETTING_SHIP_ROUTE_OK[]={"下载航线正确！"};

const uint8 UP_SETTING_OBSTACLE_ERR1[]={"下载避障点帧序号出错！"};
const uint8 UP_SETTING_OBSTACLE_ERR2[]={"下载避障点校验和出错！"};
const uint8 UP_SETTING_OBSTACLE_ERR3[]={"下载避障点写文件出错！"};
const uint8 UP_SETTING_OBSTACLE_OK[]={"下载避障点正确！"};

const uint8 UP_MMI_ERR1[]={"下载MMI参数出错！"};


	

// ########################################################
//			填工具发送程序区
// ########################################################
//填写操作缓存区
void input_ask_report(uint8 *content,uint8 error_flag,uint8 report_type,uint8 sub_type,uint8 func)
{
uint8	report_len;

	monitor_ask_operation.send_flag =SWITCH_ON;			//置发送标志
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

//发送操作返回报文
uint16 send_monitor_ask(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=54+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;			//子类型码
	if(monitor_ask_operation.error_flag==SWITCH_ON){						//操作正确
		p_udp_monitor_report->func =DOWN_FUNC_OPERATE_ERR;					//功能码
	}
	else{						//操作正确
		p_udp_monitor_report->func =DOWN_FUNC_OPERATE_OK;					//功能码
	}
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();	
	p_udp_monitor_report->model_addr =0;						//ARM模块	

	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写报文信息
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

//填写通讯信息
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_COMM_MONITOR;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块

	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送5个结构体
	for(loop_i=0;loop_i<5;loop_i++){
		p_uint32 = (uint32 *)&monitor_all_inf.monitor_comm_inf[current_ID].send_ok_number ;
		memcpy_32((uint32*)p_uint8,p_uint32,4);			//取4个32位
		p_uint8=p_uint8 +4*4;
		p_uint16 =(uint16 *)&monitor_all_inf.monitor_comm_inf[current_ID].send_err_ID;
		memcpy_16((uint16*)p_uint8,p_uint16,2);			//取2个16位
		current_ID++;
		p_uint8=p_uint8 +2*2;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//填写告警信息
uint16	monitor_send_input_alarm(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=32+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_ALARM;					//功能码
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARM模块

	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送5个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.alm_state[0];
	memcpy_32((uint32*)p_uint8,p_uint32,MONITOR_USV_ALM_NUMBER);			//取8个32位
	p_uint8 =p_uint8 +4*MONITOR_USV_ALM_NUMBER;
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//填写调试信息
uint16	monitor_send_input_test_inf(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_TEST;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*25;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送5个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.tmp_data[current_ID] ;
	memcpy_32((uint32*)p_uint8,p_uint32,25);			//取4个32位
	p_uint8=p_uint8 +25*4;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}


//填写配置出错信息
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT)+6+2;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_CFG_ERR_INF;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq;					
	if((monitor_udp_check_send_report.send_report_seq+1)>=monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	

	if(monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number==0){		//没有出错信息
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
		p_udp_monitor_report->report_len=6+2;
		*p_uint8=0;		//总条数
		p_uint8++;
		*p_uint8=0;				//当前条数
		p_uint8++;
	}
	else{
		*p_uint8=monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number;		//总条数
		p_uint8++;
		*p_uint8=current_ID;				//当前条数
		p_uint8++;
//填写实时数据，每次发送5个结构体
		p_uint32 = (uint32 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[current_ID];
		memcpy(p_uint8,(uint8 *)p_uint32,sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT));
		p_uint8=p_uint8 +sizeof(EACH_MONITOR_CFG_ALM_DESCRIBE_STRUCT);
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//主要实时参数监视
uint16	monitor_send_input_main_run_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_MAIN_PARAM;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*6;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_main_real_data[current_ID].expect_data ;
	memcpy_32((uint32*)p_uint8,p_uint32,24);			//取4个32位
	p_uint8=p_uint8 +6*sizeof(MONITOR_MAIN_REAL_DATA_STRUCT);

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//其它参数监视
uint16	monitor_send_input_other_run_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_OTHER_PARAM;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*25;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_sub_real_data[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			//取4个32位
	p_uint8=p_uint8 +25*sizeof(uint32);

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//运行状态监视
uint16	monitor_send_input_run_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;
uint8	current_ID;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_TEST;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_RUN_STATE;					//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*100;					
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.usv_equ_state[current_ID];
	memcpy((uint8*)p_uint8,(uint8 *)p_uint32,100);			
	p_uint8=p_uint8 +100;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}




//曲线实时数据
uint16	monitor_send_input_curve_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_CURVE;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_CURVE_REAL_DATA;					//功能码
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送100个byte
	p_uint32 = (uint32 *)&monitor_all_inf.monitor_curve_real_data;
	memcpy_32((uint32*)p_uint8,p_uint32,25);					
	p_uint8=p_uint8 +100;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);

	return apdu_len;
}

//各模块报文监视
//sub_equ_ID--设备序号
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=100+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_MODEL_REPORT;						//子类型码
	p_udp_monitor_report->func =sub_equ_ID;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	report_NO=monitor_udp_check_send_report.send_report_seq*100;					
	if(monitor_udp_check_send_report.send_report_seq>=3){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.module_report_inf[current_ID].report_detail[report_NO] ;
	memcpy((uint8*)p_uint8,(uint8 *)p_uint32,100);			
	p_uint8=p_uint8 +100;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}

//PID参数设定
//上送参数
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_PID_PARAM;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*8;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;

	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[0], 24*4);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}


//S面参数设定
//上送参数
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_S_FACE_PARAM;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*8;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;

	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)p_udp_monitor_report->buff[0], 24*4);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//平移参数设定
//上送参数
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
	p_udp_monitor_report->report_type = UP_TYPE_ARM_2_PC_REPORT;		//报文类型
	p_udp_monitor_report->report_len   = 96+6;
	p_udp_monitor_report->sub_type    = DOWN_SUB_ASK_SETTING_PARAM;		//子类型码
	p_udp_monitor_report->func          = DOWN_FUNC_TRANS_PARAM;			//功能码
	p_udp_monitor_report->socket_ID_l =  0;
	p_udp_monitor_report->socket_ID_h = 0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;				//ARM模块
	p_uint8 = (uint8 *)&p_udp_monitor_report->buff[0];
	
	//填写实时数据
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);
	p_uint8 = p_uint8 + 96;

	//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;	
}

//旋转参数设定
//上送参数
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
	p_udp_monitor_report->report_type = UP_TYPE_ARM_2_PC_REPORT;		//报文类型
	p_udp_monitor_report->report_len   = 96+6;
	p_udp_monitor_report->sub_type    = DOWN_SUB_ASK_SETTING_PARAM;		//子类型码
	p_udp_monitor_report->func          = DOWN_FUNC_ROTATE_PARAM;			//功能码
	p_udp_monitor_report->socket_ID_l =  0;
	p_udp_monitor_report->socket_ID_h = 0x80;
	clear_monitor_udp_life_time();

	p_udp_monitor_report->model_addr =0;				//ARM模块
	p_uint8 = (uint8 *)&p_udp_monitor_report->buff[0];

	//填写实时数据
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);
	p_uint8 = p_uint8 + 96;

	//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;	
	
}


//航线设定
//上送参数
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SHIP_ROUTE;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;			//每帧5个航点
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=2){				//本帧为结束帧 发送3帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//航点总个数
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;		//总航点数
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
//本帧航点个数
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}

//障碍点设定
//上送参数
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_OBSTACLE_LOCATE;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧 2
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//航点总个数
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.obstacle_number;		//总航点数
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
//填写实时数据，每次发送5个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		//p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}


//湖面大小信息
//上送参数
uint16	monitor_send_input_lake_large_inf(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint32 *p_uint32;
uint8  *p_uint8;
uint8  *p_cal_sum;


	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=96+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_LAKE_LARGE_INF;										//功能码
	p_udp_monitor_report->socket_ID_l=0;	
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
		
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//航点总个数
	*p_uint8 =monitor_all_inf.rec_monitor_all_set_param.lake_point_number;		//总湖面点数
	p_uint8++;
	*p_uint8=12;				//本帧点数
	p_uint8++;
//填写实时数据，每次发送5个结构体
	p_uint32 = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.lake_point[0];
	memcpy_32((uint32*)p_uint8,p_uint32,24);			
	p_uint8=p_uint8 +96;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//仿真船的初始状态
uint16	monitor_send_input_ship_start_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=16*4+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SIM_SHIP_START_STATE;					//功能码
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
	p_uint8_src =(uint8 *)&ship_simulate_param.monitor_ship_run_inf.ship_init_state.Intellingent_Switch;
	memcpy(p_uint8,p_uint8_src,4);
	p_uint8 =p_uint8 +4;
	p_uint8_src =(uint8 *)&ship_simulate_param.monitor_ship_run_inf.ship_init_state.ship_locate;
	memcpy_32((uint32 *)p_uint8,(uint32 *)p_uint8_src,15);

	p_uint8=p_uint8 +15*4;
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//仿真船的航点坐标
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SIM_ROUTE;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;			//每帧5个航点
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=2){				//本帧为结束帧 发送3帧
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//航点总个数
	*p_uint8 =ship_simulate_param.monitor_ship_run_inf.stop_number;		//总航点数
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
//本帧航点个数
//填写实时数据，每次发送6个结构体
	p_uint32 = (uint32 *)&ship_simulate_param.monitor_ship_run_inf.ship_stop_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;
	all_sum =all_sum +sys_CalcCheckSum((uint8 *)p_udp_monitor_report->buff[2], 100);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
		p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}
//仿真船的障碍点坐标
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
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=102+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SIM_OBSTACLE;										//功能码
	p_udp_monitor_report->socket_ID_l=monitor_udp_check_send_report.send_report_seq;
	current_ID=monitor_udp_check_send_report.send_report_seq*5;	
	if(monitor_udp_check_send_report.send_report_seq ==0){			//起始帧
		all_sum =0;
	}
	if(monitor_udp_check_send_report.send_report_seq>=1){				//本帧为结束帧 2
		p_udp_monitor_report->socket_ID_h=0x80;
		clear_monitor_udp_life_time();
	}
	else{
		p_udp_monitor_report->socket_ID_h=0;
		monitor_udp_check_send_report.send_report_seq++;
	}
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];
	//航点总个数
	*p_uint8 =ship_simulate_param.monitor_ship_run_inf.obstacle_number;		//总航点数
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
//填写实时数据，每次发送5个结构体
	p_uint32 = (uint32 *)&ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[current_ID];
	memcpy_32((uint32*)p_uint8,p_uint32,25);			
	p_uint8=p_uint8 +100;

	all_sum =all_sum +sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->buff[2], 100);
//最后一帧
	if(p_udp_monitor_report->socket_ID_h==0x80){								//
		*p_uint8= all_sum;
		p_uint8++;
	//	p_udp_monitor_report->report_len ++;
	}
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//仿真船的实时输入值
uint16	monitor_send_input_real_data(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=20*4+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_SHIP_MODEL;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SIM_INPUT_REAL_DATA;					//功能码
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
	p_uint8_src =(uint8 *)&ship_simulate_param.ship_adj_output_param.Intellingent_Switch;
	memcpy(p_uint8,p_uint8_src,4);
	p_uint8 =p_uint8 +4;
	p_uint8_src =(uint8 *)&ship_simulate_param.ship_adj_output_param.Accelerator_L;
	memcpy_32((uint32 *)p_uint8,(uint32 *)p_uint8_src,19);

	p_uint8=p_uint8 +19*4;
//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;
}

//发送面板当前状态
uint16	monitor_send_input_MMI_state(uint8 *report_buff)
{
uint16	apdu_len=0;
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8  *p_uint8_src;
uint8  *p_uint8;
uint8  *p_cal_sum;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
	p_udp_monitor_report->report_type =UP_TYPE_ARM_2_PC_REPORT;				//报文类型
	p_udp_monitor_report->report_len=33+6;	
	p_udp_monitor_report->sub_type = DOWN_SUB_ASK_SETTING_PARAM;						//子类型码
	p_udp_monitor_report->func =DOWN_FUNC_SHIP_ROUTE;										//功能码
	p_udp_monitor_report->socket_ID_l=0;
	p_udp_monitor_report->socket_ID_h=0x80;
	clear_monitor_udp_life_time();
	
	p_udp_monitor_report->model_addr =0;						//ARM模块	
	p_uint8 =(uint8 *)&p_udp_monitor_report->buff[0];	
//填写实时数据，每次发送5个结构体
	*p_uint8=monitor_all_inf.rec_pc_simulate_mmi.mmi_state;
	p_uint8++;
	p_uint8_src = (uint8 *)&monitor_all_inf.rec_pc_simulate_mmi.mmi_can_report[0].rec_mmi_buff[0];
	memcpy(p_uint8,p_uint8_src,32);			
	p_uint8=p_uint8 +32;

//计算报文校验和
	p_cal_sum =&p_udp_monitor_report->report_type;
	*p_uint8=sys_CalcCheckSum((uint8 *)p_cal_sum, p_udp_monitor_report->report_len);
	apdu_len=input_apdu_report(p_udp_monitor_report,p_udp_monitor_report->report_len);
	return apdu_len;

}


// ########################################################
//			填工具接收处理程序区
// ########################################################
//PID参数设定
//下行参数
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

//起始帧	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8 *)&UP_SETTING_PID_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
		return;
	}
	current_ID=p_udp_monitor_report->socket_ID_l *96;
	for(loop_i=0;loop_i<96;loop_i++){							//填入数据缓存区
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i];
	}
	p_uint8 =&p_udp_monitor_report->buff[96];
	if(p_udp_monitor_report->socket_ID_h==0x80){			//为末尾帧,填接收正确报文
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<96*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//校验和不等
		//	input_ask_report((uint8 *)&UP_SETTING_PID_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
		//	return;
		//}
	}
    //	monitor_all_inf.rec_monitor_all_set_param.PID_number= 16;
	monitor_all_inf.rec_monitor_all_set_param.PID_number = MAX_PID_GROUP_NUMBER;
//修改结构体
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,48);
	if(write_setting_file()==TRUE){
		input_ask_report((uint8 *)&UP_SETTING_PID_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_PID_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_PID_PARAM);
	}
	//PID参数设定到控制系统中
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

//S面参数设定
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

//起始帧	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8 *)&UP_SETTING_S_FACE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
		return;
	}
	current_ID=p_udp_monitor_report->socket_ID_l *96;
	for(loop_i=0;loop_i<96;loop_i++){							//填入数据缓存区
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i];
	}
	p_uint8 =&p_udp_monitor_report->buff[96];
	if(p_udp_monitor_report->socket_ID_h==0x80){			//为末尾帧,填接收正确报文
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<96*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		if(cal_sum!=report_sum){					//校验和不等
			input_ask_report((uint8 *)&UP_SETTING_S_FACE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_S_FACE_PARAM);
			return;
		}
	}
//修改结构体
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


//平移参数设定
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

	//单帧计算校验和
		p_uint8 =&p_udp_monitor_report->buff[96];
		report_sum =*p_uint8;

		cal_sum	= sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->report_type,p_udp_monitor_report->report_len);

		if(cal_sum!=report_sum){					//校验和不等
			input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
			return;
		}
	
	//修改结构体
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[0];
	p_uint32_src =(uint32 *)&p_udp_monitor_report->buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,24);
	
	//平移参数范围限定
	{
		for(loop_i=0;loop_i<MAX_TRANSLATION_MOTION_NUMBER;loop_i++)
		{
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L>255)		//转速上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L<0)			//转速下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 0;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R>255)		//转速上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R<0)			//转速下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 0;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L>255)		//档位上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L<-255)			//档位下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = -255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R>255)		//档位上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R<-255)			//档位下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L>255)		//舵角上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L<-255)		//舵角下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R>255)		//舵角上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R<-255)		//舵角下限
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


//旋转参数设定
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

	//单帧计算校验和
	p_uint8 =&p_udp_monitor_report->buff[96];
	report_sum =*p_uint8;

	cal_sum	= sys_CalcCheckSum((uint8 *)&p_udp_monitor_report->report_type,p_udp_monitor_report->report_len);

	if(cal_sum!=report_sum){					//校验和不等
		input_ask_report((uint8 *)&UP_SETTING_TRANS_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_TRANS_PARAM);
		return;
	}

	//修改结构体
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[0];
	p_uint32_src =(uint32 *)&p_udp_monitor_report->buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,24);

	//旋转参数范围限定
	{
		for(loop_i=0;loop_i<MAX_TRANSLATION_MOTION_NUMBER;loop_i++)
		{
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L>255)		//转速上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L<0)			//转速下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L = 0;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R>255)		//转速上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R<0)			//转速下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R = 0;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L>255)		//档位上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L<-255)			//档位下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L = -255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R>255)		//档位上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R<-255)			//档位下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L>255)		//舵角上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L<-255)		//舵角下限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L = -255;

			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R>255)		//舵角上限
				monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R = 255;
			if(monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R<-255)		//舵角下限
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



//航线设定
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

//起始帧	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=3){
		input_ask_report((uint8*)&UP_SETTING_SHIP_ROUTE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
		return;
	}
	ship_stop_all_number =p_udp_monitor_report->buff[0];			//总的航点数
	current_ID=p_udp_monitor_report->socket_ID_l *100;
	for(loop_i=0;loop_i<100;loop_i++){							//填入数据缓存区
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i+2];
	}
	
	if(p_udp_monitor_report->socket_ID_h==0x80){			//为末尾帧,填接收正确报文
		p_uint8 =&p_udp_monitor_report->buff[102];
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<100*3;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//校验和不等
		//	input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
		//	return;
		//}
	}
	monitor_all_inf.rec_monitor_all_set_param.ship_stop_number= ship_stop_all_number;
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
	memcpy_32((uint32*)p_uint32_dest,p_uint32_src,48);
//修改结构体
	for(loop_i=0;loop_i<MONITOR_STOP_MAX_NUMBER;loop_i++){
		memcpy_32(p_uint32_dest,p_uint32_src,4);				//2个坐标+2个备用
		p_uint32_src=p_uint32_src+4;
		p_uint32_dest=p_uint32_dest+4;
		memcpy_16((uint16 *)p_uint32_dest,(uint16 *)p_uint32_src,2);	//停靠时间+备用
		p_uint32_src=p_uint32_src++;
		p_uint32_dest=p_uint32_dest++;		
	}
	if(write_setting_file()==TRUE){
		input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_OK[0],0,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
	}
	else{
		input_ask_report((uint8 *)&UP_SETTING_SHIP_ROUTE_ERR3[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_SHIP_ROUTE);
	}

	//航线设定成功后，航行任务状态
	if(1)	//校验通过
	{

		Sailing_Cnt_Old=1;
		USV_State.USV_Sailing_Intel_Sign=1;//收到航行任务，待开启
		USV_State.Sailing_Nummber = monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[0].bak_16_1+1;// monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;	   //航行任务个数
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
		//航点类型
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Type = (monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_1 & 0x000000ff);

		if(monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat>0)		//纬度标志
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


		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_speed=  1000;//monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_16_1;	//bak_16_1 定义为航行速度
		USV_Sailing.USV_Sailing_Message[Sailing_Sign].Dradio_Waypoint[loop_i].Waypoint_Stop_Time=monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_time;
		}
		Get_Compensating_Dst();
	}


}


//障碍点设定
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

//起始帧	
	if(p_udp_monitor_report->socket_ID_l ==0){
		memset(&rec_report_buff[0],0,sizeof(rec_report_buff));
	}
	if(p_udp_monitor_report->socket_ID_l>=2){
		input_ask_report((uint8*)&UP_SETTING_OBSTACLE_ERR1[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
		return;
	}
	obstacle_all_number =p_udp_monitor_report->buff[0];		//总个数
	current_ID=p_udp_monitor_report->socket_ID_l *100;
	for(loop_i=0;loop_i<100;loop_i++){							//填入数据缓存区
		rec_report_buff[current_ID+loop_i]=p_udp_monitor_report->buff[loop_i+2];
	}
	
	if(p_udp_monitor_report->socket_ID_h==0x80){			//为末尾帧,填接收正确报文
		p_uint8 =&p_udp_monitor_report->buff[102];
		report_sum= *p_uint8;
		cal_sum =0;
		for(loop_i=0;loop_i<100*2;loop_i++){
			cal_sum=cal_sum+rec_report_buff[loop_i];
		}
		//if(cal_sum!=report_sum){					//校验和不等
		//	input_ask_report((uint8 *)&UP_SETTING_OBSTACLE_ERR2[0],SWITCH_ON,DOWN_TPYE_ASK_PC_2_ARM_REPORT,DOWN_SUB_SETTING_PARAM,DOWN_FUNC_OBSTACLE_LOCATE);
		//	return;
		//}
	}
	monitor_all_inf.rec_monitor_all_set_param.obstacle_number= obstacle_all_number;
	p_uint32_dest = (uint32 *)&monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[0];
	p_uint32_src =(uint32 *)&rec_report_buff[0];
//修改结构体
	memcpy_32(p_uint32_dest,p_uint32_src,MONITOR_OBSTACLE_MAX_NUMBER*5);				//每个障碍点有5个long	
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

//下载控制面板的数据
void	monitor_rec_deal_mmi_code(uint8 *report_buff)
{
UDP_MONITOR_REPORT_STRUCT	*p_udp_monitor_report;
uint8	*p_uint8;
uint8  *p_uint8_dest;

	p_udp_monitor_report=(UDP_MONITOR_REPORT_STRUCT	*)report_buff;
//起始帧	
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

//写定值区，REC_MONITOR_ALL_SET_PARAM_STRUCT
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

//写湖面大小
	ret_len=sprintf_usv((int8 *)&buff[0],";湖面大小\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Lake_Point_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Lake_point_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.lake_point_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.lake_point_number;loop_i++){
		p_co = &monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Lake_Point_%d=%d,%d   ;经度，纬度\n",loop_i,p_co->lng,p_co->lat);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//写PID参数
	ret_len=sprintf_usv((int8 *)&buff[0],";PID参数\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[PID_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"PID_Setting_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.PID_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.PID_number;loop_i++){
		p_pid = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"PID_Param_%d=%8.6f,%8.6f,%8.6f    ;P,I,D\n",loop_i,p_pid->P,p_pid->I,p_pid->D);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}
//写S面参数
	ret_len=sprintf_usv((int8 *)&buff[0],";S面参数\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[S_Face_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"S_Face_Setting_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.s_face_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.s_face_number;loop_i++){
		p_s_face = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"S_Face_Param_%d=%4.3f,%4.3f,%4.3f    ;k1,k2,k3\n",loop_i,p_s_face->k1,p_s_face->k2,p_s_face->k3);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//写平移参数
	monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number=3;// 临时用

	ret_len=sprintf_usv((int8 *)&buff[0],";平移参数 第1个左平移参数 第2个右平移参数 第3个停止参数\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Translation_Motion_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Translation_Motion_Setting_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number);
	printf("trans_number = %d\n",monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
		for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number;loop_i++){
		p_trans = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Translation_Param_%d=%d,%d,%d,%d,%d,%d    ;左舵角，右舵角，左档位，右档位，左发动机，右发动机\n",loop_i,p_trans->rudder_L,p_trans->rudder_R,p_trans->gear_L,p_trans->gear_R,p_trans->accelerator_L,p_trans->accelerator_R);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//写旋转参数
	monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number=4;//临时
	ret_len=sprintf_usv((int8 *)&buff[0],";旋转参数 第1个逆时针旋转参数，第2个顺时针平移参数 第三个停止参数\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Rotation_Motion_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Rotation_Motion_Setting_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
		for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number;loop_i++){
		p_rotate = &monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Rotation_Param_%d=%d,%d,%d,%d,%d,%d    ;左舵角，右舵角，左档位，右档位，左发动机，右发动机\n",loop_i,p_rotate->rudder_L,p_rotate->rudder_R,p_rotate->gear_L,p_rotate->gear_R,p_rotate->accelerator_L,p_rotate->accelerator_R);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//写各航点信息
	ret_len=sprintf_usv((int8 *)&buff[0],";各航点信息\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Ship_Route_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"Ship_Route_Number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.ship_stop_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.ship_stop_number;loop_i++){
		p_stop = &monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Ship_Route_%d=%d,%d,%d,%d,%d,%d    ;经度，纬度,备用1_32,备用2_32, 停靠(s),备用0_16\n",loop_i,p_stop->stop_locate.lng,p_stop->stop_locate.lat,p_stop->bak_32_1,p_stop->bak_32_2,p_stop->stop_time,p_stop->bak_16_1);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}

//各障碍点信息
	ret_len=sprintf_usv((int8 *)&buff[0],";各障碍点信息\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"[Obstacle_Setting]\n");
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	ret_len=sprintf_usv((int8 *)&buff[0],"obstacle_number = %d    ;个数\n",monitor_all_inf.rec_monitor_all_set_param.obstacle_number);
	fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.obstacle_number;loop_i++){
		p_obs = &monitor_all_inf.rec_monitor_all_set_param.obstacle_locate_inf[loop_i];
		ret_len=sprintf_usv((int8 *)&buff[0],"Obstacle_%d=%d,%d,%4.3f,%4.3f,%4.3f    ;经度，纬度, 速度,方向,安全半径\n",loop_i,p_obs->obstacle_locate.lng,p_obs->obstacle_locate.lat,p_obs->obstacle_speed,p_obs->obstacle_direction,p_obs->obstacle_radius);
		fwrite((int8 *)&buff[0],sizeof(char),ret_len,pFile);		
	}
	
	fclose(pFile);
	printf("write setting ok \n");

	return TRUE;
}

//读各子相的数据
//输入：字段，序号，
//输出 FALSE--出错 ，TRUE--正确
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

//读各子相的数据
//输入：字段，序号，
//输出 FALSE--出错 ，TRUE--正确
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

//读取字符串参数
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





//读定值区
int8 read_setting_file(void)
{
FILE *pFile;
int8 *p_file_memory;				//缓存区
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
	
// 获取文件大小 
    fseek (pFile , 0 , SEEK_END);  
    lSize = ftell (pFile);  
    rewind (pFile);					//将指针指向文件开头
  
	if(lSize>=0xffff){
		sprintf_usv(s1,"USV setting read file");
		sprintf_usv(s2,"too large");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}
  
   
    result = fread (p_file_memory,1,lSize,pFile);			 // 将文件拷贝到buffer中   
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

	if(result!=0){										//文件初始化出错
		sprintf_usv(s1,"USV setting  memory");
		sprintf_usv(s2,"explain error");
		input_cfg_ini_err_sub(s1,s2,0);
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		return FALSE;
	}

// 开始解析配置文件
//解析湖面大小
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
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_co->lng,INT_TYPE)==FALSE){		//经度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_co->lat,INT_TYPE)==FALSE){		//纬度
			ret_val =FALSE;
		}
	}

//解析PID参数
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


//解析平移参数
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
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_trans->rudder_L,INT_TYPE)==FALSE){		//左舵角
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_trans->rudder_R,INT_TYPE)==FALSE){		//右舵角
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_trans->gear_L,INT_TYPE)==FALSE){		//左档位
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_trans->gear_R,INT_TYPE)==FALSE){		//右档位
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_trans->accelerator_L,INT_TYPE)==FALSE){		//左发动机转速
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_trans->accelerator_R,INT_TYPE)==FALSE){		//右发动机转速
			ret_val = FALSE;
		}
	}

	//解析旋转参数
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
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_rotate->rudder_L,INT_TYPE)==FALSE){		//左舵角
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_rotate->rudder_R,INT_TYPE)==FALSE){		//右舵角
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_rotate->gear_L,INT_TYPE)==FALSE){		//左档位
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_rotate->gear_R,INT_TYPE)==FALSE){		//右档位
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_rotate->accelerator_L,INT_TYPE)==FALSE){		//右档位
			ret_val = FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_rotate->accelerator_R,INT_TYPE)==FALSE){		//右档位
			ret_val = FALSE;
		}
	}



//解析S_FACE参数
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

//解析各航点信息
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
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_stop->stop_locate.lng,INT_TYPE)==FALSE){		//经度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_stop->stop_locate.lat,INT_TYPE)==FALSE){		//纬度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_stop->bak_32_1,INT_TYPE)==FALSE){		//32位备用1 用作航行模式选择
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_stop->bak_32_2,INT_TYPE)==FALSE){		//32位备用2
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_stop->stop_time,INT_TYPE)==FALSE){		//停留时间
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,5, (uint32 *)&p_stop->bak_16_1,INT_TYPE)==FALSE){		//16位备用1 用作航速设置
			ret_val =FALSE;
		}
	}


//解析各障碍点信息
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
		if(read_sub_setting(s1,s2,0, (uint32 *)&p_obs->obstacle_locate.lng,INT_TYPE)==FALSE){		//经度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,1, (uint32 *)&p_obs->obstacle_locate.lat,INT_TYPE)==FALSE){		//纬度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,2, (uint32 *)&p_obs->obstacle_speed,FLOAT_TYPE)==FALSE){		//速度
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,3, (uint32 *)&p_obs->obstacle_direction,FLOAT_TYPE)==FALSE){		//方向
			ret_val =FALSE;
		}
		if(read_sub_setting(s1,s2,4, (uint32 *)&p_obs->obstacle_radius,FLOAT_TYPE)==FALSE){		//安全半径
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
//湖面信息
	monitor_all_inf.rec_monitor_all_set_param.lake_point_number =4;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.lake_point_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i].lng=123456;
		monitor_all_inf.rec_monitor_all_set_param.lake_point[loop_i].lat=456789;
	}
//PID参数设定
	monitor_all_inf.rec_monitor_all_set_param.PID_number=16;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.PID_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].P=float(0.1+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].I=float(0.2+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[loop_i].D=float(0.3+loop_i);
	}
//S面参数
	monitor_all_inf.rec_monitor_all_set_param.s_face_number=4;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.s_face_number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k1=float(0.4+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k2=float(0.5+loop_i);
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_s_face_param[loop_i].k3=float(0.6+loop_i);
	}
//平移参数
	monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number=3;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Translation_Motion_Setting_Number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].rudder_L=125;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].rudder_R=125;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].gear_L=110;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].gear_R=110;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].accelerator_L=40;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_trans_param[loop_i].accelerator_R=40;
	}
//旋转参数
	monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number=3;
	for(loop_i=0;loop_i<monitor_all_inf.rec_monitor_all_set_param.Rotation_Motion_Setting_Number;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].rudder_R=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].gear_R=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_L=0;
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_rotate_param[loop_i].accelerator_R=0;
	}


//各航点信息
	monitor_all_inf.rec_monitor_all_set_param.ship_stop_number=4;
	for(loop_i=0;loop_i<MONITOR_STOP_MAX_NUMBER;loop_i++){
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lng =111111+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_locate.lat =222222+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].stop_time =3333+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_1= 444444+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_32_2= 55555+loop_i;
		monitor_all_inf.rec_monitor_all_set_param.ship_stop_inf[loop_i].bak_16_1 =6666+loop_i;
	}
//障碍点信息
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
	//主要监视数据
	////左发动机转速
	//monitor_all_inf.monitor_main_real_data[0].expect_data=float(Accelerator_L/8.0);
	//monitor_all_inf.monitor_main_real_data[0].output_data=float(Accelerator_L/8.0);
	//monitor_all_inf.monitor_main_real_data[0].real_data= float(USV_State.Dradio_USV_Drive_State.Accelerator_Left_St);
	//monitor_all_inf.monitor_main_real_data[0].delta_data=float(monitor_all_inf.monitor_main_real_data[0].real_data-monitor_all_inf.monitor_main_real_data[0].expect_data);
	////右发动机转速
	//monitor_all_inf.monitor_main_real_data[1].expect_data=float(Accelerator_R/8.0);
	//monitor_all_inf.monitor_main_real_data[1].output_data=float(Accelerator_R/8.0);
	//monitor_all_inf.monitor_main_real_data[1].real_data= float(USV_State.Dradio_USV_Drive_State.Accelerator_Right_St);
	//monitor_all_inf.monitor_main_real_data[1].delta_data=float(monitor_all_inf.monitor_main_real_data[1].real_data-monitor_all_inf.monitor_main_real_data[1].expect_data);
	////左舵转角
	//monitor_all_inf.monitor_main_real_data[2].expect_data=float(Rudder_L);
	//monitor_all_inf.monitor_main_real_data[2].output_data=float(Rudder_L);
	//monitor_all_inf.monitor_main_real_data[2].real_data  =float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Left_St);
	//monitor_all_inf.monitor_main_real_data[2].delta_data =float(monitor_all_inf.monitor_main_real_data[2].real_data-monitor_all_inf.monitor_main_real_data[2].expect_data);
	////右舵转角
	//monitor_all_inf.monitor_main_real_data[3].expect_data=float(Rudder_R);
	//monitor_all_inf.monitor_main_real_data[3].output_data=float(Rudder_R);
	//monitor_all_inf.monitor_main_real_data[3].real_data= float(USV_State.Dradio_USV_Drive_State.Rudder_Angle_Right_St);
	//monitor_all_inf.monitor_main_real_data[3].delta_data=float(monitor_all_inf.monitor_main_real_data[3].real_data-monitor_all_inf.monitor_main_real_data[3].expect_data);
	////左档位
	//monitor_all_inf.monitor_main_real_data[4].expect_data=float(Gear_L);
	//monitor_all_inf.monitor_main_real_data[4].output_data=float(Gear_L);
	//monitor_all_inf.monitor_main_real_data[4].real_data  =float(USV_State.Dradio_USV_Drive_State.Gear_Left_St);
	//monitor_all_inf.monitor_main_real_data[4].delta_data=float(monitor_all_inf.monitor_main_real_data[4].real_data-monitor_all_inf.monitor_main_real_data[4].expect_data);
	////右档位
	//monitor_all_inf.monitor_main_real_data[5].expect_data=float(Gear_R);
	//monitor_all_inf.monitor_main_real_data[5].output_data=float(Gear_R);
	//monitor_all_inf.monitor_main_real_data[5].real_data= float(USV_State.Dradio_USV_Drive_State.Gear_Right_St);
	//monitor_all_inf.monitor_main_real_data[5].delta_data=float(monitor_all_inf.monitor_main_real_data[5].real_data-monitor_all_inf.monitor_main_real_data[5].expect_data);
	//
	//左发动机
	monitor_all_inf.monitor_main_real_data[0].expect_data = float(jet_system.jetL.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[0].output_data = float(jet_system.jetL.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[0].real_data	  = float(IHC_rev_msg.u16_St_Motor1Rpm);
	monitor_all_inf.monitor_main_real_data[0].delta_data  = 0;

	//右发动机
	monitor_all_inf.monitor_main_real_data[1].expect_data = float(jet_system.jetR.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[1].output_data = float(jet_system.jetR.u8_Cmd_MotorOpenDeg);
	monitor_all_inf.monitor_main_real_data[1].real_data	  = float(IHC_rev_msg.u16_St_Motor2Rpm);
	monitor_all_inf.monitor_main_real_data[1].delta_data  = 0;
	
	//左舵
	monitor_all_inf.monitor_main_real_data[2].expect_data = float(jet_system.jetL.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[2].output_data = float(jet_system.jetL.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[2].real_data   = float(IHC_rev_msg.i16_St_Motor1Rudder);
	monitor_all_inf.monitor_main_real_data[2].delta_data  = 0;

	//右舵
	monitor_all_inf.monitor_main_real_data[3].expect_data = float(jet_system.jetR.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[3].output_data = float(jet_system.jetR.i16_Cmd_MotorRudderDeg);
	monitor_all_inf.monitor_main_real_data[3].real_data   = float(IHC_rev_msg.i16_St_Motor2Rudder);
	monitor_all_inf.monitor_main_real_data[3].delta_data  = 0;

	//左档位
	monitor_all_inf.monitor_main_real_data[4].expect_data = float(jet_system.jetL.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[4].output_data = float(jet_system.jetL.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[4].real_data   = float(IHC_rev_msg.i16_St_Motor1Gear);
	monitor_all_inf.monitor_main_real_data[4].delta_data  = 0;

	//右档位
	monitor_all_inf.monitor_main_real_data[5].expect_data = float(jet_system.jetR.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[5].output_data = float(jet_system.jetR.i16_Cmd_MotorGearDeg);
	monitor_all_inf.monitor_main_real_data[5].real_data   = float(IHC_rev_msg.i16_St_Motor2Gear);
	monitor_all_inf.monitor_main_real_data[5].delta_data  = 0;

	
	
	
	
	//航速
	monitor_all_inf.monitor_main_real_data[6].expect_data=float(autoNaviSt.double_speed_exp);
	monitor_all_inf.monitor_main_real_data[6].output_data=float(autoNaviSt.double_speed_exp);
	monitor_all_inf.monitor_main_real_data[6].real_data= float(ins_msg.speed);
	monitor_all_inf.monitor_main_real_data[6].delta_data=float(monitor_all_inf.monitor_main_real_data[6].real_data-monitor_all_inf.monitor_main_real_data[6].expect_data);
	//航向
	monitor_all_inf.monitor_main_real_data[7].expect_data=float(autoNaviSt.double_heading_exp);
	monitor_all_inf.monitor_main_real_data[7].output_data=float(autoNaviSt.double_heading_exp);
	monitor_all_inf.monitor_main_real_data[7].real_data= float(ins_msg.heading);
	monitor_all_inf.monitor_main_real_data[7].delta_data=float(monitor_all_inf.monitor_main_real_data[7].real_data-monitor_all_inf.monitor_main_real_data[7].expect_data);

			
	//其他运行状态
	int32 memTemp;
	memTemp = (int32)(ins_msg.latitude *360000);
	memcpy_32((uint32*)&monitor_all_inf.monitor_sub_real_data[0],(uint32*)&memTemp,1);//本船纬度
	memTemp = (int32)(ins_msg.longitude*360000);
	memcpy_32((uint32*)&monitor_all_inf.monitor_sub_real_data[1],(uint32*)&memTemp,1);//本船经度
	//monitor_all_inf.monitor_sub_real_data[0]	=		;	
	//monitor_all_inf.monitor_sub_real_data[1]	=	(ins_msg.longitude*360000)	;	
	monitor_all_inf.monitor_sub_real_data[2]	=	(float)(ins_msg.heading)			;	//本船航向
	monitor_all_inf.monitor_sub_real_data[3]	=	(float)(ins_msg.speed)				;	//本船航速
	monitor_all_inf.monitor_sub_real_data[4]	=	(float)(ins_msg.rotRate)		;	//转向率
	monitor_all_inf.monitor_sub_real_data[5]	=	(float)(ins_msg.i16_pitch/10.0)		;	//俯仰
	monitor_all_inf.monitor_sub_real_data[6]	=	(float)(ins_msg.i16_roll/10.0)		;	//横滚
	monitor_all_inf.monitor_sub_real_data[7]	=	(float)(ins_msg.i16_heaving/10.0)	;	//升沉




	//设备投运状态
	monitor_all_inf.usv_equ_state[0]  =  ship_version	;			//1: 天极号		2: 水质船

	monitor_all_inf.usv_equ_state[1]  =  command_signal.sail_mode_cmd.u8_authority			;		//操控权限                                                                           
	monitor_all_inf.usv_equ_state[2]  =	 command_signal.sail_mode_cmd.b2_sailMode			;		//航行模式                                                                       
	monitor_all_inf.usv_equ_state[3]  =  command_signal.func_mode_cmd.b1_headingConstant	;		//定向航行                                                                       
	monitor_all_inf.usv_equ_state[4]  =  command_signal.func_mode_cmd.b1_speedConstant		;		//定速航行                                                                       
	monitor_all_inf.usv_equ_state[5]  =  command_signal.func_mode_cmd.b1_dock_cmd			;		//进出坞                                                                       
	monitor_all_inf.usv_equ_state[6]  =  command_signal.func_mode_cmd.b1_autoReturn			;		//自动返航                                                                       
	monitor_all_inf.usv_equ_state[7]  =  command_signal.func_mode_cmd.b1_setReturn			;		//设置返航点                                                                     
	monitor_all_inf.usv_equ_state[8]  =  sailTask.u8_St_sailMsgRev							;		//航行任务                                                                       
	monitor_all_inf.usv_equ_state[9]  =  command_signal.sail_mode_cmd.b2_sailTask			;		//自动航行命令                                                                   
	monitor_all_inf.usv_equ_state[10]  =  command_signal.sail_mode_cmd.b1_emergencyMode		;		//应急模式                                                                       
	monitor_all_inf.usv_equ_state[11] =  jet_system.b1_cmd_emergencyStop					;		//急停                                                                           

	monitor_all_inf.usv_equ_state[12] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type					;		//航点类型	0:不停船航点 1:采样点                        
	monitor_all_inf.usv_equ_state[13] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete		;		//采样完成：0:采样未完成 1:采样完成                      
	monitor_all_inf.usv_equ_state[14] =  smpRep.sampleStatus		;		//采样任务下发 0=任务未下发,1=采样任务已接收,2=采样中,3=采样完成,4=采样出错,5=采样忙
	monitor_all_inf.usv_equ_state[15] =  sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival				;		//航线到达：0：未到达 1：已经达到                        

	monitor_all_inf.usv_equ_state[16] =  *(int8*)&autoNaviSt.b1_st_apf 			;		//航线到达：0：未到达 1：已经达到                                                            

	monitor_all_inf.usv_equ_state[17] =  *(int8*)&obs_var.obsSenserValid	;                                                                                                        
	monitor_all_inf.usv_equ_state[18] =  *(int8*)&obs_var.obsInsValid		;         

	monitor_all_inf.usv_equ_state[19] = pAutoReturnInst->isAutoReturnCfgValid();	//自动返航配置文件有效
	monitor_all_inf.usv_equ_state[20] = pAutoReturnInst->isAutoReturnRunning();		//自动返航是否启动
	monitor_all_inf.usv_equ_state[21] = pAutoReturnInst->getReturnState();			//自动返航状态


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

	//monitor_all_inf.usv_equ_state[68] = command_signal.equpment_cmd.b1_elecWindlass1OnOff | (command_signal.equpment_cmd.b1_elecWindlass1UpDown << 1);	//锚机1                      
	//monitor_all_inf.usv_equ_state[69] = command_signal.equpment_cmd.b1_elecWindlass2OnOff | (command_signal.equpment_cmd.b1_elecWindlass2UpDown << 1);	//锚机2                      
	//monitor_all_inf.usv_equ_state[70] = command_signal.equpment_cmd.b1_sprayStrip1OnOff | (command_signal.equpment_cmd.b1_sprayStrip1UpDown<<1);		//压浪板1                    
	//monitor_all_inf.usv_equ_state[71] = command_signal.equpment_cmd.b1_sprayStrip2OnOff | (command_signal.equpment_cmd.b1_sprayStrip2UpDown<<1);		//压浪板2       







	//调试信息
	monitor_all_inf.tmp_data[0]	= (uint32)(USV_Meter_Clock_Data.run_time_second);			//运行时间			单位 秒
	monitor_all_inf.tmp_data[1] = (uint32)(USV_Meter_Clock_Data.run_total_dist);			//总里程			单位 海里*10
	monitor_all_inf.tmp_data[2] = (uint32)(USV_Meter_Clock_Data.run_total_dist_this_time);	//总里程			单位 海里*10
	monitor_all_inf.tmp_data[3] = (uint32)(USV_Meter_Clock_Data.run_average_speed);			//平均航速 单位	节*10
	monitor_all_inf.tmp_data[4] = (uint32)(USV_Meter_Clock_Data.run_time_second_this_time);	//本次运行时间		单位 秒

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

	//告警
	//DRIOP
	monitor_all_inf.alm_state[0]  = 0;	
	monitor_all_inf.alm_state[1]  = 0;	
	monitor_all_inf.alm_state[2]  = 0;

	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_warn					)   	;		//总告警
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_mainBoardPower &0x01	)<<1	;		//主板电源异常
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_mainBoardTempe &0x01	)<<2	;		//主板温度异常
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_int			&0x01	)<<3	;		//中断异常
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_CANComm		&0x01	)<<4	;		//CAN2通讯断
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_SCIA			&0x01	)<<5	;		//SCIA	串口
	monitor_all_inf.alm_state[0] += (DrIOP_rev_msg.b1_Wn_SCIB			&0x01	)<<6	;		//SCIB	串口
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP1		&0x01	)<<7	;		//扩展串口1
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP2		&0x01	)<<8	;		//扩展串口2
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP3		&0x01	)<<9	;		//扩展串口3
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP4		&0x01	)<<10	;		//扩展串口4
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP5		&0x01	)<<11	;		//扩展串口5
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP6		&0x01	)<<12	;		//扩展串口6
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP7		&0x01	)<<13	;		//扩展串口7
	monitor_all_inf.alm_state[0] +=	(DrIOP_rev_msg.b1_Wn_externSP8		&0x01	)<<14	;		//扩展串口8
	//IDU
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_warn				&0x01	)<<15	;		//总告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<16	;		//主板电源告警
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<17	;		//主板温度异常
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_int				&0x01	)<<18	;		//中断异常
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_CANComm			&0x01	)<<19	;		//CAN2通讯断
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_SCIA				&0x01	)<<20	;		//SCIA	串口
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_SCIB				&0x01	)<<21	;		//SCIB	串口
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_batLow			&0x01	)<<22	;		//电量低告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_oilLow			&0x01	)<<23	;		//油量低告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_autoOff			&0x01	)<<24	;		//自动脱扣告警
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_oilLevelGauge	&0x01	)<<25	;		//油位计无效告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreVolOver		&0x01	)<<26	;		//岸电过压告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreVolBelow	&0x01	)<<27	;		//岸电欠压告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_ShoreCurOver		&0x01	)<<28	;		//岸电过流告警
	monitor_all_inf.alm_state[0] +=	(IDU_rev_msg.b1_Wn_PORTVolOver		&0x01	)<<29	;		//PORT电池过压告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_PORTVolBelow		&0x01	)<<30	;		//PORT电池欠压告警
	monitor_all_inf.alm_state[0] += (IDU_rev_msg.b1_Wn_PORTCurOver		&0x01	)<<31	;		//PORT电池过流告警
	
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDVolOver		&0x01	)<<0	;		//STBD电池过压告警
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDVolBelow		&0x01	)<<1	;		//STBD电池欠压告警
	monitor_all_inf.alm_state[1] += (IDU_rev_msg.b1_Wn_STBDCurOver		&0x01	)<<2	;		//STBD电池过流告警

	//IOP
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_warn				&0x01	)<<3   	 ;		//总告警
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<4	 ;		//主板电源异常
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<5	 ;		//主板温度异常
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_int				&0x01	)<<6	 ;		//中断异常
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_CANComm			&0x01	)<<7	 ;		//CAN2通讯断
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_SCIA				&0x01	)<<8	 ;		//SCIA	串口
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_SCIB				&0x01	)<<9	 ;		//SCIB	串口
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP1		&0x01	)<<10	 ;		//扩展串口1
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP2		&0x01	)<<11	 ;		//扩展串口2
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP3		&0x01	)<<12	 ;		//扩展串口3
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP4		&0x01	)<<13	 ;		//扩展串口4
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP5		&0x01	)<<14	 ;		//扩展串口5
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP6		&0x01	)<<15	 ;		//扩展串口6
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP7		&0x01	)<<16	 ;		//扩展串口7
	monitor_all_inf.alm_state[1] += (IOP_rev_msg.b1_Wn_externSP8		&0x01	)<<17	 ;		//扩展串口8
	//IHC
	monitor_all_inf.alm_state[1] +=	(IHC_rev_msg.b1_Wn_ClassIWarn		&0x01	)<<18	;		//I级总告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_mainBoardPower	&0x01	)<<19	;		//主板电源异常
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_mainBoardTempe	&0x01	)<<20	;		//主板温度异常
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_SPI				&0x01	)<<21	;		//SPI通讯阻塞
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_1Comm	&0x01	)<<22	;		//扩展串口1告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_2Comm	&0x01	)<<23	;		//扩展串口2告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_3Comm	&0x01	)<<24	;		//扩展串口3告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_TL16C554_4Comm	&0x01	)<<25	;		//扩展串口4告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_ClassIIWarn		&0x01	)<<26	;		//II级总告警
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTMotorWarn	&0x01	)<<27	;		//PORT发动机故障
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_STBDMotorWarn	&0x01	)<<28	;		//STBD发动机故障
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTGearWarn		&0x01	)<<29	;		//PORT制动器故障
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_STBDGearWarn		&0x01	)<<30	;		//STBD制动器故障
	monitor_all_inf.alm_state[1] += (IHC_rev_msg.b1_Wn_PORTRudderWarn	&0x01	)<<31	;		//PORT转向器故障
	monitor_all_inf.alm_state[2] += (IHC_rev_msg.b1_Wn_STBDRudderWarn	&0x01	)<<0	;		//STBD转向器故障
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK1		&0x01   )<<1	;		//K1航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK2		&0x01   )<<2	;		//K2航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK3		&0x01   )<<3	;		//K3航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK4		&0x01   )<<4	;		//K4航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK5		&0x01   )<<5	;		//K5航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK6		&0x01   )<<6	;		//K6航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK7		&0x01   )<<7	;		//K7航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK8		&0x01   )<<8	;		//K8航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK9 		&0x01   )<<9 	;		//K9航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK10		&0x01   )<<10	;		//K10航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK11		&0x01   )<<11	;		//K11航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK12		&0x01   )<<12	;		//K12航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK13		&0x01   )<<13	;		//K13航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK14		&0x01   )<<14	;		//K14航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK15		&0x01   )<<15	;		//K15航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK16		&0x01   )<<16	;		//K16航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK17		&0x01   )<<17	;		//K17航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK18		&0x01   )<<18	;		//K18航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK19		&0x01   )<<19	;		//K19航插连接
	monitor_all_inf.alm_state[2] += (IDU_rev_msg.b1_St_plugConK20		&0x01   )<<20	;		//K20航插连接

	monitor_all_inf.alm_state[2] += (ins_hardState.b_primary	&0x01) << 21;		//K16航插连接
	monitor_all_inf.alm_state[2] += (ins_hardState.b_secondary	&0x01) << 22;		//K17航插连接
	monitor_all_inf.alm_state[2] += (ins_hardState.b_diff		&0x01) << 23;		//K18航插连接
	monitor_all_inf.alm_state[2] += (ins_hardState.b_dGps		&0x01) << 24;		//K19航插连接
	monitor_all_inf.alm_state[2] += (ins_hardState.b_rtcErr		&0x01) << 25;		//K20航插连接


	//通讯监视
	//插入到各个通许模块中更新

	//配置出错信息，暂时不加
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number=0;
	
	//障碍物信息更新 USV_RM_MSG.RM_radar_Msg[i].obstacle_locate.lat
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


	//apf_obs 障碍物信息更新
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

	//zmq_obs 障碍物信息更新
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






	//船的轨迹
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

	//人工势场 避障航点
	if(apf_valid == 1)
	{
		monitor_all_inf.monitor_curve_real_data.exp_destination.lat = int32(apf_dstCalc.lat * 360000.0);
		monitor_all_inf.monitor_curve_real_data.exp_destination.lng = int32(apf_dstCalc.lng * 360000.0);
	}
	else if (pAutoReturnInst->isAutoReturnRunning()){	//自动返航航点
		monitor_all_inf.monitor_curve_real_data.exp_destination.lat = int32(pAutoReturnInst->getDestPoint().lat * 360000);
		monitor_all_inf.monitor_curve_real_data.exp_destination.lng = int32(pAutoReturnInst->getDestPoint().lng * 360000);
	}
	else{	//航行任务航点
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

//通讯信息
//	test_start =test_start+10;
	for(loop_i=0;loop_i<ALL_MONITOR_COMM_NUMBER;loop_i++){		
		monitor_all_inf.monitor_comm_inf[loop_i].send_ok_number =test_start+loop_i+1;		//发送正确帧数
		monitor_all_inf.monitor_comm_inf[loop_i].send_error_number =test_start+loop_i+2;	//发送出错帧数
		monitor_all_inf.monitor_comm_inf[loop_i].rec_ok_number =test_start+loop_i+3;		//接收正确帧数
		monitor_all_inf.monitor_comm_inf[loop_i].rec_error_number =test_start+loop_i+4;	//接收出错帧数
		monitor_all_inf.monitor_comm_inf[loop_i].send_err_ID =test_start+loop_i+5;			//发送出错代号
		monitor_all_inf.monitor_comm_inf[loop_i].rec_err_ID =test_start+loop_i+6;			//接收出错代号						
	}
//主要监视参数
	for(loop_i=0;loop_i<MONITOR_REAL_DATA_NUMBER;loop_i++){
	//	monitor_all_inf.monitor_main_real_data[loop_i].expect_data = (float)2.0; //(test_start+loop_i+0.01);
		monitor_all_inf.monitor_main_real_data[loop_i].output_data = (float)2.0; //(test_start+loop_i+0.02);
		monitor_all_inf.monitor_main_real_data[loop_i].real_data   = (float)2.0; //(test_start+loop_i+0.03);
		monitor_all_inf.monitor_main_real_data[loop_i].delta_data  = (float)2.0; //(test_start+loop_i+0.4);
	}
//次要监视参数
	for(loop_i=0;loop_i<MONITOR_SUB_REAL_DATA_NUMBER;loop_i++){
		monitor_all_inf.monitor_sub_real_data[loop_i]=(float)(test_start+loop_i+0.1);
	}

//曲线实时数据
	////当前航点
	//monitor_all_inf.monitor_curve_real_data.current_locate.lng =test_start*10000+1;	//维度
	//monitor_all_inf.monitor_curve_real_data.current_locate.lat =test_start*10000+2;	//经度
	////航速
	//monitor_all_inf.monitor_curve_real_data.ship_speed.expect_data =(float)(test_start+1+0.01);
	//monitor_all_inf.monitor_curve_real_data.ship_speed.output_data =(float)(test_start+2+0.01);
	////航向
	//monitor_all_inf.monitor_curve_real_data.ship_direction.expect_data =(float)(test_start+3+0.01);
	//monitor_all_inf.monitor_curve_real_data.ship_direction.output_data =(float)(test_start+4+0.01);
	////左发动机
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.expect_data =(float)(test_start+5+0.01);
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.output_data =(float)(test_start+6+0.02);
	//monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.real_data =(float)(test_start+7+0.03);
	////右发动机		
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.expect_data =(float)(test_start+8+0.04);
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.output_data =(float)(test_start+9+0.05);
	//monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.real_data =(float)(test_start+10+0.06);
	////左舵
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.expect_data =(float)(test_start+11+0.07);
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.output_data =(float)(test_start+12+0.08);
	//monitor_all_inf.monitor_curve_real_data.left_rudder_speed.real_data =(float)(test_start+13+0.09);
	////右舵
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.expect_data =(float)(test_start+14+0.10);
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.output_data =(float)(test_start+15+0.11);
	//monitor_all_inf.monitor_curve_real_data.right_rudder_speed.real_data =(float)(test_start+16+0.12);
	////左档位
	//monitor_all_inf.monitor_curve_real_data.left_gear.expect_data =(float)(test_start+17+0.13);
	//monitor_all_inf.monitor_curve_real_data.left_gear.output_data =(float)(test_start+18+0.14);

	////右档位
	//monitor_all_inf.monitor_curve_real_data.right_gear.expect_data =(float)(test_start+20+0.15);
	//monitor_all_inf.monitor_curve_real_data.right_gear.output_data =(float)(test_start+21+0.16);
		
	//当前航点
	monitor_all_inf.monitor_curve_real_data.current_locate.lng += 2	;	//维度
	monitor_all_inf.monitor_curve_real_data.current_locate.lat += 1	;	//经度
	//航速
	monitor_all_inf.monitor_curve_real_data.ship_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.ship_speed.output_data =(float)(2.0);
	//航向
	monitor_all_inf.monitor_curve_real_data.ship_direction.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.ship_direction.output_data =(float)(2.0);
	
	monitor_all_inf.monitor_curve_real_data.ship_direction.output_data = (monitor_all_inf.monitor_curve_real_data.ship_direction.output_data+1)>360? 0:(monitor_all_inf.monitor_curve_real_data.ship_direction.output_data+1);


	//左发动机
//	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_Accelerator_speed.real_data =(float)(2.0);
	//右发动机		
//	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_Accelerator_speed.real_data =(float)(2.0);
	//左舵
//	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_rudder_speed.real_data =(float)(2.0);
	//右舵
//	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.output_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_rudder_speed.real_data =(float)(2.0);
	////左档位
	//monitor_all_inf.monitor_curve_real_data.left_gear.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.left_gear.output_data =(float)(2.0);

	////右档位
	//monitor_all_inf.monitor_curve_real_data.right_gear.expect_data =(float)(2.0);
	monitor_all_inf.monitor_curve_real_data.right_gear.output_data =(float)(2.0);

	//目标航点
	monitor_all_inf.monitor_curve_real_data.exp_destination.lng = 20;
	monitor_all_inf.monitor_curve_real_data.exp_destination.lat = 10;

	//航迹偏差
	monitor_all_inf.monitor_curve_real_data.track_error = 0.1;


//无人船运行状态
	for(loop_i=0;loop_i<MONITOR_USV_EQU_STATE_NUMBER;loop_i++){
		monitor_all_inf.usv_equ_state[loop_i]=0x1;
	}
	monitor_all_inf.usv_equ_state[0]=0;
	monitor_all_inf.usv_equ_state[1]=1;
	monitor_all_inf.usv_equ_state[2]=2;
	monitor_all_inf.usv_equ_state[3]=3;
	monitor_all_inf.usv_equ_state[4]=4;
	monitor_all_inf.usv_equ_state[5]=5;
//告警状态
	for(loop_i=0;loop_i<MONITOR_USV_ALM_NUMBER;loop_i++){
		monitor_all_inf.alm_state[loop_i]=0x00000000;
	}
//调试信息
	for(loop_i=0;loop_i<MONITOR_TMP_DATA_NUMBER;loop_i++){
		monitor_all_inf.tmp_data[loop_i]=test_start+1000+loop_i;
	}
	monitor_all_inf.tmp_data[0]=100;
	monitor_all_inf.tmp_data[1]=0x200;
	tmp_float =(float)12.34;
	p_uint32 =(uint32 *)&tmp_float;
	monitor_all_inf.tmp_data[2]=*p_uint32;
	monitor_all_inf.tmp_data[3]=((10*60+20)*60+30)*100+45;

//配置出错信息
	monitor_all_inf.monitor_cfg_alm_describe.cfg_alm_item_number=MONITOR_MAX_CFG_ERROR_ITEM;
	for(loop_i=0;loop_i<MONITOR_MAX_CFG_ERROR_ITEM;loop_i++){
		p_int8=(int8 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].string_title[0];
		sprintf_usv(p_int8,"aaaaaa%d",loop_i);
		p_int8=(int8 *)&monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].string_line[0];
		sprintf_usv(p_int8,"bbbbbb%d",loop_i);
		monitor_all_inf.monitor_cfg_alm_describe.monitor_cfg_alm_describe[loop_i].param1=loop_i+1;
	}
//各模块报文
	for(loop_i=0;loop_i<ALL_MONITOR_COMM_NUMBER;loop_i++){
		for(loop_j=0;loop_j<400;loop_j++){
			monitor_all_inf.module_report_inf[loop_i].report_detail[loop_j]=(uint8)(loop_j+loop_i);
		}
	}
//仿真船障碍物
	ship_simulate_param.monitor_ship_run_inf.obstacle_number = 10;
	for(loop_i=0;loop_i<MONITOR_OBSTACLE_MAX_NUMBER;loop_i++){
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lat = loop_i*150;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_locate.lng = loop_i*100;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_direction  = loop_i+2;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_radius     = loop_i+3;
	ship_simulate_param.monitor_ship_run_inf.obstacle_locate_inf[loop_i].obstacle_speed      = loop_i+4;
	}

//PID参数设置
	init_flash_setting_default();
//PC仿真面板
	monitor_all_inf.rec_pc_simulate_mmi.mmi_state=SWITCH_ON;
	for(loop_i=0;loop_i<4;loop_i++){
		for(loop_j=0;loop_j<8;loop_j++){
			monitor_all_inf.rec_pc_simulate_mmi.mmi_can_report[loop_i].rec_mmi_buff[loop_j]=loop_i*0x10+loop_j;
		}
	}
}

