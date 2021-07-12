#include "stdafx.h"
#include "../include/auto_return.h"
#include <stdarg.h>
#include <string>
#include <iostream>
#include <planning/pathplanner.h>


using std::cout;
using std::endl;

CAutoReturn *pAutoReturnInst;	//�͵����Զ�����
static const float second_ord_limt = 0.25;
static const float p = 0.1;
static bool switch_wp = true;


CAutoReturn::CAutoReturn(const string& cfgFileName)
{
	m_IsCfgValid = read_AutoReturn_Setting(cfgFileName);
	reset_AutoReturn(); //״̬��ʼ��
}


CAutoReturn::~CAutoReturn()
{
}

bool CAutoReturn::read_AutoReturn_Setting(string cfgFileName)
{
	FILE *pFile;
	int8 *p_file_memory;				//������
	int32 *p_buffer;
	uint32 lSize;
	int32 result;
	uint32 len;
	int8 s1[32];
	int8 s2[50];
	uint16	loop_i;
	int8	ret_val = TRUE;

	pFile = fopen(cfgFileName.c_str(), "rb+");

	if (pFile == NULL){
		printf("read auto return setting error\n");
		m_IsCfgValid = false;
	//	SysLogMsgPost("�Զ����������ļ�����,�ļ���ʧ��");
	//	SysPubMsgPost("�Զ����������ļ�����,�ļ���ʧ��");
		return FALSE;
	}

	p_file_memory = (int8 *)malloc(0x4fff);				//16K
	if (NULL == p_file_memory)
	{
		printf("auto return setting file memory not enough\n");
		fclose(pFile);
		return FALSE;
	}

	p_buffer = (int32 *)malloc(0x10000);				//64K
	if (NULL == p_buffer)
	{
		printf("auto return setting explain memory\n");
		free(p_file_memory);
		fclose(pFile);
		m_IsCfgValid = false;
	//	SysLogMsgPost("�Զ����������ļ�����,��������");
	//	SysPubMsgPost("�Զ����������ļ�����,��������");
		return FALSE;
	}

	// ��ȡ�ļ���С 
	fseek(pFile, 0, SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile);					//��ָ��ָ���ļ���ͷ

	if (lSize >= 0xffff){
		printf("auto return setting read file too large\n");
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		m_IsCfgValid = false;
		//SysLogMsgPost("�Զ����������ļ�����,�ļ�����");
		//SysPubMsgPost("�Զ����������ļ�����,�ļ�����");
		return FALSE;
	}


	result = fread(p_file_memory, 1, lSize, pFile);			 // ���ļ�������buffer�� 


	if (32768 < lSize) len = 2 * lSize;
	else len = 1280 + 2 * lSize;
	len = len * 2;
	result = ini_Initialize((char *)p_file_memory, p_buffer, len);

	if (result != 0){										//�ļ���ʼ������
		printf("auto return setting  memory explain error\n");
		free(p_file_memory);
		free(p_buffer);
		fclose(pFile);
		m_IsCfgValid = false;
		//SysLogMsgPost("�Զ����������ļ�����,�ڴ����");
		//SysPubMsgPost("�Զ����������ļ�����,�ڴ����");
		return FALSE;
	}

	//��ʼ���������ļ�
	sprintf_usv(s1, "AUTO_RETURN");
	//����ʹ��
	sprintf_usv(s2, "func_enable");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&m_FuncEnable, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//������
	sprintf_usv(s2, "dock_ID");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&m_DockID, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}
	//����λ��
	sprintf_usv(s2, "dock_pos");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_DockPos.lng), FLOAT_TYPE) == FALSE){		//lng
		ret_val = FALSE;
	}
	if (read_sub_setting_df(s1, s2, 1, (uint64 *)&(m_DockPos.lat), FLOAT_TYPE) == FALSE){		//lat
		ret_val = FALSE;
	}
	//�������λ��
	sprintf_usv(s2, "dock_getInPos");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_ReturnPos.lng), FLOAT_TYPE) == FALSE){		//lng
		ret_val = FALSE;
	}
	if (read_sub_setting_df(s1, s2, 1, (uint64 *)&(m_ReturnPos.lat), FLOAT_TYPE) == FALSE){		//lat
		ret_val = FALSE;
	}

	//������·
	uint32 pathPointNum = 0;
	sprintf_usv(s2, "return_path_pointSum");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&pathPointNum, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//�Զ���������
	sprintf_usv(s2, "return_power_limit");
	if (read_sub_setting(s1, s2, 0, (uint32 *)&m_ReturnPowerLimit, INT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//��������
	sprintf_usv(s2, "exp_speed");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_ExpSpeed), FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//���ﺽ��
	sprintf_usv(s2, "arr_speed");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_ArrSpeed), FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//�������
	sprintf_usv(s2, "arr_dst");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_ArrDst), FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	//���پ���
	sprintf_usv(s2, "dec_dst");
	if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(m_DecDst), FLOAT_TYPE) == FALSE){
		ret_val = FALSE;
	}

	m_ReturnPath.clear();

	for (loop_i = 0; loop_i < pathPointNum; loop_i++){
		sprintf_usv(s2, "return_path_point_%d", loop_i);
		POSITION tmp_Position;
		if (read_sub_setting_df(s1, s2, 0, (uint64 *)&(tmp_Position.lng), FLOAT_TYPE) == FALSE){		//lng
			ret_val = FALSE;
		}
		if (read_sub_setting_df(s1, s2, 1, (uint64 *)&(tmp_Position.lat), FLOAT_TYPE) == FALSE){		//lat
			ret_val = FALSE;
		}
		//cout<< "********" << tmp_Position.lng << " ," << tmp_Position.lat << endl;
		m_ReturnPath.push_back(tmp_Position);
	}



	free(p_file_memory);
	free(p_buffer);
	fclose(pFile);
	printf("read autoReturn setting ok\n");

	return ret_val;
}

void CAutoReturn::JudgeNeedPowerOffAutoReturn(uint32 powerPercent)
{
	static int judge_count = 0;//3���ж� �õ��͵���������Զ�����
	if (powerPercent < m_ReturnPowerLimit){
		judge_count++;
		if (3 == judge_count)
		{
			if (m_bPowerLowNeedReturn == false){
				WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_POWER_LOW_RETURN, WARN_ON);
			}
			m_bPowerLowNeedReturn = true;
			judge_count = 0;
		}

	}
	else{
		if (m_bPowerLowNeedReturn == true){
			judge_count = 0;
			reset_AutoReturn();
			WarnMsgQueuePut(WARN_SRC_ARM, ARM_WARN_POWER_LOW_RETURN, WARN_OFF);
		}
		m_bPowerLowNeedReturn = false;
	}
}

bool CAutoReturn::IsLowPowerAutoReturnNeeded(void)
{
	if (m_FuncEnable == 1){
		return (m_bPowerLowNeedReturn);
	}
	else{
		return false;
	}

}

void CAutoReturn::reset_AutoReturn(void)
{
	m_retProcess = RETURN_NONE;

	m_ReturnPointSn = 0;				//�Զ�������·ִ�����
	m_bIsReturnPathProc = false;		//�Զ������Ƿ�ִ��
	m_bIsReturnPathFinish = false;		//�Զ�������·�Ƿ�ִ����
	m_bIsReturnPosArrived = false;		//�Զ���������λ���Ƿ񵽴�
	m_bIsAutoGetInDockReady = false;	//�Զ������Ƿ����
	TurnOffAutoReturn();
}

int8 CAutoReturn::ControlAutoReturnMain(void)
{
	//printf("---AutoReturn == %d \t m_retProcess == %d \n", m_retProcess,m_FuncEnable);
	if (m_FuncEnable != 1)	//���ܹر�
	{
		return m_retProcess;
	}
	

	if (IsLowPowerAutoReturnNeeded()){
		usv_sign.log_b1_return_state = 2;	//�͵�������
		//ȡ����������
		if (isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID) == FALSE && sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type == 1){
			sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
		//	SysPubMsgPost("ˮ�ʲ����������͵�����������������,ִ�е�%d����", sailTask.u8_PointNum + 1);
		}
	}
	else if (IsForceAutoReturnNeeded()){
		usv_sign.log_b1_return_state = 1;	//һ������
		//ȡ����������
		if (isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID) == FALSE && sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type == 1){
			sampleCancel(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID);
			sailTask.u8_PointNum++;
			//SysPubMsgPost("ˮ�ʲ���������һ����������������,ִ�е�%d����", sailTask.u8_PointNum + 1);
		}
	}
	else{
		usv_sign.log_b1_return_state = 0;
	}
	switch (m_retProcess)
	{
	case CAutoReturn::RETURN_NONE:AutoReturn_JudgeStart();
		break;
	case CAutoReturn::RETURN_PATH:AutoReturn_SailPath();  //预设航路
		break;
	case CAutoReturn::RETURN_GPOINT:AutoReturn_GDPoint(); //低电量返航点 / 进坞预设点
		break;
	case CAutoReturn::RETURN_DOCK:AutoReturn_GDock();
		break;
	default:
		break;
	}
	autoNaviSt.b1_st_apf = auto_return_st.b1_st_apf;
	//cout << "AutoReturn :" << m_retProcess << endl;
	return m_retProcess;
}

void CAutoReturn::AutoReturn_JudgeStart(void)
{
	if (m_IsCfgValid && (m_bPowerLowNeedReturn || m_bForceAutoReturn))
	{
		//m_retProcess = CAutoReturn::RETURN_PATH;
		m_retProcess = CAutoReturn::RETURN_GPOINT; //RETURN_DOCK

		cout << "###begin auto return sail way" << endl;
	}
	else{
	}
	init_ret_point.lat = ins_msg.latitude;	 //����һ������ λ�û���
	init_ret_point.lng = ins_msg.longitude;	 //
}

void CAutoReturn::AutoReturn_SailPath(void)
{
	POSITION last_point, target_point;

	if (m_ReturnPointSn >= m_ReturnPath.size())
	{
			//返航航路执行完毕
		m_retProcess = CAutoReturn::RETURN_GPOINT;
		cout << "### begin to return point " << endl;
		return;
	}

	last_point = init_ret_point;
	target_point = m_ReturnPath.at(m_ReturnPointSn);

	//	如果需要，避障功能在此处实现

	//计算距离

	double dst = Get_distance(ins_msg.latitude,ins_msg.longitude,target_point.lat,target_point.lng);

	double heading = Get_heading(ins_msg.latitude,ins_msg.longitude,target_point.lat,target_point.lng);

	//避障检测与避障
	auto_return_st.b1_st_apf = APF_calc(target_point);
	if (auto_return_st.b1_st_apf == 1)	{ //避障
		auto_aovid_return_plan(last_point, target_point);	//避障规划
	}
	else{
		auto_return_plan(last_point, target_point);			//LOS跟踪航迹设定
	}


	if (auto_return_st.double_dst < m_DecDst){  //到达 切换
		cout << "##### sail point " << m_ReturnPointSn << "reach" << endl;

		init_ret_point = m_ReturnPath.at(m_ReturnPointSn);	 //多个返航路径点情况下 重新初始化返航触发位置

		m_ReturnPointSn++;
		switch_wp = true;
		return;
	}else{

		nCalRudderOpenDeg(auto_return_st.double_heading_exp, m_ExpSpeed);//速度航向 输出

	}
}

void CAutoReturn::AutoReturn_GDPoint(void)
{
	POSITION last_point, target_point;
	last_point = init_ret_point;

	getLatLng(return_point[2].latitude, return_point[2].longitude, temp_dockin_distance, (180 + return_point[2].heading), &target_point.lat, &target_point.lng); //��ȡ���󷽵ķ�����
	double dst = Get_distance(ins_msg.latitude,ins_msg.longitude,target_point.lat,target_point.lng);
	double heading = Get_heading(ins_msg.latitude,ins_msg.longitude,target_point.lat,target_point.lng);

	// auto_return_st.b1_st_apf = APF_calc(target_point);
	auto_return_st.double_dst = Get_distance(ins_msg.latitude, ins_msg.longitude, target_point.lat, target_point.lng);

	// if (auto_return_st.b1_st_apf == 1)	{ 
	// 	auto_aovid_return_plan(last_point, target_point);	
	// }
	// else{
	// 	auto_return_plan(last_point, target_point);			
	// }


    // translate current lat-lng into xy  
    LOC::Location _oa_origin;            // intermediate origin during avoidance
    LOC::Location _oa_destination;       // intermediate destination during avoidance
    LOC::Location _origin;               // origin Location (vehicle will travel from the origin to the destination)
    LOC::Location _destination;          // destination Location when in Guided_WP

    _origin.lat = last_point.lat*1e7;
    _origin.lng = last_point.lng*1e7;
    _origin.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
    _destination.lat = target_point.lat*1e7;
    _destination.lng = target_point.lng*1e7;
    _destination.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    Location current_loc;
    current_loc.lat = ins_msg.latitude*1e7;
    current_loc.lng = ins_msg.longitude*1e7;
    current_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    // run path planning around obstacles
    bool stop_vehicle = false;
    // true if OA has been recently active;
	bool _oa_active = false;

	 AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc, _origin, _destination,ins_msg.heading, _oa_origin, _oa_destination);
        switch (oa_retstate) {
        case AP_OAPathPlanner::OA_RetState::OA_NOT_REQUIRED:
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_PROCESSING:
        case AP_OAPathPlanner::OA_RetState::OA_ERROR:
            // during processing or in case of error, slow vehicle to a stop
            stop_vehicle = true;
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_SUCCESS:
            _oa_active = true;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_CAN_NOT_ARRIVAL:
             stop_vehicle = true;
            _oa_active = false;
            break;
        }
    }
    if (!_oa_active) {
        _oa_origin = _origin;
        _oa_destination = _destination;
    }

	POSITION start,end;
	start.lat = 1e-7*_oa_origin.lat;
	start.lng = 1e-7*_oa_origin.lng;
	end.lat   = 1e-7*_oa_destination.lat;
	end.lng   = 1e-7*_oa_destination.lng;

	auto_return_plan(start, end);

	if (auto_return_st.double_dst < autoNaviCfg.u16_arrival_distance1){ 
		if (usv_sign.entry_docking == 1){
			m_retProcess = CAutoReturn::RETURN_DOCK;
			cout << "receive the dock entrance" << endl;
			return;
		}
		else{
			//nCalRudderOpenDeg(heading, 1);
			set_steering(0.0);
			set_throttle(0.0);
		}
	}
	else{

		nCalRudderOpenDeg(auto_return_st.double_heading_exp, auto_return_st.double_speed_exp); //�ٶȺ��� ���
	}
}

void CAutoReturn::AutoReturn_GDock(void)
{
	if (/*usv_sign.entry_docking == 1 && */dockInUSVControl()){//�����Ƿ�ɹ�
			usv_sign.log_b1_dock_state = 1;//�ѽ���
			reset_AutoReturn();//��λ�Զ�����
		    set_throttle(0.25);
		    set_steering(0);
		//	nCalRudderOpenDeg(ins_msg.heading, 0);//����ɹ��� �ٶȹ���
	}
	else{
			usv_sign.log_b1_dock_state = 0;
	}

	//����ԭ����ͣ
	//arriveStandby2(m_ReturnPos.lat, m_ReturnPos.lng);
}

void CAutoReturn::DoRudderOpenDeg(double speed, double heading)
{
	double openDeg;
	double rudder;

	//�л���ARM��������

	//�ֺ��� ����PID
	if (ins_msg.speed > autoNaviCfg.double_pid_speed_threshold)
		rudder = simu_heading_TJ_PID(heading, ins_msg.heading);
	else
		rudder = simu_heading_TJ_PID_LowSpeed(heading, ins_msg.heading);

	openDeg = simu_speed_PID(speed, ins_msg.speed);

	//cout << "rudder = " << rudder << " openDeg = " << openDeg << endl;

	//���ſ���
	jet_system.jetL.u8_Cmd_MotorOpenDeg = openDeg;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = openDeg;
	//���
	jet_system.jetL.i16_Cmd_MotorRudderDeg = rudder;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = rudder;
	//��λ
	jetGearFoward();	//ǰ��
}

void CAutoReturn::TurnOnAutoReturn(void)
{
	m_bForceAutoReturn = true;
}

void CAutoReturn::TurnOffAutoReturn(void)
{
	m_bForceAutoReturn = false;
	usv_sign.log_b1_return_state = 0;
}

CAutoReturn::retProcess CAutoReturn::getReturnState(void)
{
	return m_retProcess;
}

bool CAutoReturn::isAutoReturnRunning(void)
{
	return m_IsCfgValid && (m_bPowerLowNeedReturn || m_bForceAutoReturn);
}

POSITION CAutoReturn::getDestPoint(void)
{
	POSITION iretPostion = {0.0,0.0};

	switch (m_retProcess)
	{
	case CAutoReturn::RETURN_NONE:
		break;
	case CAutoReturn::RETURN_PATH:
		if (m_ReturnPath.size() > 0){
			iretPostion = m_ReturnPath[m_ReturnPointSn];
		}
		break;
	case CAutoReturn::RETURN_GPOINT:
		iretPostion = m_ReturnPos;
		break;
	case CAutoReturn::RETURN_DOCK:
		iretPostion = m_DockPos;
		break;
	default:
		break;
	}
	return iretPostion;
}

bool CAutoReturn::isAutoReturnCfgValid(void)
{
	return m_IsCfgValid;
}

bool CAutoReturn::IsForceAutoReturnNeeded(void)
{
	if (m_FuncEnable == 1){
		return  m_bForceAutoReturn;
	}
	else{
		return false;
	}
}

bool CAutoReturn::IsAutoReturnNeeded(void)
{
	if (m_FuncEnable == 1){
		return (m_bPowerLowNeedReturn || m_bForceAutoReturn);
	}
	else{
		return false;
	}
}

void CAutoReturn::auto_return_plan(POSITION last_point, POSITION target_point)
{

	float angleFix = 0.0;
	double exp_speed;
	//	static double start_lat, start_lng;

	float final_speed = 0;

	exp_speed = m_ExpSpeed; 

	final_speed = ms2kn(p * autoNaviCfg.u16_arrival_distance1);

	auto_return_st.double_speed_exp = constrain_value(sqrt_controller(auto_return_st.double_dst, p, second_ord_limt), 0, exp_speed);
	auto_return_st.double_heading_exp = Get_heading(ins_msg.latitude, ins_msg.longitude, target_point.lat, target_point.lng);

		float wp_heading = Get_heading(last_point.lat, last_point.lng, target_point.lat, target_point.lng);

		float  yaw_err = wrap_180(wp_heading - ins_msg.heading, 1);

		/*if (switch_wp == true)
		{
			if (fabsf(yaw_err) <= 5.0f && fabsf(ins_msg.rotRate) <= 1)
			{
				switch_wp = false;
			}

			float ratio = 1 - 0.5 *constrain_value(fabsf(yaw_err / 90), 0, 1);
			auto_return_st.double_speed_exp = final_speed * ratio;
			auto_return_st.double_heading_exp = wp_heading;
		}
		else
		{*/
			auto_return_st.double_speed_exp = constrain_value(sqrt_controller(auto_return_st.double_dst, p, second_ord_limt), 0, exp_speed);

			auto_return_st.double_heading_exp = trackingPathLos(last_point.lat, last_point.lng, \
				target_point.lat, target_point.lng, \
				ins_msg.latitude, ins_msg.longitude);
		//}
	
}

float CAutoReturn::sqrt_controller(float error, float p, float second_ord_lim)
{
	if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p))
	{
		return error*p;
	}
	float linear_dist = second_ord_lim / sq(p);
	if (error > linear_dist)
	{
		return sqrtf(2.0f*second_ord_lim*(error - (linear_dist / 2.0f)));
	}
	else if (error < -linear_dist)
	{
		return -sqrtf(2.0f*second_ord_lim*(-error - (linear_dist / 2.0f)));
	}
	else
	{
		return error*p;
	}
}

void CAutoReturn::auto_aovid_return_plan(POSITION last_point, POSITION target_point)
{
	//�˹��Ƴ�����


	// ��κܿ���Ŀ�꺽�㣬�����ڱ��ϣ���ʱͣ���л�����
	if (auto_return_st.double_dst <= 10)
	{
		auto_return_st.double_speed_exp = 0;
		auto_return_st.double_dst = 0; //��Ϊ���� �����ϰ��︽���ĺ�·��
	}

	auto_return_st.double_heading_exp = apf_heading_cd;
}

