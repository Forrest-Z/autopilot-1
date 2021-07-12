/********************************************************************
-	Copyright (c),2017-	,�ķ��̱����人���������޹�˾
-	File name  : docking_usv_control.cpp
-	Author	   : fushuai
-	Date	   : 2019/06/03 19:48
-   Version    : 1.0
-	Description:
-	Others:
*********************************************************************/
#include "stdafx.h"
#include "../include/docking_usv_control.h"
#include "../include/scu_io.h"
#include "../include/docking_main.h"

int16_t	heading_err = 0;
uint8_t	gnss_ctrl_err = 0;
uint8   re_dock_in = 0;
static int16_t gear_cmd = 0;
static int16_t sterr_cmd = 0;
static uint16_t throttle_cmd = 0;
static float throttle_last=0;//out dock throttle init value;


static uint8 gridBackReturn(RETURN_POINT point);
static int8 autoDockingModeJudge(RETURN_POINT last_point, RETURN_POINT point);
static RETURN_POINT return_recv_point = { 0.000, 0.000, 0.000 };
static int8 autoReturn(RETURN_POINT last_point, RETURN_POINT point);
static uint8 autoReturnLOS(double fix_heading);
static int8 autoReturnLOSLidar(double target_dis, double track_error, double angle_exp);
static uint8 gridBackReturnLidar(double target_dis, double track_err);
static uint8 backMotion();
static uint8 idleMotion();

//�����������
int8 dockOutUSVControl(void)
{
	static int count = 0;
	static time_t start;
	time_t end;
	
	static uint32 tlast = millis();
	uint32 tnow = millis();
	float throttle_this = 0.0;

	int ret = 0;
	static uint16  start_sec_out, start_min_out;
	uint16  end_sec_out, end_min_out;
	static uint8 dockout_state = 0;
	RETURN_POINT usv_startup = {0.0,ins_msg.longitude, ins_msg.latitude };
	RETURN_POINT usv_dockout = { 0.0, ins_msg.longitude, ins_msg.latitude };

#if 1

// 	if (10 <= Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude)){//λ�ó���ɹ�
// 
// 		dspControlOFF();
// 
// 	}
// 	else{
// #ifndef WINNT
// 		
//#else
// 		//TODO:win32�޿��ƺ��˿���
// #endif
// 
// 	}

	//(0 == dock_sign.x_in_entrance) && 

	//�ߵ����¶�λ ���������Ϊ ����8 ˫���߶���
#if 0


	if ((ins_msg.insState.c_rmcValid == 'A' && ins_msg.latitude != 0 && ins_msg.longitude != 0 && \
		return_point[2].latitude != 0 && return_point[2].longitude != 0 && \
		(4 * l_ship <= Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude)) || 0 == dock_sign.x_in_entrance)
		|| (1 == dock_sumlink.x_outsuccess)){
#endif
		if (0 == dock_sign.x_in_entrance||1 == dock_sumlink.x_outsuccess){
		usv_sign.succcessed_out = 1;
		usv_sign.out_docking = 0;
		set_steering(0.0);
		set_throttle(0.0);
		throttle_last = 0.0;
		ret = 1;
	}else{
		set_steering(0.0);
		throttle_this = throttle_last*0.95+(-0.40)*0.05;
		throttle_this = throttle_this <= -0.4?-0.4:throttle_this;
		throttle_last = throttle_this;
		set_throttle(throttle_this);//此时没有GPS，后退给固定油门
		ret = 0;
	}
	return ret;
#endif
#if 0
	if (return_point[2].longitude != 0 && return_point[2].latitude != 0){ //�жϵ��Ƿ���Ч

		getLatLng(return_point[2].latitude, return_point[2].longitude, temp_dockin_distance, (180 + return_point[2].heading), &usv_dockout.latitude, &usv_dockout.longitude); //todo 

	}

	if ((ins_msg.insState.c_rmcValid == 'A' && ins_msg.latitude != 0 && ins_msg.longitude != 0 && return_point[2].latitude != 0 && return_point[2].longitude != 0)&& \
		(4 >= Get_distance(ins_msg.latitude, ins_msg.longitude, usv_dockout.latitude, usv_dockout.longitude) ||
		40 <= Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude))){

		dockout_state = 2;
	}
	if ((tnow - tlast) >= 200){
		dockout_state = 0;
	}
	switch (dockout_state)
	{
	case 0: //����
		if ((ins_msg.insState.c_rmcValid == 'A' && ins_msg.latitude != 0 && ins_msg.longitude != 0 && \
			return_point[2].latitude != 0 &&return_point[2].longitude != 0 && \
			(6 * l_ship <= Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude))) || (1 == dock_sumlink.x_outsuccess)){

			dockout_state = 1; //�������
			dspControlOFF();//ֹͣ
			usv_startup.latitude = ins_msg.latitude; //���浱ǰ��γ��
			usv_startup.longitude = ins_msg.longitude;//���浱ǰ��γ��
		}
		else{
			dspControlCmd(ins_msg.heading, -2, 0); //����
		}
		break;
	case 1: //�Զ�Ѳ������ʼ��
		if (ins_msg.insState.u8_sysState1 == 4 && ins_msg.insState.c_rmcValid == 'A'){ //GNSS���� + GPS��λ��Ч ���Զ�Ѳ������ʼ��
			float tracker_angle = trackingPathLos(usv_startup.latitude, usv_startup.longitude, usv_dockout.latitude, usv_dockout.longitude, ins_msg.latitude, ins_msg.longitude);
			dspControlCmd(tracker_angle , docking_speed , 0);
		}
		break;
	case 2: //������ʼ�� ����ɹ�
	//	SysPubMsgPost("successed out dock!\n");
		dspControlOFF();//ֹͣ
		usv_sign.succcessed_out = 1;//����ɹ�
		usv_sign.out_docking = 0;
		dockout_state = 0;
		break;
	default:
		break;
	}
	tlast = tnow;
	return ret;
#endif
}
//���봬����
static int suc_enter=0;//

int8 dockInUSVControl(void)
{
	int ret = 0;
	int autoreturn = 0;
	static int ret_point1_flag = 0;
	static int in_flag = 0;
	double image_fix_deg = 0.0;
	int motor_open_deg;
	float rudder_deg;
	double dst;
	double exp_speed, exp_heading;
	static int dockin_event = 0;//初始化目标远坞点
	static int first_visual_on = 0;
	double target_error;

	if (usv_sign.succcessed_entry == 1) //已经进坞成功则返回1
	{
		return 1;
	}

	return_recv_point.heading = ins_msg.heading; //缓存接收进坞命令后的起始点 航向
	return_recv_point.latitude = ins_msg.latitude;//缓存接收进坞命令后的起始点 纬度
	return_recv_point.longitude = ins_msg.longitude;//缓存接收进坞命令后的起始点 经度

	target_error = Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude); //����Ŀ�����ƫ��


	//lidar ����λ��
	double angle_exp = return_point[2].heading;
	double angle_real = ins_msg.heading;
	double angle_diff = angle_exp - angle_real;					//计算偏航角度
	double err_x = target_pos_lidar.delta_x / 100.0;			//目标船坞在船坐标轴x方向偏差
	double err_y = target_pos_lidar.delta_y / 100.0;			//目标船坞在船坐标轴y方向偏差
	double local_distance = sqrt(err_x*err_x + err_y*err_y);	//计算当前点到目标点距离
	double track_error = local_distance * sin(atan(err_y / err_x) - Radian(angle_diff));

	//����Lidar����״̬
	sumlink_lidardockin_state.u8_lidar_lock_on = target_pos_lidar.lock_on;
	sumlink_lidardockin_state.f32_lidar_x = err_x;
	sumlink_lidardockin_state.f32_lidar_y = err_y;
	sumlink_lidardockin_state.f64_dis = local_distance;
	sumlink_lidardockin_state.f64_real_angle = angle_real;
	sumlink_lidardockin_state.f64_target_angle = angle_exp;
	switch (dockin_event)
	{
	case 0://第0个返航点 目标为远坞点后退演算出来的点 以带避障自动航行的方式处理
		getLatLng(return_point[2].latitude, return_point[2].longitude, temp_dockin_distance, (180 + return_point[2].heading), &return_point[1].latitude, &return_point[1].longitude); //获取正后方的返航点
		dockin_event = 1;
		break;

	case 1:
		autoreturn = autoReturn(return_recv_point, return_point[1]);
		if (1 == autoreturn)
		{
			dockin_event = 2;
		}else{
			dockin_event = 1;
		}
		break;

	case 2://人为设置的返航点 目标为坞点无需避障 由出坞后自动缓存 
		autoreturn= autoReturnLOS(image_fix_deg);//LOS
		if(autoreturn !=1)
		{
			dockin_event = 0;//重新返回进坞
			re_dock_in = 1;
		}
		break;

	case 3: //���½���״̬
		dspControlCmd(ins_msg.heading, -docking_speed, 0); //当前航向后退
		if (target_error > 8){
			dockin_event = 0;//重新返回进坞
			re_dock_in = 1;
		}
		break;
	default:
		break;
	}
	if (dock_sign.x_in || dock_sumlink.x_in){//进坞成功//test @foo
		usv_sign.succcessed_entry = 1;
		//SysLogMsgPost("�յ�����ɹ��ź�...\n");
		//SysPubMsgPost("successed entry dock!\n");
	//	SysLogMsgPost("successed entry dock!\n");
		printf("successed entry dock!\n");
		ret = 1;
		first_visual_on = 0; //����Ŀ������״̬��λ
		dockin_event = 0;
		dock_zmq_cmd.dockin_cmd = CMAERA_TRACKOFF;
		dspControlOFF(); //�ر�DSP�������� ����
		suc_enter=0;
	}
	else if (dockin_event == 2 && dock_sign.x_in != 1){  //未进坞成功
		RETURN_POINT temp_point;
		temp_point.heading = ins_msg.heading;
		temp_point.latitude = ins_msg.latitude;
		temp_point.longitude = ins_msg.longitude;

		uint8 bak_return = gridBackReturn(temp_point);
		if (bak_return == 1){
			//dockin_event = 3;//���·����¼�
		}
	}
	else{
	//	SysLogMsgPost("�ȴ�����ɹ��ź�...\n");
	}
	s_sleep(10);
	return ret;
}


//�жϷ������Ƿ񵽴�
int8 autoDockingModeJudge(RETURN_POINT last_point, RETURN_POINT point)
{
	int8 iret = 0;
	if (autoNaviSt.double_dst < 3)
	{
		iret = 1;
	}
	if (autoNaviSt.double_dst < 0.8) //暂时先不用配置的文件
	{
		iret = 2;
	}
	return iret;
}

//�ޱ����Զ�����
int8 autoReturn(RETURN_POINT last_point, RETURN_POINT point)
{
	int ret = 0;
	double  dev_angle;
	float modify_angle;
	double target_distance = Get_distance(last_point.latitude, last_point.longitude, point.latitude, point.longitude);
	double angle_exp = wrap_360_cd(Get_heading(last_point.latitude, last_point.longitude, point.latitude, point.longitude)); //����apf�õ��ĺ���ĺ���


	if (target_distance <= autoNaviCfg.u16_arrival_distance1) // 到达
	{
		ret = 1; //到达航点
		//dspControlCmd(0,0,0); //原地保持目标航向
		set_throttle(0);
		set_steering(0);
		suc_enter=0;

	}else{
		ret = 0;//
		dspControlCmd(angle_exp,docking_speed,0);//按进坞指定速度前行
	}
	return ret;
}
uint8 autoReturnLOS(double fix_heading)
{
	int8 ret;
	float speed_fix;
	RETURN_POINT temp_dock;
	getLatLng(return_point[2].latitude, return_point[2].longitude, 5*l_ship, return_point[2].heading, &temp_dock.latitude, &temp_dock.longitude); //��ȡ��ǰ5�������������
	float tracker_angle = trackingPathLos(return_point[1].latitude, return_point[1].longitude, temp_dock.latitude, temp_dock.longitude, ins_msg.latitude, ins_msg.longitude);
	double local_distance = Get_distance(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude);	//���㵱ǰ�㵽Ŀ������
// 	double angle_real = Get_heading(ins_msg.latitude, ins_msg.longitude, return_point[2].latitude, return_point[2].longitude);	//���㵱ǰ�㵽Ŀ���ĺ���ƫ��
// 	double angle_diff = wrap_180_cd(return_point[2].heading - angle_real);		//计算偏航角度
	double dockin_exp_speed;

	if(1 != dockingIn())
	{
		if(1 == dock_sign.x_in_entrance)
		{
			set_steering(0);
			set_throttle(-0.3);
			return 1;
		}
		else
		{
			return 0;
		}
			
	}

	if ((1 == dock_sign.x_in_entrance /*&& local_distance < 2.5*/)||suc_enter==1){ //接近船坞查询超声波传感器检测船是否到船坞口

		speed_fix = 0.5;
		set_steering(0);
		set_throttle(0.30);
		suc_enter=1;

	}else{
		speed_fix = 1;
		float speed_temp  = constrain_value(sqrt_controller(local_distance,0.1,0.25,0.05),1,docking_speed);
		dockin_exp_speed = MAX(speed_temp,1.0f);
		//if((tracker_angle + fix_heading)<20)
			dspControlCmd(tracker_angle + fix_heading, dockin_exp_speed, 0);
		//else
		  //  dspControlCmd(tracker_angle + fix_heading, 0, 0);
	}
	return 1;
}

uint8 gridBackReturn(RETURN_POINT point)
{
	double ret_err = 0.0;
	double angle_exp = Get_heading(return_point[1].latitude, return_point[1].longitude, return_point[2].latitude, return_point[2].longitude);	//������һ���㵽��һ���㺽���
	double angle_real = Get_heading(point.latitude, point.longitude, return_point[2].latitude, return_point[2].longitude);	//������һ���㵽����λ�õĺ����
	double angle_diff = angle_exp - angle_real;		//����ǶȲ�

	if (angle_diff < 0)
	{
		return 0;
	}

	double local_distance = Get_distance(point.latitude, point.longitude, return_point[2].latitude, return_point[2].longitude);	//���㺽�о���
	double track_error = local_distance * sin(Radian(angle_diff)); //�����˸������ĸ���
	if (local_distance <= 20 && local_distance > 10){
		ret_err = 10;
	}
	else if (local_distance <= 10 && local_distance >= 5){
		ret_err = 5;
	}
	else if (local_distance <= 5 && local_distance >= 1){
		ret_err = 4;
	}
	else if (local_distance <= 1 && local_distance >= 0){
		ret_err = 1.5;
	}
	else{
		return 0;
	}

	if (track_error > ret_err || track_error < -ret_err){
		printf("local_distance == %f \t track_error == %f\n", local_distance, track_error);
		return 1;
	}
	else
	{
		return 0;
	}
}
void getLatLng(double lat, double lng, double dst, double heading, double *Lat, double *Lng)
{
	//	double lat4=30.43393466;//��γΪ������γȡ-39.90744
	//	double lng4=114.40480219;//����Ϊ��������ȡ-116.41615
	//	double dst=580.9095;
	//	double heading=293.4138;
	double c = dst / Earth_Radius;
	double a = acos(cos(Pi*(90.0 - lat) / 180.0)*cos(c) + sin(Pi*(90 - lat) / 180.0)*sin(c)*cos(Pi*(heading) / 180.0));
	double C = asin(sin(c)*sin(Pi*(heading) / 180.0) / sin(a));

	*Lat = 90 - a*180.0 / Pi;
	*Lng = lng + C*180.0 / Pi;
	return;
}

uint8 reDockIn()
{
	return re_dock_in;
}

void resetDockControlEvent()
{
	re_dock_in = 0;
	memset(&dock_sign, 0, sizeof(dock_sign));
}

int8 autoReturnLOSLidar(double target_dis, double track_error, double angle_exp)
{
	double tracker_angle;
#if 1
	int8 ret;
	double delta_los = n_ship*l_ship; // l_ship n_ship���ò���

	//double trans_error_x = 0.0;
	//double trans_error_y = 0.0;
	//trans_error_x = cos(Radian(angle_diff))*error_x + sin(Radian(angle_diff))*error_y;
	//trans_error_y = sin(Radian(angle_diff))*error_x - cos(Radian(angle_diff))*error_y;
	double tracking_path_sigma = Radian(angle_exp) + atan(track_error / delta_los); //����
	static float speed_fix;
#endif

#if 0

	int8 ret;
	double tracker_angle;
	double angle_exp = return_point[2].heading;
	double angle_real = ins_msg.heading;
	double angle_diff = angle_exp - angle_real;		//����ƫ���Ƕ�
	double delta_los = n_ship*l_ship; // l_ship n_ship���ò���
	double trans_error_x = 0.0;
	double trans_error_y = 0.0;

	//trans_error_x = cos(Radian(angle_diff))*error_x + sin(Radian(angle_diff))*error_y;
	//trans_error_y = sin(Radian(angle_diff))*error_x - cos(Radian(angle_diff))*error_y;

	double xyDiff = atan2(error_y, error_x) * 180.0 / Pi; //����������
	//double angleDiff = xyDiff - (angle_real - angle_exp);
	//double dis = sqrt(pow(error_x, 2) + pow(error_y, 2));
	//trans_error_x = cos(Radian(angleDiff))*dis;
	//trans_error_y = sin(Radian(angleDiff))*dis;

	trans_error_x = error_x;
	trans_error_y = error_y;


	double local_distance = sqrt(trans_error_x*trans_error_x + trans_error_y*trans_error_y);	//���㵱ǰ�㵽Ŀ������
	//double track_error = error_y*cos(Radian(angle_diff));
	double track_error = trans_error_y;
	//double track_error = local_distance * sin(atan(error_y / error_x) - Radian(angle_diff));
	//double tracking_path_sigma = Radian(angle_exp) - atan(track_error / delta_los); //����
	double tracking_path_sigma = Radian(angle_exp) + xyDiff;


	printf("locking on = %d\n", target_pos_lidar.lock_on);
	printf("d_x == %f \t d_y == %f\n", error_x, error_y);
	printf("lidar_distance == %f\n", local_distance);
	printf("expectAngle to realAngle = %f\n", (angle_real - angle_exp));
	printf("track_error == %f\n", track_error);

	float speed_fix;
#endif

	if (tracking_path_sigma < 0){ //(-pi,pi) 
		tracker_angle = tracking_path_sigma + 2 * Pi;
	}
	else if (tracking_path_sigma > 2 * Pi)
	{
		tracker_angle = 2 * Pi - tracking_path_sigma;
	}
	else
	{
		tracker_angle = tracking_path_sigma;
	}
	tracker_angle = tracker_angle * 180 / Pi;//ת�ɽǶ�

	//�������͵�sumlink

	sumlink_lidardockin_state.f64_track_angle = tracker_angle;
	sumlink_lidardockin_state.f64_track_err = track_error;

	printf("track_error      == %f\n", track_error);
	printf("trackpath_sigma  == %f\n", tracking_path_sigma);

	if (target_dis <= 1.0 && dock_sign.x_in_entrance == 1){ //�ӽ�����1m�Ҳ�ѯ����������������⵽���Ƿ񵽴ﴬ���

		speed_fix = 0.6; 
	}
	else{
		speed_fix = 1;
	}

	if (target_dis < delta_los || ins_msg.speed < docking_speed*0.8){//Ŀ�����С��Los���߳������ٶȺ�С�� 
		dspControlCmd(ins_msg.heading, docking_speed,0); //ֱ�Ӱ���ǰ����ǰ��
		ret = 1;
	}
	else{
		dspControlCmd(tracker_angle, docking_speed * speed_fix, 0); //�����״���ٽ׶� ֱ�Ӹ��ݸ��ٺ���ǰ��
		ret = 0;
	}
	return ret;
}

uint8 gridBackReturnLidar(double target_dis, double track_err)
{
	double ret_err = 0.0;
	double vertical_distance = 0.0; //��λ����Ŀ�괬���X��������
	vertical_distance = sqrt(target_dis*target_dis - track_err*track_err);
	if (vertical_distance <= 20 && vertical_distance > 10){
		ret_err = vertical_distance*0.3;
	}
	else if (vertical_distance <= 10 && vertical_distance >= 5){
		ret_err = vertical_distance*0.25;
	}
	else if (vertical_distance <= 5 && vertical_distance >= 1){
		ret_err = vertical_distance*0.55;
	}
	else if (vertical_distance <= 1 && vertical_distance >= 0){
		ret_err = vertical_distance * 1;
	}
	else{
		return 0;
	}

	if (track_err > ret_err || track_err < -ret_err){
		printf("vertical_distance == %f \t track_err == %f\n", vertical_distance, track_err);
		return 1;
	}
	else
	{
		return 0;
	}
}

void dspControlCmd(float _heading, float _vx, float _wr)
{
	//todo consvaule -1,1;

	if (_vx < 0)
	{
		backMotion(); //����
	}else{
		nCalRudderOpenDeg(_heading, _vx);//ǰ��
	}
// 	constrain_value(_vx, -3, 3); //���ٶȹ�һ�� �����3kn
// 	docking_control_cmd.cmd_state = 1;
// 	docking_control_cmd.vx = _vx / 3.0; //����IHC������������
// 	docking_control_cmd.heading = _heading; //��������
// 	docking_control_cmd.wr = _wr;


}

void dspControlOFF()
{
	docking_control_cmd.cmd_state = 0;
	docking_control_cmd.vx = 0; //����IHC������������
	docking_control_cmd.heading = ins_msg.heading; //��������
	docking_control_cmd.wr = 0;
}

uint8 backMotion()
{
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 255*0.3;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = 255*0.3;


	jet_system.jetL.i16_Cmd_MotorGearDeg = GEAR_DOWN;
	jet_system.jetR.i16_Cmd_MotorGearDeg = GEAR_DOWN;
	return 1;
}

// throttle in range [-1 ,1]
void set_throttle(float throttle)
{
	jet_system.jetL.u8_Cmd_MotorOpenDeg = fabsf(255 * throttle);
	jet_system.jetR.u8_Cmd_MotorOpenDeg = fabsf(255 * throttle);

	jet_system.jetL.i16_Cmd_MotorGearDeg = (throttle > 0 )?GEAR_UP:GEAR_DOWN;
	jet_system.jetR.i16_Cmd_MotorGearDeg = (throttle > 0 )?GEAR_UP:GEAR_DOWN;

}

void set_steering(float steering)
{
	jet_system.jetL.i16_Cmd_MotorRudderDeg = steering * 255;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = steering * 255;
}


uint8 idleMotion()
{
	jet_system.jetL.u8_Cmd_MotorOpenDeg = 0;
	jet_system.jetR.u8_Cmd_MotorOpenDeg = 0;

	jet_system.jetL.i16_Cmd_MotorGearDeg = 0;
	jet_system.jetR.i16_Cmd_MotorGearDeg = 0;

	jet_system.jetL.i16_Cmd_MotorRudderDeg = 0;
	jet_system.jetR.i16_Cmd_MotorRudderDeg = 0;
	return 1;
}
