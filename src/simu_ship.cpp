#include "stdafx.h"
#include "../include/simu_ship.h"
#include <math.h>

#define	REV_T 80
#define SPEED_T 250
#define RUDDER_T 50 
#define HEADING_T 50
#define SAMPLE_TIME 20

SHIP_INFO ship_info;	//�����˶�ģ��



void ship_info_init(void)
{
	ship_info.ship_module.heading_ship = 0.0			;
	ship_info.ship_module.speed_ship   = 0.0			;
	ship_info.ship_module.ship_lat	   = 30.4830276968	;
	ship_info.ship_module.ship_lng	   = 114.3487400564	;

	ship_info.ship_inertial.heading_value_back	= 0;
	ship_info.ship_inertial.rev_value_back		= 0;
	ship_info.ship_inertial.rudder_value_back	= 0;
	ship_info.ship_inertial.speed_value_back	= 0;

	ship_info.ship_cordinate.x_meter = 0;
	ship_info.ship_cordinate.y_meter = 0;

	ship_info.ship_control.motor_rev		= 0;
	ship_info.ship_control.motor_rev_exp	= 0;
	ship_info.ship_control.rudder			= 0;
	ship_info.ship_control.rudder_exp		= 0;
}



double inertial_element_rev(double T,	double GiveValue)	//������ת�ٹ��Ի���
{
	double result;
	result = (T*ship_info.ship_inertial.rev_value_back+GiveValue)/(1+T);
	ship_info.ship_inertial.rev_value_back = result;
	return result;
}

double inertial_element_speed(double T,double GiveValue)	//���ٹ��Ի��ڣ���ʱ��δ�����ö���ʽģ�⣩
{
	double result;
	result = (T*ship_info.ship_inertial.speed_value_back+GiveValue)/(1+T);
	ship_info.ship_inertial.speed_value_back = result;
	return result;
}

double inertial_element_rudder(double T,double GiveValue)
{
	double result;
	result = (T*ship_info.ship_inertial.rudder_value_back+GiveValue)/(1+T);
	ship_info.ship_inertial.rudder_value_back = result;
	return result;
}



double inertial_element_heading(double T,double GiveValue)
{
	double result;
	double dif_value;
	dif_value = (GiveValue - ship_info.ship_inertial.heading_value_back);
	if(dif_value > -180 && dif_value <= 180)
		result = (T*ship_info.ship_inertial.heading_value_back + GiveValue)/(1+T);
	else if(dif_value>180)
		result = (T*ship_info.ship_inertial.heading_value_back + GiveValue-360)/(1+T);
	else
		result = (T*ship_info.ship_inertial.heading_value_back + GiveValue+360)/(1+T);

	if(result > 360)
		ship_info.ship_inertial.heading_value_back = result - 360	;
	else if(result <0)
		ship_info.ship_inertial.heading_value_back = result + 360	;
	else
		ship_info.ship_inertial.heading_value_back = result;

	return result;
}




void ship_move_one_step(void)
{	
	double sail_speed_motor_rev;
	double vx,vy;
	double dx,dy;
	double speed_v_meter;

	//���㺽�� Ŀ��ת��--> ʵ��ת��--> ʵ�ʺ���
	ship_info.ship_control.motor_rev = inertial_element_rev(REV_T,ship_info.ship_control.motor_rev_exp);
	sail_speed_motor_rev = ship_info.ship_control.motor_rev *40/3000;
	ship_info.ship_module.speed_ship = inertial_element_speed(SPEED_T,sail_speed_motor_rev);

	//���ٵ�λת�� m/s
	speed_v_meter = 1852.0/3600.0*ship_info.ship_module.speed_ship;

	//���㺽��
	ship_info.ship_control.rudder	   = inertial_element_rudder(RUDDER_T,ship_info.ship_control.rudder_exp);
	ship_info.ship_module.heading_ship = inertial_element_heading(HEADING_T,ship_info.ship_control.rudder*0.2+ship_info.ship_module.heading_ship); 

	//�˶�����
	vx  = sin(ship_info.ship_module.heading_ship/180.0*PI)*speed_v_meter + ship_info.ship_interfering.speed_vx_interfering;
	vy  = cos(ship_info.ship_module.heading_ship/180.0*PI)*speed_v_meter + ship_info.ship_interfering.speed_vy_interfering;
	
	dx  = vx*SAMPLE_TIME/1000.0;
	dy	= vy*SAMPLE_TIME/1000.0;

	//��γ�ȱ仯������ͶӰ
	ship_info.ship_module.ship_lng += dx/Re * 180/PI/cos(ship_info.ship_module.ship_lat/180.0*PI);
	ship_info.ship_module.ship_lat += dy/Re * 180/PI;

	//xyƽ��ͶӰ
	ship_info.ship_cordinate.x_meter += dx;
	ship_info.ship_cordinate.y_meter += dy;

}