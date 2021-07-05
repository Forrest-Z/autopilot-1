#ifndef SIMU_SHIP
#define SIMU_SHIP

#define PI 3.1415926
#define Re  6371393					//����뾶

//�˶������ô������
typedef struct{
	double		heading_ship	;	//��������
	double		speed_ship		;	//��������	��λkn

	double		ship_lat		;	//����γ��
	double		ship_lng		;	//��������
}SHIP_MODULE;

typedef struct{
	double		x_meter	;
	double		y_meter	;
}SHIP_CORDINATE;


typedef struct{
	double		motor_rev		;	//������ת��
	double		rudder			;	//���
	double		motor_rev_exp	;	//������ת������
	double		rudder_exp		;	//�������
}SHIP_CONTROL;


typedef struct{
	double		speed_value_back	;	//�����ٶȷ���
	double		rev_value_back		;	//������ת�ٷ���
	double		rudder_value_back	;	//��ǹ���
	double		heading_value_back	;	//�������
}SHIP_INERTIAL;


typedef struct{
	double		speed_vx_interfering	;	//��������x����ĺ�����
	double		speed_vy_interfering	;	//��������y����ĺ�����
	double		heading_interfering		;	//��������w����ĺ�����
}SHIP_INTERFERING;

typedef struct{
	SHIP_MODULE			ship_module			;
	SHIP_INERTIAL		ship_inertial		;
	SHIP_INTERFERING	ship_interfering	;
	SHIP_CONTROL		ship_control		;
	SHIP_CORDINATE		ship_cordinate		;
}SHIP_INFO;



extern SHIP_INFO ship_info;	//�����˶�ģ��


extern void ship_move_one_step(void);
extern void ship_info_init(void);

#endif