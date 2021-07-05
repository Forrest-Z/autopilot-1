#ifndef SIMU_SHIP
#define SIMU_SHIP

#define PI 3.1415926
#define Re  6371393					//地球半径

//运动方程用船体参数
typedef struct{
	double		heading_ship	;	//本船航向
	double		speed_ship		;	//本船航速	单位kn

	double		ship_lat		;	//本船纬度
	double		ship_lng		;	//本船经度
}SHIP_MODULE;

typedef struct{
	double		x_meter	;
	double		y_meter	;
}SHIP_CORDINATE;


typedef struct{
	double		motor_rev		;	//发动机转速
	double		rudder			;	//舵角
	double		motor_rev_exp	;	//发动机转速期望
	double		rudder_exp		;	//舵角期望
}SHIP_CONTROL;


typedef struct{
	double		speed_value_back	;	//本船速度反馈
	double		rev_value_back		;	//发动机转速反馈
	double		rudder_value_back	;	//舵角惯性
	double		heading_value_back	;	//航向惯性
}SHIP_INERTIAL;


typedef struct{
	double		speed_vx_interfering	;	//干扰量在x方向的合作用
	double		speed_vy_interfering	;	//干扰量在y方向的合作用
	double		heading_interfering		;	//干扰量在w方向的合作用
}SHIP_INTERFERING;

typedef struct{
	SHIP_MODULE			ship_module			;
	SHIP_INERTIAL		ship_inertial		;
	SHIP_INTERFERING	ship_interfering	;
	SHIP_CONTROL		ship_control		;
	SHIP_CORDINATE		ship_cordinate		;
}SHIP_INFO;



extern SHIP_INFO ship_info;	//船体运动模型


extern void ship_move_one_step(void);
extern void ship_info_init(void);

#endif