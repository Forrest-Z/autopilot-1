#include "sim_boat.h"

#include <user_time/user_time.h>
#include <math/common_math.h>
#include <math/math_utils.h>
#include <math/geo.h>
#include <include/uart_deal_ins.h>
#include <stdio.h>

#define SIM_BOAT_INIT_LAT   (int32_t)(304459725)
#define SIM_BOAT_INIT_LNG   (int32_t)(1143975965)

SimBoat *SimBoat::singleton_;

SimBoat *SimBoat::get_singleton()
{
    return singleton_;
}

namespace AP{
    SimBoat *sim_boat(){
        return SimBoat::get_singleton();
    }
};


void SimBoat::init(void)
{
    param_.u_ka = 1/1.9438f;
    param_.u_kv = 3/1.9438f;
    param_.u_kc = 0;

    param_.K = 7.92;
    param_.alpha= 30;
    param_.T=13.88;
    reset();
}

void SimBoat::reset(void)
{
    state_.fai = 0;
	state_.gama = 0;
	state_.latitude  = 1e-7 *SIM_BOAT_INIT_LAT;
	state_.longitude = 1e-7 *SIM_BOAT_INIT_LNG;
	state_.r = 0;
	state_.u = 0;
	state_.v = 0;
	state_.groundspeed = 0;
    ds = 0;
}

void SimBoat::update(double throttle,double steering,double ts)
{
    uint64_t tnow = user_time::get_millis();
    if(last_update_time_ == 0 || tnow - last_update_time_ >= 500){
        reset();
    }

	last_update_time_ = tnow;

    double du,dv,dr;

    du = ts *(param_.u_ka * throttle -param_.u_ka / param_.u_kv * state_.u + param_.u_kc);
	dv = 0;
	state_.u += du;
	state_.v += dv;

    dr = ts *(param_.K*steering - state_.r - param_.alpha*state_.r*state_.r*state_.r)/param_.T;
	state_.r += dr;
	
	state_.fai += state_.r * ts;
	state_.fai = math::wrap_2PI(state_.fai);
	state_.gama = math::wrap_2PI(state_.fai + atan2(state_.v,state_.u));
	state_.groundspeed = math::safe_sqrt(math::Sqr(state_.u) + math::Sqr(state_.v));
	ds = state_.groundspeed * ts;

    double lat_next = 0,lon_next =0;

	Get_lat_lng(state_.latitude,state_.longitude,ds,math::degrees(state_.fai),&lat_next,&lon_next);

	state_.latitude = lat_next;
	state_.longitude = lon_next;
	

    // update ins_msg
    ins_msg.longitude = state_.longitude;
    ins_msg.latitude  = state_.latitude;
    ins_msg.motionDirection = math::degrees(state_.fai);
    ins_msg.rotRate = math::degrees(state_.r);
    ins_msg.speed = ms2kn(state_.groundspeed);
    ins_msg.heading = math::degrees(state_.fai);

    ins_msg.pseudorRangeError = 0;		
	ins_msg.locationState = 4;	

	ins_msg.u8_year		=21;	
	ins_msg.u8_month	=3;	
	ins_msg.u8_date		=10;	
	ins_msg.u8_hour		=9;	
	ins_msg.u8_minute	=0;	
	ins_msg.u8_second	=0;	
	ins_msg.u8_second_2	=0;	

	ins_msg.u16_speed	= ins_msg.speed*10;	
	ins_msg.u16_heading	= ins_msg.heading*10 ;	
	ins_msg.u16_volecityDir = ins_msg.motionDirection*10;
	ins_msg.i16_heaving = 5;
	
	ins_msg.i16_rot = ins_msg.rotRate*60;	
	ins_msg.i16_pitch = 0;	
	ins_msg.i16_roll = 0;

    uint8_t deg,min,sec,msec;
    intchangle_double_to_string(state_.longitude,deg,min,sec,msec);
	ins_msg.u8_longiSt = 0;
	ins_msg.u8_longiDeg = deg;
	ins_msg.u8_longiMin = min;
	ins_msg.u8_longiSec = sec;
	ins_msg.u8_longiSecDec = msec;
	ins_msg.u8_longiSecDec2 =0;

    intchangle_double_to_string(state_.latitude,deg,min,sec,msec);
	ins_msg.u8_latiSt = 0;
	ins_msg.u8_latiDeg = deg;
	ins_msg.u8_latiMin = min;
	ins_msg.u8_latiSec = sec;
	ins_msg.u8_latiSecDec= msec;	
	ins_msg.u8_latiSecDec2 =0;
	ins_msg.insState.c_rmcValid = 'A';
	ins_msg.insState.b1_dateValid = true;
	ins_msg.insState.b1_diffPostionValid =true;
	ins_msg.insState.b1_diffSignalValid = true;
	ins_msg.insState.b1_diffSignalValid_old = true;
	ins_msg.insState.b1_timeValid = 1;
	ins_msg.insState.u8_sateliteNum1=38;
	ins_msg.insState.u8_sateliteNum2 =38;
}
