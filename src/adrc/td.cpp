#include "td.h"

#include <math/common_math.h>

void TD::Init(const TdConf &td_conf)
{
    z1_ = 0;
    z2_ = 0;
    initialized_ = false;
    SetTd(td_conf);
}

void TD::SetTd(const TdConf &td_conf)
{
    r_ = td_conf.r_;
    h_ = td_conf.h_;
    is_angle_ = td_conf.angle_;
}

void TD::Reset()
{
    initialized_ = false;
    z1_ = 0;
    z2_ = 0;
}

void TD::Update(double u,double y,double ts)
{
    if(!initialized_){
        initialized_ = true;
        z1_ = y;
        z2_ = 0;
    }

    double err = z1_ - u;
    if(is_angle_){err = math::wrap_180(err);}

    double fv = math::fhan(err,z2_,r_,h_);
    z1_ += ts * z2_;
    if(is_angle_){
        z1_ = math::wrap_180(z1_);
    }
        z2_ += ts * fv;
}
