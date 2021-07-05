#include "eso.h"
#include <math/common_math.h>   
#include <math/math_utils.h>
#include <util/easylogging++.h>
   
ESO::ESO()
{
}
ESO::~ESO()
{
}

void ESO::init(const ESOConf &eso_conf)
{
    initialized_ = false;
    z1_=0;z2_=0;z3_=0;
    set_eso(eso_conf);
}

void ESO::set_eso(const ESOConf &eso_conf)
{
    angle_ = eso_conf.angle_;
    wo_ = eso_conf.wo_;
    b0_ = eso_conf.b0_;
    delta_ = eso_conf.delta_;
    model_order_ = eso_conf.model_order_;
}

void ESO::reset()
{
    initialized_ = false;
    z1_=0;z2_=0;z3_=0;
}

void ESO::update(double u,double y,double dy,double ts)
{
    if(!initialized_){
        initialized_ = true;
        z1_ = y;
        z2_ = dy;
        z3_ = 0;
    }

    double e = z1_ - y;

    if(angle_){e = math::wrap_180(e);}

    switch (model_order_)
    {
        case 1:
        {
            #if LINEAR_ESO_ENABLE == 1
                z1_  = z1_ + ts * (z2_ - 2*wo_ * e + b0_ * u);
                if(angle_){z1_ = math::wrap_180(z1_);}
                z2_  = z2_ + ts * (- wo_ * wo_ * e);
            #else
                double fe = math::fal(e,0.5,delta_);
                z1_ = z1_ + ts * (z2_ - 2*wo_*e + b0_ *u);
                if(angle_){z1_ = math::wrap_180(z1_);}
                z2_ = z2_ + ts * (-wo_*wo_*fe);
            #endif
        }
        break;
        case 2:
        {
           #if LINEAR_ESO_ENABLE == 1
                z1_  = z1_ + ts * (z2_ - 3*wo_ * e);
                if(angle_){z1_ = math::wrap_180(z1_);}
                z2_  = z2_ + ts * (z3_ - 3*wo_*wo_*e + b0_ *u);
                z3_  = z3_ + ts * (- wo_ * wo_ * wo_ * e);
            #else
                double fe = math::fal(e,0.5,delta_);
                double fe1 = math::fal(e,0.25,delta_);
                z1_  = z1_ + ts * (z2_ - 3*wo_ * e);
                if(angle_){z1_ = math::wrap_180(z1_);}
                z2_  = z2_ + ts * (z3_ - 3*wo_*wo_*fe + b0_ *u);
                z3_  = z3_ + ts * (- wo_ * wo_ * wo_ * fe1);
            #endif
        }
        break;
        default:
            LOG(ERROR)<<"LESO: unkonw model order";
        break;
    }

}