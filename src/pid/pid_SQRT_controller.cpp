#include "pid_SQRT_controller.h"
#include <math/math_utils.h>

void PIDSqrtController::Init( const PIDSqrtConf &conf)
{
     initialized_ = false;
     integral_ = 0.0;
     SetPID(conf);
}

void PIDSqrtController::SetPID(const PIDSqrtConf &conf)
{
    kp_ = conf.kp_;
    ki_ = conf.ki_;
    second_limit_ = conf.second_limit_;
    integrator_saturation_level_ = conf.integrator_saturation_level_;
    output_saturation_level_ = conf.output_saturation_level_;
}

void PIDSqrtController::Reset()
{
     initialized_ = false;
     integral_ = 0.0;
}

double PIDSqrtController::Control(const double error,const double dt)
{
    integral_+= error;
    integral_ = math::Clamp(integral_,-integrator_saturation_level_,integrator_saturation_level_);

    double control = math::SqrtController(error,kp_,second_limit_,dt) + ki_*integral_;
    control = math::Clamp(control,-output_saturation_level_,output_saturation_level_);
    return control;
}
