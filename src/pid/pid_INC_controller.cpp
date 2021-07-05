#include "pid_INC_controller.h"
#include <math/math_utils.h>

void PIDIncController::Init( const PIDIncConf &conf)
{
     control_previous_ = 0;
     error_=0;
     error_previous_ = 0;
     cmd_previous_ = 0;
     initialized_ = false;
     SetPID(conf);
}

void PIDIncController::SetPID(const PIDIncConf &conf)
{
    kp_ = conf.kp_;
    ki_ = conf.ki_;
    max_acceleration_ = conf.max_acceleration_;
    output_saturation_level_ = conf.output_saturation_level_;
}

void PIDIncController::Reset()
{
     control_previous_ = 0;
     error_=0;
     error_previous_ = 0;
     cmd_previous_ = 0;
     initialized_ = false;
}

double PIDIncController::Control(const double u, const double y,const double dt)
{

    double cmd = u;
    double fdb = y;
    const double change = max_acceleration_*dt;

    if(cmd >(change + cmd_previous_)){
        cmd = change + cmd_previous_;
    }
    if(cmd < (cmd_previous_ - change)){
        cmd = cmd_previous_ - change;
    }

    error_ = cmd - fdb;

    double control = control_previous_;
    control += kp_*(error_ - error_previous_) + ki_*error_;

    control = math::Clamp(control,0.0,output_saturation_level_);

    control_previous_ = control;
    error_previous_ = error_;
    cmd_previous_ = cmd; 
    return control;
}