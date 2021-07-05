#include "controller.h"
#include <iostream>
#include <user_time/user_time.h>

// Include CANBUS message
#include <include/can_deal_IHC.h>


bool Controller::Init(std::shared_ptr<VehicleState> injector,
                              const ControlConf *control_conf)
{
    control_conf_ = control_conf;
    if(control_conf_ == nullptr){
        std::cout << "Controller:: control_conf =nullptr" << std::endl;
        return false;
    }
    injector_ = injector;
    
    Enter();

    return true;
}


bool Controller::ComputeControlCommand(double target,ControlCommand *cmd,double ts)
{
    uint64_t tnow = user_time::get_millis();

    // Check vehicle state info 
    if(tnow - injector_->TimeStamp() >= 500 || 
       injector_->sensor_status() == VehicleState::GPS_ERR_RADAR_ERR ||
       IHC_rev_msg.mid_st.b1_St_MotorOn == 0 ||
       injector_->stop_boat() == true){
        cmd->set_throttle(0);
        cmd->set_steering_target(0);
        Reset();
        std::cout << name_ <<"Controller::set throrttle and rudder to zero,no vehicel state data update!"<<std::endl;
        return false;
    }

    // Check timeout to update
    if(tnow - last_update_time_ >= 500 || last_update_time_==0){
        Reset();
        std::cout<< name_ <<"Controller::called timeout,reset module!"<<std::endl;
    }
    last_update_time_ = tnow;

    // compute control commands
    Run(target,cmd,ts);
    return true;

}


int Controller::BoundOutput(const double output_unbounded, const double previous_output, double *output,double ts)
{
    int status = 0;
    double ts_ = ts;
    if (output_unbounded > bound_command_ ||
        output_unbounded > previous_output + bound_command_rate_ * ts_) {
        *output = (bound_command_ < previous_output + bound_command_rate_ * ts_)
                    ? bound_command_
                    : previous_output + bound_command_rate_ * ts_;
        // if output exceeds the upper bound, then status = 1; while if output
        // changing rate exceeds the upper rate bound, then status = 2
        status =
            (bound_command_ < previous_output + bound_command_rate_ * ts_) ? 1 : 2;
    } else if (output_unbounded < -bound_command_ ||
                output_unbounded < previous_output - bound_command_rate_ * ts_) {
        *output = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                    ? -bound_command_
                    : previous_output - bound_command_rate_ * ts_;
        // if output exceeds the lower bound, then status = -1; while if output
        // changing rate exceeds the lower rate bound, then status = -2
        status = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                    ? -1
                    : -2;
    } else {
        *output = output_unbounded;
        // if output does not violate neither bound nor rate bound, then status = 0
        status = 0;
    }
    return status;
}