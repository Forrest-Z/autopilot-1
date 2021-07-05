#include "lon_mrac_controller.h"

#include <iostream>
#include <stdio.h>
#include <util/easylogging++.h>

#include <user_time/user_time.h>
#include <math/common_math.h>
#include <math/math_utils.h>


LonMracController::LonMracController(){
        name_ = "LON_MRAC_CONTROLLER";
        LOG(INFO) << "LON_MRAC_CONTROLLER Create!";
        std::cout << name_ << std::endl;
}

LonMracController::~LonMracController(){}

void LonMracController::Enter(void)
{
    // Initialize common variables
    bound_command_           = control_conf_->lon_controller_output_limit_;
    bound_command_rate_      = control_conf_->lon_controller_output_rate_limit_;
    bound_error_             = control_conf_->lon_controller_error_limit_;
    radar_hold_thtottle_cmd_ = control_conf_->lon_radar_hold_thtottle_cmd_;
    speed_acceleration_max_  = control_conf_->lon_controller_input_rate_limit_;

    // Initialize local class variables
    speed_leso_controller_.init(control_conf_->lon_speed_leso_conf_);
    speed_mrac_controller_.Init(control_conf_->lon_speed_mrac_conf_);    

    control_previous_ = 0.0;    
    std::cout << "LonMRACController:: Initialise completed!"<< std::endl;    
}

void LonMracController::Run(double target,ControlCommand *cmd,double ts)
{
    // Keep const throttle for radar navigation
    if(injector_->sensor_status() == VehicleState::GPS_ERR_RADAR_OK){
        // consider the boat start task in the bridge!
        /*  Reset();  */
        if(control_previous_ == 0.0){
            cmd->set_throttle(radar_hold_thtottle_cmd_ * bound_command_);
            control_previous_ = radar_hold_thtottle_cmd_*bound_command_;
        }
        
        std::cout << "LonPIDController:: hold constance throttle for radar navigation!" << std::endl;
        return;
    }
    
    // get states
    double linear_velocity = injector_->linear_velocity();
    double heading = injector_->heading_error();

    double control_unbounded = 0;
    double desired_speed = target;
    double speed = linear_velocity;

   // smooth desired speed
     const uint64_t now = user_time::get_millis();
    if ((lon_control_last_ms_ == 0) || ((now- lon_control_last_ms_) > 200))
     {
         desired_speed_ = speed;
     }
     lon_control_last_ms_ = now;

     if(speed_acceleration_max_ > 0){
          const double change_max = speed_acceleration_max_ *ts;
          desired_speed = math::Clamp(desired_speed,desired_speed_-change_max,desired_speed_ + change_max);
     }
    desired_speed_ = desired_speed;

    speed_leso_controller_.update(control_previous_,speed,0.0,ts);
    
    Matrix state = Matrix::Zero(2,1);
    state(0,0) = speed_leso_controller_.z1();
    state(1,0) = speed_leso_controller_.z2();

    double b0  = speed_leso_controller_.b0();
    control_unbounded = speed_mrac_controller_.Control(desired_speed_,state,b0,ts);

    // Anti saturation control
    double control = 0;
    BoundOutput(control_unbounded, control_previous_, &control,ts);
    control= math::Clamp(control,0.0,bound_command_);
    control_previous_ = control;
    cmd->set_throttle(control);

    printf("LonMRACController:: desired_speed = %f,speed = %f,control_unbound=%f,control=%f\n",desired_speed,speed,control_unbounded,control);
}


void LonMracController::Reset()
{
    speed_leso_controller_.reset();
    speed_mrac_controller_.Reset();
    control_previous_ = 0;
}

 void LonMracController::Stop(){
     // Close log file
 }

std::string LonMracController::Name() const { return name_; }




