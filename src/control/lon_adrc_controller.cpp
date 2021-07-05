#include <user_time/user_time.h>
#include <math/common_math.h>
#include <math/math_utils.h>
#include <iostream>
#include <stdio.h>

#include <util/easylogging++.h>
#include "lon_adrc_controller.h"

LonADRCController::LonADRCController(){
        name_ = "LON_ADRC_CONTROLLER";
        LOG(INFO) << "LON_ADRC_CONTROLLER Create!";
        std::cout << name_ << std::endl;
}

LonADRCController::~LonADRCController(){}

void LonADRCController::Enter(void)
{
    // Initialize common variables
    bound_command_        = control_conf_->lon_controller_output_limit_;
    bound_command_rate_   = control_conf_->lon_controller_output_rate_limit_;
    bound_error_          = control_conf_->lon_controller_error_limit_;
    radar_hold_thtottle_cmd_ = control_conf_->lon_radar_hold_thtottle_cmd_;
    speed_acceleration_max_  = control_conf_->lon_controller_input_rate_limit_;

    // Initialize local class variables
    speed_leso_controller_.init(control_conf_->lon_speed_leso_conf_);
    speed_sqrt_controller_.Init(control_conf_->lon_speed_sqrt_conf_);    

    control_previous_ = 0.0;    
    std::cout << "LonADRCController:: Initialise completed!"<< std::endl;    
}

void LonADRCController::Run(double target,ControlCommand *cmd,double ts)
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
    double error = desired_speed_ - speed_leso_controller_.z1();
    double error_limited = math::Clamp(error,-bound_error_,bound_error_);

    double disturb = speed_leso_controller_.z2();
    double b0 = speed_leso_controller_.b0();

    control_unbounded = (speed_sqrt_controller_.Control(error_limited,ts) - disturb)/b0;

    // Anti saturation control
    double control = 0;
    BoundOutput(control_unbounded, control_previous_, &control,ts);
    control= math::Clamp(control,0.0,bound_command_);
    control_previous_ = control;
    cmd->set_throttle(control);

  //  printf("LonADRCController:: desired_speed = %f,speed = %f,control_unbound=%f,control=%f\n",desired_speed,speed,control_unbounded,control);
}


void LonADRCController::Reset()
{
    speed_leso_controller_.reset();
    speed_sqrt_controller_.Reset();
    control_previous_ = 0;
}

 void LonADRCController::Stop(){
     // Close log file
 }

std::string LonADRCController::Name() const { return name_; }




