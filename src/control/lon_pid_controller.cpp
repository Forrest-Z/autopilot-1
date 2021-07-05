#include <user_time/user_time.h>
#include <iostream>
#include <math/common_math.h>
#include <math/math_utils.h>
#include "lon_pid_controller.h"
#include <stdio.h>


LonPIDController::LonPIDController(){
        name_ = "LON_PID_CONTROLLER";
         std::cout << name_ << std::endl;
}

LonPIDController::~LonPIDController(){}

void LonPIDController::Enter(void)
{
    // Initialize common variables
    bound_command_        = control_conf_->lon_controller_output_limit_;
    bound_command_rate_   = control_conf_->lon_controller_output_rate_limit_;
    bound_error_          = control_conf_->lon_controller_error_limit_;
    radar_hold_thtottle_cmd_ = control_conf_->lon_radar_hold_thtottle_cmd_;

    // Initialize local class variables
    speed_pid_controller_.Init(control_conf_->lon_speed_pid_conf_);     

    control_previous_ = 0.0;    
    std::cout << "LonPIDController:: Initialise completed!"<< std::endl;    
}

void LonPIDController::Run(double target,ControlCommand *cmd,double ts)
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


    // PID controller
    control_unbounded = speed_pid_controller_.Control(desired_speed,speed,ts);

    // Anti saturation control
    double control = 0;
    BoundOutput(control_unbounded, control_previous_, &control,ts);
    control= math::Clamp(control,0.0,bound_command_);
    control_previous_ = control;
    cmd->set_throttle(control);

  // printf("LonPIDController:: desired_speed = %f,speed = %f,control_unbound=%f,control=%f\n",desired_speed,speed,control_unbounded,control);
}


void LonPIDController::Reset()
{
    speed_pid_controller_.Reset();
    control_previous_ = 0;
}

 void LonPIDController::Stop(){
     // Close log file
 }

std::string LonPIDController::Name() const { return name_; }




