#include <user_time/user_time.h>
#include <iostream>
#include <math/common_math.h>
#include <math/math_utils.h>
#include "lat_pid_controller.h"
#include <stdio.h>

LatPIDController::LatPIDController(){
        name_ = "LAT_PID_CONTROLLER";
         std::cout << name_ << std::endl;
}

LatPIDController::~LatPIDController(){}

void LatPIDController::Enter(void)
{
    // Initialize common variables
    bound_command_        = control_conf_->lat_controller_output_limit_;
    bound_command_rate_   = control_conf_->lat_controller_output_rate_limit_;
    bound_error_          = control_conf_->lat_controller_error_limit_;
    sight_track_distance_ = control_conf_->lat_sight_track_distance_;
  

    // Initialize local class variables
    steering_angle_pid_controller_.Init(control_conf_->lat_steering_angle_pid_conf_);
    steering_rate_pid_controller_.Init(control_conf_->lat_steering_rate_pid_conf_);      

    control_previous_ = 0.0;    
    std::cout << "LatPIDController:: Initialise completed!"<< std::endl;    
}

void LatPIDController::Run(double target,ControlCommand *cmd,double ts)
{
    double linear_velocity = injector_->linear_velocity();
    double track_error     = injector_->lateral_error();
    double heading         = injector_->heading_error();
    double heading_rate    = injector_->angular_velocity();

    // get desired yaw angle;
    track_error = math::Clamp(track_error,-sight_track_distance_,+sight_track_distance_);
    double desired_heading = -math::degrees(atan(track_error / sight_track_distance_));

    double control_unbounded = 0;

    // Compute control error
    double error = math::wrap_180(desired_heading - heading);
    double error_limited = math::Clamp(error,-bound_error_,bound_error_);

    // Compute reference yaw rate
    double desired_rate = steering_angle_pid_controller_.Control(error_limited,ts);

    // Compute control output
    double rate_error = desired_rate - heading_rate;
    control_unbounded = steering_rate_pid_controller_.Control(rate_error,ts);
 
    // Anti saturation control
    double control = 0;
    BoundOutput(control_unbounded, control_previous_, &control,ts);
    control_previous_ = control;
    cmd->set_steering_target(control);

  //  printf("LatPIDController: track_error=%f, desired_yaw = %f,yaw = %f,control_unbound=%f,control=%f\n",track_error,desired_heading,heading,control_unbounded,control);
}


void LatPIDController::Reset()
{
    steering_angle_pid_controller_.Reset();
    steering_rate_pid_controller_.Reset();
    control_previous_ = 0;
}

 void LatPIDController::Stop(){
     // Close log file
 }


std::string LatPIDController::Name() const { return name_; }




