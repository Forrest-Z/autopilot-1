#include <user_time/user_time.h>

#include <math/common_math.h>
#include <math/math_utils.h>

#include "lat_adrc_controller.h"

#include <stdio.h>
#include <iostream>


LatADRCController::LatADRCController(){
        name_ = "LAT_ADRC_CONTROLLER";
         std::cout << name_ << std::endl;
}

LatADRCController::~LatADRCController(){}

void LatADRCController::Enter(void)
{
    // Initialize common variables
    bound_command_        = control_conf_->lat_controller_output_limit_;
    bound_command_rate_   = control_conf_->lat_controller_output_rate_limit_;
    bound_error_          = control_conf_->lat_controller_error_limit_;
    sight_track_distance_ = control_conf_->lat_sight_track_distance_;
    heading_rate_max_     = control_conf_->lat_controller_input_rate_limit_;

    // Initialize local class variables
    steering_angle_leso_controller_.init(control_conf_->lat_steering_angle_leso_conf_);
    steering_angle_npd_controller_.Init(control_conf_->lat_steering_angle_npd_conf_);
   
    control_previous_ = 0.0;    
    std::cout << "LatADRCController:: Initialise completed!"<< std::endl;    
}

void LatADRCController::Run(double target,ControlCommand *cmd,double ts)
{
    double linear_velocity = injector_->linear_velocity();
    double track_error     = injector_->lateral_error();
    double heading         = injector_->heading_error();
    double heading_rate    = injector_->angular_velocity();

    // get desired yaw angle;
    track_error = math::Clamp(track_error,-sight_track_distance_,+sight_track_distance_);
    double desired_heading = -math::degrees(atan(track_error / sight_track_distance_));


    // smooth desired command
    const uint64_t now = user_time::get_millis();
     if ((lat_control_last_ms_ == 0) || ((now- lat_control_last_ms_) > 200))
     {
         desired_heading_ = heading;
     }
     lat_control_last_ms_ = now;
    
    if(heading_rate_max_ > 0){
        const double change_max = heading_rate_max_ *ts;
        desired_heading = math::Clamp(desired_heading,desired_heading_ - change_max,desired_heading_+change_max);
        desired_heading = math::wrap_180(desired_heading);
    }
    desired_heading_ = desired_heading;


    // ESO 
    steering_angle_leso_controller_.update(control_previous_,heading,heading_rate,ts);
    double error = math::wrap_180(desired_heading_ - steering_angle_leso_controller_.z1());
    double error_limited = math::Clamp(error,-bound_error_,bound_error_);

    double de       = steering_angle_leso_controller_.z2();
    double disturb  = steering_angle_leso_controller_.z3();
    double b0       = steering_angle_leso_controller_.b0();

    // NLSEF
    double control_unbounded = 0;
    control_unbounded = (steering_angle_npd_controller_.Update(error_limited,de) - disturb)/b0;
    
    // Anti saturation control
    double control = 0;
    BoundOutput(control_unbounded, control_previous_, &control,ts);
    control_previous_ = control;

    cmd->set_steering_target(control);

   // printf("LatADRCController: track_error=%f, desired_yaw = %f,yaw = %f,control_unbound=%f,control=%f\n",track_error,desired_heading,heading,control_unbounded,control);
}


void LatADRCController::Reset()
{
    steering_angle_leso_controller_.reset();
    steering_angle_npd_controller_.Reset();
    control_previous_ = 0;
}

 void LatADRCController::Stop(){
     // Close log file
 }

std::string LatADRCController::Name() const { return name_; }




