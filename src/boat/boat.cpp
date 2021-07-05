
#include "boat.h"
#include <include/usv_include.h>
#include <math/geo.h>
#include <user_time/user_time.h>
#include <util/easylogging++.h>
#include <math/math_utils.h>
#include <math/common_math.h>
Boat *Boat::singleton_;

Boat::Boat()
{
    singleton_ = this;
    injector_  = std::make_shared<VehicleState>();
}

INITIALIZE_EASYLOGGINGPP
void Boat::setup(void)
{
    el::Configurations conf("./log.conf");
    el::Loggers::reconfigureAllLoggers(conf);
    LOG(INFO) << "****** Boat Setup *****";

    // Load all parameters
    std::cout <<endl;
    std::cout <<"read configuration information from usv_cfg.xml" << std::endl;
    std::string error = "";
	
	read_config_xml_.open_file(CFG_FILE_PATH);
    if(!read_config_xml_.read_config_value(conf_,error))
    {
        std::cout << "config file read failed and breakout!!!\n";
        return;
    }

    std::cout << "config file read in and startup!!!\n";
    std::cout <<endl;


    // Initialize Console
    console_.Init(conf_.boat_conf_.console_ip_,conf_.boat_conf_.local_ip_,
    conf_.boat_conf_.console_port_,conf_.boat_conf_.local_port_);

    
    // load boat parameters
    route_switch_steering_cmd = conf_.boat_conf_.route_switch_steering_cmd_;
    throttle_limit            = conf_.control_conf_.lon_controller_output_limit_;
    steering_limit            = conf_.control_conf_.lat_controller_output_limit_;
    position_second_limit     = conf_.boat_conf_.position_second_limit_;
    position_kp               = conf_.boat_conf_.position_kp_;


    // initialize controller
    lat_controller_.Init(injector_,&conf_.control_conf_);
    lon_controller_.Init(injector_,&conf_.control_conf_);

    // initialize communication
    radar_message.init(conf_.radar_conf_);

    // vehicle state initialize
    injector_->Init(conf_.vehicle_state_conf_);

    // Simulation for debug or test
    sim_boat_.init();

    // initialise object avoidance
    oa.init();

    LOG(INFO) << "Boat setup finished!";
}

 bool Boat::finished_sail_task()
 {
    int8_t ret = 0;
    if(sailTask.u8_PointNum >= sailTask.sailMsg.u8_pointSum){
        #if 1
        ret = arriveStandby2(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_latitude,
                            sailTask.sailMsg.wayPoint[sailTask.u8_PointNum-1].f64_longitude);
        #else
         cmd.set_steering_target(0);
         cmd.set_throttle(0.0);
         set_steering(cmd.get_steering_target()/steering_limit);
         set_throttle(cmd.get_throttle()/throttle_limit);
         #endif

        if(ret == -1){
            sailTask.u8_St_sailMsgRev = 0;
            sailTask.sailMsg.u8_sailNum = 0;
        }else{
            sailTask.u8_St_sailMsgRev = 2;
        }
        command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_NONE;
        return true;
    }else{
        command_signal.sail_feedBack.b2_sailTask = SAIL_TASK_ON;
		return false;
    }
 }

 bool Boat::reached_destination()
 {
     double lat_now   = injector_->lat_now(); 
     double lon_now   = injector_->lon_now(); 
	 double lat_start = injector_->lat_start();  
     double lon_start = injector_->lon_start();   
     double lat_end   = injector_->lat_end();  
     double lon_end   = injector_->lon_end();  
    
     struct crosstrack_error_s crosstrack_error;
   // if(ins_msg.insState.c_rmcValid == 'A'){
        autoNaviSt.double_dst = Get_distance(lat_now,lon_now,lat_end,lon_end);
        get_distance_to_line(&crosstrack_error,lat_now,lon_now,lat_start,  lon_start,  lat_end,  lon_end);
   // }

     if((autoNaviSt.double_dst < autoNaviCfg.u16_arrival_distance1) ||
       (sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival == 1)||
       (crosstrack_error.past_end == true) || 
       (injector_->waypoint_unreachable() == true))
	{
		sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_sailArrival = 1;
		return true;
	}
    return false;
 }


 void Boat::update_sampling_task()
 {
     if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type != 0)
	{
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u16_sampleVolume > 0){
        #if 1
			sampleStandby();
        #else
           cmd.set_steering_target(0);
           cmd.set_throttle(0.0);
           set_steering(cmd.get_steering_target()/steering_limit);
           set_throttle(cmd.get_throttle()/throttle_limit);
         #endif
		}
		
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 0){
			sampleStart2(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_type,sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u16_sampleVolume);
			sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand = 2;
		}
		
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 2){
			if(isSampleFinished(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].u64_sailPointID)){
				sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete = 1;
			}
		}
		
		if(sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingCommand == 2
           &&sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].b1_samplingComplete == 1){
			printf("Arrival of waypoint[%d],waypoint with sampling, follow-up task",sailTask.u8_PointNum+1);
			sailTask.u8_PointNum++;
            route_switch_ = true;
			printf("waypoint with sampling ++\n");
		}

	}else{
		printf("Arrival of waypoint[%d],waypoint without sampling, follow-up task",sailTask.u8_PointNum+1);
		sailTask.u8_PointNum++;
        route_switch_ = true;
		printf("waypoint without sampling++\n");
	}
 }

void Boat::update_navigation_task()
 {
    double relative_angle = injector_->heading_error();
    double relative_angular_velocity = injector_->angular_velocity();
    double lateral_error = injector_->lateral_error();

    double desired_speed = sailTask.sailMsg.wayPoint[sailTask.u8_PointNum].f64_expSpeed;

    if(route_switch_ == true){
        if(fabs(relative_angle) <= 20.0 /*&& fabs(relative_angular_velocity) <=2.0*/ ){
            route_switch_ = false;
        }
        double steer_cmd = steering_limit*route_switch_steering_cmd*(relative_angle >0 ? -1:+1);
        cmd.set_steering_target(steer_cmd);
        cmd.set_throttle(0.0);
    }else{
        double speed_cd = sqrt_controller(autoNaviSt.double_dst,position_kp,position_second_limit,loop_ts_);
        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%,90deg of error reduces speed to 50%
        double yaw_error_ratio = 1.0 - math::Clamp(abs(relative_angle / 90.0),0.0,1.0);
        speed_cd *= yaw_error_ratio;

        // reduce desired speed if lateral error is large
        double track_error_ratio = 1.0 -math::Clamp(abs(lateral_error/5.0),0.0,1.0) * 0.5;
        speed_cd *= track_error_ratio;

        speed_cd = math::Clamp(speed_cd,0.0,desired_speed);

        lat_controller_.ComputeControlCommand(0.0,&cmd,loop_ts_);
        lon_controller_.ComputeControlCommand(speed_cd,&cmd,loop_ts_);
    }
      set_steering(cmd.get_steering_target()/steering_limit);
      set_throttle(cmd.get_throttle()/throttle_limit);
}


void Boat::loop(void)
{
   uint64_t tnow = user_time::get_millis();  
   if(tnow - last_update_time >= 500 || last_update_time == 0){
       route_switch_ = true;
   }
    last_update_time = tnow;

    if(finished_sail_task()){
        return;
    }

    injector_->update(loop_ts_);

    if(!reached_destination()){
        update_navigation_task();
    }else{
        update_sampling_task();
    }
}

Boat *Boat::get_singleton()
{
    return singleton_;
}

namespace AP{
    Boat *boat(){
        return Boat::get_singleton();
    }
};

Boat boat;