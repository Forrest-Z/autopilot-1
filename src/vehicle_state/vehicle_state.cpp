#include <cmath>
#include <string>

#include <user_time/user_time.h>
#include <math/math_utils.h>
#include <math/common_math.h>
#include <radar/radar_message.h>
#include <include/usv_include.h>

#include <conf/conf.h>
#include <boat/boat.h>

#include <util/easylogging++.h>

#include "vehicle_state.h"


void VehicleState::Init(VehicleStateConf &conf)
{
    use_ground_speed_angle_ = conf.use_ground_speed_angle_;
    use_ground_speed_ = conf.use_ground_speed_;
    use_external_angular_velocity_ = conf.use_external_angular_velocity_;
    heading_error_td_filter.Init(conf.heading_error_td_filter_conf_);
    initialized_ = false;
}


void VehicleState::Reset(void)
{
    heading_error_td_filter.Reset();
    initialized_ = false;
}

bool VehicleState::get_ekf_origin(Location &origin) const
{
    if(ekf_origin_is_set){
        origin = ekf_origin_;
    }
    return ekf_origin_is_set;
}

void VehicleState::update(double ts)
{
    // if called timeout reset current state
    uint64_t cur_time = user_time::get_millis();
    if(cur_time - last_update_time_ >= 500 || last_update_time_ == 0){
        LOG(INFO) << "VehicleState:: called timeout,reset module";
        Reset();
    }
    last_update_time_ = cur_time;

    // check GNSS and lidar state
    bool radar_msg_ok = true;
    bool gps_msg_ok   = true;
    if((ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A')||
       ins_msg.insState.u8_sysState1==3 ){
        gps_msg_ok = false;
    }
    if(cur_time - AP::radar_message()->radar_message_time_stamp() >= 500 || 
        AP::radar_message()->ipc2arm_msg().flag == 0){
        radar_msg_ok = false;
    }

    // get current route with start point,goal point
    uint16_t id     = sailTask.u8_PointNum;
    uint16_t max_id = sailTask.sailMsg.u8_pointSum;
    lat_now_ = ins_msg.latitude;
    lon_now_ = ins_msg.longitude;

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();

    if(!initialized_ && id == 0 && gps_msg_ok && max_id >= 1){
          LOG(INFO)<< "VehicleState:: Set initial sail task point!";
          lat_start_ = lat_now_;
          lon_start_ = lon_now_;

          // get ekf_origin
          ekf_origin_.lat = lat_start_*1e7;
          ekf_origin_.lng = lon_start_*1e7;
          ekf_origin_.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
        
          ekf_origin_is_set = true;
          initialized_ = true;
          if(oa != nullptr){
              oa->set_ekf_origin(ekf_origin_);
          }
    }
    if(id >= 1){
        lat_start_     = sailTask.sailMsg.wayPoint[id-1].f64_latitude;
        lon_start_     = sailTask.sailMsg.wayPoint[id-1].f64_longitude;
    }
    lat_end_            = sailTask.sailMsg.wayPoint[id].f64_latitude;
    lon_end_            = sailTask.sailMsg.wayPoint[id].f64_longitude;

    // translate current lat-lng into xy  
    _origin.lat = lat_start_*1e7;
    _origin.lng = lon_start_*1e7;
    _origin.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
    _destination.lat = lat_end_*1e7;
    _destination.lng = lon_end_*1e7;
    _destination.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    Location current_loc;
    current_loc.lat = lat_now_*1e7;
    current_loc.lng = lon_now_*1e7;
    current_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    // run path planning around obstacles
     stop_vehicle = false;
    // true if OA has been recently active;
    bool _oa_was_active = _oa_active;
    _oa_dest_unreachable = false;

    if (oa != nullptr) {
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc, _origin, _destination,ins_msg.heading, _oa_origin, _oa_destination);
        switch (oa_retstate) {
        case AP_OAPathPlanner::OA_RetState::OA_NOT_REQUIRED:
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_PROCESSING:
        case AP_OAPathPlanner::OA_RetState::OA_ERROR:
            // during processing or in case of error, slow vehicle to a stop
            stop_vehicle = true;
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_SUCCESS:
            _oa_active = true;
            break;
        case AP_OAPathPlanner::OA_RetState::OA_CAN_NOT_ARRIVAL:
            _oa_dest_unreachable = true;
             stop_vehicle = true;
            _oa_active = false;
            break;
        }
    }
    if (!_oa_active) {
        _oa_origin = _origin;
        _oa_destination = _destination;
    }

    // update state relative new route
    // check if vehicle is in recovering state after switching off OA
    if (!_oa_active && _oa_was_active) {
        if (oa->get_options() & AP_OAPathPlanner::OARecoveryOptions::OA_OPTION_WP_RESET) {
            //reset wp origin to vehicles current location
            _origin = current_loc;
        }
    }

    // update current state 
    double gps_heading = 0.0,gps_track_error = 0.0,gps_course = 0.0;
    // compute vector of route
    double OA_distance = _oa_origin.get_distance(_oa_destination);
    double OA_bearing  = _oa_origin.get_bearing_to(_oa_destination) *0.01;
    
    math::Vec2d OA;
    OA.set_x(OA_distance * cos(math::radians(OA_bearing)));
    OA.set_y(OA_distance * sin(math::radians(OA_bearing)));

    // compute PA
    double PA_distance =  current_loc.get_distance(_oa_destination);
    double PA_bearing  =  current_loc.get_bearing_to(_oa_destination)*0.01;
    math::Vec2d PA;
    PA.set_x(PA_distance * cos(math::radians(PA_bearing)));
    PA.set_y(PA_distance * sin(math::radians(PA_bearing)));

    lateral_error_ = (OA.Length() >= 1e-6)?PA.CrossProd(OA)/OA.Length():0.0;
    heading_error_ = math::wrap_180(ins_msg.heading - OA_bearing);
    linear_velocity_ = (gps_msg_ok == true)?(ins_msg.speed):(linear_velocity_);
    angular_velocity_ = (use_external_angular_velocity_ == true)?(ins_msg.rotRate):heading_error_td_filter.z2();
    time_stamp_ = cur_time;


    // update current localization state
    if(radar_msg_ok == true && gps_msg_ok == true){
        status_ = GPS_OK_RADAR_OK;
    }else if(radar_msg_ok == true && gps_msg_ok == false){
        status_ = GPS_ERR_RADAR_OK;
    }else if(radar_msg_ok == false && gps_msg_ok == true){
        status_ = GPS_OK_RADAR_ERR;
    }else{
        status_ = GPS_ERR_RADAR_ERR;
    }

    // initialization not completed,navigation not allowed
    if(!initialized_ && id == 0){
       LOG(INFO)<< "VehicleState:: Initialization is not completed,navigation not allowed!";
        status_ = GPS_ERR_RADAR_ERR;
    }

    // compute angular velocity
    if(status_ == GPS_ERR_RADAR_ERR){
        heading_error_td_filter.Reset();
    }else{
        heading_error_td_filter.Update(heading_error_,heading_error_,ts);
    }

}



void VehicleState::Update(double ts)
{
    uint64_t cur_time = user_time::get_millis();
    if(cur_time - last_update_time_ >= 500 || last_update_time_ == 0){
        LOG(INFO) << "VehicleState:: called timeout,reset module";
        Reset();
    }
    last_update_time_ = cur_time;

    // states of radar and gps
    bool radar_msg_ok = true;
    bool gps_msg_ok   = true;
    if((ins_msg.insState.c_rmcValid != 'A' && irtk_msg.rtk_state.c_rmcValid != 'A')||ins_msg.insState.u8_sysState1==3 ){
    //    std::cout<<"VehicleState::gps data lost"<<std::endl;
        gps_msg_ok = false;
    }

    if(cur_time - AP::radar_message()->radar_message_time_stamp() >= 500 || 
        AP::radar_message()->ipc2arm_msg().flag == 0){
        radar_msg_ok = false;
     // std::cout<<"VehicleState::radar data lost"<<std::endl;
    }

    // update current route
    uint16_t id     = sailTask.u8_PointNum;
    uint16_t max_id = sailTask.sailMsg.u8_pointSum;

    lat_now_ = ins_msg.latitude;
    lon_now_ = ins_msg.longitude;

    if(!initialized_ && id == 0 && gps_msg_ok && max_id >= 1){
          LOG(INFO)<< "VehicleState:: Set initial sail task point!";
          lat_start_ = lat_now_;
          lon_start_ = lon_now_;
          initialized_ = true;
    }

    if(id >= 1){
        lat_start_     = sailTask.sailMsg.wayPoint[id-1].f64_latitude;
        lon_start_     = sailTask.sailMsg.wayPoint[id-1].f64_longitude;
    }
    lat_end_            = sailTask.sailMsg.wayPoint[id].f64_latitude;
    lon_end_            = sailTask.sailMsg.wayPoint[id].f64_longitude;


    // update sensor message
    double radar_heading =0.0,radar_track_error = 0.0,radar_course = 0.0;
    if(radar_msg_ok == true){
        radar_track_error     =  AP::radar_message()->ipc2arm_msg().relative_distance;
        radar_heading         =  AP::radar_message()->ipc2arm_msg().relative_heading;
        radar_course          =  radar_heading;
    }
    
    double gps_heading = 0.0,gps_track_error = 0.0,gps_course = 0.0;
    if(gps_msg_ok){
        // compute vector of route
        double OA_distance = Get_distance(lat_start_,lon_start_,lat_end_,lon_end_);
        double OA_bearing  = Get_heading(lat_start_,lon_start_,lat_end_,lon_end_);
        math::Vec2d OA;
        OA.set_x(OA_distance * cos(math::radians(OA_bearing)));
        OA.set_y(OA_distance * sin(math::radians(OA_bearing)));
    
        // compute PA
        double PA_distance = Get_distance(lat_now_,lon_now_,lat_end_,lon_end_);
        double PA_bearing  = Get_heading(lat_now_,lon_now_,lat_end_,lon_end_);
        math::Vec2d PA;
        PA.set_x(PA_distance * cos(math::radians(PA_bearing)));
        PA.set_y(PA_distance * sin(math::radians(PA_bearing)));

        // Get distance from Vehicle to route
        gps_track_error = (OA.Length() >= 1e-6)?PA.CrossProd(OA)/OA.Length():0.0;

        // update pos infomation
        gps_heading = math::wrap_180(ins_msg.heading - OA_bearing);
        gps_course  = math::wrap_180(ins_msg.motionDirection - OA_bearing);
    }

    // update vehicle state sourece
    if(radar_msg_ok == true && gps_msg_ok == true){
        status_ = GPS_OK_RADAR_OK;
    }else if(radar_msg_ok == true && gps_msg_ok == false){
        status_ = GPS_ERR_RADAR_OK;
    }else if(radar_msg_ok == false && gps_msg_ok == true){
        status_ = GPS_OK_RADAR_ERR;
    }else{
        status_ = GPS_ERR_RADAR_ERR;
    }

    // select data source
    uint8_t nav_source = AP::conf()->navigation_info_source;
    if(nav_source == 1){
        status_ = GPS_OK_RADAR_ERR; // Only use GNSS data
    }else if(nav_source == 2){
        status_ = GPS_ERR_RADAR_OK; // Only use Radar data
    } 

    // initialization not completed,navigation not allowed
    if(!initialized_ && id == 0){
       LOG(INFO)<< "VehicleState:: Initialization not completed,navigation not allowed!";
        status_ = GPS_ERR_RADAR_ERR;
    }

    // update curent relative position and attitude
    switch (status_)
    {
    case GPS_OK_RADAR_OK:
        if(AP::boat()->route_switch() == true){
           lateral_error_     = gps_track_error;
           heading_error_     = (use_ground_speed_angle_ == true)?(gps_course):(gps_heading);
       
        }else{
            lateral_error_    = radar_track_error;
            heading_error_    = (use_ground_speed_angle_ == true && ins_msg.speed >= 1.0)?(radar_course):(radar_heading);
        }

        linear_velocity_       = (use_ground_speed_ == true && ins_msg.speed >= 1.0)?(ins_msg.speed):(ins_msg.speed * cos(math::radians(gps_course)));
        angular_velocity_      = (use_external_angular_velocity_ == true)?(ins_msg.rotRate):heading_error_td_filter.z2();
        time_stamp_ = cur_time;

        break;
    case GPS_ERR_RADAR_OK:
        // only use radar data for navigation
        lateral_error_          = radar_track_error;
        heading_error_          = (use_ground_speed_angle_ == true)?(radar_course):(radar_heading);
        angular_velocity_       = heading_error_td_filter.z2();
        time_stamp_             = cur_time;
 
        break;
    case GPS_OK_RADAR_ERR:
        lateral_error_    = gps_track_error;
        heading_error_    = (use_ground_speed_angle_ == true && ins_msg.speed >= 1.0)?(gps_course):(gps_heading);
    
        linear_velocity_  = (use_ground_speed_ == true && ins_msg.speed >= 1.0)?(ins_msg.speed):(ins_msg.speed * cos(math::radians(gps_course)));
        angular_velocity_ = (use_external_angular_velocity_ == true)?(ins_msg.rotRate):heading_error_td_filter.z2();
        time_stamp_       = cur_time;
    
        break;
    case GPS_ERR_RADAR_ERR:
        LOG(ERROR) << "VehicleState:: sensor state  is GPS_ERR_RADAR_ERR";
        break;
    default:
        break;
    }

    // update td filter
    if(status_ == GPS_ERR_RADAR_ERR){
        heading_error_td_filter.Reset();
    }else{
        heading_error_td_filter.Update(heading_error_,heading_error_,ts);
    }

}

VehicleState *VehicleState::singleton_ = nullptr;

VehicleState *VehicleState::get_singleton()
{
    return singleton_;
}

namespace AP {

VehicleState *vehicle_state()
{
    return VehicleState::get_singleton();
}

};
