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

extern LOC::Location ekf_origin_;

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

    if(cur_time - AP::radar_message()->radar_message_time_stamp() >= 500){
        radar_msg_ok = false;
    }

    // get current route with start point,goal point
    uint16_t id     = sailTask.u8_PointNum;
    uint16_t max_id = sailTask.sailMsg.u8_pointSum;
    lat_now_        = ins_msg.latitude;
    lon_now_        = ins_msg.longitude;

   
    if(last_id_ != id){
        if(id == 0){
            lat_start_ = lat_now_;
            lon_start_ = lon_now_;
        }else{
            lat_start_ = sailTask.sailMsg.wayPoint[id-1].f64_latitude;
            lon_start_ = sailTask.sailMsg.wayPoint[id-1].f64_longitude;
        }
        last_id_ = id;
    }

     if(!initialized_ /*&& id == 0*/ && gps_msg_ok && max_id >= 1){
        LOG(INFO)<< "VehicleState:: Set initial sail task point!";
        lat_start_ = lat_now_;
        lon_start_ = lon_now_;
        initialized_ = true;
    }

    lat_end_   = sailTask.sailMsg.wayPoint[id].f64_latitude;
    lon_end_   = sailTask.sailMsg.wayPoint[id].f64_longitude;

    if(max_id == 0){
        lat_next_ = lat_now_;
        lon_next_ = lon_now_;
    }else{
        uint16_t next_wp_id =id+1;
        next_wp_id = (next_wp_id >= max_id-1)?(max_id-1):(next_wp_id);
        lat_next_   = sailTask.sailMsg.wayPoint[next_wp_id].f64_latitude;
        lon_next_   = sailTask.sailMsg.wayPoint[next_wp_id].f64_longitude;
     }


    desired_speed_ms_   = kn2ms(sailTask.sailMsg.wayPoint[id].f64_expSpeed);

    // translate current lat-lng into xy  
    _origin.lat = lat_start_*1e7;
    _origin.lng = lon_start_*1e7;
    _origin.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);
    _destination.lat = lat_end_*1e7;
    _destination.lng = lon_end_*1e7;
    _destination.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    // next route destination
    Location next_destination;
    next_destination.lat = lat_next_ * 1e7;
    next_destination.lng = lon_next_ * 1e7;
    next_destination.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    // current location
    Location current_loc;
    current_loc.lat = lat_now_*1e7;
    current_loc.lng = lon_now_*1e7;
    current_loc.set_alt_cm(0,Location::AltFrame::ABOVE_ORIGIN);

    // run path planning around obstacles
     stop_vehicle = false;

    // true if OA has been recently active;
    bool _oa_was_active = _oa_active;
    _oa_dest_unreachable = false;
    
    double current_yaw = 0.0;
     current_yaw = ins_msg.heading;

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc, _origin, _destination,desired_speed_ms_,current_yaw, _oa_origin, _oa_destination,_oa_speed_ms);
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
             //stop_vehicle = true;
            _oa_active = false;
            break;
        }
    }

    if (!_oa_active) {
        _oa_origin      = _origin;
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

    _oa_advance_finished = false;

    if(_oa_dest_unreachable == true){
        if((id +1) < max_id){
            const float next_route_bearing  = _destination.get_bearing_to(next_destination)*0.01f;
            const float next_route_distance = _destination.get_distance(next_destination);
            const float advance_distance = (next_route_distance > 10.0f)?(10.0f):(next_route_distance);

            Location sub_wp = _destination;
            sub_wp.offset_bearing(next_route_bearing,advance_distance);
            sailTask.sailMsg.wayPoint[id].f64_latitude  = sub_wp.lat * 1e-7;
            sailTask.sailMsg.wayPoint[id].f64_longitude = sub_wp.lng * 1e-7;

            if(next_route_distance <1.0){
            _oa_advance_finished = true;
            }
            printf("vehicle_state::remain_search_distance[%d] = %f\n",id,next_route_distance);
            initialized_ = false;
        }else{
            _oa_advance_finished = true;
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

    lateral_error_     = (OA.Length() >= 1e-6)?PA.CrossProd(OA)/OA.Length():0.0;
    heading_error_     = math::wrap_180(current_yaw - OA_bearing);

    linear_velocity_   = (gps_msg_ok == true)?(ins_msg.speed):(linear_velocity_);
    angular_velocity_  = (use_external_angular_velocity_ == true)?(ins_msg.rotRate):heading_error_td_filter.z2();
    time_stamp_        = cur_time;


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
