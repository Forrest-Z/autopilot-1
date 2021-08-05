#include "pathplanner.h"
#include <thread>
#include <functional>
#include <user_time/user_time.h>
#include <math_utils.h>
// parameter defaults
const float OA_MARGIN_MAX_DEFAULT = 5;
const int16_t OA_OPTIONS_DEFAULT = 1;

const int16_t OA_UPDATE_MS  = 200;      // path planning updates run at 5hz
const int16_t OA_TIMEOUT_MS = 3000;     // results over 3 seconds old are ignored


/// Constructor
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;
}

// perform any required initialisation
void AP_OAPathPlanner::init(const PlanningConf &conf)
{
    // load parameters
    _type = conf.PP_type;
     lon_scan_distance  = conf.lon_scan_distance;
     lon_scan_angle     = conf.lon_scan_angle;
     lon_time_constance = conf.lon_time_constance;
     lon_dccel_speed    = conf.lon_dccel_speed;

    // run background task looking for best alternative destination
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        return;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            _oabendyruler->Init(conf);
        }
        break;
    }

    _oadatabase.init(conf);
    start_thread();
}


bool AP_OAPathPlanner::start_thread()
{
    std::lock_guard<std::mutex> lock(_rsem);

    if (_thread_created) {
        return true;
    }
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    std::thread t(std::bind(&AP_OAPathPlanner::avoidance_thread,this));
    t.detach();

    _thread_created = true;
    return true;
}

// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_loc with an intermediate location
AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         const float desired_speed_ms,
                                         const float ground_course_deg,
                                         Location &result_origin,
                                         Location &result_destination,
                                         float  &result_desired_speed_ms)
{
    // exit immediately if disabled or thread is not running from a failed init
    if (_type == OA_PATHPLAN_DISABLED || !_thread_created) {
        return OA_NOT_REQUIRED;
    }

    const uint32_t now = user_time::get_millis();
     std::lock_guard<std::mutex> lock(_rsem);

    // place new request for the thread to work on
    avoidance_request.current_loc = current_loc;
    avoidance_request.origin = origin;
    avoidance_request.destination = destination;
    avoidance_request.ground_course_deg = ground_course_deg;
    avoidance_request.request_time_ms  = now;
    avoidance_request.desired_speed_ms = desired_speed_ms;

    // check result's destination matches our request
    const bool destination_matches = (destination.lat == avoidance_result.destination.lat) && (destination.lng == avoidance_result.destination.lng);

    // check results have not timed out
    const bool timed_out = now - avoidance_result.result_time_ms > OA_TIMEOUT_MS;

    // return results from background thread's latest checks
    if (destination_matches && !timed_out) {
        // we have a result from the thread
        result_origin = avoidance_result.origin_new;
        result_destination = avoidance_result.destination_new;
        result_desired_speed_ms = avoidance_result.result_desired_speed_ms;
        return avoidance_result.ret_state;
    }

    // if timeout then path planner is taking too long to respond
    if (timed_out) {
        return OA_ERROR;
    }

    // background thread is working on a new destination
    return OA_PROCESSING;
}

extern LOC::Location ekf_origin_;
// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
void AP_OAPathPlanner::avoidance_thread()
{   
    // require ekf origin to have been set
    while (!origin_set) {
        origin_set =  (ekf_origin_.lat != 0 && ekf_origin_.lng != 0)?(true):(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        printf("need set ekf original\n");
    }

    while (true) {

        // if database queue needs attention, service it faster
        if (_oadatabase.process_queue()) {
           std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        const uint32_t now = user_time::get_millis();
        if (now - avoidance_latest_ms < OA_UPDATE_MS) {
            continue;
        }
        avoidance_latest_ms = now;

        _oadatabase.update();

        Location origin_new;
        Location destination_new;
        float desired_speed_new;
        {
              std::lock_guard<std::mutex> lock(_rsem);
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // this is a very old request, don't process it
                continue;
            }

            // copy request to avoid conflict with main thread
            avoidance_request2 = avoidance_request;

            // store passed in origin and destination so we can return it if object avoidance is not required
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
            desired_speed_new = avoidance_request.desired_speed_ms;
        }

        // run background task looking for best alternative destination
        OA_RetState res = OA_NOT_REQUIRED;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER:
            if (_oabendyruler == nullptr) {
                printf("OAPathPlanner need reboot");
                continue;
            }
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.origin,avoidance_request2.destination, avoidance_request2.ground_course_deg, origin_new, destination_new, false)) {
                res = OA_SUCCESS;
            }else if(_oabendyruler->give_up_current_waypoint() == true){
                res = OA_CAN_NOT_ARRIVAL;
            }
            break;
        }

        // longitude control
         adjust_desired_speed(avoidance_request2.current_loc,avoidance_request2.ground_course_deg,desired_speed_new);

        {
            // give the main thread the avoidance result
             std::lock_guard<std::mutex> lock(_rsem);
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
            avoidance_result.result_desired_speed_ms = desired_speed_new;
            avoidance_result.result_time_ms = user_time::get_millis();
            avoidance_result.ret_state = res;
            
        }
    }
}

void AP_OAPathPlanner::adjust_desired_speed(const Location &current_loc,const float ground_course_deg,float &desired_speed_ms)
{
    // exit immediately if db is empty
    if(!_oadatabase.healthy()){
        return;
    }

    //check each obstacle's distance from segment
    float smallest_distance = FLT_MAX;
    for(uint16_t i = 0; i<_oadatabase.database_count(); i++){
        const AP_OADatabase::OA_DbItem& item = _oadatabase.get_item(i);

        // compute relative distance to obstacle
        Location obstacle(Vector3f(item.pos.x * 100.0f,item.pos.y *100.0f,0.0f),ekf_origin_);
        const float distance_to_obstacle  = current_loc.get_distance(obstacle);
        if(distance_to_obstacle > lon_scan_distance){
            continue;
        }

        // compute relative heading to obstacle
        const float heading_to_obstacle = math::wrap_180(current_loc.get_bearing_to(obstacle) *0.01f - ground_course_deg);

        // compute smallest distance with a certain heading
        if(heading_to_obstacle <= lon_scan_angle && heading_to_obstacle >= -lon_scan_angle){
            if(smallest_distance > distance_to_obstacle){
                smallest_distance = distance_to_obstacle;
            }
        }
    }
    if(smallest_distance>lon_scan_distance) 
        return;
  
    const float speed_ms = math::SqrtController(smallest_distance,1.0f/lon_time_constance, lon_dccel_speed, 0.001f * OA_UPDATE_MS);
    //printf("[INFO]:: reference_speed_ms = %f,filtered_speed_ms = %f\n",desired_speed_ms,speed_ms);
    desired_speed_ms = math::Clamp(speed_ms,0.0f,desired_speed_ms);

}





// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
