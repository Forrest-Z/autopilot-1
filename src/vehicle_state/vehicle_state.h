#pragma once

#include <stdint.h>
#include <adrc/td.h>
#include "vehicle_state_conf.h"
#include <iostream>
#include <planning/pathplanner.h>

//using namespace LOC;

class VehicleState{
public:
    enum SensorStatus{
        GPS_OK_RADAR_OK = 0,
        GPS_OK_RADAR_ERR = 1,
        GPS_ERR_RADAR_ERR = 2,
        GPS_ERR_RADAR_OK  = 3
    };

    VehicleState(){
        if(singleton_){
            std::cout<<"Too many VehicleState Class" << std::endl;
            return;
        }
        singleton_ = this;
    }
    virtual ~VehicleState() = default;


    /* Do not allow copies */
    VehicleState(const VehicleState &other) = delete;
    VehicleState &operator=(const VehicleState&) = delete;

    static VehicleState *get_singleton();

    void Init(VehicleStateConf &conf);

    void Update(double ts);

    void update(double ts);

    void Reset(void);


    SensorStatus sensor_status() const{return status_;}
    uint64_t TimeStamp() const{return time_stamp_;}

    double lateral_error() const{return lateral_error_;}
    double heading_error() const{return heading_error_;}

    double angular_velocity() const{return angular_velocity_;}
    double linear_velocity() const {return linear_velocity_;}

    double lat_start() const{return lat_start_;}
    double lon_start() const{return lon_start_;}
    double lat_end() const{return lat_end_;}
    double lon_end() const{return lon_end_;}
    double lat_now() const{return lat_now_;}
    double lon_now() const{return lon_now_;}

    bool   get_ekf_origin(Location &origin) const;
    bool   stop_boat(void) const { return stop_vehicle;}
    bool   waypoint_unreachable(void) const { return _oa_dest_unreachable;}

private:

    uint64_t time_stamp_ = 0;

    SensorStatus status_;

    // pose
    double  lateral_error_ = 0;    // m
    double  heading_error_ = 0;    // deg

    // velocity
    double  angular_velocity_ = 0; // deg/s
    double  linear_velocity_ = 0;  // m/s

    // position
    double  lat_start_;
    double  lon_start_;
    double  lat_end_;
    double  lon_end_;
    double  lat_now_;
    double  lon_now_;

private:

    static VehicleState *singleton_;
    uint64_t last_update_time_ = 0;
    bool initialized_ = false;

private:
    bool use_ground_speed_angle_;
    bool use_ground_speed_;
    bool use_external_angular_velocity_;
    TD   heading_error_td_filter;

private:
    bool _oa_active;                // true if we should use alternative destination to avoid obstacles
    LOC::Location ekf_origin_;
    bool ekf_origin_is_set{false};
    bool stop_vehicle{false};
    bool _oa_dest_unreachable{false};

    LOC::Location _oa_origin;            // intermediate origin during avoidance
    LOC::Location _oa_destination;       // intermediate destination during avoidance
    LOC::Location _origin;               // origin Location (vehicle will travel from the origin to the destination)
    LOC::Location _destination;          // destination Location when in Guided_WP
};

namespace AP {
    VehicleState *vehicle_state();
};