#pragma once

#include <stdint.h>
#include <vector>
#include <string>
#include <map>

// adrc controller
#include <adrc/eso_conf.h>
#include <adrc/lpd_conf.h>
#include <adrc/td_conf.h>
#include <adrc/npd_conf.h>

// pid controller
#include <pid/pid_conf.h>
#include <pid/pid_inc_conf.h>
#include <pid/pid_sqrt_conf.h>

// mrac controller
#include <mrac/mrac_adrc_conf.h>


class ControlConf{
public:
    ControlConf()
    {
        controller_type.insert(std::make_pair("LAT_CONTROLLER_PID",LAT_CONTROLLER_PID));
        controller_type.insert(std::make_pair("LAT_CONTROLLER_ADRC",LAT_CONTROLLER_ADRC));
        controller_type.insert(std::make_pair("LAT_CONTROLLER_MRAC",LAT_CONTROLLER_MRAC));

        controller_type.insert(std::make_pair("LON_CONTROLLER_PID",LON_CONTROLLER_PID));
        controller_type.insert(std::make_pair("LON_CONTROLLER_ADRC",LON_CONTROLLER_ADRC));
        controller_type.insert(std::make_pair("LON_CONTROLLER_MRAC",LON_CONTROLLER_MRAC));
    }
    virtual ~ControlConf() = default;

    // active_controllers
    enum ControllerType:uint8_t{
        LAT_CONTROLLER_PID = 0,
        LAT_CONTROLLER_ADRC,
        LAT_CONTROLLER_MRAC,

        LON_CONTROLLER_PID,
        LON_CONTROLLER_ADRC,
        LON_CONTROLLER_MRAC,
    };

     
    std::map<std::string,ControllerType> controller_type;
    std::string lat_controller_type_;
    std::string lon_controller_type_;

    // lateral controller common parameters
    // [LAT_CONTROLLER_COMMON]
    double  lat_controller_error_limit_;
    double  lat_controller_input_rate_limit_;
    double  lat_controller_output_limit_;
    double  lat_controller_output_rate_limit_;
    double  lat_sight_track_distance_;

 
    // lateral controller with PID
    // [LAT_CONTROLLER_PID]
    PIDSqrtConf lat_steering_angle_pid_conf_;
    PIDSqrtConf lat_steering_rate_pid_conf_;

    // lateral controller with ADRC
    // [LAT_CONTROLLER_ADRC]
    ESOConf    lat_steering_angle_leso_conf_;
    NpdConf     lat_steering_angle_npd_conf_;

    // lateral controller with MRAC 
    // [LAT_CONTROLLER_MRAC]
    MracAdrcConf lat_steering_angle_mrac_conf_;

    // longitudinal_controller common paramters
    // [LON_CONTROLLER_COMMON]
    double  lon_controller_error_limit_;
    double  lon_controller_input_rate_limit_;
    double  lon_controller_output_limit_;
    double  lon_controller_output_rate_limit_;
    double  lon_radar_hold_thtottle_cmd_;
    
    // longitudinal_controller with PID
    // [LON_CONTROLLER_PID]
    PIDIncConf lon_speed_pid_conf_;

    // longitudinal_controller with ADRC
    // [LON_CONTROLLER_ADRC]
    ESOConf    lon_speed_leso_conf_;
    PIDSqrtConf lon_speed_sqrt_conf_;

    // lateral controller with MRAC 
    // [LON_CONTROLLER_MRAC]
    MracAdrcConf lon_speed_mrac_conf_;

};