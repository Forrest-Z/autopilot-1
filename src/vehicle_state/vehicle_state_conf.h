#pragma once

#include <stdint.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include <adrc/td_conf.h>

class VehicleStateConf{
public:
    VehicleStateConf() = default;
    virtual ~VehicleStateConf() = default;
    
    bool use_ground_speed_angle_;
    bool use_ground_speed_;
    bool use_external_angular_velocity_;

    TdConf      heading_error_td_filter_conf_; 
};