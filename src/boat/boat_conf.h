
#pragma once

#include <stdint.h>

class BoatConf{
public:
    BoatConf() = default;
    virtual ~BoatConf() =default;

    double  route_switch_steering_cmd_;
    double  position_kp_;
    double  position_second_limit_;

    // console
    std::string local_ip_;
    std::string console_ip_;
    uint16_t    local_port_;
    uint16_t    console_port_;
                
};