#pragma once

#include <stdint.h>
#include <string.h>
#include <string>

/* Include all parameter class */
#include <boat/boat_conf.h>
#include <control/control_conf.h>
#include <radar/radar_conf.h>
#include <vehicle_state/vehicle_state_conf.h>

class Conf{
public:

    Conf();

    // empty descontuctor function
    virtual ~Conf(){}

   // get singleton instance
    static Conf *get_singleton() {
        return singleton_;
    }

    // boat_conf
    BoatConf boat_conf_;

    // control_conf
    ControlConf control_conf_;

    // radar_conf
    RadarConf radar_conf_;

    // vehicle_state_conf
    VehicleStateConf vehicle_state_conf_;
  
    /* debug */
    uint8_t navigation_info_source; // 0: auto 1:GNSS 2: Radar
    bool    simualtion_enable;      // true or false
    bool    old_code_test_enable;   // true or false

private:
    static Conf *singleton_;
 
};

namespace AP{
    Conf *conf();
};