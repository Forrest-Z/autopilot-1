#pragma once

#include <cmath>
#include <stdarg.h>

// Libraries
#include <conf/conf.h>

#include <hal/console.hpp>

#include <control/lon_controller.h>
#include <control/lat_controller.h>

#include <vehicle_state/vehicle_state.h>
#include <radar/radar_message.h>
#include <simulation/sim_boat.h>

#include <common/commom.h>
#include <conf/read_config_xml.h>



class Boat{
public:

    Boat();
    virtual ~Boat() = default;

   /* Do not allow copies */
    Boat(const Boat &other) = delete;
    Boat &operator=(const Boat&) = delete;

    static Boat *get_singleton();

    void setup(void);

    void loop(void);

    bool route_switch() const{return route_switch_;}

protected:
    bool finished_sail_task();
    bool reached_destination();
    void update_sampling_task();
    void update_navigation_task();

private:
    static Boat* singleton_;

    // Configuration
	ReadConfigXml read_config_xml_;

    Conf conf_;

    // Console
    Console console_;

    // command bus
    ControlCommand cmd; 
    
    // controller
    LatController lat_controller_;
    LonController lon_controller_;

    // vehicle state
    std::shared_ptr<VehicleState> injector_;

    // radar communication
    RadarMessage radar_message;
    
    // simulation
    SimBoat sim_boat_;

    // pathplanner
    AP_OAPathPlanner oa;
    
private:
    double loop_ts_ = 0.05;
    bool   route_switch_ = true;
    uint64_t last_update_time = 0;
private:
// parameters
   double route_switch_steering_cmd = 0;
   double throttle_limit =0;
   double steering_limit = 0;
   double position_kp;
   double position_second_limit;

private:
   float _overshoot{1.0};    // [0~10m,inc:0.1m]
   float _turn_radius{1.0};  // [0~10m,inc:0.1m]
   float _turn_max_g{0.1}; // [0~0.3g,inc:0.01g]

};

namespace AP {
    Boat *boat();
};

extern Boat boat;
