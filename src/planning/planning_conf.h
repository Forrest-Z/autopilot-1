#pragma once
#include <stdint.h>

class PlanningConf{
public:
    PlanningConf(){}
    virtual ~PlanningConf() = default;

    // bendyruler
    float BR_lookahead{10.0f};
    float BR_bendy_ratio{1.5f};
    float BR_bendy_angle{75.0f};
    float BR_margin_max{2.0f};

    float BR_max_deviate_angle{100.0f};
    float BR_min_near_distance{5.0f};

    float BR_shoreline_safe_pb{0.1f};
    float BR_shoreline_safe_distance{15.0f};
    float BR_shoreline_safe_angle{50.0f};

    // database
    int16_t      DB_queue_size_param{360};                       // queue size
    int16_t      DB_database_size_param{720};                    // db size
    int16_t      DB_database_expiry_seconds{5};                  // objects expire after this timeout
    float        DB_beam_width{10.0};                            // beam width used when converting lidar readings to object radius
    float        DB_radius_min{1.5f};                            // objects minimum radius (in meters)
    float        DB_dist_max{50.0f};                             // objects maximum distance (in meters)

    // pathplanner
    int8_t PP_type{1};

    // 
    float lon_scan_distance{30.0};
    float lon_scan_angle{30};
    float lon_time_constance{5};
    float lon_dccel_speed{0.5};
};