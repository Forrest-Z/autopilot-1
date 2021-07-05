#pragma once

class PlannerOpenSpaceConfig{
public:
PlannerOpenSpaceConfig() = default;

double grid_a_star_xy_resolution{0.1};
double node_radius{0.5};
double xy_grid_resolution{0.2};
double phi_grid_resolution{0.05};
double step_size{0.5};
double delta_t{0.05};
uint64_t next_node_num{10};
double traj_forward_penalty{0.0};
double traj_back_penalty {0.0};
double traj_gear_switch_penalty{10.0};
double traj_steer_penalty{100.0};
double traj_steer_change_penalty{10.0};
};

class VehicleParam{
public:
    // vehicle parameters
    double vehicle_length{1.65};
    double vehicle_width{0.5};
    double vehicle_back_edge_to_center{0.8};
    double max_steer_angle{35.0};
    double steer_ratio{1.0};
    double wheel_base{0.8};
};


class BendyRulerConfig{
public:
    // OA common parameters
    float _margin_max;           // object avoidance will ignore objects more than this many meters from vehicle

    // BendyRuler parameters
    float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    float _bendy_ratio;          // object avoidance will avoid major directional change if change in margin ratio is less than this param
    float _bendy_angle;          // object avoidance will try avoding change in direction over this much angle
};

class DataBaseConfig{
public:
    
    // parameters
    int16_t      _queue_size_param;                     // queue size
    int16_t      _database_size_param;                  // db size
    int16_t      _database_expiry_seconds;               // objects expire after this timeout
    int16_t      _output_level;                          // controls which items should be sent to GCS
    float        _beam_width;                            // beam width used when converting lidar readings to object radius
    float        _radius_min;                            // objects minimum radius (in meters)
    float        _dist_max;                              // objects maximum distance (in meters)
    float        _min_alt;                               // OADatabase minimum vehicle height check (in meters)
};