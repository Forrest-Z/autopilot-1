#pragma once

#include <common/commom.h>
#include <math/Location.h>
#include <math/vector2.h>


using namespace LOC;

/*
 * BendyRuler avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OABendyRuler {
public:
    AP_OABendyRuler();

    /* Do not allow copies */
    AP_OABendyRuler(const AP_OABendyRuler &other) = delete;
    AP_OABendyRuler &operator=(const AP_OABendyRuler&) = delete;

    // send configuration info stored in front end parameters
    void set_ekf_origin(const Location &ekf_origin){ _ekf_origin = ekf_origin;}

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    bool update(const Location& current_loc, const Location& destination, const float ground_course_deg, Location &origin_new, Location &destination_new, bool proximity_only);

    bool give_up_current_waypoint(void) const { return (destinatoin_near_obstacle_ == true) || (destination_unreachable_ == true);}

    // search for path in XY direction
    bool search_xy_path(const Location& current_loc, const Location& destination, float ground_course_deg, Location &destination_new, float lookahead_step_1_dist, float lookahead_step_2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);

private:

    // calculate minimum distance between a path and any obstacle
    float calc_avoidance_margin(const Location &start, const Location &end, bool proximity_only) const;

    // determine if BendyRuler should accept the new bearing or try and resist it. Returns true if bearing is not changed  
    bool resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const;    

    // calculate minimum distance between a path and proximity sensor obstacles
    // on success returns true and updates margin
    bool calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const;

    // internal variables used by background thread
    float _current_lookahead;       // distance (in meters) ahead of the vehicle we are looking for obstacles
    float _bearing_prev;            // stored bearing in degrees 
    Location _destination_prev;     // previous destination, to check if there has been a change in destination
    Location _ekf_origin;

    bool destinatoin_near_obstacle_{false};
    bool destination_unreachable_{false};

private:
    
    // OA common parameters
    float _margin_max{1.5f};           // object avoidance will ignore objects more than this many meters from vehicle
    float _lookahead{15.0f};            // object avoidance will look this many meters ahead of vehicle
    float _bendy_ratio{1.5f};           // object avoidance will avoid major directional change if change in margin ratio is less than this param
    float _bendy_angle{75.0f};          // object avoidance will try avoding change in direction over this much angle
};
