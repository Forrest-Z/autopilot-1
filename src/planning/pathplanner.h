#pragma once

#include "bendyruler.h"
#include "database.h"
#include <mutex>
#include <stdio.h>

using namespace LOC;

/*
 * This class provides path planning around fence, stay-out zones and moving obstacles
 */
class AP_OAPathPlanner {

public:
    AP_OAPathPlanner();

    /* Do not allow copies */
    AP_OAPathPlanner(const AP_OAPathPlanner &other) = delete;
    AP_OAPathPlanner &operator=(const AP_OAPathPlanner&) = delete;

    // get singleton instance
    static AP_OAPathPlanner *get_singleton() {
        return _singleton;
    }

    // perform any required initialisation
    void init();

   // void set_ekf_origin(const Location &ekf_origin){ _ekf_origin = ekf_origin;origin_set = true;printf("SET ekf original successed for AP_OAPathPlanner!\n");}

    // object avoidance processing return status enum
    enum OA_RetState : uint8_t {
        OA_NOT_REQUIRED = 0,            // object avoidance is not required
        OA_PROCESSING,                  // still calculating alternative path
        OA_ERROR,                       // error during calculation
        OA_SUCCESS,                     // success
        OA_CAN_NOT_ARRIVAL
    };

    // provides an alternative target location if path planning around obstacles is required
    // returns true and updates result_origin and result_destination with an intermediate path
    OA_RetState mission_avoidance(const Location &current_loc,
                           const Location &origin,
                           const Location &destination,
                           const float ground_course_deg,
                           Location &result_origin,
                           Location &result_destination) WARN_IF_UNUSED;

    // enumerations for _TYPE parameter
    enum OAPathPlanTypes {
        OA_PATHPLAN_DISABLED = 0,
        OA_PATHPLAN_BENDYRULER = 1,
    };

    // enumeration for _OPTION parameter
    enum OARecoveryOptions {
        OA_OPTION_DISABLED = 0,
        OA_OPTION_WP_RESET = (1 << 0),
    };

    uint16_t get_options() const { return _options;}

private:

    // avoidance thread that continually updates the avoidance_result structure based on avoidance_request
    void avoidance_thread();
    bool start_thread();

    // an avoidance request from the navigation code
    struct avoidance_info {
        Location current_loc;
        Location origin;
        Location destination;
        float    ground_course_deg;
        uint32_t request_time_ms;
    } avoidance_request, avoidance_request2;

    // an avoidance result from the avoidance thread
    struct {
        Location destination;       // destination vehicle is trying to get to (also used to verify the result matches a recent request)
        Location origin_new;        // intermediate origin.  The start of line segment that vehicle should follow
        Location destination_new;   // intermediate destination vehicle should move towards
        uint32_t result_time_ms;    // system time the result was calculated (used to verify the result is recent)
        OA_RetState ret_state;      // OA_SUCCESS if the vehicle should move along the path from origin_new to destination_new
    } avoidance_result;

    // parameters
 
    
    // internal variables used by front end
     std::mutex _rsem;            // semaphore for multi-thread use of avoidance_request and avoidance_result
    bool _thread_created;           // true once background thread has been created
    AP_OABendyRuler *_oabendyruler; // Bendy Ruler algorithm

    AP_OADatabase _oadatabase;      // Database of dynamic objects to avoid
    uint32_t avoidance_latest_ms;   // last time Dijkstra's or BendyRuler algorithms ran

    bool proximity_only = true;
    static AP_OAPathPlanner *_singleton;
  //  Location _ekf_origin;
    bool origin_set{false};

private:
    // paramters
    int8_t _type{OA_PATHPLAN_BENDYRULER};                  // avoidance algorithm to be used
    int16_t _options{0};                                   // Bitmask for options while recovering from Object Avoidance

};

namespace AP {
    AP_OAPathPlanner *ap_oapathplanner();
};
