/*
 * Location.cpp
 */

#include "Location.h"
#include "definitions.h"

namespace LOC{
/// constructors
Location::Location()
{
    zero();
}

const Location definitely_zero{};
// bool Location::is_zero() const
// {
//     return !memcmp(this, &definitely_zero, sizeof(*this));
// }

void Location::zero(void)
{
    memset(this, 0, sizeof(*this));
}

Location::Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}

Location::Location(const Vector3f &ekf_offset_neu,const Location &ekf_origin)
{
    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, AltFrame::ABOVE_ORIGIN);

    // calculate lat, lon
    lat = ekf_origin.lat;
    lng = ekf_origin.lng;
    offset(ekf_offset_neu.x / 100.0f, ekf_offset_neu.y / 100.0f);
    
}

void Location::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            // do nothing
            break;
        case AltFrame::ABOVE_HOME:
            relative_alt = true;
            break;
        case AltFrame::ABOVE_ORIGIN:
            origin_alt = true;
            break;
        case AltFrame::ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            relative_alt = true;
            terrain_alt = true;
            break;
    }
}


// get altitude frame
Location::AltFrame Location::get_alt_frame() const
{
    if (terrain_alt) {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt) {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt) {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}


bool Location::get_vector_xy_from_origin_NE(Vector2f &vec_ne,const Location &ekf_origin) const
{
    if(ekf_origin.lat == 0 || ekf_origin.lng == 0){
        return false;
    }
    vec_ne.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_ne.y = (lng-ekf_origin.lng) * LATLON_TO_CM * ekf_origin.longitude_scale();
    return true;
}


// return distance in meters between two locations
float Location::get_distance(const struct Location &loc2) const
{
    float dlat = (float)(loc2.lat - lat);
    float dlng = ((float)(loc2.lng - lng)) * loc2.longitude_scale();
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}


/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f Location::get_distance_NE(const Location &loc2) const
{
    return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale());
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3f Location::get_distance_NED(const Location &loc2) const
{
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale(),
                    (alt - loc2.alt) * 0.01f);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset(float ofs_north, float ofs_east)
{
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale();
    lat += dlat;
    lng += dlng;
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location::offset_bearing(float bearing, float distance)
{
    const float ofs_north = cosf(math::radians(bearing)) * distance;
    const float ofs_east  = sinf(math::radians(bearing)) * distance;
    offset(ofs_north, ofs_east);
}

// extrapolate latitude/longitude given bearing, pitch and distance
void Location::offset_bearing_and_pitch(float bearing, float pitch, float distance)
{
    const float ofs_north =  cosf(math::radians(pitch)) * cosf(math::radians(bearing)) * distance;
    const float ofs_east  =  cosf(math::radians(pitch)) * sinf(math::radians(bearing)) * distance;
    offset(ofs_north, ofs_east);
    const int32_t dalt =  sinf(math::radians(pitch)) * distance *100.0f;
    alt += dalt; 
}


float Location::longitude_scale() const
{
    float scale = cosf(lat * (1.0e-7f * DEG_TO_RAD));
    return std::fmax(scale, 0.01f);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool Location::sanitize(const Location &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt) {
        relative_alt = false;
        alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng()) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

// make sure we know what size the Location object is:
assert_storage_size<Location, 16> _assert_storage_size_Location;


// return bearing in centi-degrees from location to loc2
int32_t Location::get_bearing_to(const struct Location &loc2) const
{
    const int32_t off_x = loc2.lng - lng;
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale();
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Location::same_latlon_as(const Location &loc2) const
{
    return (lat == loc2.lat) && (lng == loc2.lng);
}

// return true when lat and lng are within range
bool Location::check_latlng() const
{
    return (labs(lat) <= 90*1e7) && (labs(lng) <= 180*1e7);
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Location::past_interval_finish_line(const Location &point1, const Location &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float Location::line_path_proportion(const Location &point1, const Location &point2) const
{
    const Vector2f vec1 = point1.get_distance_NE(point2);
    const Vector2f vec2 = point1.get_distance_NE(*this);
    const float dsquared = SQ(vec1.x) + SQ(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}

}