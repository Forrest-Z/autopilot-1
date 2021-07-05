#pragma once

#ifndef MATH_CHECK_INDEXES
#define MATH_CHECK_INDEXES 0
#endif

#include "rotations.h"
#include "definitions.h"

#include <stdint.h>
#include <cmath>
#include <float.h>
#include <string.h>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif
#include <common/commom.h>

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    constexpr Vector3<T>()
        : x(0)
        , y(0)
        , z(0) {}

    // setting ctor
    constexpr Vector3<T>(const T x0, const T y0, const T z0)
        : x(x0)
        , y(y0)
        , z(z0) {}

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // non-uniform scaling
    Vector3<T> &operator *=(const Vector3<T> &v) {
        x *= v.x; y *= v.y; z *= v.z;
        return *this;
    }

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const;

    // dot product for Lua
    T dot(const Vector3<T> &v) const {
        return *this * v;
    }
    
    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // cross product for Lua
    Vector3<T> cross(const Vector3<T> &v) const {
        return *this % v;
    }

    // scale a vector3
    Vector3<T> scale(const float v) const {
        return *this * v;
    }
    
    // computes the angle between this vector and another vector
    float angle(const Vector3<T> &v2) const;

    // check if all elements are zero
  //  bool is_zero()   {
   //     return true;
       // return ((fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON) && (fabsf(z) < FLT_EPSILON));
 //   }


    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float length(void) const;

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }

    // distance from the tip of this vector to another vector squared (so as to avoid the sqrt calculation)
    float distance_squared(const Vector3<T> &v) const {
        const float dist_x = x-v.x;
        const float dist_y = y-v.y;
        const float dist_z = z-v.z;
        return (dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    }

    // distance from the tip of this vector to a line segment specified by two vectors
    float distance_to_segment(const Vector3<T> &seg_start, const Vector3<T> &seg_end) const;

    // extrapolate position given bearing and pitch (in degrees) and distance
    void offset_bearing(float bearing, float pitch, float distance);
    
    // given a position p1 and a velocity v1 produce a vector
    // perpendicular to v1 maximising distance from p1.  If p1 is the
    // zero vector the return from the function will always be the
    // zero vector - that should be checked for.
    static Vector3<T> perpendicular(const Vector3<T> &p1, const Vector3<T> &v1)
    {
        const T d = p1 * v1;
        if (fabsf(d) < FLT_EPSILON) {
            return p1;
        }
        const Vector3<T> parallel = (v1 * d) / v1.length_squared();
        Vector3<T> perpendicular = p1 - parallel;

        return perpendicular;
    }


    // Shortest distance between point(p) to a point contained in the line segment defined by w1,w2
    static float closest_distance_between_line_and_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    // Point in the line segment defined by w1,w2 which is closest to point(p)
    static Vector3<T> point_on_line_closest_to_other_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    // This implementation is borrowed from: http://geomalgorithms.com/a07-_distance.html
    // INPUT: 4 points corresponding to start and end of two line segments
    // OUTPUT: shortest distance between segments, and closest point on segment 2, from segment 1, gets passed on reference as "intersection" 
    static float segment_to_segment_dist(const Vector3<T>& seg1_start, const Vector3<T>& seg1_end, const Vector3<T>& seg2_start, const Vector3<T>& seg2_end, Vector3<T>& intersection) WARN_IF_UNUSED;
};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;
