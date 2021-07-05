#pragma once

#include <cmath>
#include <stdint.h>
#include <float.h>
#include <type_traits>

namespace math{

#ifdef M_PI
# undef M_PI
#endif
#define M_PI      (3.141592653589793f)

#ifdef M_PI_2
# undef M_PI_2
#endif
#define M_PI_2    (M_PI / 2)

#define M_GOLDEN  1.6180339f

#define M_2PI         (M_PI * 2)

// MATH_CHECK_INDEXES modifies some objects (e.g. SoloGimbalEKF) to
// include more debug information.  It is also used by some functions
// to add extra code for debugging purposes. If you wish to activate
// this, do it here or as part of the top-level Makefile -
// e.g. Tools/Replay/Makefile
#ifndef MATH_CHECK_INDEXES
  #define MATH_CHECK_INDEXES 0
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// Centi-degrees to radians
#define DEGX100 5729.57795f

/* 
 * @brief: Check whether a float is zero
 */
template <typename T>
bool IsZero(const T fVal1) {
    
    return (fabsf(static_cast<float>(fVal1)) < FLT_EPSILON);
}

/* 
 * @brief: Check whether a float is greater than zero
 */
template <typename T>
inline bool IsPositive(const T fVal1) {
  
    return (static_cast<float>(fVal1) >= FLT_EPSILON);
}

/* 
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool IsNegative(const T fVal1) {
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}


template <typename T>
float safe_asin(const T v)
{
    const float f = static_cast<const float>(v);
    if (std::isnan(f)) {
        return 0.0f;
    }
    if (f >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (f <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(f);
}

template float safe_asin<int>(const int v);
template float safe_asin<short>(const short v);
template float safe_asin<float>(const float v);
template float safe_asin<double>(const double v);

template <typename T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (std::isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);


inline float sign(float val)
{
    if(val > 0.0f)
		return 1.0f;
	else if(val < 0.0f)
		return -1.0f;
    else
    {
        return 0;
    }
}

double fhan(double v1, double v2, double r0, double h0);
double fal(double e, double alpha, double delta);

/*
 * Constrain an euler angle to be within the range: 0 to 360 degrees. The
 * second parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
inline float wrap_360(const float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res < 0) {
        res += 360.0f;
    }
    return res;
}
inline double wrap_360(const double angle)
{
     double res = fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}
inline int wrap_360(const int angle)
{
    int res = angle % 360;
    if (res < 0) {
        res += 360;
    }
    return res;
}


inline float wrap_360_cd(const float angle)
{
    float res = fmodf(angle, 36000.0f);
    if (res < 0) {
        res += 36000.0f;
    }
    return res;
}

inline double wrap_360_cd(const double angle)
{
     double res = fmod(angle, 36000.0);
    if (res < 0) {
        res += 36000.0;
    }
    return res;
}

inline int wrap_360_cd(const int angle)
{
    int res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

inline long wrap_360_cd(const long angle)
{
     long res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

/*
 * Constrain an angle to be within the range: -180 to 180 degrees. The second
 * parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <typename T>
T wrap_180(const T angle)
{
     auto res = wrap_360(angle);
    if (res > T(180)) {
        res -= T(360);
    }
    return res;
}

/*
 * Wrap an angle in centi-degrees. See wrap_180().
 */
template <typename T>
T wrap_180_cd(const T angle)
{
    auto res = wrap_360_cd(angle);
    if (res > T(18000)) {
        res -= T(36000);
    }
    return res;
}



/*
 * wrap an angle in radians to 0..2PI
 */

template <typename T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}
template float wrap_2PI<int>(const int radian);
template float wrap_2PI<short>(const short radian);
template float wrap_2PI<float>(const float radian);
template float wrap_2PI<double>(const double radian);

/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <typename T>
float wrap_PI(const T radian)
{
    auto res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}
template float wrap_PI<int>(const int radian);
template float wrap_PI<short>(const short radian);
template float wrap_PI<float>(const float radian);
template float wrap_PI<double>(const double radian);

// degrees -> radians
inline  float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
inline  float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

};