//
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

/*
  Note that this filter can be used in 2 ways:

   1) providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   2) providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

#pragma once

#include "filter_class.h"
#include <Eigen/Core>

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, double cutoff_freq, double dt);
    T apply(const T &sample);

    void compute_alpha(double sample_freq, double cutoff_freq);
    
    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);

private:
    T _output;
    double alpha = 1.0f;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(double cutoff_freq);
    LowPassFilter(double sample_freq, double cutoff_freq);

    // change parameters
    void set_cutoff_frequency(double cutoff_freq);
    void set_cutoff_frequency(double sample_freq, double cutoff_freq);

    // return the cutoff frequency
    double get_cutoff_freq(void) const;
    T apply(T sample, double dt);
    T apply(T sample);
    const T &get() const;
    void reset(T value);
    void reset(void) { reset(T()); }
    
protected:
    double _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter<T>::LowPassFilter() : _cutoff_freq(0.0f) { 
  
}
// constructor
template <class T>
LowPassFilter<T>::LowPassFilter(float cutoff_freq) : _cutoff_freq(cutoff_freq) { 
  
}
*/

// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Eigen::Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Eigen::Vector3f> LowPassFilterVector3f;
