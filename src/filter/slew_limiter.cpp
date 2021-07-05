#include "slew_limiter.h"
#include <math/math_utils.h>

SlewLimiter::SlewLimiter(const double &_slew_rate_max, const double &_slew_rate_tau) :
    slew_rate_max(_slew_rate_max),
    slew_rate_tau(_slew_rate_tau)
{
    slew_filter.set_cutoff_frequency(10.0);
    slew_filter.reset(0.0);
}

/*
  apply filter to sample, returning multiplier between 0 and 1 to keep
  output within slew rate
 */
double SlewLimiter::modifier(double sample, double dt)
{
    if (slew_rate_max <= 0) {
        return 1.0;
    }

    // Calculate the slew rate amplitude produced by the unmodified sample
    // calculate a low pass filtered slew rate
    double Pterm_slew_rate = slew_filter.apply((fabsf(sample - last_sample)/ dt), dt);

    // rectify and apply a decaying envelope filter. The 10 in the
    // constrain limits the modifier to be between 0.1 and 1.0, so we
    // never drop PID gains below 10% of configured value
    double alpha = 1.0 - math::Clamp(dt/slew_rate_tau, 0.0, 1.0);
    slew_amplitude = math::Clamp(Pterm_slew_rate, alpha * slew_amplitude, 10 * slew_rate_max);

    // Calculate the gain adjustment
    double mod = slew_rate_max / fmaxf(slew_amplitude, slew_rate_max);
    last_sample = mod * sample;

    return mod;
}
