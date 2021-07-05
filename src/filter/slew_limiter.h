#pragma once

/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#include <stdint.h>
#include "low_pass_filter.h"

class SlewLimiter {
public:
    SlewLimiter(const double &slew_rate_max, const double &slew_rate_tau);

    /*
      apply filter to sample, returning multiplier between 0 and 1 to keep
      output within slew rate
    */
    double modifier(double sample, double dt);

private:
    const double &slew_rate_max;
    const double &slew_rate_tau;
    LowPassFilterFloat slew_filter;
    double slew_amplitude;
    double last_sample;
};
