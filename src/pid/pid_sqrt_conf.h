#pragma once

#include <stdint.h>

class PIDSqrtConf{
public:
    double kp_;
    double ki_;
    double second_limit_;
    double output_saturation_level_;
    double integrator_saturation_level_;
};