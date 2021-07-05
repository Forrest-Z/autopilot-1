#pragma once

#include <stdint.h>

class PidConf{
public:
    bool   integrator_enable_ = false;
    double integrator_saturation_level_ = 0.85;
    double kp_ = 0.1;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double kaw_= 0.0;
    double output_saturation_level_ = 1.0;
};