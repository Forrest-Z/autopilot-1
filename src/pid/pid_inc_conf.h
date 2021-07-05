#pragma once

#include <stdint.h>

class PIDIncConf{
public:
        double kp_;
        double ki_;
        double output_saturation_level_;
        double max_acceleration_;
};