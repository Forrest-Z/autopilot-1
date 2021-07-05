#pragma once
#include <stdint.h>

class ESOConf{
public:
    bool    angle_;
    double  wo_;
    double  b0_;
    double  delta_;
    uint8_t model_order_;
};