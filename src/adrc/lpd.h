#pragma once 
#include <stdint.h>
#include "lpd_conf.h"


class LPD{
public:
    LPD() = default;
    virtual ~LPD()=default;

    void init(const LpdConf &lpd_conf);

    void set_lpd(const LpdConf &lpd_conf);

    void reset();

    double update(double e,double de);

private:
    double wc_;
    double yita_;
    uint8_t model_order_;
};