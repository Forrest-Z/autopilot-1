#include "lpd.h"
#include <iostream>
#include <stdio.h>
void LPD::init(const LpdConf &lpd_conf)
{
    set_lpd(lpd_conf);
}

void LPD::set_lpd(const LpdConf &lpd_conf)
{
    model_order_ = lpd_conf.model_order_;
    wc_ = lpd_conf.wc_;
    yita_ = lpd_conf.yita_;
}

void LPD::reset()
{

}

double LPD::update(double e,double de)
{
    switch (model_order_)
    {
    case 1:
      return wc_ *e;
        break;
    case 2:
        return wc_*wc_*e - 2*wc_*yita_*de;
        break;
    default:
    std::cout << "lpd:unkonw model order" << std::endl;
        return 0;
        break;
    }
} 