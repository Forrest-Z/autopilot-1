#include <math/common_math.h>

#include "npd_conf.h"
#include "npd.h"

void NPD::Init(const NpdConf &conf)
{
    SetNpd(conf);
}

void NPD::SetNpd(const NpdConf &conf)
{
    r_ = conf.r_;
    h_ = conf.h_;
    c_ = conf.c_; 
}

void NPD::Reset()
{

}

double NPD::Update(double e,double de)
{
    return -math::fhan(e,-c_*de,r_,h_);
}

