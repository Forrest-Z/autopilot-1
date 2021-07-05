/// @file LESO.h
/// @brief Linear extend state observer
#pragma once

#include <stdint.h>
#include "eso_conf.h"

#define LINEAR_ESO_ENABLE  0

class ESO{
public:

    ESO();
    virtual ~ESO();
    
    void init(const ESOConf &eso_conf);

    void set_eso(const ESOConf &eso_conf);

    void reset();

    void update(double u,double y,double dy,double ts);
    
    double z1()const{return z1_;}
    double z2()const{return z2_;}
    double z3()const{return z3_;}
    double b0()const{return b0_;}

private:

// Parameters
    bool   angle_;
    double wo_;
    double b0_;
    double delta_;
    uint8_t model_order_;
private:
// middle variables and states
    double z1_ = 0;
    double z2_ = 0;
    double z3_  =0;
    bool   initialized_ = false;
};