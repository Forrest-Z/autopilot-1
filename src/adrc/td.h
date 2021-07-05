#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <cmath>

#include <iostream>
#include "td_conf.h"


class TD{
public:
   TD() = default;
   virtual ~TD() = default;

    void Init(const TdConf &td_conf);

    void SetTd(const TdConf &td_conf);
    void Reset();

    void Update(double u,double y,double ts);


    double z1()const{return z1_;}
    double z2()const{return z2_;}

    void   z1(double z1){z1_ = z1;}
    void   z2(double z2){z2_ = z2;}

private:

    double r_;
    double h_;
    bool   is_angle_;

    double z1_;
    double z2_;
    bool initialized_;
};