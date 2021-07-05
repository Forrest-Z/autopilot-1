#pragma once
#include <stdint.h>

class LpdConf{
public:
    void wc(double wc){wc_  =wc;}
    void yita(double yita){yita_ = yita;}
    void model_order(uint8_t model_order){model_order_ = model_order;}

    double wc()const{return wc_;}
    double yita()const {return yita_;}
    uint8_t model_order()const{return model_order_;}
private:
    double wc_;
    double yita_;
    uint8_t model_order_;
};