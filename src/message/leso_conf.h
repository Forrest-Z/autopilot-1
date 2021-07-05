#pragma once
#include <stdint.h>

class LesoConf{
public:
   void angle(bool angle){angle_ = angle;}
   void wo(double wo){wo_ = wo;}
   void b0(double b0){b0_ = b0;}
   void model_order(uint8_t model_order){model_order_ = model_order;}

   bool angle()const{return angle_;}
   double wo()const{return wo_;}
   double b0()const{return b0_;}
   double model_order()const{return model_order_;}
private:
    bool   angle_;
    double wo_;
    double b0_;
    uint8_t model_order_;
};