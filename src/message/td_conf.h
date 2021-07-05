#pragma once

class TdConf{
public:
    void r(const double r){r_ = r;}
    void h(const double h){h_ = h;}
    void is_angle(const bool is_angle){is_angle_ = is_angle;}

    double r()const{return r_;}
    double h()const{return h_;}
    bool is_angle()const{return is_angle_;}
private:
    double r_;
    double h_; 
    bool   is_angle_;
};