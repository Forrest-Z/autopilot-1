#pragma once
#include <stdint.h>

class ControlCommand{
public:

    // Set functions
    void set_throttle(double throttle){throttle_ = throttle;}
    void set_gear(int8_t gear){gear_ = gear;}
    void set_steering_target(double steering_angle){steering_angle_ = steering_angle;}
    void set_steering_rate(double steering_rate){steering_rate_ = steering_rate;}
    void set_speed_target(double speed){speed_ =speed;}
    void set_acceleration_target(double acceleration){acceleration_ = acceleration;}
    void set_engine_on_ff(bool engine_on_off){engine_on_off_ = engine_on_off;}
    
    // Get functions
    double get_throttle()const{return throttle_;}
    int8_t get_gear()const{return gear_;}
    double get_steering_rate() const{return steering_rate_;}
    double get_steering_target() const{return steering_angle_;}
    double get_speed_target() const{return speed_;}
    double get_acceleration_target() const{return acceleration_;}
    double get_engine_on_off()const{return engine_on_off_;}

private:
    //target throttle in percentage [0, 100]
    double throttle_;

    //taarget gear in percent [-1,1]
    int8_t gear_;

    // target non-directional steering rate, in percentage of full scale per
    // second [-100, 100]
    double steering_rate_;

    // target steering angle, in percentage of full scale [-100, 100]
    double steering_angle_;

    // target speed, in m/s
    double speed_;

    // target acceleration in m`s^-2
    double acceleration_;

     // engine on/off, true: engine on
    bool engine_on_off_;
};