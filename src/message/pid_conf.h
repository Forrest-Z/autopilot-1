#pragma once

#include <stdint.h>

class PidConf{
public:

    // Set functions
    void integrator_enable(const bool integrator_enable) {
        integrator_enable_ = integrator_enable;
    }
    void integrator_saturation_level(const double integrator_saturation_level) {
        integrator_saturation_level_ = integrator_saturation_level;
    }
    void kp(const double kp){kp_ = kp;}
    void ki(const double ki){ki_ = ki;}
    void kd(const double kd){kd_ = kd;}
    void kaw(const double kaw){kaw_ = kaw;}
    void output_saturation_level(const double output_saturation_level){
        output_saturation_level_ = output_saturation_level;
    }

    // Get functions
    bool   integrator_enable() const {return integrator_enable_;}
    double integrator_saturation_level() const {return integrator_saturation_level_;}
    double kp() const {return kp_;}
    double ki() const {return ki_;}
    double kd() const {return kd_;}
    double kaw() const {return kaw_;}
    double output_saturation_level() const {return output_saturation_level_;}

private:
    bool   integrator_enable_ = false;
    double integrator_saturation_level_ = 0.85;
    double kp_ = 0.1;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double kaw_= 0.0;
    double output_saturation_level_ = 1.0;
};