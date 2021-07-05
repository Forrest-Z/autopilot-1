#pragma once

#include <stdint.h>

class PIDSqrtConf{
    public:
        void kp(double kp){kp_ = kp;}
        void ki(double ki){ki_= ki;}
        void output_satration_level(double output_saturation_level){output_saturation_level_ = output_saturation_level;}
        void second_limit(double second_limit){second_limit_ = second_limit;}
        void integrator_saturation_level(double integrator_saturation_level){integrator_saturation_level_ = integrator_saturation_level;}
    
        double kp() const {return kp_;}
        double ki() const {return ki_;}
        double output_saturation_level() const{return output_saturation_level_;}
        double second_limit() const {return second_limit_;}
        double integrator_saturation_level() const {return integrator_saturation_level_;}
    
    private:
        double kp_;
        double ki_;
        double output_saturation_level_;
        double integrator_saturation_level_;
        double second_limit_;
};