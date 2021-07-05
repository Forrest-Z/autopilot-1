#pragma once

#include <stdint.h>

class PIDIncConf{
    public:
        void kp(double kp){kp_ = kp;}
        void ki(double ki){ki_= ki;}
        void output_saturation_level(double output_saturation_level){output_saturation_level_ = output_saturation_level;}
        void max_acceleration(double max_acceleration){max_acceleration_ = max_acceleration;}
    
        double kp() const {return kp_;}
        double ki() const {return ki_;}
        double output_saturation_level() const{return output_saturation_level_;}
        double max_acceleration() const {return max_acceleration_;}
    
    private:
        double kp_;
        double ki_;
        double output_saturation_level_;
        double max_acceleration_;
};