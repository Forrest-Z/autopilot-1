#include "pid_IC_controller.h"

#include <cmath>
#include <iostream>
#include <math/math_utils.h>



double PIDICController::Control(const double error, const double dt) {
  if (dt <= 0) {
     std::cout <<"dt <= 0, will use the last output" << std::endl;
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }
  // integral clamping
  if (!integrator_enabled_) {
    integral_ = 0;
  } else {
    double u = error * kp_ + integral_ + error * dt * ki_ + diff * kd_;
    if (((error * u) > 0) &&
        ((u > output_saturation_high_) || (u < output_saturation_low_))) {
    } else {
      // Only update integral then
      integral_ += error * dt * ki_;
    }
  }

  previous_error_ = error;
  output = error * kp_ + integral_ + diff * kd_;

  if (output >= output_saturation_high_) {
    output_saturation_status_ = 1;
  } else if (output <= output_saturation_low_) {
    output_saturation_status_ = -1;
  } else {
    output_saturation_status_ = 0;
  }

  output = math::Clamp(error * kp_ + integral_ + diff * kd_,
                               output_saturation_high_,
                               output_saturation_low_);  // Ki already applied
  previous_output_ = output;
  return output;
}

int PIDICController::OutputSaturationStatus() {
  return output_saturation_status_;
}



