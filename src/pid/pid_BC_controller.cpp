#include "pid_BC_controller.h"

#include <cmath>
#include <iostream>
#include <math/math_utils.h>

double PIDBCController::Control(const double error, const double dt) {
  if (dt <= 0) {
    std::cout << "dt <= 0, will use the last output"<<std::endl;
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) /dt;
  }

  // backward calculation
  if (!integrator_enabled_) {
    integral_ = 0;
  } else {
    double u = error * kp_ + integral_ + error * dt * ki_ + diff * kd_;
    double aw_term = math::Clamp(u, output_saturation_high_,
                                         output_saturation_low_) -
                     u;
    if (aw_term > 1e-6) {
      output_saturation_status_ = -1;
    } else if (aw_term < -1e-6) {
      output_saturation_status_ = 1;
    } else {
      output_saturation_status_ = 0;
    }
    integral_ += kaw_ * aw_term + error * dt;
  }

  previous_error_ = error;
  output = math::Clamp(error * kp_ + integral_ + diff * kd_,
                               output_saturation_high_,
                               output_saturation_low_);  // Ki already applied
  previous_output_ = output;
  return output;
}

int PIDBCController::OutputSaturationStatus() {
  return output_saturation_status_;
}



