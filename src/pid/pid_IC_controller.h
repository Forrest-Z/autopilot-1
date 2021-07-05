/**
 * @file pid_controller.h
 * @brief Defines the PIDICController class.
 */

#pragma once

#include "pid_controller.h"



/**
 * @class PIDICController
 * @brief A proportional-integral-derivative controller for speed and steering
with integral-clamping-anti-windup
 */
class PIDICController : public PIDController {
 public:
  /**
   * @brief compute control value based on the error,
   with integral-clamping-anti-windup
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  virtual double Control(const double error, const double dt);

  virtual int OutputSaturationStatus();

 private:
};



