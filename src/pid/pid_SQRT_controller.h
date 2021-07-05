#pragma once

#include "pid_sqrt_conf.h"

class PIDSqrtController{
public:
  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void Init( const PIDSqrtConf &conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void SetPID(const PIDSqrtConf &conf);

  /**
   * @brief reset variables for pid controller
   */
  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error = u-y error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
   double Control(const double error,const double dt);

   virtual ~PIDSqrtController() = default;
private:
    double kp_ = 0;
    double ki_ = 0;
    double second_limit_ = 0;
    double output_saturation_level_;
    double integrator_saturation_level_;
private:
    bool   initialized_ = false;
    double integral_ = 0.0;
};