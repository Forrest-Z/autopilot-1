#pragma once

#include "pid_inc_conf.h"

class PIDIncController{
public:
  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void Init( const PIDIncConf &conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void SetPID(const PIDIncConf &conf);

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
   double Control(const double u, const double y,const double dt);

   virtual ~PIDIncController() = default;
private:
    double kp_ = 0;
    double ki_ = 0;
    double max_acceleration_ = 0;
    double output_saturation_level_;
private:
    bool   initialized_ = false;
    double control_previous_ = 0;
    double error_;
    double error_previous_ = 0;
    double cmd_previous_ = 0;
};