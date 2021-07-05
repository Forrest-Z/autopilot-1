#pragma once

#include <fstream>
#include <memory>

#include "controller.h"
#include <pid/pid_SQRT_controller.h>

class LatPIDController : public Controller{
public:
  /**
   * @brief constructor
   */
  LatPIDController();

  /**
   * @brief destructor
   */
   ~LatPIDController(); 

 /**
   * @brief initialize Lateral Controller
   * @return Status initialization status
   */
  void Enter(void) override;

  /**
   * @brief compute steering target based on current vehicle status
   *        and target trajectory
   * @param status vehicle status
   * @param cmd control command
   * @return Status computation status
   */
  void Run(double target,ControlCommand *cmd,double ts) override;

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  void Reset() override;

  /**
   * @brief stop Lateral controller
   */
  void Stop() override;

   /**
   * @brief Lateral controller name
   * @return string controller name in string
   */
  std::string Name() const override;

private:
  
 // Paramters
  double sight_track_distance_;

  // Controller
  PIDSqrtController steering_angle_pid_controller_;
  PIDSqrtController steering_rate_pid_controller_;
};
