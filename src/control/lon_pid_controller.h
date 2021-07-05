#pragma once

#include <fstream>
#include <memory>

#include "controller.h"
#include <pid/pid_INC_controller.h>


class LonPIDController : public Controller{
public:
  /**
   * @brief constructor
   */
  LonPIDController();

  /**
   * @brief destructor
   */
   ~LonPIDController(); 

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
  double radar_hold_thtottle_cmd_;

  // Controller
  PIDIncController speed_pid_controller_;
};
