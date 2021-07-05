#pragma once

#include <fstream>
#include <memory>

#include "controller.h"
#include <adrc/td.h>
#include <adrc/eso.h>
#include <adrc/lpd.h>


#include <pid/pid_SQRT_controller.h>


class LonADRCController : public Controller{
public:
  /**
   * @brief constructor
   */
  LonADRCController();

  /**
   * @brief destructor
   */
   ~LonADRCController(); 

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
  double     radar_hold_thtottle_cmd_;
  double     speed_acceleration_max_;

  // Controller
  ESO                 speed_leso_controller_;
  PIDSqrtController   speed_sqrt_controller_;

  // Ohters
  uint64_t   lon_control_last_ms_ =0;
  double     desired_speed_=0;
};
