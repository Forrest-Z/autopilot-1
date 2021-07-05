#pragma once

#include <fstream>
#include <memory>

#include "controller.h"
#include <mrac/mrac_adrc_controller.h>
#include <adrc/eso.h>

class LonMracController : public Controller{
public:
  /**
   * @brief constructor
   */
  LonMracController();

  /**
   * @brief destructor
   */
   ~LonMracController(); 

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
  double speed_acceleration_max_;

  // Controller
  ESO                speed_leso_controller_;
  MracAdrcController speed_mrac_controller_;

   // Ohters
  uint64_t   lon_control_last_ms_{0};
  double     desired_speed_{0.0};
};
