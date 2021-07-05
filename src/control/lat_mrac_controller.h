#pragma once

#include <fstream>
#include <memory>

#include "controller.h"
#include <mrac/mrac_adrc_controller.h>
#include <adrc/eso.h>

class LatMracController : public Controller{
public:
  /**
   * @brief constructor
   */
  LatMracController();

  /**
   * @brief destructor
   */
   ~LatMracController(); 

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
  double heading_rate_max_;

  // Controller
  ESO                steering_angle_leso_controller_;
  MracAdrcController steering_angle_mrac_controller_;

  // ohters
  uint64_t lat_control_last_ms_ =0;
  double desired_heading_=0;
};
