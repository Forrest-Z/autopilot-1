#pragma once

#include <fstream>
#include <memory>

#include "controller.h"

class LatController{
public:
  /**
   * @brief constructor
   */
  LatController()=default;

  /**
   * @brief destructor
   */
  virtual ~LatController() = default;

  /**
   * @brief initialize Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
   bool Init(std::shared_ptr<VehicleState> injector,
                              const ControlConf *control_conf);

  /**
   * @brief compute control command based on current vehicle status
   *        and target trajectory
   * @param target control reference signal
   * @param cmd control command
   * @param ts  control period
   * @return Status computation status
   */
   void ComputeControlCommand(double target,ControlCommand *cmd,double ts);

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  void Reset();

  /**
   * @brief stop Lateral controller
   */
  void Stop();

   /**
   * @brief Lateral controller name
   * @return string controller name in string
   */
  std::string Name() const;

private:
   std::unique_ptr<Controller> controller_ptr_;
};
