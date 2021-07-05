
/**
 * @file
 * @brief Defines the Controller base class.
 */
#pragma once

#include <memory>
#include <string>

#include <vehicle_state/vehicle_state.h>
#include "control_conf.h"
#include "control_cmd.h"
#include <string>

/**
 * @class Controller
 *
 * @brief base class for all controllers.
 */
class Controller {
 public:
  /**
   * @brief constructor
   */
  Controller() = default;

  /**
   * @brief destructor
   */
  virtual ~Controller(){delete control_conf_;control_conf_ = nullptr;}

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
   bool ComputeControlCommand(double target,ControlCommand *cmd,double ts);

   int BoundOutput(const double output_unbounded, const double previous_output,
                   double *output,double ts);

   virtual void Reset() = 0;
   virtual std::string Name() const = 0;
   virtual void Stop() = 0;

protected:
  virtual void Enter(void) = 0;
  virtual void Run(double target,ControlCommand *cmd,double ts) = 0;

   
protected:
  std::string name_;
  uint64_t last_update_time_ = 0;
  const ControlConf *control_conf_ = nullptr;
  std::shared_ptr<VehicleState> injector_;

  //common variables
  double bound_command_ = 0.0;
  double bound_command_rate_ = 0.0;
  double bound_error_ = 0;
  double control_previous_ = 0.0;

};
