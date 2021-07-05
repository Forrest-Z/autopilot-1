#include "lon_controller.h"

#include <common/commom.h>

// include all lateral controller
#include "lon_pid_controller.h"
#include "lon_adrc_controller.h"
#include "lon_mrac_controller.h"
#include <util/easylogging++.h>

bool LonController::Init(std::shared_ptr<VehicleState> injector,
                           const ControlConf *control_conf)
{
    const std::string control_type = control_conf->lon_controller_type_;
    ControlConf::ControllerType control_type_id  = control_conf->controller_type.find(control_type)->second;
    std::cout << "lon controller type="<< control_type<< "  controller type id= "<< control_type_id<< std::endl;

    switch (control_type_id)
    {
    case ControlConf::ControllerType::LON_CONTROLLER_PID:
        controller_ptr_ = std::unique_ptr<LonPIDController>(new LonPIDController());
        break;
    case ControlConf::ControllerType::LON_CONTROLLER_ADRC:
        controller_ptr_ = std::unique_ptr<LonADRCController>(new LonADRCController());
        break;
    case ControlConf::ControllerType::LON_CONTROLLER_MRAC:
        controller_ptr_ = std::unique_ptr<LonMracController>(new LonMracController());
        break;
    default:
        std::cout << "Unknow longitudinal controller!"<<std::endl;
        LOG(ERROR) << "Unknow longitudinal controller!";
        return false;
        break;
    }

    // Initialize controller
    controller_ptr_->Init(injector,control_conf);
    return true;
}

void LonController::ComputeControlCommand(double target,ControlCommand *cmd,double ts)
{
    controller_ptr_->ComputeControlCommand(target,cmd,ts);
}


void LonController::Reset()
{
    controller_ptr_->Reset();
}

 void LonController::Stop()
 {
     controller_ptr_->Stop();
     // Close log file
 }

std::string LonController::Name() const { controller_ptr_->Name(); }

