#include "lat_controller.h"

#include <common/commom.h>

// include all lateral controller
#include "lat_pid_controller.h"
#include "lat_adrc_controller.h"
#include "lat_mrac_controller.h"

#include <util/easylogging++.h>

bool LatController::Init(std::shared_ptr<VehicleState> injector,
                           const ControlConf *control_conf)
{
    const std::string control_type = control_conf->lat_controller_type_;
    ControlConf::ControllerType control_type_id         = control_conf->controller_type.find(control_type)->second;
    std::cout << "lat controller type="<< control_type<< "  controller type id= "<< control_type_id<< std::endl;

    switch (control_type_id)
    {
    case ControlConf::ControllerType::LAT_CONTROLLER_PID:
        controller_ptr_ = std::unique_ptr<LatPIDController>(new LatPIDController());
        break;
    case ControlConf::ControllerType::LAT_CONTROLLER_ADRC:
        controller_ptr_ = std::unique_ptr<LatADRCController>(new LatADRCController());
        break;
    case ControlConf::ControllerType::LAT_CONTROLLER_MRAC:
        controller_ptr_ = std::unique_ptr<LatMracController>(new LatMracController());
        break;
    default:
        std::cout << "Unknow lateral controller!"<<std::endl;
        LOG(ERROR) << "Unknow lateral controller!";
        return false;
        break;
    }

    // Initialize controller
    controller_ptr_->Init(injector,control_conf);
    return true;

}

void LatController::ComputeControlCommand(double target,ControlCommand *cmd,double ts)
{
    controller_ptr_->ComputeControlCommand(target,cmd,ts);
}


void LatController::Reset()
{
    controller_ptr_->Reset();
}

 void LatController::Stop()
 {
     controller_ptr_->Stop();
     // Close log file
 }

std::string LatController::Name() const { controller_ptr_->Name(); }

