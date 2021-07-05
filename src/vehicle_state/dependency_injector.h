#pragma once

#include "vehicle_state.h"

class DependencyInjector{
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  VehicleState* vehicle_state() {
    return &vehicle_state_;
  }

 private:
  VehicleState vehicle_state_;
};


