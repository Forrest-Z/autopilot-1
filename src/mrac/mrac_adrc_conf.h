#pragma once

#include <stdint.h>
#include <Eigen/Core>
using Matrix  = Eigen::MatrixXd;


class MracAdrcConf{
private:
    const static uint8_t MODEL_ORDER_MAX = 2;

public:
    uint8_t model_order;
    double wc;
    double wa;
    double gama;
    double Q[MODEL_ORDER_MAX];
    double e0;
    double lv;

    // For Proj class
    double param_tolerance;
    double param_limit;
};