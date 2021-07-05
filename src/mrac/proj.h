#pragma once

#include <Eigen/Core>

class Proj{
public:
    Proj()=default;
    virtual ~Proj() = default;

    void Init(double tolerance,double limit);
    
    // return paramter_derivatives by projective revise
    Eigen::VectorXd Update(Eigen::VectorXd paramter,Eigen::VectorXd paramter_derivatives);

private:
    double tolerance_{0.5};
    double limit_{10};
};