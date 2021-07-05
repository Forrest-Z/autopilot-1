#include "proj.h"
#include <math/math_utils.h>

void Proj::Init(double tolerance,double limit)
{
    tolerance_ = tolerance;
    limit_     = limit;
}

Eigen::VectorXd  Proj::Update(Eigen::VectorXd parameter,Eigen::VectorXd paramter_derivatives)
{
    Eigen::VectorXd  gradient = 2*(1+tolerance_)/(tolerance_ * limit_)*parameter;

    double convex_function = ((1+ tolerance_)*math::Sqr(parameter.norm()) - math::Sqr(limit_))
                            /(tolerance_*math::Sqr(limit_));

    if(convex_function >0 && paramter_derivatives.transpose()*gradient>0){
        return paramter_derivatives-(gradient*gradient.transpose()*paramter_derivatives*convex_function)
               /(gradient.transpose() * gradient);
    }else{
        return paramter_derivatives;
    }
}