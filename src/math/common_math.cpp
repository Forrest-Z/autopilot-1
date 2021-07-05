#include "common_math.h"

namespace math{

double fhan(double v1, double v2, double r0, double h0)
    {
        double d = h0 * h0 * r0;
        double a0 = h0 * v2;
        double y = v1 + a0;
        double a1 = safe_sqrt(d*(d + 8.0f*fabsf(y)));
        double a2 = a0 + sign(y)*(a1-d)*0.5f;
        double sy = (sign(y+d) - sign(y-d))*0.5f;
        double a = (a0 + y - a2)*sy + a2;
        double sa = (sign(a+d) - sign(a-d))*0.5f;

        return -r0*(a/d - sign(a))*sa - r0*sign(a);
    }


 double fal(double e, double alpha, double delta)
    {
        if(fabs(e) <= delta){
            return e / (pow(delta, 1.0f-alpha));
        }else{
            return pow(fabs(e), alpha) * sign(e);
        }
    }
}