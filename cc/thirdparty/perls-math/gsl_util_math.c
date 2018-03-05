#include <math.h>

#include "gsl_util_math.h"

double
gslu_math_minimized_angle (double angle)
{
    while (angle < -M_PI)
        angle += 2*M_PI;
    while (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}

