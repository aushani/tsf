#ifndef __PERLS_MATH_GSL_UTIL_MATH_H__
#define __PERLS_MATH_GSL_UTIL_MATH_H__

#include <gsl/gsl_math.h>

/**
 * @defgroup PerlsMathGsluMath GSL Util Math
 * @brief GSL utility math.
 * @ingroup PerlsMath
 * 
 * @{
 */

/** wrap circular quantities between [-pi,pi]*/
double
gslu_math_minimized_angle (double angle);

/**
 * @}
 */

#endif // __PERLS_MATH_GSL_UTIL_MATH_H__
