#ifndef __PERLS_MATH_GSL_UTIL_H__
#define __PERLS_MATH_GSL_UTIL_H__

#ifndef GSL_RANGE_CHECK_OFF
#define GSL_RANGE_CHECK_OFF
#define TURN_RANGE_CHECK_BACK_ON
#endif

#include "gsl_util_array.h"
#include "gsl_util_blas.h"
#include "gsl_util_boolean.h"
#include "gsl_util_eigen.h"
#include "gsl_util_index.h"
#include "gsl_util_linalg.h"
#include "gsl_util_math.h"
#include "gsl_util_matrix.h"
#include "gsl_util_permutation.h"
#include "gsl_util_rand.h"
#include "gsl_util_statistics.h"
#include "gsl_util_vector.h"

#ifdef TURN_RANGE_CHECK_BACK_ON
#undef GSL_RANGE_CHECK_OFF
#endif

#endif // __PERLS_MATH_GSL_UTIL_H__
