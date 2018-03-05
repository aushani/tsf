#ifndef __PERLS_MATH_GSL_UTIL_PERMUTATION_H__
#define __PERLS_MATH_GSL_UTIL_PERMUTATION_H__

#include <gsl/gsl_permutation.h>

/**
 * @defgroup PerlsMathGsluPermutation GSL Util Permutation
 * @brief GSL utility for permutations.
 * @ingroup PerlsMath
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * custom "free" that checks for existence
 */
static inline void
gslu_permutation_free (gsl_permutation *p)
{
    if (p)
        gsl_permutation_free (p);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_PERMUTATION_H__
