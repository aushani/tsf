#ifndef __PERLS_MATH_GSL_UTIL_ARRAY_H__
#define __PERLS_MATH_GSL_UTIL_ARRAY_H__

#include <gsl/gsl_blas.h>

/**
 * @defgroup PerlsMathGsluArray GSL Util Array
 * @brief GSL utility for array.
 * @ingroup PerlsMath
 * 
 * @{
 */

/**
 * prints a double array as a size1 by size2 gsl_matrix \n
 * e.g., gslu_array_printf (data, 2, 3, "A"); prints data as 2x3 matrix
 */
void
gslu_array_printf (const double data[], size_t size1, size_t size2, const char *name);

/**
 * custom version of gslu_array_printf
 * @see gslu_array_printf
 *
 * @param fmt if it is NULL, then it defaults to "%f"
 * @param trans one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
gslu_array_printfc (const double data[], size_t size1, size_t size2, const char *name, 
                    const char *fmt, CBLAS_TRANSPOSE_t trans);

/**
 * @}
 */

#endif // __PERLS_MATH_GSL_UTIL_H__
