#ifndef __PERLS_MATH_GSL_UTIL_STATISTICS_H__
#define __PERLS_MATH_GSL_UTIL_STATISTICS_H__

#include <gsl/gsl_vector.h>

/**
 * @defgroup PerlsMathGsluStatistics GSL Util Statistics
 * @brief GSL utility statistics.
 * @ingroup PerlsMath
 * 
 * @{
 */

/** find median, note input vector is sorted in place. */
double
gslu_stats_median_array (double data[], size_t stride, size_t n);

/** find median for vector */
double
gslu_stats_median_vector (const gsl_vector *v);

/** find the NxN sample covariance for a NxM matrix, where each column is a sample from
 * the N-dimensional random vector */
gsl_matrix *
gslu_sample_cov (const gsl_matrix *A);

/**
 * @}
 */

#endif // __PERLS_MATH_GSL_UTIL_STATISTICS_H__
