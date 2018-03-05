#ifndef __PERLS_MATH_GSL_UTIL_EIGEN_H__
#define __PERLS_MATH_GSL_UTIL_EIGEN_H__

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_eigen.h>

#include "gsl_util_vector.h"
#include "gsl_util_matrix.h"

/**
 * @defgroup PerlsMathGsluEigen GSL Util Eigen
 * @brief GSL utility for eigenvalue decomposition.
 * @ingroup PerlsMath
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Eigen value decomposition structure */
typedef struct _gslu_eigen gslu_eigen;
struct _gslu_eigen {
    gsl_matrix *V;
    gsl_vector *D;
};

/** free Eigen value decomposition structure */
static inline void
gslu_eigen_free (gslu_eigen *eigen)
{
    if (eigen) {
        gslu_matrix_free (eigen->V);
        gslu_vector_free (eigen->D);
        free (eigen);
    }
}

/**
 * compute Eigen value decomposition, returns NULL on error
 */
gslu_eigen *
gslu_eigen_decomp_alloc (const gsl_matrix *A);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_EIGEN_H__
