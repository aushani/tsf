#ifndef __PERLS_MATH_GSL_UTIL_LINALG_H__
#define __PERLS_MATH_GSL_UTIL_LINALG_H__

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_linalg.h>

#include "gsl_util_blas.h"
#include "gsl_util_matrix.h"
#include "gsl_util_permutation.h"
#include "gsl_util_vector.h"

/**
 * @defgroup PerlsMathGsluLinalg GSL Util Linear Algebra
 * @brief GSL utility for Linear Algebra.
 * @ingroup PerlsMath
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*============================LINEAR ALGEBRA===============================*/

/** LU decomposition structure */
typedef struct _gslu_linalg_LU gslu_linalg_LU;
struct _gslu_linalg_LU {
    gsl_matrix *LU;
    gsl_permutation *p;
    int signum;
};

/** free LU decomposition structure */
static inline void
gslu_linalg_LU_free (gslu_linalg_LU *lu)
{
    if (lu) {
        gslu_matrix_free (lu->LU);
        gslu_permutation_free (lu->p);
        free (lu);
    }
}

/**
 * compute LU decomposition, returns NULL on error
 */
gslu_linalg_LU *
gslu_linalg_LU_decomp_alloc (const gsl_matrix *A);


/** QR decomposition structure */
typedef struct _gslu_linalg_QR gslu_linalg_QR;
struct _gslu_linalg_QR {
    gsl_matrix *QR;
    gsl_vector *tau;
};

/** free QR decomposition structure */
static inline void
gslu_linalg_QR_free (gslu_linalg_QR *qr)
{
    if (qr) {
        gslu_matrix_free (qr->QR);
        gslu_vector_free (qr->tau);
        free (qr);
    }
}

/**
 * compute QR decomposition, returns NULL on error
 */
gslu_linalg_QR *
gslu_linalg_QR_decomp_alloc (const gsl_matrix *A);


/**
 * compute cholesky decomposition
 * @return GSL_SUCEESS. GSL_EDOM on error.
 */
int
gslu_linalg_cholesky_decomp_lower (gsl_matrix *A);

/** SVD structure */
typedef struct _gslu_linalg_SV gslu_linalg_SV;
struct _gslu_linalg_SV {
    gsl_matrix *U;
    gsl_vector *S;
    gsl_matrix *V;
};

/** free SVD structure */
static inline void
gslu_linalg_SV_free (gslu_linalg_SV *sv)
{
    if (sv) {
        gslu_matrix_free (sv->U);
        gslu_vector_free (sv->S);
        gslu_matrix_free (sv->V);
        free (sv);
    }
}

/**
 * compute "economy size" SV decomposition using gsl
 * returns NULL on error. For A, M x N matrix, 
 * if A is a thin matrix \f$ M \geq N \f$, use gsl_linalg_SV_decomp.
 * If A is a fat matrix, we use fast as in
 * http://ugrad.stat.ubc.ca/R/library/GeneTS/html/fast.svd.html
 *
 * @note If A is a fat matrix with \f$M<N\f$, only the first M columns
 * of V are computed as in Matlab's svd. No null space basis will be
 * returned.
 * @see gslu_linalg_SV_decomp_full_alloc
 */
gslu_linalg_SV *
gslu_linalg_SV_decomp_econ_alloc (const gsl_matrix *A);

/**
 * compute "full size" SV decomposition using gsl
 *
 * returns NULL on error.
 */
gslu_linalg_SV *
gslu_linalg_SV_decomp_full_alloc (const gsl_matrix *A);

/**
 * Reduced row echelon form. The original matrix A will be changed. (http://rosettacode.org)
 *
 * @param A input matrix
 * @param m2 do rref up to m2-th row. Do full rref if m2 is 0
 *
 * @note input matrix A will be changed after the function call
 * @see gslu_linalg_rref_alloc
 */
void
gslu_linalg_rref (gsl_matrix *A, const int m2);

/**
 * allocation version of gslu_linalg_rref
 * @see gslu_linalg_rref
 */
static inline gsl_matrix *
gslu_linalg_rref_alloc (const gsl_matrix *A, const int m2)
{
    gsl_matrix *B = gslu_matrix_clone (A);
    gslu_linalg_rref (B, m2);
    return B;
}

#ifdef GSL_VER_LESS_THAN_1_12
int 
gsl_linalg_cholesky_invert (gsl_matrix *cholesky);
#endif

/**
 * Compute the square of a matrix defined sqrtm() command in MATLAB. This can be used for
 * the unscented transform (instead of Cholesky Decomposition)
 * 
 * @see http://en.wikipedia.org/wiki/Square_root_of_a_matrix
 * @see http://www.mathworks.com/help/techdoc/ref/sqrtm.html
 */
int
gslu_linalg_sqrtm (const gsl_matrix *R, gsl_matrix *sqrtmR);

/**
 * allocation version of gslu_linalg_sqrtm
 * @see gslu_linalg_sqrtm
 */
static inline gsl_matrix *
gslu_linalg_sqrtm_alloc (const gsl_matrix *R)
{
    gsl_matrix *sqrtmR = gsl_matrix_alloc(R->size1, R->size2);
    gslu_linalg_sqrtm (R, sqrtmR);
    return sqrtmR;
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_LINALG_H__
