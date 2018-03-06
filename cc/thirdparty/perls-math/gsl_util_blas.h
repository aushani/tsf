#ifndef __PERLS_MATH_GSL_UTIL_BLAS_H__
#define __PERLS_MATH_GSL_UTIL_BLAS_H__

#include <assert.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_complex_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

/**
 * @defgroup PerlsMathGsluBlas GSL Util BLAS
 * @brief GSL utility: blas wrapper functions.
 * @ingroup PerlsMath
 *
 * @{
 */

/*===============================BLAS======================================*/

/**
 *\f$ d = alpha * op(A)*x + beta*y \f$
 *
 * @param TransA CblasNoTrans or CblasTrans
 */
static inline gsl_vector *
gslu_blas_dgemv_alloc (CBLAS_TRANSPOSE_t TransA, double alpha,
                       const gsl_matrix *A, const gsl_vector *x, double beta, const gsl_vector *y)
{
    gsl_vector *d = gsl_vector_alloc (y->size);
    gsl_vector_memcpy (d, y);
    gsl_blas_dgemv (TransA, alpha, A, x, beta, d);
    return d;
}

/**
 * \f$ D = alpha * op(A)*op(B) + beta*C\f$
 */
static inline gsl_matrix *
gslu_blas_dgemm_alloc (CBLAS_TRANSPOSE_t TransA, CBLAS_TRANSPOSE_t TransB, double alpha,
                       const gsl_matrix *A, const gsl_matrix *B, double beta, const gsl_matrix *C)
{
    gsl_matrix *D = gsl_matrix_alloc (C->size1, C->size2);
    gsl_matrix_memcpy (D, C);
    gsl_blas_dgemm (TransA, TransB, alpha, A, B, beta, D);
    return D;
}

/*===================SIMPLE MATRIX VECTOR OPS=================================*/

/**
 * \f$ b = A*x \f$
 */
static inline int
gslu_blas_mv (gsl_vector *b, const gsl_matrix *A, const gsl_vector *x)
{
    assert (b->size==A->size1 && A->size2==x->size);
    return gsl_blas_dgemv (CblasNoTrans, 1.0, A, x, 0.0, b);
}

/**
 * allocation version of gslu_blas_mv. \f$ b = A*x \f$
 * @see gslu_blas_mv
 */
static inline gsl_vector *
gslu_blas_mv_alloc (const gsl_matrix *A, const gsl_vector *x)
{
    assert (A->size2 == x->size);
    gsl_vector *b = gsl_vector_alloc (A->size1);
    gslu_blas_mv (b, A, x);
    return b;
}

/**
 * \f$ b^T = x^T*A \f$  i.e. \f$b = A^T*x \f$
 */
static inline int
gslu_blas_vTm (gsl_vector *b, const gsl_vector *x, const gsl_matrix *A)
{
    assert (b->size==A->size2 && A->size1==x->size);
    return gsl_blas_dgemv (CblasTrans, 1.0, A, x, 0.0, b);
}

/**
 * allocation version of gslu_blas_vTm \f$b = A^T*x \f$
 * @see gslu_blas_vTm
 */
static inline gsl_vector *
gslu_blas_vTm_alloc (const gsl_vector *x, const gsl_matrix *A)
{
    assert (A->size1==x->size);
    gsl_vector *b = gsl_vector_alloc (A->size2);
    gslu_blas_vTm (b, x, A);
    return b;
}

/**
 * \f$s = x^T*A*y\f$
 */
static inline double
gslu_blas_vTmv (const gsl_vector *x, const gsl_matrix *A, const gsl_vector *y)
{
    assert (x->size==A->size1 && A->size2==y->size);
    double s = 0.0;
    for (size_t i=0; i<A->size1; i++) {
        double xi = gsl_vector_get (x, i);
        for (size_t j=0; j<A->size2; j++)
            s += xi * gsl_matrix_get (A, i, j) * gsl_vector_get (y, j);
    }
    return s;
}

/**
 * \f$C = A*B\f$
 */
static inline int
gslu_blas_mm (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (C->size1==A->size1 && C->size2==B->size2 && A->size2==B->size1);
    return gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, C);
}

/**
 * allocation version of gslu_blas_mm. \f$C = A*B\f$
 * @see gslu_blas_mm
 */
static inline gsl_matrix *
gslu_blas_mm_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    assert (A->size2 == B->size1);
    gsl_matrix *C = gsl_matrix_alloc (A->size1, B->size2);
    gslu_blas_mm (C, A, B);
    return C;
}

/**
 * \f$C = A*B^T\f$
 */
static inline int
gslu_blas_mmT (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (C->size1 == A->size1 && C->size2 == B->size1 && A->size2 == B->size2);
    return gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, A, B, 0.0, C);
}

/**
 * allocation version of gslu_blas_mmT. \f$C = A*B^T\f$
 * @see gslu_blas_mmT
 */
static inline gsl_matrix *
gslu_blas_mmT_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    assert (A->size2 == B->size2);
    gsl_matrix *C = gsl_matrix_alloc (A->size1, B->size1);
    gslu_blas_mmT (C, A, B);
    return C;
}

/**
 * \f$C = A^T*B\f$
 */
static inline int
gslu_blas_mTm (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (C->size1 == A->size2 && C->size2 == B->size2 && A->size1 == B->size1);
    return gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1.0, A, B, 0.0, C);
}

/**
 * allocation version of gslu_blas_mTm. \f$C = A^T*B\f$
 * @see gslu_blas_mTm
 */
static inline gsl_matrix *
gslu_blas_mTm_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    assert (A->size1 == B->size1);
    gsl_matrix *C = gsl_matrix_alloc (A->size2, B->size2);
    gslu_blas_mTm (C, A, B);
    return C;
}

/**
 * \f$C = A^T*B^T\f$
 */
static inline int
gslu_blas_mTmT (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (C->size1 == A->size2 && C->size2 == B->size1 && A->size1 == B->size2);
    return gsl_blas_dgemm (CblasTrans, CblasTrans, 1.0, A, B, 0.0, C);
}

/**
 * allocation version of gslu_blas_mTmT. \f$C = A^T*B^T\f$
 * @see gslu_blas_mTmT
 */
static inline gsl_matrix *
gslu_blas_mTmT_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    assert (A->size1 == B->size2);
    gsl_matrix *C = gsl_matrix_alloc (A->size2, B->size1);
    gslu_blas_mTmT (C, A, B);
    return C;
}

/**
 * \f$D = A*B*C\f$
 * @note It requires gsl_matrix work space of size A->size1 x B->size2, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mmm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mmm. \f$D = A*B*C\f$
 * @gslu_blas_mmm
 */
static inline gsl_matrix *
gslu_blas_mmm_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size2 == B->size1 && B->size2 == C->size1);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size2);
    gslu_blas_mmm (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B*C^T\f$
 * @note It requires gsl_matrix work space of size A->size1 x B->size2, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mmmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mmmT. \f$D = A*B*C^T\f$
 * @see gslu_blas_mmmT
 */
static inline gsl_matrix *
gslu_blas_mmmT_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size2 == B->size1 && B->size2 == C->size2);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size1);
    gslu_blas_mmmT (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B^T*C\f$
 * @note It requires gsl_matrix work space of size A->size1 x B->size1, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mmTm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mmTm. \f$D = A*B^T*C\f$
 * @see gslu_blas_mmTm
 */
static inline gsl_matrix *
gslu_blas_mmTm_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size2 == B->size2 && B->size1 == C->size1);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size2);
    gslu_blas_mmTm (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^T*B*C\f$
 * @note It requires gsl_matrix work space of size A->size2 x B->size2, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mTmm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mTmm. \f$D = A^T*B*C\f$
 * @see gslu_blas_mTmm
 */
static inline gsl_matrix *
gslu_blas_mTmm_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size1 == B->size1 && B->size2 == C->size1);
    gsl_matrix *D = gsl_matrix_alloc (A->size2, C->size2);
    gslu_blas_mTmm (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B^T*C^T\f$
 * @note It requires gsl_matrix work space of size A->size1 x B->size1, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mmTmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mmTmT. \f$D = A*B^T*C^T\f$
 * @see gslu_blas_mmTmT
 */
static inline gsl_matrix *
gslu_blas_mmTmT_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size2 == B->size2 && B->size1 == C->size2);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size1);
    gslu_blas_mmTmT (D, A, B, C, _work);
    return D;
}


/**
 * \f$D = A^T*B^T*C\f$
 * @note It requires gsl_matrix work space of size A->size2 x B->size1, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mTmTm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mTmTm. \f$D = A^T*B^T*C\f$
 * @see gslu_blas_mTmTm
 */
static inline gsl_matrix *
gslu_blas_mTmTm_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size1 == B->size2 && B->size1 == C->size1);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size1);
    gslu_blas_mTmTm (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^T*B*C^T\f$
 * @note It requires gsl_matrix work space of size A->size2 x B->size2, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mTmmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mTmmT. \f$D = A^T*B*C^T\f$
 * @see gslu_blas_mTmmT
 */
static inline gsl_matrix *
gslu_blas_mTmmT_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size1 == B->size1 && B->size2 == C->size2);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size1);
    gslu_blas_mTmmT (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^T*B^T*C^T\f$
 * @note Itrequires gsl_matrix work space of size A->size2 x B->size1, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mTmTmT (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work);

/**
 * allocation version of gslu_blas_mTmTmT. \f$D = A^T*B^T*C^T\f$
 * @see gslu_blas_mTmTmT
 */
static inline gsl_matrix *
gslu_blas_mTmTmT_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C, gsl_matrix *_work)
{
    assert (A->size1 == B->size2 && B->size1 == C->size2);
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size1);
    gslu_blas_mTmTmT (D, A, B, C, _work);
    return D;
}

/** computes outer product \f$C = ab^T\f$ where a and b are vector (can be different dimms)
 * @param a Nx1 vector
 * @param b Mx1 vector
 * @param C NxM matrix (returned)
 */
int
gslu_blas_vvT (gsl_matrix *C, const gsl_vector *a, const gsl_vector *b);

/**
 * allocation version of gslu_blas_vvT. \f$C = ab^T\f$
 * @see gslu_blas_vvT.
 */
static inline gsl_matrix *
gslu_blas_vvT_alloc (const gsl_vector *a, const gsl_vector *b)
{
    //assert (a->size == b->size);

    gsl_matrix *C = gsl_matrix_alloc (a->size, b->size);
    gslu_blas_vvT (C, a, b);
    return C;
}



/*===============================COMPLEX======================================*/

/**
 * \f$d = alpha * op(A)*x + beta*y\f$
 */
static inline gsl_vector_complex *
gslu_blas_zgemv_alloc (CBLAS_TRANSPOSE_t TransA, gsl_complex alpha,
                       const gsl_matrix_complex *A, const gsl_vector_complex *x, gsl_complex beta, const gsl_vector_complex *y)
{
    gsl_vector_complex *d = gsl_vector_complex_alloc (y->size);
    gsl_vector_complex_memcpy (d, y);
    gsl_blas_zgemv (TransA, alpha, A, x, beta, d);
    return d;
}

/**
 * \f$D = alpha * op(A)*op(B) + beta*C\f$
 */
static inline gsl_matrix_complex *
gslu_blas_zgemm_alloc (CBLAS_TRANSPOSE_t TransA, CBLAS_TRANSPOSE_t TransB, gsl_complex alpha,
                       const gsl_matrix_complex *A, const gsl_matrix_complex *B, gsl_complex beta, const gsl_matrix_complex *C)
{
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (C->size1, C->size2);
    gsl_matrix_complex_memcpy (D, C);
    gsl_blas_zgemm (TransA, TransB, alpha, A, B, beta, D);
    return D;
}

/*===================SIMPLE MATRIX VECTOR OPS=================================*/

/**
 * \f$b = A*x\f$ for complex matrix A and complex vector x
 */
static inline int
gslu_blas_mv_complex (gsl_vector_complex *b, const gsl_matrix_complex *A, const gsl_vector_complex *x)
{
    assert (b->size==A->size1 && A->size2==x->size);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);

    return gsl_blas_zgemv (CblasNoTrans, alpha, A, x, beta, b);
}

/**
 * allocation version of gslu_blas_mv_complex.
 * @see gslu_blas_mv_complex.
 */
static inline gsl_vector_complex *
gslu_blas_mv_complex_alloc (const gsl_matrix_complex *A, const gsl_vector_complex *x)
{
    assert (A->size2 == x->size);
    gsl_vector_complex *b = gsl_vector_complex_alloc (A->size1);
    gslu_blas_mv_complex (b, A, x);
    return b;
}

/**
 * \f$b^H = x^H*A\f$ i.e. \f$b = A^H*x\f$ for complex matrix A and complex vector x.
 */
static inline int
gslu_blas_vHm_complex (gsl_vector_complex *b, const gsl_matrix_complex *A, const gsl_vector_complex *x)
{
    assert (b->size==A->size2 && A->size1==x->size);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);
    return gsl_blas_zgemv (CblasConjTrans, alpha, A, x, beta, b);
}

/**
 * allocation version of gslu_blas_vHm_complex. \f$b^H = x^H*A\f$ i.e. \f$b = A^H*x\f$
 * @see gslu_blas_vHm_complex
 */
static inline gsl_vector_complex *
gslu_blas_vHm_complex_alloc (const gsl_matrix_complex *A, const gsl_vector_complex *x)
{
    assert (A->size1==x->size);
    gsl_vector_complex *b = gsl_vector_complex_alloc (A->size2);
    gslu_blas_vHm_complex (b, A, x);
    return b;
}

/**
 * \f$ C = A*B \f$ for complex matrix A and B.
 */
static inline int
gslu_blas_mm_complex (gsl_matrix_complex *C, const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (C->size1==A->size1 && C->size2==B->size2 && A->size2==B->size1);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);
    return gsl_blas_zgemm (CblasNoTrans, CblasNoTrans, alpha, A, B, beta, C);
}

/**
 * allocation version of gslu_blas_mm_complex. \f$ C = A*B \f$
 * @see gslu_blas_mm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (A->size2 == B->size1);
    gsl_matrix_complex *C = gsl_matrix_complex_alloc (A->size1, B->size2);
    gslu_blas_mm_complex (C, A, B);
    return C;
}

/**
 * \f$C = A*B^H \f$ (transpose and conjugate) for complex matrix A and B.
 */
static inline int
gslu_blas_mmH_complex (gsl_matrix_complex *C, const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (C->size1 == A->size1 && C->size2 == B->size1 && A->size2 == B->size2);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);
    return gsl_blas_zgemm (CblasNoTrans, CblasConjTrans, alpha, A, B, beta, C);
}

/**
 * -allocation version of gslu_blas_mmH_complex. \f$C = A*B^H \f$
 * @see gslu_blas_mmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (A->size2 == B->size2);
    gsl_matrix_complex *C = gsl_matrix_complex_alloc (A->size1, B->size1);
    gslu_blas_mmH_complex (C, A, B);
    return C;
}

/**
 * \f$C = A^H*B\f$ for complex matrix A and B.
 */
static inline int
gslu_blas_mHm_complex (gsl_matrix_complex *C, const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (C->size1 == A->size2 && C->size2 == B->size2 && A->size1 == B->size1);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);
    return gsl_blas_zgemm (CblasConjTrans, CblasNoTrans, alpha, A, B, beta, C);
}

/**
 * allocation version of gslu_blas_mHm_complex. \f$C = A^H*B\f$.
 * @see gslu_blas_mHm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (A->size1 == B->size1);
    gsl_matrix_complex *C = gsl_matrix_complex_alloc (A->size2, B->size2);
    gslu_blas_mHm_complex (C, A, B);
    return C;
}

/**
 * \f$C = A^H*B^H\f$ for complex matrix A and B.
 */
static inline int
gslu_blas_mHmH_complex (gsl_matrix_complex *C, const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (C->size1 == A->size2 && C->size2 == B->size1 && A->size1 == B->size2);
    gsl_complex alpha = gsl_complex_rect (1.0, 0.0);
    gsl_complex beta = gsl_complex_rect (0.0, 0.0);
    return gsl_blas_zgemm (CblasConjTrans, CblasConjTrans, alpha, A, B, beta, C);
}

/**
 * allocation version of gslu_blas_mHmH_complex. \f$C = A^H*B^H\f$
 * @see gslu_blas_mHmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B)
{
    assert (A->size1 == B->size2);
    gsl_matrix_complex *C = gsl_matrix_complex_alloc (A->size2, B->size1);
    gslu_blas_mHmH_complex (C, A, B);
    return C;
}

/**
 * \f$D = A*B*C\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix_complex work space of size A->size1 x B->size2, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mmm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                  const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mmm_complex. \f$D = A*B*C\f$
 * @see gslu_blas_mmm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mmm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                        const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size2 == B->size1 && B->size2 == C->size1);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size2);
    gslu_blas_mmm_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B*C^H\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix work space of size A->size1 x B->size2, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mmmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mmmH_complex. \f$D = A*B*C^H\f$.
 * @see gslu_blas_mmmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mmmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                         const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size2 == B->size1 && B->size2 == C->size2);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size1);
    gslu_blas_mmmH_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B^H*C\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix work space of size A->size1 x B->size1, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mmHm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mmHm_complex. \f$D = A*B^H*C\f$.
 * @see gslu_blas_mmHm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mmHm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                         const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size2 == B->size2 && B->size1 == C->size1);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size2);
    gslu_blas_mmHm_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^H*B*C\f$ for complex matrix A,B and C.
 * @note requires gsl_matrix_complex work space of size A->size2 x B->size2, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mHmm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                   const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mHmm_complex. \f$D = A^H*B*C\f$.
 * @see gslu_blas_mHmm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHmm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                         const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size1 == B->size1 && B->size2 == C->size1);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size2, C->size2);
    gslu_blas_mHmm_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A*B^H*C^H\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix work space of size A->size1 x B->size1, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mmHmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mmHmH_complex. \f$D = A*B^H*C^H\f$.
 * @see gslu_blas_mmHmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mmHmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                          const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size2 == B->size2 && B->size1 == C->size2);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size1);
    gslu_blas_mmHmH_complex (D, A, B, C, _work);
    return D;
}


/**
 * \f$D = A^H*B^H*C\f$ for complex matrix A,B and C.
 * @note equires gsl_matrix work space of size A->size2 x B->size1, (pass NULL to dynamically alloc and free).
 */
int
gslu_blas_mHmHm_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mHmHm_complex
 * @see gslu_blas_mHmHm_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHmHm_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                          const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size1 == B->size2 && B->size1 == C->size1);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size1);
    gslu_blas_mHmHm_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^H*B*C^H\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix work space of size A->size2 x B->size2, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mHmmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                    const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mHmmH_complex. \f$D = A^H*B*C^H\f$.
 * @see gslu_blas_mHmmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHmmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                          const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size1 == B->size1 && B->size2 == C->size2);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size1);
    gslu_blas_mHmmH_complex (D, A, B, C, _work);
    return D;
}

/**
 * \f$D = A^H*B^H*C^H\f$ for complex matrix A, B and C.
 * @note requires gsl_matrix work space of size A->size2 x B->size1, (pass NULL to dynamically alloc and free)
 */
int
gslu_blas_mHmHmH_complex (gsl_matrix_complex *D, const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                     const gsl_matrix_complex *C, gsl_matrix_complex *_work);

/**
 * allocation version of gslu_blas_mHmHmH_complex. \f$D = A^H*B^H*C^H\f$.
 * @see gslu_blas_mHmHmH_complex
 */
static inline gsl_matrix_complex *
gslu_blas_mHmHmH_complex_alloc (const gsl_matrix_complex *A, const gsl_matrix_complex *B,
                           const gsl_matrix_complex *C, gsl_matrix_complex *_work)
{
    assert (A->size1 == B->size2 && B->size1 == C->size2);
    gsl_matrix_complex *D = gsl_matrix_complex_alloc (A->size1, C->size1);
    gslu_blas_mHmHmH_complex (D, A, B, C, _work);
    return D;
}

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_BLAS_H__
