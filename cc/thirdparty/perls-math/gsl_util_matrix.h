#ifndef __PERLS_MATH_GSL_UTIL_MATRIX_H__
#define __PERLS_MATH_GSL_UTIL_MATRIX_H__

#include <math.h>
#include <assert.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util_index.h"

/**
 * @defgroup PerlsMathGsluMatrix GSL Util Matrix
 * @brief GSL utility for matrices.
 * @ingroup PerlsMath
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Macro for creating a gsl_matrix_view object and data on the stack.
 * Use gslu macros to create matrix elements on the stack with matrix
 * views.  var will have subfields .data and .matrix.  The advantage
 * is that elements created on the stack do not have to be manually
 * freed by the programmer, they automatically have limited scope
 */
#define GSLU_MATRIX_VIEW(var,i,j,...)                                   \
    struct {                                                            \
        double data[i*j];                                               \
        gsl_matrix matrix;                                              \
    } var = {__VA_ARGS__};                                              \
    {   /* _view_ has local scope */                                    \
        gsl_matrix_view _view_ = gsl_matrix_view_array (var.data, i, j); \
        var.matrix = _view_.matrix;                                     \
    }

/**
 * Macro for type conversion, e.g. the following command would
 * convert gsl_matrix_float *f, to gsl_matrix *d
 * GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, f, gsl_matrix, d);
 */
#define GSLU_MATRIX_TYPEA_TO_TYPEB(typeA, A, typeB, B) {                \
        const typeA *_AA_ = A;                                          \
        const typeB *_BB_ = B;                                          \
        assert (_AA_->size1 == _BB_->size1 && _AA_->size2 == _BB_->size2); \
        for (size_t _i_=0; _i_<_AA_->size1; _i_++) {                    \
            for (size_t _j_=0; _j_<_BB_->size2; _j_++) {                \
                typeB ## _set (B, _i_, _j_, typeA ## _get (A, _i_, _j_)); \
            }                                                           \
        };                                                              \
    }


/*==============================MATRIX====================================*/

/**
 * prints the contents of a matrix to stdout.  each element is formatted
 * using the printf-style format specifier fmt.
 *
 * @param fmt if it is NULL, then it defaults to "%f"
 * @param trans one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
gslu_matrix_printfc (const gsl_matrix *m, const char *name,
                     const char *fmt, CBLAS_TRANSPOSE_t trans);

/**
 * prints the contents of a matrix to stdout. e.g. gslu_matrix_printf (A, "A");
 */
static inline void
gslu_matrix_printf (const gsl_matrix *A, const char *name)
{
    gslu_matrix_printfc (A, name, NULL, CblasNoTrans);
}

/**
 * custom "free" that checks for existence
 */
static inline void
gslu_matrix_free (gsl_matrix *A)
{
    if (A)
        gsl_matrix_free (A);
}

/**
 * custom "free" that checks for existence for float matrix
 */
static inline void
gslu_matrix_float_free (gsl_matrix_float *A)
{
    if (A)
        gsl_matrix_float_free (A);
}

/**
 * clones a matrix
 *
 * @param A matrix to be cloned
 * @return clone of A
 */
static inline gsl_matrix *
gslu_matrix_clone (const gsl_matrix *A)
{
    gsl_matrix *B = gsl_matrix_alloc (A->size1, A->size2);
    gsl_matrix_memcpy (B, A);
    return B;
}

/**
 * select a subset of rows of matrix A according to index p
 *
 * @param A original matrix
 * @param p index
 * @param B submatrix of A to be returned.
 */
static inline void
gslu_matrix_selrow (gsl_matrix *B, const gsl_matrix *A, const gslu_index *isel)
{
    assert (B->size2 == A->size2 && B->size1 == isel->size);
    for (size_t i=0; i<B->size1; i++) {
        const size_t ii = gslu_index_get (isel, i);
        assert (ii < A->size1);
        gsl_vector_const_view r = gsl_matrix_const_row (A, ii);
        gsl_matrix_set_row (B, i, &r.vector);
    }
}

/**
 * allocation version of gslu_matrix_selrow
 * @see gslu_matrix_selrow
 */
static inline gsl_matrix *
gslu_matrix_selrow_alloc (const gsl_matrix *A, const gslu_index *isel)
{
    gsl_matrix *B = gsl_matrix_alloc (isel->size, A->size2);
    gslu_matrix_selrow (B, A, isel);
    return B;
}

/**
 * select a subset of cols of matrix A according to index p
 *
 * @param A original input matirx
 * @param p index
 * @param B submatrix of A to be returned.
 */
static inline void
gslu_matrix_selcol (gsl_matrix *B, const gsl_matrix *A, const gslu_index *isel)
{
    assert (B->size1 == A->size1 && B->size2 == isel->size);
    for (size_t j=0; j<B->size2; j++) {
        const size_t jj = gslu_index_get (isel, j);
        assert (jj < A->size2);
        gsl_vector_const_view c = gsl_matrix_const_column (A, jj);
        gsl_matrix_set_col (B, j, &c.vector);
    }
}

/**
 * allocation version of gslu_matrix_selcol
 * @see gslu_matrix_selcol
 */
static inline gsl_matrix *
gslu_matrix_selcol_alloc (const gsl_matrix *A, const gslu_index *isel)
{
    gsl_matrix *B = gsl_matrix_alloc (A->size1, isel->size);
    gslu_matrix_selcol (B, A, isel);
    return B;
}

/**
 * @return 1 if matrices A and B have the same size
 */
static inline int
gslu_matrix_is_same_size (const gsl_matrix *A, const gsl_matrix *B)
{
    return A->size1 == B->size1 && A->size2 == B->size2;
}

/**
 * @return 1 if matrix A is square in size
 */
static inline int
gslu_matrix_is_square (const gsl_matrix *A)
{
    return A->size1 == A->size2;
}

/**
 * @return 1 if all elements of A and B are the same to within epsilon
 */
static inline int
gslu_matrix_is_equal (const gsl_matrix *A, const gsl_matrix *B, const double epsilon)
{
    assert (A->size1 == B->size1 && A->size2 == B->size2);
    for (size_t i=0; i<A->size1; i++) {
        for (size_t j=0; j<A->size2; j++) {
            double delta = gsl_matrix_get (A, i, j) - gsl_matrix_get (B, i, j);
            if (fabs (delta) > epsilon)
                return 0;
        }
    }
    return 1;
}

/**
 * @return number of elements in matrix
 */
static inline size_t
gslu_matrix_numel (const gsl_matrix *A)
{
    return (A->size1 * A->size2);
}

/**
 * copies matrix B into a submatrix of matrix A starting at A(i,j)
 */
static inline int
gslu_matrix_set_submatrix (gsl_matrix *A, const size_t i, const size_t j, const gsl_matrix *B)
{
    assert ((B->size1+i) <= A->size1 && (B->size2+j) <= A->size2);
    gsl_matrix_view Asub = gsl_matrix_submatrix (A, i, j, B->size1, B->size2);
    return gsl_matrix_memcpy (&Asub.matrix, B);
}

/**
 * Makes a block-diagonal matrix as in
 ABDiag = [A 0
           0 B]
 */
void
gslu_matrix_mdiag (gsl_matrix *ABDiag, const gsl_matrix *A, const gsl_matrix *B);

static inline gsl_matrix *
gslu_matrix_mdiag_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    gsl_matrix *ABDiag = gsl_matrix_calloc (A->size1+B->size1, A->size2+B->size2);
    gslu_matrix_mdiag (ABDiag, A, B);
    return ABDiag;
}

/**
 * copies vector b into a column j of matrix A
 */
static inline int
gslu_matrix_set_column (gsl_matrix *A, const size_t j, const gsl_vector *b)
{
    assert (A->size1 == b->size);
    gsl_vector_view Acol = gsl_matrix_column (A, j);
    return gsl_vector_memcpy (&Acol.vector, b);
}

/**
 * copies vector b into a row i of matrix A
 */
static inline int
gslu_matrix_set_row (gsl_matrix *A, const size_t i, const gsl_vector *b)
{
    assert (A->size2 == b->size);
    gsl_vector_view Acol = gsl_matrix_row (A, i);
    return gsl_vector_memcpy (&Acol.vector, b);
}

/**
 * transpose of matrix
 */
static inline gsl_matrix *
gslu_matrix_transpose_alloc (const gsl_matrix *A)
{
    gsl_matrix *B = gsl_matrix_alloc (A->size2, A->size1);
    gsl_matrix_transpose_memcpy (B, A);
    return B;
}

/**
 * determinant of matrix
 */
double
gslu_matrix_det (const gsl_matrix *A);

/**
 * Get trace of matrix A
 * @return tr(A)
 */
static inline double
gslu_matrix_trace (const gsl_matrix *A)
{
    gsl_vector_const_view d = gsl_matrix_const_diagonal (A);
    double trace = 0.0;
    for (size_t i=0; i<d.vector.size; i++)
        trace += gsl_vector_get (&d.vector, i);
    return trace;
}

/**
 * inverse of matrix
 * @note Ainv and A can be the same arg here allowing for in-place calculation w/o concern for memory aliasing int.
 */
int
gslu_matrix_inv (gsl_matrix *Ainv, const gsl_matrix *A);

/**
 * allocation version of gslu_matrix_inv
 * @see gslu_matrix_inv
 */
static inline gsl_matrix *
gslu_matrix_inv_alloc (const gsl_matrix *A)
{
    assert (A->size1 == A->size2);
    gsl_matrix *Ainv = gsl_matrix_alloc (A->size1, A->size2);
    gslu_matrix_inv (Ainv, A);
    return Ainv;
}

/**
 * use cholesky decomposition to find inverse for symmetric positive definite matrix A
 */
int
gslu_matrix_spdinv (gsl_matrix *Ainv, const gsl_matrix *A);


/**
 * allocation version of gslu_matrix_spdinv
 */
static inline gsl_matrix *
gslu_matrix_spdinv_alloc (const gsl_matrix *A)
{
    gsl_matrix *Ainv = gslu_matrix_clone (A);
    gslu_matrix_spdinv (Ainv, A);
    return Ainv;
}

/**
 * reshapes matrix A into matrix B:
 * taken in column order if TransA = CblasNoTrans
 * taken in row order if TransA = CblasTrans
 */
int
gslu_matrix_reshape (gsl_matrix *B, const gsl_matrix *A, CBLAS_TRANSPOSE_t TransA);

/**
 * stacks matrix A into a column vector:
 * taken in column order if TransA = CblasNoTrans
 * taken in row order if TransA = CblasTrans
 */
int
gslu_matrix_stack (gsl_vector *a, const gsl_matrix *A, CBLAS_TRANSPOSE_t TransA);


/**
 * calculate skew sym matrix from 3 element vector
 * \f[S = \left[\begin{array}{ccc} 0 & -s(3) & s(2) \\ s(3) &  0  &  -s(1)\\ -s(2) &  s(1) &  0 \\ \end{array} \right] \f]
 */
static inline void
gslu_matrix_skewsym (gsl_matrix *S, const gsl_vector *s)
{
    assert (s->size == 3);

    double s1 = gsl_vector_get (s, 0);
    double s2 = gsl_vector_get (s, 1);
    double s3 = gsl_vector_get (s, 2);

    gsl_matrix_set (S, 0, 0, 0.0);  gsl_matrix_set (S, 0, 1, -s3);   gsl_matrix_set (S, 0, 2,  s2);
    gsl_matrix_set (S, 1, 0,  s3);  gsl_matrix_set (S, 1, 1, 0.0);   gsl_matrix_set (S, 1, 2, -s1);
    gsl_matrix_set (S, 2, 0, -s2);  gsl_matrix_set (S, 2, 1,  s1);   gsl_matrix_set (S, 2, 2, 0.0);
}

/**
 * allocation version of gslu_matrix_skewsym.
 * @see gslu_matrix_skewsym
 */
static inline gsl_matrix *
gslu_matrix_skewsym_alloc (const gsl_vector *s)
{
    assert (s->size == 3);
    gsl_matrix *S = gsl_matrix_alloc (3, 3);
    gslu_matrix_skewsym (S, s);
    return S;
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_MATRIX_H__
