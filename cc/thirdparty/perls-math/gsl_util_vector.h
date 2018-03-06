#ifndef __PERLS_MATH_GSL_UTIL_VECTOR_H__
#define __PERLS_MATH_GSL_UTIL_VECTOR_H__

#include <assert.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector_complex_double.h>

#include "gsl_util_index.h"
#include "gsl_util_blas.h"
#include "gsl_util_matrix.h"
#include "gsl_util_math.h"

/**
 * @defgroup PerlsMathGsluVector GSL Util Vector
 * @brief GSL utility for Vectors.
 * @ingroup PerlsMath
 *
 * @{
 */

/**
 * Macro for creating a gsl_vector_view object and data on the stack.
 * Use gslu macros to create vector elements on the stack with vector
 * views.  var will have subfields .data and .vector.  The advantage
 * is that elements created on the stack do not have to be manually
 * freed by the programmer, they automatically have limited scope
 */
#define GSLU_VECTOR_VIEW(var,i,...)                                     \
    struct {                                                            \
        double data[i];                                                 \
        gsl_vector vector;                                              \
    } var = {__VA_ARGS__};                                              \
    {   /* _view_ has local scope */                                    \
        gsl_vector_view _view_ = gsl_vector_view_array (var.data, i);   \
        var.vector = _view_.vector;                                     \
    }

/**
 * Macro for type conversion, e.g. the following command would
 * convert gsl_vector_float *f, to gsl_vector *d
 * GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, f, gsl_vector, d);
 */
#define GSLU_VECTOR_TYPEA_TO_TYPEB(typea, a, typeb, b) {                \
        const typea *_aa_ = a;                                          \
        assert (_aa_->size == ((const typeb*) b)->size);                \
        for (size_t _i_=0; _i_<_aa_->size; _i_++) {                     \
            typeb ## _set (b, _i_, typea ## _get (a, _i_));             \
        };                                                              \
    }

/*==============================VECTOR====================================*/


/**
 * simple version of gslu_vector_printf. e.g. gslu_vector_printf (a, "a");
 * @see gslu_vector_printf
 */
void
gslu_vector_printf (const gsl_vector *v, const char *name);

/**
 * prints the contents of a vector to stdout.  each element is formatted
 * using the printf-style format specifier fmt.
 *
 * @param fmt if it is NULL, then it defaults to "%f"
 * @param trans one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
gslu_vector_printfc (const gsl_vector *v, const char *name,
                     const char *fmt, CBLAS_TRANSPOSE_t trans);

/**
 * gslu_vector_printf for complex vector
 * @see gslu_vector_printf
 */
void
gslu_vector_complex_printf (const gsl_vector_complex *v, const char *name);

/**
 * complex custom version of gslu_vector_printf
 */
void
gslu_vector_complex_printfc (const gsl_vector_complex *v, const char *name,
                             const char *fmt, CBLAS_TRANSPOSE_t trans);

/**
 * custom "free" that checks for existence
 */
static inline void
gslu_vector_free (gsl_vector *v)
{
    if (v)
        gsl_vector_free (v);
}

/**
 * gslu_vector_free for float vector
 */
static inline void
gslu_vector_float_free (gsl_vector_float *v)
{
    if (v)
        gsl_vector_float_free (v);
}

/**
 * clones a vector
 */
static inline gsl_vector *
gslu_vector_clone (const gsl_vector *a)
{
    gsl_vector *b = gsl_vector_alloc (a->size);
    gsl_vector_memcpy (b, a);
    return b;
}

/**
 * clones a vector in float
 */
static inline gsl_vector_float *
gslu_vector_float_clone (const gsl_vector_float *a)
{
    gsl_vector_float *b = gsl_vector_float_alloc (a->size);
    gsl_vector_float_memcpy (b, a);
    return b;
}


/**
 * select a subset of vector a according to index isel
 */
static inline void
gslu_vector_sel (gsl_vector *b, const gsl_vector *a, const gslu_index *isel)
{
    assert (b->size == isel->size);
    for (size_t i=0; i<b->size; i++) {
        size_t ii = gslu_index_get (isel, i);
        assert (ii < a->size);
        gsl_vector_set (b, i, gsl_vector_get (a, ii));
    }
}

/**
 * allocation version of gslu_vector_sel
 * @see gslu_vector_sel
 */
static inline gsl_vector *
gslu_vector_sel_alloc (const gsl_vector *a, const gslu_index *isel)
{
    gsl_vector *b = gsl_vector_alloc (isel->size);
    gslu_vector_sel (b, a, isel);
    return b;
}

/**
 * @return 1 if vectors a and b have the same size
 */
static inline int
gslu_vector_is_same_size (const gsl_vector *a, const gsl_vector *b)
{
    return a->size == b->size;
}

/**
 * @return 1 if all elements of a and b are exactly the same
 */
int
gslu_vector_is_equal (const gsl_vector *a, const gsl_vector *b);


/**
 * copies vector b into a subvector of vector a starting at a(i)
 */
static inline int
gslu_vector_set_subvector (gsl_vector *a, const size_t i, const gsl_vector *b)
{
    assert ((b->size+i) <= a->size);
    gsl_vector_view asub = gsl_vector_subvector (a, i, b->size);
    return gsl_vector_memcpy (&asub.vector, b);
}

/**
 * copies subvector from b and store in a. a = b(idx:idx+len)
 */
static inline void
gslu_vector_get_subvector (gsl_vector *a, const gsl_vector *b, const size_t idx, const size_t len)
{
    assert (a->size <= b->size && a->size <= len && (idx+len) <= b->size);

    size_t a_i=0;
    for (size_t i=idx; i<idx+len; i++) {
        gsl_vector_set (a, a_i, gsl_vector_get (b, i));
        a_i++;
    }
}

/**
 * allocation version of gslu_vector_get_subvector
 * @see gslu_vector_get_subvector
 */
static inline gsl_vector *
gslu_vector_get_subvector_alloc (const gsl_vector *b, const size_t idx, const size_t len)
{
    gsl_vector *a = gsl_vector_alloc (len);
    gslu_vector_get_subvector (a, b, idx, len);
    return a;
}

/**
 * sum of the individual elements of a vector
 */
static inline double
gslu_vector_sum (const gsl_vector *v)
{
    double sum = 0.0;
    for (size_t i=0; i<v->size; i++)
        sum += gsl_vector_get (v, i);
    return sum;
}

/**
 * absolute sum of the individual elements of a vector
 */
static inline double
gslu_vector_abs_sum (const gsl_vector *v)
{
    return gsl_blas_dasum (v);
}

/**
 * cumulative sum of a vector
 */
static inline void
gslu_vector_cumsum (gsl_vector *v)
{
    for (size_t i=1 ; i < v->size ; i++)
        gsl_vector_set (v, i, gsl_vector_get(v, i) + gsl_vector_get(v, i-1));
}

/**
 * magnitude of a vector
 */
static inline double
gslu_vector_norm (const gsl_vector *v)
{
    return gsl_blas_dnrm2 (v);
}

/**
 * magnitude of a vector for float vector
 * @see gslu_vector_norm
 */
static inline float
gslu_vector_float_norm (const gsl_vector_float *v)
{
    return gsl_blas_snrm2 (v);
}

/**
 * computes the euclidean distance between two vectors.
 */
static inline double
gslu_vector_dist (const gsl_vector *a, const gsl_vector *b)
{
    assert (a->size == b->size);
    double dist2 = 0.0;
    for (size_t i=0; i<a->size; i++) {
        double delta = gsl_vector_get (a, i) - gsl_vector_get (b, i);
        dist2 += delta*delta;
    }
    return sqrt (dist2);
}

/**
 * computes the euclidean distance between two vectors, taking care to properly handle
 * indicies with circular quantities.
 *
 * @param c an index vector of circular indicies, elements must be specified in increasing order
 */
double
gslu_vector_circ_dist (const gsl_vector *a, const gsl_vector *b, const gslu_index *c);


/**
 * computes the Mahalanobis distance between two vectors.
 * \f[d = (a-b)^T * S^{-1} * (a-b) \f]
 *
 * @Note the user supplies the matrix inverse in the above
 */
double
gslu_vector_mahal_dist (const gsl_vector *a, const gsl_vector *b, const gsl_matrix *Sinv);

/**
 * computes the Mahalanobis distance between two vectors, taking care to properly handle
 * indicies with circular quantities.
 *
 * @param c an index vector of circular indicies, elements must be specified in increasing order
 * @see gslu_vector_mahal_dist
 */
double
gslu_vector_mahal_circ_dist (const gsl_vector *a, const gsl_vector *b, const gsl_matrix *Sinv, const gslu_index *c);

/**
 * computes vector dot product \f$ a^T*b \f$
 */
static inline double
gslu_vector_dot (const gsl_vector *a, const gsl_vector *b)
{
    assert (a->size == b->size);
    double result;
    gsl_blas_ddot (a, b, &result);
    return result;
}

/**
 * computes vector dot product \f$ a^T*b \f$ for float vector
 * @see gslu_vector_dot
 */
static inline float
gslu_vector_float_dot (const gsl_vector_float *a, const gsl_vector_float *b)
{
    assert (a->size == b->size);
    float result;
    gsl_blas_sdot (a, b, &result);
    return result;
}

/**
 * computes vector cross product \f$c = a x b\f$
 */
static inline int
gslu_vector_cross (gsl_vector *c, const gsl_vector *a, const gsl_vector *b)
{
    assert (c->size == 3 && a->size == 3 && b->size == 3);
    double a0 = gsl_vector_get (a, 0), b0 = gsl_vector_get (b, 0);
    double a1 = gsl_vector_get (a, 1), b1 = gsl_vector_get (b, 1);
    double a2 = gsl_vector_get (a, 2), b2 = gsl_vector_get (b, 2);
    gsl_vector_set (c, 0, a1*b2 - a2*b1);
    gsl_vector_set (c, 1, a2*b0 - a0*b2);
    gsl_vector_set (c, 2, a0*b1 - a1*b0);
    return GSL_SUCCESS;
}

/**
 * allocation version of gslu_vector_cross
 * @see gslu_vector_cross
 */
static inline gsl_vector *
gslu_vector_cross_alloc (const gsl_vector *a, const gsl_vector *b)
{
    assert (a->size == 3 && b->size == 3);
    gsl_vector *c = gsl_vector_alloc (3);
    gslu_vector_cross (c, a, b);
    return c;
}

/**
 * computes scalar triple product \f$ a^T * (b x c) \f$
 */
static inline double
gslu_vector_scalar_triple_prod (const gsl_vector *a, const gsl_vector *b, const gsl_vector *c)
{
    assert (a->size == 3 && b->size == 3 && c->size == 3);
    GSLU_VECTOR_VIEW (bxc, 3);
    gslu_vector_cross (&bxc.vector, b, c);
    return gslu_vector_dot (a, &bxc.vector);
}

/**
 * computes vector triple product \f$ a x (b x c) \f$
 */
static inline int
gslu_vector_triple_prod (gsl_vector *r, const gsl_vector *a, const gsl_vector *b, const gsl_vector *c)
{
    assert (r->size == 3 && a->size == 3 && b->size == 3 && c->size == 3);
    GSLU_VECTOR_VIEW (bxc, 3);
    gslu_vector_cross (&bxc.vector, b, c);
    return gslu_vector_cross (r, a, &bxc.vector);
}

/**
 * allocation version of gslu_vector_triple_prod
 * @see gslu_vector_triple_prod
 */
static inline gsl_vector *
gslu_vector_triple_prod_alloc (const gsl_vector *a, const gsl_vector *b, const gsl_vector *c)
{
    assert (a->size == 3 && b->size == 3 && c->size == 3);
    gsl_vector *r = gsl_vector_alloc (3);
    gslu_vector_triple_prod (r, a, b, c);
    return r;
}


/**
 * reshapes a vector into matrix
 * stuffed in column order if TransA = CblasNoTrans
 * stuffed in row order if TransA = CblasTrans
 *
 * @param TransA CblasNoTrans to stuff in column order. CblasTrans to stuff in row order
 * @return GSL_SUCCESS when successful. GSL_ERROR on error
 */
int
gslu_vector_reshape (gsl_matrix *A, const gsl_vector *a, CBLAS_TRANSPOSE_t TransA);

/**
 * normalize vector to have unit length \f$ a = a / norm(a); \f$
 * @note input a will be changed after the operation.
 */
static inline void
gslu_vector_normalize (gsl_vector *a)
{
    double anorm = gslu_vector_norm (a);
    gsl_vector_scale (a, 1/anorm);
}

/**
 * allocation version of gslu_vector_normalize. \f$ b = a / norm(a)\f$
 * @note input vector a will not be changed.
 */
static inline gsl_vector *
gslu_vector_normalize_alloc (const gsl_vector *a)
{
    double anorm = gslu_vector_norm (a);
    gsl_vector *b = gslu_vector_clone (a);
    gsl_vector_scale (b, 1/anorm);
    return b;
}

/**
 * normalize vector to have unit length \f$ a = a / norm(a); \f$
 * @note input a will be changed after the operation.
 */
static inline void
gslu_vector_float_normalize (gsl_vector_float *a)
{
    float anorm = gslu_vector_float_norm (a);
    gsl_vector_float_scale (a, 1/anorm);
}

/**
 * allocation version of gslu_vector_float_normalize. \f$ b = a / norm(a)\f$
 * @note input vector a will not be changed.
 */
static inline gsl_vector_float *
gslu_vector_float_normalize_alloc (gsl_vector_float *a)
{
    float anorm = gslu_vector_float_norm (a);
    gsl_vector_float *b = gslu_vector_float_clone (a);
    gsl_vector_float_scale (b, 1/anorm);

    return b;
}

/**
 * Take square root for each element in vector a
 * @note this operation will change the input vector a.
 */
static inline void
gslu_vector_sqrt (gsl_vector *a)
{
    size_t n = a->size;

    for (size_t i=0; i<n; i++)
        gsl_vector_set (a, i, sqrt(gsl_vector_get (a, i)));
}

/**
 * allocation version of gslu_vector_sqrt
 */
static inline gsl_vector *
gslu_vector_sqrt_alloc (const gsl_vector *a)
{
    size_t n = a->size;
    gsl_vector *b = gsl_vector_alloc (n);
    gsl_vector_memcpy (b, a);

    for (size_t i=0; i<n; i++)
        gsl_vector_set (b, i, sqrt(gsl_vector_get (b, i)));

    return b;
}

/**
 * Invert each element in vector a
 * @note this operation will change the input vector a.
 */
static inline void
gslu_vector_inv (gsl_vector *a)
{
    size_t n = a->size;

    for (size_t i=0; i<n; i++)
        gsl_vector_set (a, i, 1.0/(gsl_vector_get (a, i)));
}

/**
 * allocation version of gslu_vector_inv
 */
static inline gsl_vector *
gslu_vector_inv_alloc (const gsl_vector *a)
{
    size_t n = a->size;
    gsl_vector *b = gsl_vector_alloc (n);
    for (size_t i=0; i<n; i++)
        gsl_vector_set (b, i, 1.0/(gsl_vector_get (a, i)));

    return b;
}

/**
 * returns changed vector 'a' such that each element of 'a' is the absolute value of the original element.
 * \f[ a_i = ||a_i|| \f]
 * @note input vector a will be changed
 */
static inline void
gslu_vector_abs (gsl_vector *a)
{
    size_t n = a->size;
    for (size_t i=0; i<n; i++)
        gsl_vector_set (a, i, fabs (gsl_vector_get (a, i)));
}

/**
 * returns a new vector such that each element of b is the absolute value of the original vector a.
 * \f[ b_i = ||a_i|| \f]
 */
static inline gsl_vector *
gslu_vector_abs_alloc (gsl_vector *a)
{
    size_t n = a->size;
    gsl_vector *b = gsl_vector_alloc (n);
    for (size_t i=0; i<n; i++)
        gsl_vector_set (b, i, fabs (gsl_vector_get (a, i)));

    return b;
}

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_VECTOR_H__
