#ifndef __PERLS_MATH_GSL_UTIL_RAND_H__
#define __PERLS_MATH_GSL_UTIL_RAND_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include "gsl_util_index.h"

/**
 * @defgroup PerlsMathGsluRand GSL Util Rand
 * @brief GSL utility Rand.
 * @ingroup PerlsMath
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*===========================RANDOM NUMBER GENERATOR=========================*/
/** random number generator */
int32_t
gslu_rand_seed (void);

/** generate random number and return gsl_rng */
gsl_rng *
gslu_rand_rng_alloc (void);

/*===========================RANDOM INDEX GENERATOR=========================*/

/** Randomly samples w/o replacement rsel->size unique indices from the set [0,n-1] */
void
gslu_rand_index (const gsl_rng *r, gslu_index *rsel, size_t n);


/*===========================RANDOM SCALAR/VECTOR/MATRIX=========================*/

/** uniformly distributed random numbers [0.0, 1.0) */
static inline double
gslu_rand_uniform (const gsl_rng *r)
{
    return gsl_rng_uniform (r);
}

/** populate a vector a with uniformly distributed random numbers [0.0, 1.0) */
static inline void
gslu_rand_uniform_vector (const gsl_rng *r, gsl_vector *a)
{
    for (size_t i=0; i<a->size; i++)
        gsl_vector_set (a, i, gsl_rng_uniform (r));
}

/** populate a matrix a with uniformly distributed random numbers [0.0, 1.0) */
static inline void
gslu_rand_uniform_matrix (const gsl_rng *r, gsl_matrix *A)
{
    for (size_t i=0; i<A->size1; i++)
        for (size_t j=0; j<A->size2; j++)
            gsl_matrix_set (A, i, j, gsl_rng_uniform (r));
}


/** strictly positive uniformly distributed random numbers (0.0, 1.0)*/
static inline double
gslu_rand_uniform_pos (const gsl_rng *r)
{
    return gsl_rng_uniform_pos (r);
}

/** populate a vector with strictly positive uniformly distributed random numbers (0.0, 1.0)*/
static inline void
gslu_rand_uniform_pos_vector (const gsl_rng *r, gsl_vector *a)
{
    for (size_t i=0; i<a->size; i++)
        gsl_vector_set (a, i, gsl_rng_uniform_pos (r));
}

/** populate a matrix with strictly positive uniformly distributed random numbers (0.0, 1.0)*/
static inline void
gslu_rand_uniform_pos_matrix (const gsl_rng *r, gsl_matrix *A)
{
    for (size_t i=0; i<A->size1; i++)
        for (size_t j=0; j<A->size2; j++)
            gsl_matrix_set (A, i, j, gsl_rng_uniform_pos (r));
}


/** normally distributed random numbers */
static inline double
gslu_rand_normal (const gsl_rng *r)
{
    return gsl_ran_gaussian_ziggurat (r, 1.0);
}

/** populate a vector with normally distributed random numbers */
static inline void
gslu_rand_normal_vector (const gsl_rng *r, gsl_vector *a)
{
    for (size_t i=0; i<a->size; i++)
        gsl_vector_set (a, i, gsl_ran_gaussian_ziggurat (r, 1.0));
}

/** populate a matrix with normally distributed random numbers */
static inline void
gslu_rand_normal_matrix (const gsl_rng *r, gsl_matrix *A)
{
    for (size_t i=0; i<A->size1; i++)
        for (size_t j=0; j<A->size2; j++)
            gsl_matrix_set (A, i, j, gsl_ran_gaussian_ziggurat (r, 1.0));
}


/** gaussian distributed random numbers */
static inline double
gslu_rand_gaussian (const gsl_rng *r, double mu, double std)
{
    return gsl_ran_gaussian_ziggurat (r, std) + mu;
}

/** for the vector and matrix functions provide either the covariance
 * matrix Sigma, or its lower-triangular choleksy decomposition L
 * (faster) where Sigma = L*L', but not both.  set the unused param to
 * NULL
 */
void
gslu_rand_gaussian_vector (const gsl_rng *r, gsl_vector *a, 
                           const gsl_vector *mu, const gsl_matrix *Sigma, const gsl_matrix *L);

/**
 * @see gslu_rand_gaussian_vector
 * @note each column of A is a vector sample 
 */
void 
gslu_rand_gaussian_matrix (const gsl_rng *r, gsl_matrix *A, 
                           const gsl_vector *mu, const gsl_matrix *Sigma, const gsl_matrix *L);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_MATH_GSL_UTIL_RAND_H__
