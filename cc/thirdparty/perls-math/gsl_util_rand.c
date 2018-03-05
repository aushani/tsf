#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <time.h>
#include <sys/time.h>

// external linking req'd
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector.h>

#include "gsl_util_rand.h"


/*===========================RANDOM NUMBER GENERATOR=========================*/
/** random number generator */
int32_t
gslu_rand_seed (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return tv.tv_usec;
}

/** generate random number and return gsl_rng */
gsl_rng *
gslu_rand_rng_alloc (void)
{
    const gsl_rng_type *T = gsl_rng_env_setup ();
    gsl_rng *r = gsl_rng_alloc (T);
    gsl_rng_set (r, gslu_rand_seed ());
    return r;
}


/*===========================RANDOM INDEX GENERATOR=========================*/
void
gslu_rand_index (const gsl_rng *r, gslu_index *rsel, size_t n)
{
    assert (n >= rsel->size);

    // draw sample indices
    size_t nsamps = 0;
    while (nsamps < rsel->size) {
        size_t s = floor (gsl_rng_uniform (r) * n);

        // check for uniqueness
        bool unique = 1;
        for (size_t i=0; i<nsamps; i++) {
            if (s == gslu_index_get (rsel, i)) {
                unique = 0;
                break;
            }
        }

        if (unique) // add it to the set
            gslu_index_set (rsel, nsamps++, s);
    }

    // The above can be really slow for n close to rsel->size
    // the following should work better in general but might not be faster for
    // n << rsel->size
    //gslu_index_init (rsel);
    //unsigned long set[n];
    //for (int i=0; i<n; i++)
    //    set[i] = i;
    //gsl_ran_choose (r, rsel->data, rsel->size, set, n, sizeof (unsigned long));
}

/*===========================RANDOM SCALAR/VECTOR/MATRIX=========================*/
void
gslu_rand_gaussian_vector (const gsl_rng *r, gsl_vector *a, 
                           const gsl_vector *mu, const gsl_matrix *Sigma, const gsl_matrix *L)
{
    assert (a->size == mu->size && (Sigma || L));
    for (size_t i=0; i<a->size; i++)
        gsl_vector_set (a, i, gsl_ran_gaussian_ziggurat (r, 1.0));

    if (L) {
        assert (L->size1 == L->size2 && L->size1 == mu->size);
        gsl_blas_dtrmv (CblasLower, CblasNoTrans, CblasNonUnit, L, a);
    } else {
        assert (Sigma->size1 == Sigma->size2 && Sigma->size1 == mu->size);
        gsl_matrix *_L = gsl_matrix_alloc (Sigma->size1, Sigma->size2);
        gsl_matrix_memcpy (_L, Sigma);
        gsl_linalg_cholesky_decomp (_L);
        gsl_blas_dtrmv (CblasLower, CblasNoTrans, CblasNonUnit, _L, a);
        gsl_matrix_free (_L);
    }

    gsl_vector_add (a, mu);
}

void
gslu_rand_gaussian_matrix (const gsl_rng *r, gsl_matrix *A, 
                           const gsl_vector *mu, const gsl_matrix *Sigma, const gsl_matrix *L)
{
    assert (A->size1 == mu->size && (Sigma || L));
    for (size_t i=0; i<A->size1; i++)
        for (size_t j=0; j<A->size2; j++)
            gsl_matrix_set (A, i, j, gsl_ran_gaussian_ziggurat (r, 1.0));
    
    if (L) {
        assert (L->size1 == L->size2 && L->size1 == mu->size);
        gsl_blas_dtrmm (CblasLeft, CblasLower, CblasNoTrans, CblasNonUnit, 1.0, L, A);
    }
    else {
        assert (Sigma->size1 == Sigma->size2 && Sigma->size1 == mu->size);
        gsl_matrix *_L = gsl_matrix_alloc (Sigma->size1, Sigma->size2);
        gsl_matrix_memcpy (_L, Sigma);
        gsl_linalg_cholesky_decomp (_L);
        gsl_blas_dtrmm (CblasLeft, CblasLower, CblasNoTrans, CblasNonUnit, 1.0, _L, A);
        gsl_matrix_free (_L);
    }

    for (size_t j=0; j<A->size2; j++) {
        gsl_vector_view a = gsl_matrix_column (A, j);
        gsl_vector_add (&a.vector, mu);
    }
}
