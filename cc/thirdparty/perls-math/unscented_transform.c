#include "unscented_transform.h"

#include "gsl_util_matrix.h"
#include "gsl_util_blas.h"
#include "gsl_util_linalg.h"

unscented_transform_opts_t * 
unscented_transform_def_opts ()
{
    unscented_transform_opts_t *opts = calloc (1, sizeof(*opts));
    opts->alpha = 1.0;
    opts->beta = 2.0;
    opts->kappa = 0.0;
    return opts;
}

void
unscented_transform_t_free (unscented_transform_t *ut)
{
    gsl_matrix_free (ut->sigmaPoints);
    gsl_vector_free (ut->covWeights);
    gsl_vector_free (ut->meanWeights);
    free (ut);
}

void
unscented_transform_alloc (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                           unscented_transform_t **ut)
{
    int n = mu->size;
    int numSigmaPoints = 2*n+1;

    *ut = calloc (1, sizeof **ut);
    (*ut)->sigmaPoints = gsl_matrix_calloc(n, numSigmaPoints);
    (*ut)->meanWeights = gsl_vector_calloc(numSigmaPoints);
    (*ut)->covWeights  = gsl_vector_calloc(numSigmaPoints);

    if (opts)
        unscented_transform (mu, R, opts, *ut);
    else {
        unscented_transform_opts_t *defaultOpts = unscented_transform_def_opts();
        unscented_transform (mu, R, defaultOpts, *ut);
        free (defaultOpts);
    }
}

int
unscented_transform (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                     unscented_transform_t *ut)
{
    int n = ut->sigmaPoints->size1;

    assert(ut->sigmaPoints->size1 == n && ut->sigmaPoints->size2 == (2*n+1));

    double lambda = pow(opts->alpha, 2) * (n+opts->kappa) - n;

    gsl_matrix *RCopy = gsl_matrix_alloc(R->size1, R->size2);
	gsl_matrix_memcpy(RCopy, R);
    gsl_matrix_scale(RCopy, n+lambda);
    gsl_matrix *U = gslu_linalg_sqrtm_alloc(RCopy);

    int i=0;
    gsl_vector_view UColumn;
    gsl_vector_view ithSigmaPoint;

    //set the 0th sigma point to be the mean
    ithSigmaPoint = gsl_matrix_column(ut->sigmaPoints, 0);
    gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);

    gsl_vector_set (ut->meanWeights, 0, lambda / (n+lambda));
    gsl_vector_set (ut->covWeights, 0, lambda / (n+lambda) + (1 - opts->alpha*opts->alpha + opts->beta));

    for (i=1; i<=n; i++) {
        /* Compute sigma point */
        UColumn = gsl_matrix_column(U, i-1);
        ithSigmaPoint = gsl_matrix_column(ut->sigmaPoints, i);
        gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);
        gsl_vector_add(&(ithSigmaPoint.vector), &(UColumn.vector));

        /* Compute mean weight */
        gsl_vector_set (ut->meanWeights, i, 1/(2*(n+lambda)));

        /* Compute covariance weight */
        gsl_vector_set (ut->covWeights, i, 1/(2*(n+lambda)));
    }

    for (i=n+1; i<=2*n; i++) {
        /* Compute sigma point */
        UColumn = gsl_matrix_column(U, i-n-1);
        ithSigmaPoint = gsl_matrix_column(ut->sigmaPoints, i);
        gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);
        gsl_vector_sub(&(ithSigmaPoint.vector), &(UColumn.vector));

        /* Compute mean weight */
        gsl_vector_set (ut->meanWeights, i, 1/(2*(n+lambda)));

        /* Compute covariance weight */
        gsl_vector_set (ut->covWeights, i, 1/(2*(n+lambda)));
    }

    //clean up
    gsl_matrix_free(RCopy);
    gsl_matrix_free(U);
    
    return EXIT_SUCCESS;
}

int
unscented_func_alloc (const unscented_func_t f, const unscented_transform_t *ut,
                      gsl_vector **muPrime, gsl_matrix **sigmaPrime, void *user)
{
    size_t n = ut->sigmaPoints->size2;
    gsl_vector *fout;

    /* need to get the length of the vector returned by f */
    gsl_vector_const_view tmp = gsl_matrix_const_column (ut->sigmaPoints, 0);
    fout = f (&tmp.vector, user);
    size_t m = fout->size;
    gsl_vector_free (fout);

    gsl_matrix *y = gsl_matrix_calloc (m, n);

    /* evaluate f for each sigma point */
    for (size_t col=0; col<n; ++col) {
        gsl_vector_const_view currSigmaPoint = gsl_matrix_const_column (ut->sigmaPoints, col);
        fout = f (&currSigmaPoint.vector, user);
        gslu_matrix_set_column (y, col, fout);
        gsl_vector_free (fout);
    }

    /* get the estimated mean */
    *muPrime = gsl_vector_calloc (m);
    gsl_vector *yColCpy = gsl_vector_calloc (m);
    for (size_t col=0; col<n; ++col) {
        gsl_vector_const_view yCol = gsl_matrix_const_column (y, col);
        gsl_vector_memcpy (yColCpy, &yCol.vector);
        gsl_vector_scale (yColCpy, gsl_vector_get (ut->meanWeights, col));
        gsl_vector_add (*muPrime, yColCpy);
    }

    /* get the estimated covariance */
    *sigmaPrime = gsl_matrix_calloc (m, m);
    gsl_matrix *sigmaPrimeScaled = gsl_matrix_calloc (m, m);
    gsl_vector *diff = gsl_vector_calloc (m);
    for (size_t col=0; col<n; ++col) {
        double weight = gsl_vector_get (ut->covWeights, col);
        gsl_vector_const_view yCol = gsl_matrix_const_column (y, col);
        gsl_vector_memcpy (diff, &yCol.vector);
        gsl_vector_sub (diff, *muPrime);
        gslu_blas_vvT (sigmaPrimeScaled, diff, diff);
        gsl_matrix_scale (sigmaPrimeScaled, weight);
        gsl_matrix_add (*sigmaPrime, sigmaPrimeScaled);
    }

    /* clean up */
    gsl_matrix_free (y);
    gsl_matrix_free (sigmaPrimeScaled);
    gsl_vector_free (yColCpy);
    gsl_vector_free (diff);

    return EXIT_SUCCESS;
}
