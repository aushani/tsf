#ifndef __PERLS_MATH_UNSCENTED_TRANSFORM_H__
#define __PERLS_MATH_UNSCENTED_TRANSFORM_H__

#include <math.h>
#include <assert.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_eigen.h>

typedef struct unscented_transform_opts_t {
    double alpha;
    double beta;
    double kappa;
} unscented_transform_opts_t;

typedef struct unscented_transform_t {
    gsl_matrix *sigmaPoints;
    gsl_vector *meanWeights;
    gsl_vector *covWeights;
} unscented_transform_t;

unscented_transform_opts_t * 
unscented_transform_def_opts ();

void
unscented_transform_t_free (unscented_transform_t *ut);

void
unscented_transform_alloc (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                           unscented_transform_t **ut);

int
unscented_transform (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                     unscented_transform_t *ut);

typedef gsl_vector* (*unscented_func_t) (const gsl_vector *x, void *user);

int
unscented_func_alloc (const unscented_func_t f, const unscented_transform_t *ut,
                      gsl_vector **muPrime, gsl_matrix **sigmaPrime, void *user);

#endif //__PERLS_MATH_UNSCENTED_TRANSFORM_H__
