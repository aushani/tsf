#ifndef __PERLS_MATH_HOMOGENOUS_H__
#define __PERLS_MATH_HOMOGENOUS_H__

#include <gsl/gsl_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maps [D x N] src to [(D+1) x N] dst where N is the number of points
// and D is the dimension of each point
void
homogenize (const gsl_matrix *src, gsl_matrix *dst);

gsl_matrix *
homogenize_alloc (const gsl_matrix *src);


// Maps [(D+1) x N] src to [D x N] dst
void
dehomogenize (const gsl_matrix *src, gsl_matrix *dst);

gsl_matrix *
dehomogenize_alloc (const gsl_matrix *src);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_MATH_HOMOGENOUS_H__
