#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util.h"
#include "homogenous.h"

void
homogenize (const gsl_matrix *src, gsl_matrix *dst)
{
    assert (src->size1==(dst->size1-1) && (src->size2==dst->size2));

    gslu_matrix_set_submatrix (dst, 0, 0, src);

    gsl_vector *v = gsl_vector_alloc (dst->size2);
    gsl_vector_set_all (v, 1.0);
    gsl_matrix_set_row (dst, dst->size1-1, v);
    gsl_vector_free (v);
}

gsl_matrix *
homogenize_alloc (const gsl_matrix *src)
{
    size_t rows = src->size1+1;
    size_t cols = src->size2;

    gsl_matrix *dst = gsl_matrix_alloc (rows, cols);
    homogenize (src, dst);

    return dst;
}


void
dehomogenize (const gsl_matrix *src, gsl_matrix *dst)
{
    assert (dst->size1==(src->size1-1) && (src->size2==dst->size2));

    gsl_matrix_const_view src_sub = gsl_matrix_const_submatrix (src, 0, 0, dst->size1, dst->size2);
    gsl_matrix_memcpy (dst, &src_sub.matrix);

    gsl_vector_const_view w = gsl_matrix_const_row (src, src->size1-1);
    for (size_t i=0; i<dst->size1; i++) {
        gsl_vector_view v = gsl_matrix_row (dst, i);
        gsl_vector_div (&v.vector, &w.vector);
    }
}

gsl_matrix *
dehomogenize_alloc (const gsl_matrix *src)
{
    size_t rows = src->size1-1;
    size_t cols = src->size2;

    gsl_matrix *dst = gsl_matrix_alloc (rows, cols);
    dehomogenize (src, dst);
    return dst;
}
