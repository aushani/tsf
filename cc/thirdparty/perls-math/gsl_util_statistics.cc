#include <gsl/gsl_sort.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_statistics.h>

#include "gsl_util_vector.h"
#include "gsl_util_statistics.h"

double
gslu_stats_median_array (double data[], size_t stride, size_t n)
{
    gsl_sort (data, stride, n);
    return gsl_stats_median_from_sorted_data (data, stride, n);
}


double
gslu_stats_median_vector (const gsl_vector *v)
{
    gsl_vector *b = gslu_vector_clone (v);
    gsl_sort_vector (b);
    double median = gsl_stats_median_from_sorted_data (b->data, b->stride, b->size);
    gslu_vector_free (b);
    return median;
}

gsl_matrix *
gslu_sample_cov (const gsl_matrix *A)
{
    gsl_matrix *Sigma = gsl_matrix_calloc (A->size1, A->size1);
    for (size_t i = 0; i < A->size1; i++) {
        for (size_t j = i; j < A->size1; j++) {
            gsl_vector_const_view a = gsl_matrix_const_row (A, i);
            gsl_vector_const_view b = gsl_matrix_const_row (A, j);
            double cov = gsl_stats_covariance(a.vector.data, a.vector.stride, 
                                              b.vector.data, b.vector.stride, a.vector.size);
            gsl_matrix_set (Sigma, i, j, cov);
            gsl_matrix_set (Sigma, j, i, cov);
        }
    }
    return Sigma;
}
