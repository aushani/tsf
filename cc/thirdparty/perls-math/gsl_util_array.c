#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util_matrix.h"
#include "gsl_util_array.h"

void
gslu_array_printf (const double data[], size_t size1, size_t size2, const char *name)
{
    gsl_matrix_const_view A = gsl_matrix_const_view_array (data, size1, size2);
    gslu_matrix_printfc (&A.matrix, name, NULL, CblasNoTrans);
}

void
gslu_array_printfc (const double data[], size_t size1, size_t size2, const char *name, 
                    const char *fmt, CBLAS_TRANSPOSE_t trans)
{
    gsl_matrix_const_view A = gsl_matrix_const_view_array (data, size1, size2);
    gslu_matrix_printfc (&A.matrix, name, fmt, trans);
}

