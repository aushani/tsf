#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util_vector.h"
#include "gsl_util_matrix.h"
#include "gsl_util_boolean.h"


void
gslu_boolean_printf (const gslu_boolean *i, const char *name)
{
    gsl_vector *d = gsl_vector_alloc (i->size);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gslu_boolean, i, gsl_vector, d);
    gsl_matrix_const_view m = gsl_matrix_const_view_vector (d, d->size, 1);
    gslu_matrix_printfc (&m.matrix, name, "%10.0f", CblasNoTrans);
    gslu_vector_free (d);
}


void
gslu_boolean_printfc (const gslu_boolean *i, const char *name, const char *fmt, CBLAS_TRANSPOSE_t trans)
{
    gsl_vector *d = gsl_vector_alloc (i->size);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gslu_boolean, i, gsl_vector, d);
    gsl_matrix_const_view m = gsl_matrix_const_view_vector (d, d->size, 1);
    if (fmt == NULL)
        gslu_matrix_printfc (&m.matrix, name, "%10.0f", trans);
    else
        gslu_matrix_printfc (&m.matrix, name, fmt, trans);
    gslu_vector_free (d);
}

