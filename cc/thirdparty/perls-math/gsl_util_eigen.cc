#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_eigen.h>

#include "gsl_util_eigen.h"

gslu_eigen *
gslu_eigen_decomp_alloc (const gsl_matrix *A)
{
    // A should be a square matrix
    assert (A->size1 == A->size2);

    size_t N = A->size1;

    gslu_eigen *eigen = malloc (sizeof (*eigen));
    eigen->V = gsl_matrix_alloc (N, N);
    eigen->D = gsl_vector_alloc (N);
    gsl_matrix *work = gsl_matrix_alloc (N, N);
    gsl_matrix_memcpy (work, A); // because this changes matrix when eig decomposition
    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (N);

    // turning off default gsl error handler (=abort) to check success
    gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();
    int status = gsl_eigen_symmv (work, eigen->D, eigen->V, w);
    if (w) gsl_eigen_symmv_free (w);
    gsl_eigen_symmv_sort (eigen->D, eigen->V, GSL_EIGEN_SORT_ABS_DESC);

    if (status != GSL_SUCCESS) {
        fprintf (stderr, "error in gslu_linalg_EIGEN_decomp_alloc: %s\n",
                 gsl_strerror (status));

        gslu_eigen_free (eigen);
        gslu_matrix_free (work);

        // restore back to default
        gsl_set_error_handler (default_handler);
        return NULL;
    }

    // restore back to default
    gsl_set_error_handler (default_handler);

    gslu_matrix_free (work);
    return eigen;
}
