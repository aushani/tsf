#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <float.h>

#include <gsl/gsl_linalg.h>

#include "gsl_util_eigen.h"
#include "gsl_util_linalg.h"

gslu_linalg_SV *
gslu_linalg_SV_decomp_full_alloc (const gsl_matrix *A)
{
    size_t M = A->size1;
    size_t N = A->size2;

    gslu_linalg_SV *sv = malloc (sizeof (*sv));

    // check if M >= N
    if (M >= N) {
        sv->U = gsl_matrix_alloc (M, N);
        sv->S = gsl_vector_alloc (N);
        sv->V = gsl_matrix_alloc (N, N);
        gsl_matrix_memcpy (sv->U, A);
        gsl_vector *work = gsl_vector_alloc (N);

        // turning off default gsl error handler (=abort) to check success
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();

        int ret = gsl_linalg_SV_decomp (sv->U, sv->V, sv->S, work);
        if (ret != GSL_SUCCESS) {
            fprintf (stderr, "error in gslu_linalg_SV_decomp_alloc: %s\n", 
                     gsl_strerror (ret));

            gslu_linalg_SV_free (sv);
            gslu_vector_free (work);

            // restore back to default
            gsl_set_error_handler (default_handler);
            return NULL;
        }

        // restore back to default
        gsl_set_error_handler (default_handler);

        gslu_vector_free (work);
     }
    else { // M < N

        // RUN gsl svd first
        sv->U = gsl_matrix_alloc (M, M);
        sv->S = gsl_vector_alloc (M);
        sv->V = gsl_matrix_alloc (N, N);

        gsl_matrix *AT = gslu_matrix_transpose_alloc (A); // N by M matrix
        gsl_matrix_view Vsub = gsl_matrix_submatrix (sv->V, 0, 0, N, M);
        gsl_vector *work = gsl_vector_alloc (M);
        gsl_matrix_memcpy (&Vsub.matrix, AT);

        // turning off default gsl error handler (=abort) to check success
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();

        int ret = gsl_linalg_SV_decomp (&Vsub.matrix, sv->U, sv->S, work);
        if (ret != GSL_SUCCESS) {
            fprintf (stderr, "error in gslu_linalg_SV_decomp_alloc: %s\n", 
                     gsl_strerror (ret));
            
            gslu_linalg_SV_free (sv);
            gslu_vector_free (work);
            gslu_matrix_free (AT);

            // restore back to default
            gsl_set_error_handler (default_handler);
            return NULL;
        }

        // restore back to default
        gsl_set_error_handler (default_handler);

        // adding missing elements from QR
        gslu_linalg_QR *qr = gslu_linalg_QR_decomp_alloc (AT);
        gsl_matrix *Q = gsl_matrix_alloc (AT->size1, AT->size1);
        gsl_matrix *R = gsl_matrix_alloc (AT->size1, AT->size2);
        gsl_linalg_QR_unpack (qr->QR, qr->tau, Q, R);
        gsl_matrix_view Q_null = gsl_matrix_submatrix (Q, 0, M, N, N-M);
        gsl_matrix_view V_null = gsl_matrix_submatrix (sv->V, 0, M, N, N-M);
        gsl_matrix_memcpy (&V_null.matrix, &Q_null.matrix);

        // clean up
        gslu_matrix_free (AT);
        gslu_vector_free (work);
        gslu_linalg_QR_free (qr);
        gslu_matrix_free (Q);
        gslu_matrix_free (R);
    }

    return sv;
}


gslu_linalg_LU *
gslu_linalg_LU_decomp_alloc (const gsl_matrix *A)
{
    gslu_linalg_LU *lu = malloc (sizeof (*lu));
    lu->LU = gsl_matrix_alloc (A->size1, A->size2);
    gsl_matrix_memcpy (lu->LU, A);
    lu->p = gsl_permutation_alloc (A->size2);
    if (gsl_linalg_LU_decomp (lu->LU, lu->p, &lu->signum) != GSL_SUCCESS) {
        gslu_linalg_LU_free (lu);
        return NULL;
    }
    return lu;
}


gslu_linalg_QR *
gslu_linalg_QR_decomp_alloc (const gsl_matrix *A)
{
    gslu_linalg_QR *qr = malloc (sizeof (*qr));
    qr->QR = gsl_matrix_alloc (A->size1, A->size2);
    gsl_matrix_memcpy (qr->QR, A);
    qr->tau = gsl_vector_alloc (GSL_MIN (A->size1, A->size2));
    if (gsl_linalg_QR_decomp (qr->QR, qr->tau) != GSL_SUCCESS) {
        gslu_linalg_QR_free (qr);
        return NULL;
    }
    return qr;
}

int
gslu_linalg_cholesky_decomp_lower (gsl_matrix *A)
{    
    int ret = gsl_linalg_cholesky_decomp (A);
    if (ret == GSL_EDOM)
        GSL_ERROR ("Input matrix is not positive definite!", GSL_EDOM);
    else {
        // decomp returns L and LT in full matrix
        // set upper triangular part 0 so only lower is returned
        for (size_t i=0 ; i < A->size1 ; i++)
            for (size_t j=i+1 ; j < A->size2 ; j++)
                gsl_matrix_set (A, i, j, 0);

        return ret;
    }
}


gslu_linalg_SV *
gslu_linalg_SV_decomp_econ_alloc (const gsl_matrix *A)
{
    size_t M = A->size1;
    size_t N = A->size2;

    // check if M >= N
    if (M >= N) {
        gslu_linalg_SV *sv = malloc (sizeof (*sv));
        sv->U = gsl_matrix_alloc (M, N);
        gsl_matrix_memcpy (sv->U, A);
        sv->S = gsl_vector_alloc (N);
        sv->V = gsl_matrix_alloc (N, N);
        gsl_vector *work = gsl_vector_alloc (N);

        // turning off default gsl error handler (=abort) to check success
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();

        int ret = gsl_linalg_SV_decomp (sv->U, sv->V, sv->S, work);
        if (ret != GSL_SUCCESS) {
            fprintf (stderr, "error in gslu_linalg_SV_decomp_alloc: %s\n",
                     gsl_strerror (ret));

            gslu_linalg_SV_free (sv);
            gslu_vector_free (work);

            // restore back to default
            gsl_set_error_handler (default_handler);
            return NULL;
        }

        // restore back to default
        gsl_set_error_handler (default_handler);

        gslu_vector_free (work);
        return sv;
    }
    else {
        // if M < N, implemetation from
        // http://ugrad.stat.ubc.ca/R/library/GeneTS/html/fast.svd.html

        gsl_matrix *AAT = gslu_blas_mmT_alloc (A, A);
        gslu_eigen *eig = gslu_eigen_decomp_alloc (AAT);
        if (!eig) {
            gslu_matrix_free (AAT);
            gslu_eigen_free (eig);
            return NULL;
        }

        // A A' = U S^2 U'
        gslu_linalg_SV *sv = malloc (sizeof (*sv));
        sv->U = gslu_matrix_clone (eig->V);
        sv->S = gslu_vector_clone (eig->D); 
        gslu_vector_sqrt (sv->S);

        // V = A' U D^(-1)
        gsl_matrix *Sinv = gsl_matrix_alloc (M, M); gsl_matrix_set_zero (Sinv);
        gsl_vector_view Sinv_vec = gsl_matrix_diagonal (Sinv);
        gsl_vector_memcpy (&Sinv_vec.vector, sv->S);
        gslu_vector_inv (&Sinv_vec.vector);
        sv->V = gslu_blas_mTmm_alloc (A, sv->U, Sinv, NULL);

        gslu_matrix_free (AAT);
        gslu_matrix_free (Sinv);
        gslu_eigen_free (eig);

        return sv;
    }
}

void
gslu_linalg_rref (gsl_matrix *A, const int m2)
{
    // A = M x N matrix
    size_t M = A->size1;
    size_t N = A->size2;
    size_t max_rowcount = M;

    if (m2 > 0 && m2 <=M)
        max_rowcount = m2;
        
    size_t col, rix, iix;

    // workspace
    gsl_vector *row_work = gsl_vector_alloc (N);
 
    col = 0;
    for (rix=0; rix<max_rowcount; rix++) {
        if (col >= N) return;
        iix = rix;
        while (fabs (gsl_matrix_get (A, iix, col)) < GSL_DBL_EPSILON) {
            // if pivot candidate == 0 we need to swap. iix is the index to swap
            iix++;
            if (iix == M) {
                iix = rix;
                col++;
                if (col == N) return;
            }
        }

        // Swap i-th and r-th rows.
        if ( iix != rix ) {
            gsl_vector_view A_i_row = gsl_matrix_row (A, iix);
            gsl_vector_view A_r_row = gsl_matrix_row (A, rix);
            gsl_vector_memcpy (row_work, &A_r_row.vector);          // r -> temp
            gsl_vector_memcpy (&A_r_row.vector, &A_i_row.vector);   // i -> r
            gsl_vector_memcpy (&A_i_row.vector, row_work);          // temp -> i
        }

        // normalize pivot row
        gsl_vector_view A_r_row = gsl_matrix_row (A, rix);
        gsl_vector_scale (&A_r_row.vector, 1.0/gsl_matrix_get (A, rix, col));

        // Subtract multiples of the pivot row from all the other rows.
        for (iix=0; iix<max_rowcount; iix++) {
            if ( iix != rix ) {
                gsl_vector_memcpy (row_work, &A_r_row.vector);
                gsl_vector_scale (row_work, gsl_matrix_get (A, iix, col));
                gsl_vector_view A_i_row = gsl_matrix_row (A, iix);
                gsl_vector_sub (&A_i_row.vector, row_work);
            }
        }
        col++;
    }

    // clean up
    gslu_vector_free (row_work);
}

#ifdef GSL_VER_LESS_THAN_1_12
int 
gsl_linalg_cholesky_invert (gsl_matrix *cholesky)
{
    printf ("Using workaround for gsl_linalg_cholesky_invert()");
    for (size_t i=0; i<cholesky->size1; i++)
        for (size_t j=0; j<cholesky->size2; j++)
            if (i > j)
                gsl_matrix_set (cholesky, i, j, 0);

    gsl_matrix *Rinv = gslu_matrix_inv_alloc (cholesky);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, Rinv, Rinv, 0.0, cholesky);

    gslu_matrix_free (Rinv);

    return 0; //GSL_EDOM (==1)
}
#endif

static gsl_matrix *
gslu_linalg_diag_alloc(const gsl_vector * X)
{
    gsl_matrix * mat = gsl_matrix_alloc(X->size, X->size);
    gsl_vector_view diag = gsl_matrix_diagonal(mat);
    gsl_matrix_set_all(mat, 0.0);
    gsl_vector_memcpy(&diag.vector, X);
    return mat;
}

int
gslu_linalg_sqrtm (const gsl_matrix *R, gsl_matrix *sqrtmR)
{
    assert(R->size1 == R->size2);

    int n = R->size1;

    //Perform eigenvalue decomposition on R, ie R = U * eval * U'
    gsl_vector *eval = gsl_vector_alloc(n);
    gsl_matrix *U = gsl_matrix_alloc(n,n);
    gsl_vector *sqrtEval = gsl_vector_alloc(n);
    gsl_matrix *RCopy = gsl_matrix_calloc(R->size1, R->size2);
    gsl_matrix_memcpy (RCopy, R);

    //Compute the eigenvalues
    gsl_eigen_symmv_workspace *w =  gsl_eigen_symmv_alloc(n);
    gsl_eigen_symmv (RCopy, eval, U, w);
    gsl_eigen_symmv_free (w);

    int evalIndex = 0;
    for (evalIndex = 0; evalIndex < n; evalIndex++) {
        gsl_vector_set(sqrtEval, evalIndex, sqrt(gsl_vector_get(eval, evalIndex)));
    }

    gsl_matrix *Ut = gslu_matrix_transpose_alloc(U);
    gsl_matrix *Lambda = gslu_linalg_diag_alloc(sqrtEval);

    gsl_matrix *ULambda = gslu_blas_mm_alloc(U, Lambda);

    //sqrtmR = U * Lambda * U'
    gslu_blas_mm (sqrtmR, ULambda, Ut);

    //clean up
    gsl_vector_free(eval);
    gsl_matrix_free(U);
    gsl_vector_free(sqrtEval);
    gsl_matrix_free(Lambda);
    gsl_matrix_free(ULambda);
    gsl_matrix_free(Ut);
    gsl_matrix_free(RCopy);

    return EXIT_SUCCESS;
}
