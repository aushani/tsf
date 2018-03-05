#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <float.h>
#include <sys/ioctl.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

#include "compare.h"
#include "gsl_util_matrix.h"
#include "gsl_util_linalg.h"

// determinant of matrix
double
gslu_matrix_det (const gsl_matrix *A)
{
    assert (A->size1 == A->size2);
    double det = 0.0;
    switch (A->size1) {
    case 1:
        det = gsl_matrix_get (A, 0, 0);
        break;

    case 2: {
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1);
        double c = gsl_matrix_get (A, 1, 0), d = gsl_matrix_get (A, 1, 1);
        det = a*d - b*c;
        break;
    }

    case 3: {
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1), c = gsl_matrix_get (A, 0, 2);
        double d = gsl_matrix_get (A, 1, 0), e = gsl_matrix_get (A, 1, 1), f = gsl_matrix_get (A, 1, 2);
        double g = gsl_matrix_get (A, 2, 0), h = gsl_matrix_get (A, 2, 1), i = gsl_matrix_get (A, 2, 2);
        det = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g);
        break;
    }

    case 4: {
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1), c = gsl_matrix_get (A, 0, 2), d = gsl_matrix_get (A, 0, 3);
        double e = gsl_matrix_get (A, 1, 0), f = gsl_matrix_get (A, 1, 1), g = gsl_matrix_get (A, 1, 2), h = gsl_matrix_get (A, 1, 3);
        double i = gsl_matrix_get (A, 2, 0), j = gsl_matrix_get (A, 2, 1), k = gsl_matrix_get (A, 2, 2), l = gsl_matrix_get (A, 2, 3);
        double m = gsl_matrix_get (A, 3, 0), n = gsl_matrix_get (A, 3, 1), o = gsl_matrix_get (A, 3, 2), p = gsl_matrix_get (A, 3, 3);

        det =
            a*(f*(k*p - l*o) - j*(g*p - h*o) + n*(g*l - h*k)) -
            e*(b*(k*p - l*o) - j*(c*p - d*o) + n*(c*l - d*k)) +
            i*(b*(g*p - h*o) - f*(c*p - d*o) + n*(c*h - d*g)) -
            m*(b*(g*l - h*k) - f*(c*l - d*k) + j*(c*h - d*g));
        break;
    }

    default: {
        gslu_linalg_LU *lu = gslu_linalg_LU_decomp_alloc (A);
        det = gsl_linalg_LU_det (lu->LU, lu->signum);
        gslu_linalg_LU_free (lu);
    }
    }
    return det;
}


// inverse of matrix
int
gslu_matrix_inv (gsl_matrix *Ainv, const gsl_matrix *A)
{
    assert (A->size1 == A->size2 && Ainv->size1 == A->size1 && Ainv->size2 == A->size2);
    int ret = GSL_SUCCESS;
    switch (A->size1) {
    case 1:
        gsl_matrix_set (Ainv, 0, 0, 1.0 / gsl_matrix_get (A, 0, 0));
        break;

    case 2: {
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1);
        double c = gsl_matrix_get (A, 1, 0), d = gsl_matrix_get (A, 1, 1);
        double det = a*d - b*c;
        if (0 == dblcmp (det, 0.0)) { // singular
            GSL_ERROR ("matrix is singular", GSL_ESING);
        }
        gsl_matrix_set (Ainv, 0, 0,  d/det); gsl_matrix_set (Ainv, 0, 1, -b/det);
        gsl_matrix_set (Ainv, 1, 0, -c/det); gsl_matrix_set (Ainv, 1, 1,  a/det);
        break;
    }

    case 3: {
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1), c = gsl_matrix_get (A, 0, 2);
        double d = gsl_matrix_get (A, 1, 0), e = gsl_matrix_get (A, 1, 1), f = gsl_matrix_get (A, 1, 2);
        double g = gsl_matrix_get (A, 2, 0), h = gsl_matrix_get (A, 2, 1), i = gsl_matrix_get (A, 2, 2);

        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (0 == dblcmp (det, 0.0)) { // singular
            GSL_ERROR ("matrix is singular", GSL_ESING);
        }
        gsl_matrix_set (Ainv, 0, 0, (e*i - f*h) / det);
        gsl_matrix_set (Ainv, 0, 1, (c*h - b*i) / det);
        gsl_matrix_set (Ainv, 0, 2, (b*f - c*e) / det);

        gsl_matrix_set (Ainv, 1, 0, (f*g - d*i) / det);
        gsl_matrix_set (Ainv, 1, 1, (a*i - c*g) / det);
        gsl_matrix_set (Ainv, 1, 2, (c*d - a*f) / det);

        gsl_matrix_set (Ainv, 2, 0, (d*h - e*g) / det);
        gsl_matrix_set (Ainv, 2, 1, (b*g - a*h) / det);
        gsl_matrix_set (Ainv, 2, 2, (a*e - b*d) / det);
        break;
    }

    case 4: { // thank you matlab!
        double a = gsl_matrix_get (A, 0, 0), b = gsl_matrix_get (A, 0, 1), c = gsl_matrix_get (A, 0, 2), d = gsl_matrix_get (A, 0, 3);
        double e = gsl_matrix_get (A, 1, 0), f = gsl_matrix_get (A, 1, 1), g = gsl_matrix_get (A, 1, 2), h = gsl_matrix_get (A, 1, 3);
        double i = gsl_matrix_get (A, 2, 0), j = gsl_matrix_get (A, 2, 1), k = gsl_matrix_get (A, 2, 2), l = gsl_matrix_get (A, 2, 3);
        double m = gsl_matrix_get (A, 3, 0), n = gsl_matrix_get (A, 3, 1), o = gsl_matrix_get (A, 3, 2), p = gsl_matrix_get (A, 3, 3);

        double det =
            a*(f*(k*p - l*o) - j*(g*p - h*o) + n*(g*l - h*k)) -
            e*(b*(k*p - l*o) - j*(c*p - d*o) + n*(c*l - d*k)) +
            i*(b*(g*p - h*o) - f*(c*p - d*o) + n*(c*h - d*g)) -
            m*(b*(g*l - h*k) - f*(c*l - d*k) + j*(c*h - d*g));
        if (0 == dblcmp (det, 0.0)) { // singular
            GSL_ERROR ("matrix is singular", GSL_ESING);
        }
        gsl_matrix_set (Ainv, 0, 0,  (f*(k*p - l*o) - j*(g*p - h*o) + n*(g*l - h*k)) / det);
        gsl_matrix_set (Ainv, 0, 1, -(b*(k*p - l*o) - j*(c*p - d*o) + n*(c*l - d*k)) / det);
        gsl_matrix_set (Ainv, 0, 2,  (b*(g*p - h*o) - f*(c*p - d*o) + n*(c*h - d*g)) / det);
        gsl_matrix_set (Ainv, 0, 3, -(b*(g*l - h*k) - f*(c*l - d*k) + j*(c*h - d*g)) / det);

        gsl_matrix_set (Ainv, 1, 0, -(e*(k*p - l*o) - i*(g*p - h*o) + m*(g*l - h*k)) / det);
        gsl_matrix_set (Ainv, 1, 1,  (a*(k*p - l*o) - i*(c*p - d*o) + m*(c*l - d*k)) / det);
        gsl_matrix_set (Ainv, 1, 2, -(a*(g*p - h*o) - e*(c*p - d*o) + m*(c*h - d*g)) / det);
        gsl_matrix_set (Ainv, 1, 3,  (a*(g*l - h*k) - e*(c*l - d*k) + i*(c*h - d*g)) / det);

        gsl_matrix_set (Ainv, 2, 0,  (e*(j*p - l*n) - i*(f*p - h*n) + m*(f*l - h*j)) / det);
        gsl_matrix_set (Ainv, 2, 1, -(a*(j*p - l*n) - i*(b*p - d*n) + m*(b*l - d*j)) / det);
        gsl_matrix_set (Ainv, 2, 2,  (a*(f*p - h*n) - e*(b*p - d*n) + m*(b*h - d*f)) / det);
        gsl_matrix_set (Ainv, 2, 3, -(a*(f*l - h*j) - e*(b*l - d*j) + i*(b*h - d*f)) / det);

        gsl_matrix_set (Ainv, 3, 0, -(e*(j*o - k*n) - i*(f*o - g*n) + m*(f*k - g*j)) / det);
        gsl_matrix_set (Ainv, 3, 1,  (a*(j*o - k*n) - i*(b*o - c*n) + m*(b*k - c*j)) / det);
        gsl_matrix_set (Ainv, 3, 2, -(a*(f*o - g*n) - e*(b*o - c*n) + m*(b*g - c*f)) / det);
        gsl_matrix_set (Ainv, 3, 3,  (a*(f*k - g*j) - e*(b*k - c*j) + i*(b*g - c*f)) / det);
        break;
    }

    default: {
        gslu_linalg_LU *lu = gslu_linalg_LU_decomp_alloc (A);
        ret = gsl_linalg_LU_invert (lu->LU, lu->p, Ainv);
        gslu_linalg_LU_free (lu);
    }
    }
    return ret;
}

// use cholesky decomposition to find inverse for symmetric positive definite matrix A
int
gslu_matrix_spdinv (gsl_matrix *Ainv, const gsl_matrix *A)
{
    assert (Ainv->size1 == A->size1 && Ainv->size2 == A->size2);
    gsl_matrix_memcpy (Ainv, A);
    if (gsl_linalg_cholesky_decomp (Ainv) == GSL_EDOM) {
        GSL_ERROR ("Input matrix is not positive definite!", GSL_EDOM);
        return GSL_EDOM;
    }
    else
        return gsl_linalg_cholesky_invert (Ainv);
}



// reshapes matrix A into matrix B:
// taken in column order if TransA = CblasNoTrans
// taken in row order if TransA = CblasTrans
int
gslu_matrix_reshape (gsl_matrix *B, const gsl_matrix *A, CBLAS_TRANSPOSE_t TransA)
{
    assert (A->size1*A->size2 == B->size1*B->size2);
    switch (TransA) {
    case CblasNoTrans: // column order
        for (size_t j=0, k=0; j<A->size2; j++) {
            for (size_t i=0; i<A->size1; i++, k++) {
                size_t ii = k % B->size1;
                size_t jj = k / B->size1;
                gsl_matrix_set (B, ii, jj, gsl_matrix_get (A, i, j));
            }
        }
        break;

    case CblasTrans: // row order
        for (size_t i=0, k=0; i<A->size1; i++) {
            for (size_t j=0; j<A->size2; j++, k++) {
                size_t ii = k % B->size1;
                size_t jj = k / B->size1;
                gsl_matrix_set (B, ii, jj, gsl_matrix_get (A, i, j));
            }
        }
        break;

    case CblasConjTrans:
        GSL_ERROR ("CblasConjTrans not handled!", GSL_EUNIMPL);
        break;

    default:
        GSL_ERROR ("Unknown CBLAS_TRANSPOSE_t type!", GSL_EUNIMPL);
    }
    return GSL_SUCCESS;
}

// stacks matrix A into a column vector:
// taken in column order if TransA = CblasNoTrans
// taken in row order if TransA = CblasTrans
int
gslu_matrix_stack (gsl_vector *a, const gsl_matrix *A, CBLAS_TRANSPOSE_t TransA)
{
    assert (A->size1*A->size2 == a->size);
    switch (TransA) {
    case CblasNoTrans: // column order
        for (size_t j=0, k=0; j<A->size2; j++)
            for (size_t i=0; i<A->size1; i++, k++)
                gsl_vector_set (a, k, gsl_matrix_get (A, i, j));
        break;

    case CblasTrans: // row order
        for (size_t i=0, k=0; i<A->size1; i++)
            for (size_t j=0; j<A->size2; j++, k++)
                gsl_vector_set (a, k, gsl_matrix_get (A, i, j));
        break;

    case CblasConjTrans:
        GSL_ERROR ("CblasConjTrans not handled!", GSL_EUNIMPL);
        break;

    default:
        GSL_ERROR ("Unknown CBLAS_TRANSPOSE_t type!", GSL_EUNIMPL);
    }
    return GSL_SUCCESS;
}


void
gslu_matrix_printfc (const gsl_matrix *A, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans)
{

    // figure out our terminal window size
    int ncols = 0;
    struct winsize w;
    if (ioctl (0, TIOCGWINSZ, &w))
        ncols = 80;
    else
        ncols = round (w.ws_col / 12.0);

    if (ncols == 0)
        ncols = 80;

    if (name) {
        if (Trans==CblasNoTrans)
            printf ("%s =\n", name);
        else
            printf ("%s' =\n", name);
    }

    // default printf format
    int __default__ = 1;
    if (fmt == NULL)
        fmt = "%10.4f";
    else
        __default__ = 0;

    size_t size1, size2;
    if (Trans == CblasNoTrans) {
        size1 = A->size1;
        size2 = A->size2;
    } else {
        size1 = A->size2;
        size2 = A->size1;
    }

    // emulate matlab scientific format
    bool scifmt = 0;
    const int scimin = -3;
    const int scimax =  4;
    const double ZERO = DBL_EPSILON;
    double fabsmax = fabs (gsl_matrix_max (A));
    int sci;
    if (fabsmax > ZERO)
        sci = floor (log10 (fabsmax));
    else
        sci = 0;

    if (sci < scimin) {
        sci++;
        scifmt = 1;
    }
    else if (sci > scimax) {
        sci--;
        scifmt = 1;
    }
    const double tens = pow (10, sci);
    if (scifmt)
        printf ("   1.0e%+03d *\n", sci);

    // print matrix
    for (size_t n=0; n < size2; n+=ncols) {
        if (ncols < size2) {
            if (n == (size2 - 1))
                printf ("    Column %zd\n", n);
            else
                printf ("    Columns %zd through %zd\n", n, GSL_MIN(n+ncols, size2)-1);
        }
        for (size_t i=0; i<size1; i++) {
            for (size_t j=GSL_MIN(n, size2); j<GSL_MIN(n+ncols, size2); j++) {
                double v;
                if (Trans == CblasNoTrans)
                    v = gsl_matrix_get (A, i, j);
                else
                    v = gsl_matrix_get (A, j, i);

                if (scifmt)
                    v /= tens;

                if (__default__ && fabs (v) < ZERO)
                    printf ("%10.4g", 0.0);
                else
                    printf (fmt, v);
                printf (" ");
            }
            printf ("\n");
        }
    }
}

void
gslu_matrix_mdiag (gsl_matrix *ABDiag, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (gslu_matrix_is_square (A) && gslu_matrix_is_square (B) && ABDiag->size1 == A->size1+B->size1);
    gslu_matrix_set_submatrix (ABDiag, 0, 0, A);
    gslu_matrix_set_submatrix (ABDiag, A->size1, A->size2, B);
}
