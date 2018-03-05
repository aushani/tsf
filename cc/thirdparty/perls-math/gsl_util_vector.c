#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/ioctl.h>

// external linking req'd
#include <math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util_matrix.h"
#include "gsl_util_vector.h"

void
gslu_vector_printf (const gsl_vector *a, const char *name)
{
    gslu_vector_printfc (a, name, NULL, CblasNoTrans);
}

void
gslu_vector_printfc (const gsl_vector *a, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans)
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
        size1 = a->size;
        size2 = 1;
    } else {
        size1 = 1;
        size2 = a->size;
    }

    // emulate matlab scientific format
    bool scifmt = 0;
    const int scimin = -3;
    const int scimax =  4;
    const double ZERO = DBL_EPSILON;
    double fabsmax = fabs (gsl_vector_max (a));
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
                    v = gsl_vector_get (a, i);
                else
                    v = gsl_vector_get (a, j);

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
gslu_vector_complex_printf (const gsl_vector_complex *a, const char *name)
{
    gslu_vector_complex_printfc (a, name, NULL, CblasNoTrans);
}

void
gslu_vector_complex_printfc (const gsl_vector_complex *a, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans)
{
    gsl_vector_const_view a_real = gsl_vector_complex_const_real (a);
    gsl_vector_const_view a_imag = gsl_vector_complex_const_imag (a);

    // figure out our terminal window size
    int ncols = 0;
    struct winsize w;
    if (ioctl (0, TIOCGWINSZ, &w)) {
        ncols = 80;
    } else {
        ncols = round (w.ws_col / 12.0);
    }

    if (ncols == 0)
        ncols = 80;

    if (name) {
        if (Trans==CblasNoTrans)
            printf ("%s =\n", name);
        else
            printf ("%s' =\n", name);
    }

    // default printf format
    char *fmt_real = calloc (20, sizeof(char));
    char *fmt_imag = calloc (20, sizeof(char));
    const char *imaginary = "i";

    int __default__ = 1;
    if (fmt == NULL) {
        strcpy (fmt_real, "%10.4f");
        strcpy (fmt_imag, "%+10.4fi");
    }
    else {
        __default__ = 0;
        strcpy (fmt_real, fmt);
        strcpy (fmt_imag, fmt);
        strcat (fmt_imag, imaginary);
    }

    size_t size1, size2;
    size1 = a->size;
    size2 = 2;


    // emulate matlab scientific format
    bool scifmt = 0;
    const int scimin = -3;
    const int scimax =  4;
    const double ZERO = DBL_EPSILON;
    double fabsmax = fabs (gsl_vector_max (&a_real.vector)) > fabs (gsl_vector_max (&a_imag.vector)) ? fabs (gsl_vector_max (&a_real.vector)) : fabs (gsl_vector_max (&a_imag.vector));
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
                if (j==0)
                    v = gsl_vector_get (&a_real.vector, i);
                else // (j==1)
                    v = gsl_vector_get (&a_imag.vector, i);

                if (scifmt)
                    v /= tens;

                if (__default__ && fabs (v) < ZERO) {
                    if (j==0)
                        printf (fmt_real, 0.0);
                    else //if (j==1)
                        printf (fmt_imag, 0.0);
                    printf (" ");
                }
                else {
                    if (j==0)
                        printf (fmt_real, v);
                    else {//if (j==1)
                        printf (fmt_imag, v);
                        printf (" ");
                    }
                }
            }
            if (Trans == CblasNoTrans)
                printf ("\n");
        }
    }

    free (fmt_real);
    free (fmt_imag);
}

// return 1 if all elements of a and b are exactly the same
int
gslu_vector_is_equal (const gsl_vector *a, const gsl_vector *b)
{
    assert (a->size == b->size);
    for (size_t i=0; i<a->size; i++) {
        double epsilon = gsl_vector_get (a, i) - gsl_vector_get (b, i);
        if (fabs (epsilon) > 1e-14)
            return 0;
    }
    return 1;
}


double
gslu_vector_circ_dist (const gsl_vector *a, const gsl_vector *b, const gslu_index *c)
{
    assert (a->size == b->size && 0 < c->size && c->size <= a->size);
    double dist2 = 0.0;
    size_t k = 0, c_k = gslu_index_get (c, k);
    for (size_t i=0; i<a->size; i++) {
        double delta = gsl_vector_get (a, i) - gsl_vector_get (b, i);
        if (i == c_k) {
            delta = gslu_math_minimized_angle (delta);
            c_k = ++k < c->size ? gslu_index_get (c, k) : c_k;
        }
        dist2 += delta*delta;
    }
    return sqrt (dist2);
}

double
gslu_vector_mahal_dist (const gsl_vector *a, const gsl_vector *b, const gsl_matrix *Sinv)
{
    assert (Sinv->size1 == Sinv->size2 && a->size == b->size && a->size == Sinv->size1);
    double dist2 = 0.0;
    for (size_t i=0; i<Sinv->size1; i++) {
        double ei = gsl_vector_get (a, i) - gsl_vector_get (b, i);
        for (size_t j=i; j<Sinv->size2; j++) {
            double ej = gsl_vector_get (a, j) - gsl_vector_get (b, j);
            if (i == j)
                dist2 += ei * gsl_matrix_get (Sinv, i, j) * ei;
            else
                dist2 += 2 * ei * gsl_matrix_get (Sinv, i, j) * ej;
        }
    }
    return sqrt (dist2);
}

double
gslu_vector_mahal_circ_dist (const gsl_vector *a, const gsl_vector *b, const gsl_matrix *Sinv, const gslu_index *c)
{
    assert (Sinv->size1 == Sinv->size2 && a->size == b->size && a->size == Sinv->size1 && 0 < c->size && c->size <= a->size);
    gsl_vector *nu = gsl_vector_alloc (a->size);
    size_t k = 0, c_k = gslu_index_get (c, k);
    for (size_t i=0; i<a->size; i++) {
        double delta = gsl_vector_get (a, i) - gsl_vector_get (b, i);
        if (i == c_k) {
            delta = gslu_math_minimized_angle (delta);
            c_k = ++k < c->size ? gslu_index_get (c, k) : c_k;
        }
        gsl_vector_set (nu, i, delta);
    }

    double dist2 = 0.0;
    for (size_t i=0; i<Sinv->size1; i++) {
        double ei = gsl_vector_get (nu, i);
        for (size_t j=i; j<Sinv->size2; j++) {
            double ej = gsl_vector_get (nu, j);
            if (i == j)
                dist2 += ei * gsl_matrix_get (Sinv, i, j) * ei;
            else
                dist2 += 2* ei * gsl_matrix_get (Sinv, i, j) * ej;
        }
    }
    gsl_vector_free (nu);
    return sqrt (dist2);
}


int
gslu_vector_reshape (gsl_matrix *A, const gsl_vector *a, CBLAS_TRANSPOSE_t TransA)
{
    assert (A->size1*A->size2 == a->size);
    switch (TransA) {
    case CblasNoTrans: // column order
        for (size_t j=0, k=0; j<A->size2; j++)
            for (size_t i=0; i<A->size1; i++, k++)
                gsl_matrix_set (A, i, j, gsl_vector_get (a, k));
        break;

    case CblasTrans: // row order
        for (size_t i=0, k=0; i<A->size1; i++)
            for (size_t j=0; j<A->size2; j++, k++)
                gsl_matrix_set (A, i, j, gsl_vector_get (a, k));
        break;

    case CblasConjTrans:
        GSL_ERROR ("CblasConjTrans not handled!", GSL_EUNIMPL);
        break;

    default:
        GSL_ERROR ("Unknown CBLAS_TRANSPOSE_t type!", GSL_EUNIMPL);
    }
    return GSL_SUCCESS;
}
