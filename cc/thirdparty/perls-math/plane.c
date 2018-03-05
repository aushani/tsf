#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <gsl/gsl_rng.h>

#include "gsl_util.h"
#include "homogenous.h"
#include "ransac.h"
#include "plane.h"


#define RANSAC_SAMPLE_SIZE 3

void
plane_estim_3pt (const gsl_matrix *xyz, gsl_vector *coeff)
{
    assert (xyz->size1 == 3 && xyz->size2 == 3 && coeff->size == 4);

    // model: a*x + b*y + c*z + d = 0; subject to a^2 + b^2 + c^2 = 1

    gsl_vector_const_view xxx = gsl_matrix_const_row (xyz, 0);
    gsl_vector_const_view yyy = gsl_matrix_const_row (xyz, 1);
    gsl_vector_const_view zzz = gsl_matrix_const_row (xyz, 2);
    GSLU_VECTOR_VIEW (ones, 3, {1, 1, 1});
    GSLU_MATRIX_VIEW (A, 3, 3);

    // a = -det([ones(q,1) y_sample' z_sample']);
    gsl_matrix_set_col (&A.matrix, 0, &ones.vector);
    gsl_matrix_set_col (&A.matrix, 1, &yyy.vector);
    gsl_matrix_set_col (&A.matrix, 2, &zzz.vector);
    double a = -1.0 * gslu_matrix_det (&A.matrix);

    // b = -det([x_sample' ones(q,1) z_sample']);
    gsl_matrix_set_col (&A.matrix, 0, &xxx.vector);
    gsl_matrix_set_col (&A.matrix, 1, &ones.vector);
    gsl_matrix_set_col (&A.matrix, 2, &zzz.vector);
    double b = -1.0 * gslu_matrix_det (&A.matrix);

    // c = -det([x_sample' y_sample' ones(q,1)]);
    gsl_matrix_set_col (&A.matrix, 0, &xxx.vector);
    gsl_matrix_set_col (&A.matrix, 1, &yyy.vector);
    gsl_matrix_set_col (&A.matrix, 2, &ones.vector);
    double c = -1.0 * gslu_matrix_det (&A.matrix);

    // d = det([x_sample' y_sample' z_sample']);
    gsl_matrix_set_col (&A.matrix, 0, &xxx.vector);
    gsl_matrix_set_col (&A.matrix, 1, &yyy.vector);
    gsl_matrix_set_col (&A.matrix, 2, &zzz.vector);
    double d = gslu_matrix_det (&A.matrix);

    // coeff = [a b c d]';
    gsl_vector_set (coeff, 0, a);
    gsl_vector_set (coeff, 1, b);
    gsl_vector_set (coeff, 2, c);
    gsl_vector_set (coeff, 3, d);
    if (d < 0.0)
        gsl_vector_scale (coeff, -1.0/sqrt (a*a + b*b + c*c));
    else
        gsl_vector_scale (coeff, 1.0/sqrt (a*a + b*b + c*c));
}

void
plane_estim_svd (const gsl_matrix *xyz, gsl_vector *coeff, gsl_vector **error)
{
    assert (xyz->size1 == 3 && xyz->size2 >= 3 && coeff->size == 4);

    gsl_matrix *xyz_h = homogenize_alloc (xyz);
    gsl_matrix *A = gslu_matrix_transpose_alloc (xyz_h); // [xi yi zi 1]
    gslu_linalg_SV *SVD = gslu_linalg_SV_decomp_econ_alloc (A);

    if (SVD) {
        gsl_vector_view v = gsl_matrix_column (SVD->V, 3); // [a,b,c,d]
        const double a = gsl_vector_get (&v.vector, 0);
        const double b = gsl_vector_get (&v.vector, 1);
        const double c = gsl_vector_get (&v.vector, 2);
        const double d = gsl_vector_get (&v.vector, 3);
        gsl_vector_memcpy (coeff, &v.vector);
        if (d < 0.0)
            gsl_vector_scale (coeff, -1.0/sqrt (a*a + b*b + c*c));
        else
            gsl_vector_scale (coeff, 1.0/sqrt (a*a + b*b + c*c));

        if (error)
            *error = gslu_blas_mv_alloc (A, coeff); /* orthogonal error = (a*x+b*y+c*z+d)/sqrt(a^2+b^2+c^2)
                                                  note: a^2+b^2+c^2=1 */
    }   

    // clean up
    gslu_linalg_SV_free (SVD);
    gsl_matrix_free (xyz_h);
    gsl_matrix_free (A);
}

size_t
plane_estim_ransac (const gsl_matrix *xyz, double percent_outliers,
                    gsl_vector *coeff, gsl_vector **error, gslu_index **isel)
{
    // check the input size first
    assert (xyz->size1 == 3 && xyz->size2 >= 3 && coeff->size == 4);

    // set parameters: threshold, iteration min/max
    const double percent_inliers = 1.0-percent_outliers;
    const double thresh = 1.0E-4;         // value copied from matlab
    const double p = 0.995;               // confidence-level
    const size_t n_total  = xyz->size2;   // size of the set = N
    size_t n_inlier = 0;
    size_t itr_min = 100;
    size_t itr_max = ransac_adapt_trials (percent_inliers*n_total, n_total, p, RANSAC_SAMPLE_SIZE);

    size_t *sel = malloc (n_total * sizeof (*sel));
    size_t *sel_in = malloc (n_total * sizeof (*sel_in));
    gsl_vector *error_i = gsl_vector_alloc (n_total);
    if (error)
        *error = gsl_vector_alloc (n_total);


    GSLU_VECTOR_VIEW (coeff_i, 4);
    GSLU_INDEX_VIEW (rsel, RANSAC_SAMPLE_SIZE);
    GSLU_MATRIX_VIEW (xyz_samples, 3, RANSAC_SAMPLE_SIZE);
    gsl_rng *r = gslu_rand_rng_alloc ();

    for (size_t itr=0; (itr < itr_min) || (itr < itr_max); itr++) {
        // draw samples
        gslu_rand_index (r, &rsel.vector, n_total);
        gslu_matrix_selcol (&xyz_samples.matrix, xyz, &rsel.vector);

        // fit plane model
        plane_estim_3pt (&xyz_samples.matrix, &coeff_i.vector);
        gsl_vector_const_view abc_i = gsl_vector_const_subvector (&coeff_i.vector, 0, 3);
        const double d_i = gsl_vector_get (&coeff_i.vector, 3);

        // orthogonal error = (a*x + b*y + c*z + d) / sqrt (a^2 + b^2 +c^2)    note: a^2+b^2+c^2=1
        size_t n_in = 0;
        for (size_t i=0; i<n_total; i++) {
            gsl_vector_const_view xyz_i = gsl_matrix_const_column (xyz, i);
            double e = gslu_vector_dot (&xyz_i.vector, &abc_i.vector) + d_i;
            gsl_vector_set (error_i, i, e);
            if (fabs (e) < thresh)
                sel_in[n_in++] = i;
        }

        // check for consensus
        if (n_in > n_inlier) {
            n_inlier = n_in;
            gsl_vector_memcpy (coeff, &coeff_i.vector);
            itr_max = GSL_MIN (itr_max, ransac_adapt_trials (n_inlier, n_total, p, RANSAC_SAMPLE_SIZE));
            
            // only stuff if the user requests the following outputs
            if (error)
                gsl_vector_memcpy (*error, error_i);

            if (isel) {
                for (size_t i=0; i<n_inlier; i++)
                    sel[i] = sel_in[i];
            }
        }
    }

    // assign inlier set only if user requests it
    if (isel && n_inlier) {
        *isel = gslu_index_alloc (n_inlier);
        for (size_t i=0; i<n_inlier; i++)
            gslu_index_set (*isel, i, sel[i]);
    }

    // clean up
    gsl_rng_free (r);
    gsl_vector_free (error_i);
    free (sel);
    free (sel_in);

    return n_inlier;
}   

void
plane_ray_intersection (const gsl_vector *coeff, const gsl_matrix *n_rays, gsl_matrix *xyz)
{
    assert (gslu_matrix_is_same_size (n_rays, xyz) && coeff->size == 4);

    const double a = gsl_vector_get (coeff, 0);
    const double b = gsl_vector_get (coeff, 1);
    const double c = gsl_vector_get (coeff, 2);
    const double d = gsl_vector_get (coeff, 3);
    GSLU_VECTOR_VIEW (plane_normal, 3, {a, b, c});

    // returns 3xN matrix, each column contains an intersection point (xi,yi,zi)
    size_t npts = n_rays->size2;
    for (size_t i=0; i<npts; i++) {
        gsl_vector_view xyz_i = gsl_matrix_column (xyz, i);
        gsl_vector_const_view ray_i = gsl_matrix_const_column (n_rays, i);
        double t = -d / gslu_vector_dot (&ray_i.vector, &plane_normal.vector);
        gsl_vector_memcpy (&xyz_i.vector, &ray_i.vector);
        gsl_vector_scale (&xyz_i.vector, t);
    }
}
