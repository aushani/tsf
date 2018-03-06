#include <string.h>
#include <assert.h>

#include "gsl_util.h"

#include "fasttrig.h"
#include "dm.h"


void
dm_trans2dm (const double t[3], double b[3], double J[9])
{
    double eps = 2.2204e-16;    // from matlab
    
    // compute b
    const double x = t[0]; // x = mag*cos(elev)*cos(azim)
    const double y = t[1]; // y = mag*cos(elev)*sin(azim)
    const double z = t[2]; // z = mag*sin(elev)

    const double azim = fatan2 (y, x);
    const double elev = fatan2 (z, sqrt (x*x+y*y));
    const double mag = sqrt (x*x+y*y+z*z);

    b[0] = azim; b[1] = elev; b[2] = mag;

    if(J != NULL) { // jacobian needed
        double alpha = sqrt (x*x+y*y);
        if (fabs (alpha) < eps)
            alpha = alpha+eps; // avoid division by zero

        /* analytical expression for jacobian
         * J = [-y/alpha^2,          x/alpha^2,         0; ...
         *      -z*x/(alpha*mag^2), -z*y/(alpha*mag^2), alpha/mag^2; ...
         *       x/mag,              y/mag,             z/mag];
         */
        J[0]=-y/(alpha*alpha);     J[1]=x/(alpha*alpha);      J[2]=0;
        J[3]=-z*x/(alpha*mag*mag); J[4]=-z*y/(alpha*mag*mag); J[5]=alpha/(mag*mag);
        J[6]=x/mag;                J[7]=y/mag;                J[8]=z/mag;
    }
}

void
dm_trans2dm_gsl (const gsl_vector *t, gsl_vector *b, gsl_matrix *J)
{
    assert (t->size == 3 && b->size == 3);
    double t_data[3] = {t->data[0], t->data[t->stride], t->data[2*t->stride]};
    double b_data[3];

    if (J) {
        assert (J->size1 == 3 && J->size2 == 3 && J->tda == 3);
        dm_trans2dm (t_data, b_data, J->data); 
    } else {
        dm_trans2dm (t_data, b_data, NULL); 
    }

    b->data[0] = b_data[0];
    b->data[b->stride] = b_data[1];
    b->data[2*b->stride] = b_data[2];
}

void
dm_trans2dm_cmath (const double t[3], double b[3], double J[9])
{
    double eps = 2.2204e-16;    // from matlab
    
    // compute b
    const double x = t[0]; // x = mag*cos(elev)*cos(azim)
    const double y = t[1]; // y = mag*cos(elev)*sin(azim)
    const double z = t[2]; // z = mag*sin(elev)

    const double azim = atan2 (y, x);
    const double elev = atan2 (z, sqrt (x*x+y*y));
    const double mag = sqrt (x*x+y*y+z*z);

    b[0] = azim; b[1] = elev; b[2] = mag;

    if(J != NULL) { // jacobian needed
        double alpha = sqrt (x*x+y*y);
        if (fabs (alpha) < eps)
            alpha = alpha+eps; // avoid division by zero

        /* analytical expression for jacobian
         * J = [-y/alpha^2,          x/alpha^2,         0; ...
         *      -z*x/(alpha*mag^2), -z*y/(alpha*mag^2), alpha/mag^2; ...
         *       x/mag,              y/mag,             z/mag];
         */
        J[0]=-y/(alpha*alpha);     J[1]=x/(alpha*alpha);      J[2]=0;
        J[3]=-z*x/(alpha*mag*mag); J[4]=-z*y/(alpha*mag*mag); J[5]=alpha/(mag*mag);
        J[6]=x/mag;                J[7]=y/mag;                J[8]=z/mag;
    }
}

void
dm_trans2dm_pose_cov (const gsl_vector *x, const gsl_matrix *cov, 
                      gsl_vector *dmx, gsl_matrix *dmcov, gsl_matrix *J)
{
    assert (x->size == 6 && dmx->size == 5 && cov->size1 == 6 && cov->size2 == 6
            && dmcov->size1 == 5 && dmcov->size2 == 5);

    gsl_vector_const_view t = gsl_vector_const_subvector (x, 0, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (x, 3, 3);
    gsl_vector_view rph_dm = gsl_vector_subvector (dmx, 2, 3);
    gsl_vector_memcpy (&rph_dm.vector, &rph.vector);

    GSLU_VECTOR_VIEW (b, 3);
    GSLU_MATRIX_VIEW (work, 5, 6);
    GSLU_MATRIX_VIEW (Jdm, 3, 3);

    if (J) {
        assert (J->size1 == 5 && J->size2 == 6);

        // set pose
        dm_trans2dm_gsl (&t.vector, &b.vector, &Jdm.matrix);
        gsl_vector_set (dmx, 0, gsl_vector_get (&b.vector, 0));
        gsl_vector_set (dmx, 1, gsl_vector_get (&b.vector, 1));

        // set covariance
        gsl_matrix_view Jtemp_sub = gsl_matrix_submatrix (J, 2, 3, 3, 3);
        gsl_matrix_set_identity (&Jtemp_sub.matrix);
        gsl_matrix_view Jdm_sub = gsl_matrix_submatrix (&Jdm.matrix, 0, 0, 2, 3);
        Jtemp_sub = gsl_matrix_submatrix (J, 0, 0, 2, 3);
        gsl_matrix_memcpy (&Jtemp_sub.matrix, &Jdm_sub.matrix);
        gslu_blas_mmmT (dmcov, J, cov, J, &work.matrix);

    }
    else {
        GSLU_MATRIX_VIEW (Jtemp, 5, 6);

        // set pose
        dm_trans2dm_gsl (&t.vector, &b.vector, &Jdm.matrix);
        gsl_vector_set (dmx, 0, gsl_vector_get (&b.vector, 0));
        gsl_vector_set (dmx, 1, gsl_vector_get (&b.vector, 1));

        // set covariance
        gsl_matrix_view Jtemp_sub = gsl_matrix_submatrix (&Jtemp.matrix, 2, 3, 3, 3);
        gsl_matrix_set_identity (&Jtemp_sub.matrix);
        gsl_matrix_view Jdm_sub = gsl_matrix_submatrix (&Jdm.matrix, 0, 0, 2, 3);
        Jtemp_sub = gsl_matrix_submatrix (&Jtemp.matrix, 0, 0, 2, 3);
        gsl_matrix_memcpy (&Jtemp_sub.matrix, &Jdm_sub.matrix);
        gslu_blas_mmmT (dmcov, &Jtemp.matrix, cov, &Jtemp.matrix, &work.matrix);
    }


}

void
dm_dm2trans (const double b[3], double t[3], double J[9])
{
    // convert direction-magnitude to translation vector
    const double azim = b[0];
    const double elev = b[1];
    const double mag  = b[2];

    double se, ce, sa, ca;
    fsincos (elev, &se, &ce);
    fsincos (azim, &sa, &ca);

    double tz = mag*se;
    double tx = mag*ce*ca;
    double ty = mag*ce*sa;

    t[0]=tx; t[1]=ty; t[2]=tz;

    if (J != NULL) {
        /* analytical expression for jacobian matrix
         * J = [-mag*ce*sa, -mag*se*ca, ce*ca; ...
         *       mag*ce*ca, -mag*se*sa, ce*sa; ...
         *               0,     mag*ce,    se];
         */
        J[0]=-mag*ce*sa; J[1]=-mag*se*ca; J[2]=ce*ca;
        J[3]= mag*ce*ca; J[4]=-mag*se*sa; J[5]=ce*sa;
        J[6]= 0;         J[7]= mag*ce;    J[8]=se;
    }
}

void
dm_dm2trans_gsl (const gsl_vector * b, gsl_vector *t, gsl_matrix *J)
{
    assert (t->size == 3 && b->size == 3);
    double b_data[3] = {b->data[0], b->data[b->stride], b->data[2*b->stride]};
    double t_data[3];

    if (J) {
        assert (J->size1 == 3 && J->size2 == 3 && J->tda == 3);
        dm_dm2trans (b_data, t_data, J->data); 
    } else {
        dm_dm2trans (b_data, t_data, NULL); 
    }

    t->data[0] = t_data[0];
    t->data[t->stride] = t_data[1];
    t->data[2*t->stride] = t_data[2];

}

void
dm_dm2trans_cmath (const double b[3], double t[3], double J[9])
{
    // convert direction-magnitude to translation vector
    const double azim = b[0];
    const double elev = b[1];
    const double mag  = b[2];

    double se, ce, sa, ca;
    se = sin (elev); ce = cos (elev);
    sa = sin (azim); ca = cos (azim);

    double tz = mag*se;
    double tx = mag*ce*ca;
    double ty = mag*ce*sa;

    t[0]=tx; t[1]=ty; t[2]=tz;

    if (J != NULL) {
        /* analytical expression for jacobian matrix
         * J = [-mag*ce*sa, -mag*se*ca, ce*ca; ...
         *       mag*ce*ca, -mag*se*sa, ce*sa; ...
         *               0,     mag*ce,    se];
         */
        J[0]=-mag*ce*sa; J[1]=-mag*se*ca; J[2]=ce*ca;
        J[3]= mag*ce*ca; J[4]=-mag*se*sa; J[5]=ce*sa;
        J[6]= 0;         J[7]= mag*ce;    J[8]=se;
    }
}
