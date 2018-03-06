#ifndef __PERLS_MATH_DM_H__
#define __PERLS_MATH_DM_H__

/**
 * @defgroup PerlsMathDM dm-trans
 * @brief Utility on direction-magnitude (dm)
 * @ingroup PerlsMath
 * 
 * @{
 */

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

/** 
 * translation vector to direction-magnitude.
 * function from matlab van code (TRANS2DM)
 *
 * It also returns the Jacobian of this transformation
 * which is useful for propogating 1st order covariances e.g.
 * \f$Cov_b = J*Cov_t*J^T\f$
 *
 * @param t original translation vector
 * @param b bearing (returned). [azimuth, elevation, magnitude]
 * @param J jacobian (returned). No jacobian returned if NULL provided.
 */
void
dm_trans2dm (const double t[3], double b[3], double J[9]);

/**
 * dm_trans2dm in gsl format
 * @see dm_trans2dm
 */
void
dm_trans2dm_gsl (const gsl_vector *t, gsl_vector *b, gsl_matrix *J);

/**
 * dm_trans2dm using math.h instead of fasttrig, when very accurate computations are critical.
 */
void
dm_trans2dm_cmath (const double t[3], double b[3], double J[9]);

/**
 * translation vector to direction-magnitude.
 * function from matlab van code (TRANS2DM)
 *
 * It also returns the Jacobian of this transformation
 * which is useful for propogating 1st order covariances e.g.
 * \f$Cov_b = J*Cov_t*J^T\f$
 *
 * @param b original bearing. [azimuth, elevation, magnitude]
 * @param t translation vector (returned).
 * @param J jacobian (returned). No jacobian returned if NULL provided.
*/
void
dm_dm2trans (const double b[3], double t[3], double J[9]);

/**
 * dm_dm2trans in gsl format
 * @see dm_dm2trans
 */
void
dm_dm2trans_gsl (const gsl_vector * b, gsl_vector *t, gsl_matrix *J);

/**
 * dm_dm2trans using math.h instead of fasttrig, when very accurate computations are critical.
 */
void
dm_dm2trans_cmath (const double b[3], double t[3], double J[9]);

/**
 * Converts 6 dof to 5 dof.
 * Given 6 dof pose (xyzrph) and related covariance
 * returns 5 dof pose and related covariance
 * when J is non-null, return internal jacobian for later use
 *
 * @param x 6 dof pose [x,y,z,r,p,h]
 * @param cov covriance for 6 dof pose
 * @param dmx 5 dof pose [azimuth, eleveation, r, p, h] (returned)
 * @param dmcov covriance for 5 dof pose
 * @param J jacobian used in the calculation. No jacobian returned if NULL provided.
 */
void
dm_trans2dm_pose_cov (const gsl_vector *x, const gsl_matrix *cov, 
                      gsl_vector *dmx, gsl_matrix *dmcov, gsl_matrix *J);

#endif // __PERLS_MATH_DM_H__

