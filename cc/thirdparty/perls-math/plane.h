#ifndef __PERLS_MATH_PLANE_H__
#define __PERLS_MATH_PLANE_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "gsl_util_index.h"

#ifdef __cplusplus
extern "C" {
#endif


/* finds the equation of the plane a*x + b*y + c*z + d = 0 given 3 points
 * xyz - 3 x N matrix of points, here N = 3
 * coeff - placeholder for plane coefficients [a,b,c,d]
 */
void
plane_estim_3pt (const gsl_matrix *xyz, gsl_vector *coeff);

/* finds the least-squares equation of the plane a*x + b*y + c*z + d = 0 using SVD
 * xyz - 3 x N matrix of points
 * coeff - placeholder for plane coefficients [a,b,c,d]
 * error - orthogonal error vector, set to NULL if don't want to compute it
 */
void
plane_estim_svd (const gsl_matrix *xyz, gsl_vector *coeff, gsl_vector **error);

/* given number of 3d points
 * fit a plane to it and return the plane coefficients
 * returns number of inliers on success, -1 on error
 */
size_t
plane_estim_ransac (const gsl_matrix *xyz,       /* 3xN matrix containing 3D points */
                    double percent_outliers,     /* e.g. 20% = 0.2 */
                    gsl_vector *coeff,           /* plane coeff [a b c d], ax+by+cz+d=0 to be returned */
                    gsl_vector **error,          /* orthogonal error vector, set to NULL if don't want to compute it */
                    gslu_index **isel);          /* N-vector of inlier indices, set to NULL if you don't want it */

/* given plane coefficient [a b c d] and N number of rays
 * find intersection point between the plane and rays
 * http://www.siggraph.org/education/materials/HyperGraph/raytrace/rayplane_intersection.htm
 * 
 * returns 3xN matrix, each column contains an intersection point (xi,yi,zi)
 */
void
plane_ray_intersection (const gsl_vector *coeff,    /* 4x1 vector = [a b c d], ax+by+cz+d=0 */
                        const gsl_matrix *n_rays,   /* 3xN matrix, each column defines a ray = (v1,v2,v3) */
                        gsl_matrix *xyz);           /* 3xN matrix, each column defines the xyz ray intersection point */

#ifdef __cplusplus
}
#endif

#endif // __PERLS_MATH_PLANE_H__
