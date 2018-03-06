#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "ransac.h"

size_t
ransac_adapt_trials (size_t n_inlier, size_t n_total, double p, size_t s)
{
    // fraction of outliers
    double epsilon = 1.0 - ((double)n_inlier)/((double)n_total);

    /* fraction of inliers
     * (1-epsilon)
     *
     * probability of drawing s inliers
     * (1-epsilon)^s
     *
     * probability of drawing a set with one or more outliers
     * 1-(1-epsilon)^s
     *
     * probability of drawing a set with one or more outliers N times
     * (1-(1-epsilon)^s)^N
     *
     * probability of drawing at least one set of s inliers
     * p = 1-(1-(1-epsilon)^s)^N
     */

    // calculate number of trials required to achieve desired confidence level
    // (1-(1-epsilon)^s)^N = 1-p
    const double eps = DBL_EPSILON;
    return log10 (1.0-p) / (log10 (1.0 - pow (1.0-epsilon, s))+eps);
}
