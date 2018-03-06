#ifndef __PERLS_MATH_RANSAC_H__
#define __PERLS_MATH_RANSAC_H__

/*
  %ADAPT_TRIALS  Number of trials required to achieve an outlier-free sample set.
  %  N = ADAPT_TRIALS(n_inlier,n_total,p,s) calculates number of
  %  trials needed to get a outlier-free sample set of size s with
  %  probability p from a set of n_total measurements.
  %
  %  Reference: Algorithm 3.5 Hartley/Zisserman
  %
  %-----------------------------------------------------------------
  %    History:
  %    Date            Who         What
  %    -----------     -------     -----------------------------
  %    12-2002         op          Created and written.
  %    12/20/2002      rme         Renamed to adapt_trials.m and
  %                                commented code.
  %    05-02-2010      rme         Ported to C.
*/
size_t
ransac_adapt_trials (size_t n_inlier, size_t n_total, double p, size_t s);

#endif //__PERLS_MATH_RANSAC_H__
