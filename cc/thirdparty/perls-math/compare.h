#ifndef __PERLS_MATH_COMPARE_H__
#define __PERLS_MATH_COMPARE_H__

/**
 * @defgroup PerlsMathCompare compare
 * @brief  fltcmp(), dblcmp(), dbllcmp() relicate the functionality of
 * strcmp(), but for floats.
 * The strcmp() function compares the two strings s1 and s2.  It
 * returns an integer less than, equal to, or greater than zero if s1
 * is found, respectively, to be less than, to match, or be greater
 * than s2.
 *
 * @ingroup PerlsMath
 * 
 * @{
 */

#include <math.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * compare two float variables
 * @return 0 if same, 1 if f1 is larger, -1 if f2 is larger
 */
static inline int
fltcmp (float f1, float f2)
{
    float epsilon = f1-f2;
    if (epsilon < 0.0)
        return -1;
    else if (epsilon > 0.0)
        return  1;
    else
        return  0;
}

/**
 * compare two double variables
 * @return 0 if same, 1 if d1 is larger, -1 if d2 is larger
 */
static inline int
dblcmp (double d1, double d2)
{
    double epsilon = d1-d2;
    if (epsilon < 0.0)
        return -1;
    else if (epsilon > 0.0)
        return  1;
    else
        return  0;
}

/**
 * compare two long double variables
 * @return 0 if same, 1 if d1 is larger, -1 if d2 is larger
 */
static inline int
dbllcmp (long double d1, long double d2)
{
    long double epsilon = d1-d2;
    if (epsilon < 0.0)
        return -1;
    else if (epsilon > 0.0)
        return  1;
    else
        return  0;
}

#ifdef __cplusplus
}
#endif

#endif //__PERLS_MATH_COMPARE_H__
