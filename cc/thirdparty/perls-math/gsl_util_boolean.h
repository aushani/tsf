#ifndef __PERLS_MATH_GSL_UTIL_BOOLEAN_H__
#define __PERLS_MATH_GSL_UTIL_BOOLEAN_H__

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector_ushort.h>

/**
 * @defgroup PerlsMathGsluBoolean GSL Util boolean
 * @brief GSL utility for boolean.
 * @ingroup PerlsMath
 * 
 * @{
 */

/** boolean is a wrapper for vector ulong */
#define gslu_boolean gsl_vector_ushort
/** boolean view */
#define gslu_boolean_view gsl_vector_ushort_view
/** const boolean view */
#define gslu_boolean_const_view gsl_vector_ushort_const_view

/**
 * Macro for creating a gslu_boolean_view object and data on the stack.
 * Use gslu macros to create vector elements on the stack with vector
 * views.  var will have subfields .data and .vector.  The advantage
 * is that elements created on the stack do not have to be manually
 * freed by the programmer, they automatically have limited scope
 */
#define GSLU_BOOLEAN_VIEW(var,i,...)                                    \
    struct {                                                            \
        unsigned long data[i];                                          \
        gsl_vector_ushort vector;                                       \
    } var = {__VA_ARGS__};                                              \
    {   /* _view has local scope */                                     \
        gsl_vector_ushort_view _view = gsl_vector_ushort_view_array (var.data, i); \
        var.vector = _view.vector;                                      \
    }


/** Index allocation */
#define gslu_boolean_alloc gsl_vector_ushort_alloc

/** boolean calloc. It initializes to identity boolean */
static inline gslu_boolean *
gslu_boolean_calloc (size_t n) // initialize to identity boolean
{
    gsl_vector_ushort *boolean = gsl_vector_ushort_alloc (n);
    for (size_t i=0; i<n; i++)
        gsl_vector_ushort_set (boolean, i, i);
    return boolean;
}

/** initializes to [0 1 2 3 4 .... ] */
static inline void
gslu_boolean_init (gslu_boolean *boolean)
{
    for (size_t i=0; i<boolean->size; i++)
        gsl_vector_ushort_set (boolean, i, i);
}

/** free an boolean vector */
static inline void
gslu_boolean_free (gslu_boolean *boolean)
{
    if (boolean)
        gsl_vector_ushort_free (boolean);
}

/** duplicate an boolean vector */
static inline gslu_boolean *
gslu_boolean_dup (const gslu_boolean *a)
{
    gslu_boolean *b = gslu_boolean_alloc (a->size);
    gsl_vector_ushort_memcpy (b, a);
    return b;
}

/* Accessing boolean elements */
/** get boolean elements */
#define gslu_boolean_get gsl_vector_ushort_get
/** set boolean elements */
#define gslu_boolean_set gsl_vector_ushort_set
/** ptr boolean elements */
#define gslu_boolean_ptr gsl_vector_ushort_ptr
/** const get boolean elements */
#define gslu_boolean_const_ptr gsl_vector_ushort_const_ptr

/* Initializing boolean elements */
/** set all boolean vector */
#define gslu_boolean_set_all gsl_vector_ushort_set_all
/** set zero boolean vector */
#define gslu_boolean_set_zero gsl_vector_ushort_set_zero
/** set basis boolean vector */
#define gslu_boolean_set_basis gsl_vector_ushort_set_basis

/* Reading and writing booleanes */
/** fwrite boolean vector */
#define gslu_boolean_fwrite gsl_vector_ushort_fwrite
/** fread boolean vector */
#define gslu_boolean_fread gsl_vector_ushort_fread
/** fprintf boolean vector */
#define gslu_boolean_fprintf gsl_vector_ushort_fprintf
/** fscanf boolean vector */
#define gslu_boolean_fscanf gsl_vector_ushort_fscanf

/* Index views */
/** subvector boolean vector */
#define gslu_boolean_subvector gsl_vector_ushort_subvector
/** const subvector boolean vector */
#define gslu_boolean_const_subvector gsl_vector_ushort_const_subvector
/** subvector with stride boolean vector */
#define gslu_boolean_subvector_with_stride gsl_vector_ushort_subvector_with_stride
/** const subvector with stride boolean vector */
#define gslu_boolean_const_subvector_with_stride gsl_vector_ushort_const_subvector_with_stride
/** boolean view array boolean vector */
#define gslu_boolean_view_array gsl_vector_ushort_view_array
/** const boolean view array boolean vector */
#define gslu_boolean_const_view_array gsl_vector_ushort_const_view_array
/** view array with stride boolean vector */
#define gslu_boolean_view_array_with_stride gsl_vector_ushort_view_array_with_stride
/** const view array with stride boolean vector */
#define gslu_boolean_const_view_array_with_stride gsl_vector_ushort_const_view_array_with_stride

/* Copying booleanes */
/** memcpy boolean vector */
#define gslu_boolean_memcpy gsl_vector_ushort_memcpy

/** swap boolean vector */
#define gslu_boolean_swap gsl_vector_ushort_swap

/* Exchanging elements */
/** swap elements boolean vector */
#define gslu_boolean_swap_elements gsl_vector_ushort_swap_elements

/** reverse boolean vector */
#define gslu_boolean_reverse gsl_vector_ushort_reverse

/* Index operations */
/** add boolean vector */
#define gslu_boolean_add gsl_vector_ushort_add
/** sub boolean vector */
#define gslu_boolean_sub gsl_vector_ushort_sub
/** mul boolean vector */
#define gslu_boolean_mul gsl_vector_ushort_mul
/** div boolean vector */
#define gslu_boolean_div gsl_vector_ushort_div
/** scale boolean vector */
#define gslu_boolean_scale gsl_vector_ushort_scale
/** add const boolean vector */
#define gslu_boolean_add_constant gsl_vector_ushort_add_constant

/* Finding maximum and minimum elements on booleanes */
/** max boolean vector */
#define gslu_boolean_max gsl_vector_ushort_max
/** min boolean vector */
#define gslu_boolean_min gsl_vector_ushort_min
/** minmax boolean vector */
#define gslu_boolean_minmax gsl_vector_ushort_minmax
/** max boolean boolean vector */
#define gslu_boolean_max_boolean gsl_vector_ushort_max_boolean
/** min boolean boolean vector */
#define gslu_boolean_min_boolean gsl_vector_ushort_min_boolean
/** minmax boolean boolean vector */
#define gslu_boolean_minmax_boolean gsl_vector_ushort_minmax_boolean

/* Index properties */
/** isnull boolean vector */
#define gslu_boolean_isnull gsl_vector_ushort_isnull
/** is pos def boolean vector */
#define gslu_boolean_ispos gsl_vector_ushort_ispos
/** is neg def boolean vector */
#define gslu_boolean_isneg gsl_vector_ushort_isneg
/** is non-neg def boolean vector */
#define gslu_boolean_isnonneg gsl_vector_ushort_isnonneg

/** Printing boolean vector */
void
gslu_boolean_printf (const gslu_boolean *i, const char *name);

/** custom version of printing boolean vector 
 *
 * @param fmt if it is NULL, then it defaults to "%f"
 * @param trans one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
gslu_boolean_printfc (const gslu_boolean *i, const char *name, const char *fmt, CBLAS_TRANSPOSE_t trans);

#endif // __PERLS_MATH_GSL_UTIL_BOOLEAN_H__
