#ifndef __PERLS_MATH_GSL_UTIL_INDEX_H__
#define __PERLS_MATH_GSL_UTIL_INDEX_H__

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector_ulong.h>

/**
 * @defgroup PerlsMathGsluIndex GSL Util Index
 * @brief GSL utility for index.
 * @ingroup PerlsMath
 * 
 * @{
 */

/** index is a wrapper for vector ulong */
#define gslu_index gsl_vector_ulong
/** index view */
#define gslu_index_view gsl_vector_ulong_view
/** const index view */
#define gslu_index_const_view gsl_vector_ulong_const_view

/**
 * Macro for creating a gslu_index_view object and data on the stack.
 * Use gslu macros to create vector elements on the stack with vector
 * views.  var will have subfields .data and .vector.  The advantage
 * is that elements created on the stack do not have to be manually
 * freed by the programmer, they automatically have limited scope
 */
#define GSLU_INDEX_VIEW(var,i,...)                                      \
    struct {                                                            \
        unsigned long data[i];                                          \
        gsl_vector_ulong vector;                                        \
    } var = {__VA_ARGS__};                                              \
    {   /* _view has local scope */                                     \
        gsl_vector_ulong_view _view = gsl_vector_ulong_view_array (var.data, i); \
        var.vector = _view.vector;                                      \
    }


/** Index allocation */
#define gslu_index_alloc gsl_vector_ulong_alloc

/** index calloc. It initializes to identity index */
static inline gslu_index *
gslu_index_calloc (size_t n) // initialize to identity index
{
    gsl_vector_ulong *index = gsl_vector_ulong_alloc (n);
    for (size_t i=0; i<n; i++)
        gsl_vector_ulong_set (index, i, i);
    return index;
}

/** initializes to [0 1 2 3 4 .... ] */
static inline void
gslu_index_init (gslu_index *index)
{
    for (size_t i=0; i<index->size; i++)
        gsl_vector_ulong_set (index, i, i);
}

/** free an index vector */
static inline void
gslu_index_free (gslu_index *index)
{
    if (index)
        gsl_vector_ulong_free (index);
}

/** duplicate an index vector */
static inline gslu_index *
gslu_index_dup (const gslu_index *a)
{
    gslu_index *b = gslu_index_alloc (a->size);
    gsl_vector_ulong_memcpy (b, a);
    return b;
}

/* Accessing index elements */
/** get index elements */
#define gslu_index_get gsl_vector_ulong_get
/** set index elements */
#define gslu_index_set gsl_vector_ulong_set
/** ptr index elements */
#define gslu_index_ptr gsl_vector_ulong_ptr
/** const get index elements */
#define gslu_index_const_ptr gsl_vector_ulong_const_ptr

/* Initializing index elements */
/** set all index vector */
#define gslu_index_set_all gsl_vector_ulong_set_all
/** set zero index vector */
#define gslu_index_set_zero gsl_vector_ulong_set_zero
/** set basis index vector */
#define gslu_index_set_basis gsl_vector_ulong_set_basis

/* Reading and writing indexes */
/** fwrite index vector */
#define gslu_index_fwrite gsl_vector_ulong_fwrite
/** fread index vector */
#define gslu_index_fread gsl_vector_ulong_fread
/** fprintf index vector */
#define gslu_index_fprintf gsl_vector_ulong_fprintf
/** fscanf index vector */
#define gslu_index_fscanf gsl_vector_ulong_fscanf

/* Index views */
/** subvector index vector */
#define gslu_index_subvector gsl_vector_ulong_subvector
/** const subvector index vector */
#define gslu_index_const_subvector gsl_vector_ulong_const_subvector
/** subvector with stride index vector */
#define gslu_index_subvector_with_stride gsl_vector_ulong_subvector_with_stride
/** const subvector with stride index vector */
#define gslu_index_const_subvector_with_stride gsl_vector_ulong_const_subvector_with_stride
/** index view array index vector */
#define gslu_index_view_array gsl_vector_ulong_view_array
/** const index view array index vector */
#define gslu_index_const_view_array gsl_vector_ulong_const_view_array
/** view array with stride index vector */
#define gslu_index_view_array_with_stride gsl_vector_ulong_view_array_with_stride
/** const view array with stride index vector */
#define gslu_index_const_view_array_with_stride gsl_vector_ulong_const_view_array_with_stride

/* Copying indexes */
/** memcpy index vector */
#define gslu_index_memcpy gsl_vector_ulong_memcpy

/** swap index vector */
#define gslu_index_swap gsl_vector_ulong_swap

/* Exchanging elements */
/** swap elements index vector */
#define gslu_index_swap_elements gsl_vector_ulong_swap_elements

/** reverse index vector */
#define gslu_index_reverse gsl_vector_ulong_reverse

/* Index operations */
/** add index vector */
#define gslu_index_add gsl_vector_ulong_add
/** sub index vector */
#define gslu_index_sub gsl_vector_ulong_sub
/** mul index vector */
#define gslu_index_mul gsl_vector_ulong_mul
/** div index vector */
#define gslu_index_div gsl_vector_ulong_div
/** scale index vector */
#define gslu_index_scale gsl_vector_ulong_scale
/** add const index vector */
#define gslu_index_add_constant gsl_vector_ulong_add_constant

/* Finding maximum and minimum elements on indexes */
/** max index vector */
#define gslu_index_max gsl_vector_ulong_max
/** min index vector */
#define gslu_index_min gsl_vector_ulong_min
/** minmax index vector */
#define gslu_index_minmax gsl_vector_ulong_minmax
/** max index index vector */
#define gslu_index_max_index gsl_vector_ulong_max_index
/** min index index vector */
#define gslu_index_min_index gsl_vector_ulong_min_index
/** minmax index index vector */
#define gslu_index_minmax_index gsl_vector_ulong_minmax_index

/* Index properties */
/** isnull index vector */
#define gslu_index_isnull gsl_vector_ulong_isnull
/** is pos def index vector */
#define gslu_index_ispos gsl_vector_ulong_ispos
/** is neg def index vector */
#define gslu_index_isneg gsl_vector_ulong_isneg
/** is non-neg def index vector */
#define gslu_index_isnonneg gsl_vector_ulong_isnonneg

/** Printing index vector */
void
gslu_index_printf (const gslu_index *i, const char *name);

/** custom version of printing index vector 
 *
 * @param fmt if it is NULL, then it defaults to "%f"
 * @param trans one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
gslu_index_printfc (const gslu_index *i, const char *name, const char *fmt, CBLAS_TRANSPOSE_t trans);

#endif // __PERLS_MATH_GSL_UTIL_INDEX_H__
