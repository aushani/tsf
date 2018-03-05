#ifndef __PERLS_COMMON_TEXTREAD_H__
#define __PERLS_COMMON_TEXTREAD_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h> // needed for PRId64 macro
#include <glib.h>

#include "error.h"

/**
 * @defgroup PerlsCommonTextread Textread
 * @brief Library for writing textread compatible CSV files that can be loaded by Matlab.
 * @ingroup PerlsCommon
 * @include: perls-common/textread.h
 *
 * @see lcmlog-export
 *
 * @{
 */

#define TEXTREAD_MAX_NFIELDS 100000

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _textread textread_t;
struct _textread {
    char *csvlog_prefix;
    char *csvlog_dir;
    char *csvlog_fname;
    FILE *csvlog_fid;

    char    *srcid;
    GString *textread_string;
    GString *csvheader_string;
    GString *const_string;

    int  nfields;
    char **lvalue;
    char **rvalue;

    bool mutable;
};

/**
 * \cond PRIVATE
 */
char *
textread_format (const char *format);
/**
 * \endcond
 */


/**
 * @brief Macro that adds a numeric field for export.
 * @param tr  textread_t struct pointer.
 * @param name Field's string name.
 * @param format Field's format string for export (e.g. <code>"%f", "%d", "%s", "%"PRId64</code>)
 * @param data Field's numeric value for conversion.
 */
#define TEXTREAD_ADD_FIELD(tr, name, format, data)                      \
    {                                                                   \
        if (tr->mutable) {                                              \
            if (tr->nfields < TEXTREAD_MAX_NFIELDS) {                   \
                tr->lvalue[tr->nfields] = strdup (name);                \
                tr->rvalue[tr->nfields] = textread_format (format);     \
                tr->nfields++;                                          \
            }                                                           \
            else {                                                      \
                ERROR ("max nfields [%d] exceeded on field %s", TEXTREAD_MAX_NFIELDS, tr->srcid); \
                exit (EXIT_FAILURE);                                    \
            }                                                           \
        }                                                               \
        else                                                            \
            fprintf (tr->csvlog_fid, format ";", data);                 \
    }

/**
 * @brief Macro that adds a string field for export.
 * @param tr  textread_t struct pointer.
 * @param name Field's string name.
 * @param format Field's format string for export (e.g. <code>"%f", "%d", "%s", "%"PRId64</code>).
 * @param data Field's string value for conversion.
 */
#define TEXTREAD_ADD_STRFIELD(tr, name, format, str)                    \
    {                                                                   \
        if (tr->mutable) {                                              \
            if (tr->nfields < TEXTREAD_MAX_NFIELDS) {                   \
                tr->lvalue[tr->nfields] = strdup (name);                \
                tr->rvalue[tr->nfields] = textread_format (format);     \
                tr->nfields++;                                          \
            }                                                           \
            else {                                                      \
                ERROR ("max nfields [%d] exceeded on field %s", TEXTREAD_MAX_NFIELDS, tr->srcid); \
                exit (EXIT_FAILURE);                                    \
            }                                                           \
        }                                                               \
        else                                                            \
            fprintf (tr->csvlog_fid, "%s;", str);                       \
    }

/**
 * @brief Macro that adds an integer constant value to the CSV file.
 * @param tr textread sruct pointer.
 * @param name Constant's string name.
 * @param value Constant's value.
 */
#define TEXTREAD_ADD_CONST(tr, name, value)                             \
    {                                                                   \
        if (tr->mutable) {                                              \
            typeof(value) _v = value - (int64_t) value;                 \
            if (_v<0 || _v>0)   /* floating point */                    \
                g_string_append_printf (tr->const_string, "nav_t.%s.const.%s=%.10f;\n", tr->srcid, name, (double) value); \
            else                /* integer */                           \
                g_string_append_printf (tr->const_string, "nav_t.%s.const.%s=%"PRId64";\n", tr->srcid, name, (int64_t) value); \
        }                                                               \
    }



/**
 * @brief Allocates an empty textread struct.
 * @param cvslog_dir Output CSV log directory.
 * @param csvlog_prefix Prefix to preprend to all output CSV log files.
 * @param srcid Unique source identifier string.
 */
textread_t *
textread_create (const char *csvlog_dir, const char *csvlog_prefix, const char *srcid);

/**
 * @brief Closes the exported CSV file and frees the textread struct.
 */
void
textread_destroy (textread_t *tr);


/**
 * @brief Call this function <b>before</b> calling any of the TEXTREAD_ADD_FIELD(), TEXTREAD_ADD_STRFIELD(), or TEXTREAD_ADD_CONST() macros.
 * @param tr textread struct pointer.
 */
void
textread_start (textread_t *tr);

/**
 * @brief Call this function <b>after</b> done calling the TEXTREAD_ADD_FIELD(), TEXTREAD_ADD_STRFIELD(), or TEXTREAD_ADD_CONST() macros.
 * @param tr textread struct pointer.
 */
void
textread_stop (textread_t *tr);

/**
 * @brief Writes a Matlab m-file that will automatically load all exported CSV files into the workspace.
 * @param tra Array of textread structs.
 * @param tra_len Length of textread array.
 * @param mfile_dir Directory to use for output m-file.
 * @param mfile_fname Filename to use for output m-file.
 */
void
textread_gen_matlab (textread_t *tra[], size_t tra_len, const char *mfile_dir, const char *mfile_fname);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif //__PERLS_COMMON_TEXTREAD_H__
