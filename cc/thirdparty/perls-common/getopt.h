#ifndef __PERLS_COMMON_GETOPT_H__
#define __PERLS_COMMON_GETOPT_H__

#include <glib.h>

/**
 * @defgroup PerlsCommonGetopt Command Line Options
 * @brief Wrapper to POSIX getopt.
 * @details PeRL's wrapper to POSIX getopt.
 * @ingroup PerlsCommon
 * @include: perls-common/getopt.h
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct getopt_option getopt_option_t;
struct getopt_option
{
    char *sname;
    char *lname;
    char *svalue;
    
    char *help;
    int type;
    int found;
	
    int spacer;
    int description;
    int example;
};

typedef struct getopt getopt_t;
struct getopt
{
    char        *exename;
    GHashTable  *lopts;
    GHashTable  *sopts;
    GPtrArray   *extraargs;
    GPtrArray   *options;
};

/**
 * Instantiates a new getopt_t instance.  Don't use malloc() or glib types will
 * not be created correctly.
 */
getopt_t *
getopt_create (void);

void 
getopt_destroy (getopt_t *gopt);

int 
getopt_parse (getopt_t *gopt, int argc, char *argv[], int showErrors);

/* set extraargs=NULL, to print default usage */
void 
getopt_do_usage (getopt_t *gopt, const char *extraargs);

void
getopt_add_description (getopt_t *gopt, const char *s);

/* void */
/* getopt_add_example (getopt_t *gopt, const char *s); */
#define getopt_add_example(gopt, ...)                                   \
    {                                                                   \
        getopt_option_t *goo = (getopt_option_t *) calloc (1, sizeof (*goo)); \
        goo->example = 1;                                               \
        goo->help = g_strdup_printf (__VA_ARGS__);                      \
        g_ptr_array_add (gopt->options, goo);                           \
    }

void 
getopt_add_spacer (getopt_t *gopt, const char *s);

void
getopt_add_help (getopt_t *gopt, const char *help);

/**
 * add boolean
 *
 * @param gopt   getopt_t instance
 * @param sopt   short name of the option
 * @param lname  long name used in opt parsing
 * @param def    default value
 * @param help   detailed description shown on help
 *
 * example usage:
 * @code
 *   getopt_add_bool (gopt, 'v', "visualize", 0, "visualize?");
 * @endcode
 * add boolean option with default value 0 (false) 
 *
 * @see getopt_get_bool 
 */
void
getopt_add_bool (getopt_t *gopt, char sopt, const char *lname, int def, const char *help);

/**
 * add int
 *
 * @param gopt   getopt_t instance
 * @param sopt   short name of the option
 * @param lname  long name used in opt parsing
 * @param def    default value
 * @param help   detailed description shown on help
 *
 * example usage:
 * @code
 *   getopt_add_int (gopt, 'o', "option", "1", "option on?");
 * @endcode 
 * add int option with default value 1
 *
 * @see getopt_get_int 
 */
void
getopt_add_int (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help);

/**
 * add long
 *
 * @param gopt   getopt_t instance
 * @param sopt   short name of the option
 * @param lname  long name used in opt parsing
 * @param def    default value
 * @param help   detailed description shown on help
 *
 * example usage:
 * @code
 *   getopt_add_long (gopt, 't', "time", "1234567890", "time");
 * @endcode 
 * add long option with default value 1234567890
 *
 * @see getopt_get_long 
 */
void
getopt_add_long (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help);

/**
 * add double
 *
 * @param gopt   getopt_t instance
 * @param sopt   short name of the option
 * @param lname  long name used in opt parsing
 * @param def    default value
 * @param help   detailed description shown on help
 *
 * example usage:
 * @code
 *   getopt_add_double (gopt, 'x', "position", "1.0", "x-dir position");
 * @endcode 
 * add double option with default value 1.0
 *
 * @see getopt_get_double 
 */
void
getopt_add_double (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help);

/**
 * add string
 *
 * @param gopt   getopt_t instance
 * @param sopt   short name of the option
 * @param lname  long name used in opt parsing
 * @param def    default value
 * @param help   detailed description shown on help
 *
 * example usage:
 * @code
 *   getopt_add_string (gopt, 'f', "fname", "./test.dat", "filename");
 * @endcode 
 * add string option with default value "./test.dat"
 *
 * @see getopt_get_string
 */
void
getopt_add_string (getopt_t *gopt, char sopt, const char *lname, const char *def, const char *help);

/**
 * get bool
 *
 * @param getopt  getopt_t instance
 * @param lname   long name used in opt parsing
 *
 * example usage:
 * @code
 *   double val = getopt_get_bool (gopt, "visualize");
 * @endcode 
 * get bool option with name "visualize"
 *
 * @see getopt_add_bool 
 */
int
getopt_get_bool (getopt_t *getopt, const char *lname);

/**
 * get int
 *
 * @param getopt  getopt_t instance
 * @param lname   long name used in opt parsing
 *
 * example usage:
 * @code
 *   int val = getopt_get_int (gopt, "option");
 * @endcode 
 * get int option with name "option"
 *
 * @see getopt_add_int 
 */
int
getopt_get_int (getopt_t *getopt, const char *lname);

/**
 * get long
 *
 * @param getopt  getopt_t instance
 * @param lname   long name used in opt parsing
 *
 * example usage:
 * @code
 *   long time = getopt_get_long (gopt, "time");
 * @endcode 
 * get long option with name "time"
 *
 * @see getopt_add_long 
 */
long
getopt_get_long (getopt_t *getopt, const char *lname);

/**
 * get double
 *
 * @param getopt  getopt_t instance
 * @param lname   long name used in opt parsing
 *
 * example usage:
 * @code
 *   double val = getopt_get_double (gopt, "position");
 * @endcode 
 * get double option with name "position"
 *
 * @see getopt_add_double 
 */
double
getopt_get_double (getopt_t *getopt, const char *lname);

/**
 * get string
 *
 * @param getopt  getopt_t instance
 * @param lname   long name used in opt parsing
 *
 * example usage:
 * @code
 *   char *out_fname = getopt_get_string (gopt, "fname");
 * @endcode 
 * get int option with name "fname"
 *
 * @see getopt_add_string 
 */
const char *
getopt_get_string (getopt_t *gopt, const char *lname);

int
getopt_has_flag (getopt_t *gopt, const char *lname);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif // __PERLS_COMMON_GETOPT_H__
