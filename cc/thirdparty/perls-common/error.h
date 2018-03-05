#ifndef __PERLS_COMMON_ERROR_H__
#define __PERLS_COMMON_ERROR_H__

#include <stdio.h>
#include <string.h>
#include <error.h>
#include <errno.h>

// Prints just the name of the file (i.e., w/o the path)
#define __FILENAME__ ((strrchr (__FILE__, '/') ?: __FILE__ - 1) + 1)

// Prints just the /src/path/file
#define __SRC_FILE__ ((strstr (__FILE__, "/src/") ?: __FILE__))


/* note the ", ## args"
 * This macro works even if no variable argument list is given -- the precompile will remove the comma ","
 */
#define ERROR(format, ...) \
    fprintf (stderr, "%s:%s:%d: " format "\n", __SRC_FILE__, __func__, __LINE__, ## __VA_ARGS__)

#define PERROR(format, ...) \
    fprintf (stderr, "%s:%s:%d: " format ": ", __SRC_FILE__, __func__, __LINE__, ## __VA_ARGS__), \
    perror (NULL)

#endif // __PERLS_COMMON_ERROR_H__
