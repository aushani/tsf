#ifndef __PERLS_COMMON_UNIX_H__
#define __PERLS_COMMON_UNIX_H__

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif    

/* Function with behaviour like `mkdir -p'
 * e.g., mode = 0775 
 * returns 0 success; -1 error 
 */
int
unix_mkpath (const char *path, mode_t mode);


/* checks for existence of a running process id
 * returns 0 if pid is alive; -1 if not found
 */
int
unix_pidstat (pid_t pid);

/**
 * @brief Get directory contents in dirname, including '.' and '..'
 * @param dirname name of directory
 * @param listing output content
 * @param n number of strings assigned to listing
 *
 * @note free listing in a for loop
 */
int
unix_lsdir (const char *dirname, char ***listing, size_t *n);

/**
 * @brief Create a file with given path.  Make directories appropriately
 */
int
unix_touch (const char *path, mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_UNIX_H__
