#ifndef __PERLS_COMMON_GLIB_UTIL_H__
#define __PERLS_COMMON_GLIB_UTIL_H__

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

guint
gu_int64_hash (gconstpointer v);

gboolean
gu_int64_equal (gconstpointer v1, gconstpointer v2);


/* duplicate a regular data type, e.g., 
   uint64 d = gu_dup (&utime, sizeof utime);
   returns a pointer to a copy of utime
 */
gpointer
gu_dup (gconstpointer v, size_t size);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_GLIB_UTIL_H__
