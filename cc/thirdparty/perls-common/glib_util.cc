#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>

#include "error.h"
#include "glib_util.h"


guint
gu_int64_hash (gconstpointer v)
{
    return (guint) *(const gint64*) v;
}

gboolean
gu_int64_equal (gconstpointer v1, gconstpointer v2)
{
    return *((const gint64*) v1) == *((const gint64*) v2);
}

gpointer
gu_dup (gconstpointer v, size_t size)
{
    gpointer d = g_malloc (size);
    memcpy (d, v, size);
    return d;
}
