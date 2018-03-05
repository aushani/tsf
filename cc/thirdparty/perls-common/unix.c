#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#include "unix.h"

/* Function with behaviour like `mkdir -p'  */
int
unix_mkpath (const char *s, mode_t mode)
{
    char *q, *r = NULL, *path = NULL, *up = NULL;
    int rv;
 
    rv = -1;
    if (strcmp (s, ".") == 0 || strcmp (s, "/") == 0)
        return (0);
 
    if ((path = strdup (s)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((q = strdup(s)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((r = dirname (q)) == NULL)
        goto out;
 
    if ((up = strdup (r)) == NULL)
        exit (EXIT_SUCCESS);
 
    if ((unix_mkpath (up, mode) == -1) && (errno != EEXIST))
        goto out;
 
    if ((mkdir (path, mode) == -1) && (errno != EEXIST))
        rv = -1;
    else
        rv = 0;
 
  out:
    if (up != NULL)
        free (up);
    free (q);
    free (path);
    return (rv);
}

int
unix_touch (const char *path, mode_t mode)
{
    char *s_copy = strdup (path);
    char *last_slash = strrchr (s_copy, '/');
    int rv;
    if (last_slash) {
        *last_slash = '\0';
        char *dir = s_copy;
        rv = unix_mkpath (dir, mode);

        if (rv != EXIT_SUCCESS) {
            perror ("unix_mkpath()");
            rv = EXIT_FAILURE;
            goto out;
        }
    }

    FILE *fp = fopen (path, "w");
    if (!fp) {
        perror ("fopen()");
        rv = EXIT_FAILURE;
        goto out;
    }
    fclose (fp);
    rv = EXIT_SUCCESS;

  out:
    free (s_copy);
    return rv;
}

int
unix_pidstat (pid_t pid)
{
    char procdir[64];
    snprintf (procdir, sizeof (procdir), "/proc/%d", pid);
    struct stat info;
    return stat (procdir, &info);
}

static size_t
_numFilesInDir (DIR *dir)
{
    seekdir (dir, 0);
    size_t n = 0;
    struct dirent *ent;

    while ((ent = readdir (dir)) != NULL)
        n++;

    seekdir (dir, 0);
    return n;
}

int
unix_lsdir (const char *dirname, char ***listing, size_t *n)
{
    DIR *dir = opendir (dirname);
    if (!dir) {
        perror ("opendir()");
        return EXIT_FAILURE;
    }

    struct dirent *ent;
    *n = _numFilesInDir (dir);
    *listing = malloc (*n * sizeof (**listing));

    int i = 0;
    while ((ent = readdir (dir)) != NULL) {
        (*listing)[i] = strdup (ent->d_name);
        i++;
    }

    closedir (dir);
    return EXIT_SUCCESS;
}
