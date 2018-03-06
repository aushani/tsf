#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#include "error.h"
#include "timestamp.h"
#include "timeutil.h"

struct timer_thread_context {
    struct timespec spec;
    timeutil_timer_callback callback;
    void *user;
};

static void
timer_thread_sighandler (int signo)
{
    pthread_exit (NULL);
}

static void *
timer_thread (void *arg)
{
    // copy our context to a local variable on the stack
    struct timer_thread_context *_ttc = arg;
    struct timer_thread_context ttc = *_ttc;
    free (_ttc);

    signal (SIGUSR1, timer_thread_sighandler);
    while (ttc.callback (ttc.user)) {
        struct timespec req=ttc.spec, rem;
        timeutil_nanosleep_fully (&req, &rem);
    }
    pthread_exit (0);
}

timeutil_timer_t
timeutil_timer_create (struct timespec spec, timeutil_timer_callback callback, void *user)
{
    struct timer_thread_context *ttc = malloc (sizeof (*ttc));
    ttc->spec = spec;
    ttc->callback = callback;
    ttc->user = user;

    timeutil_timer_t timer;
    pthread_create (&timer.th, NULL, timer_thread, ttc);
    pthread_detach (timer.th);
    return timer;
}


void
timeutil_timer_destroy (timeutil_timer_t *timer)
{
    pthread_kill (timer->th, SIGUSR1);
}


struct timespec
timeutil_hz_to_timespec (double hz)
{
    double dt = 1.0 / hz;
    struct timespec spec;
    spec.tv_sec  = dt;
    spec.tv_nsec = (dt - spec.tv_sec)*1E9;
    return spec;
}

int
timeutil_nanosleep_fully (const struct timespec *req, struct timespec *rem)
{
    if (nanosleep (req, rem) < 0)
        if (errno == EINTR) {
            struct timespec temp_rem;
            return timeutil_nanosleep_fully (rem, &temp_rem);
        }
        else {
            PERROR ("timeutil_nanosleep_fully()");
            return -1;
        }
    else
        return 0;
}

int
timeutil_nsleep (int64_t nsec)
{
    struct timespec rem;
    struct timespec req = {
        .tv_sec = nsec / 1000000000,
        .tv_nsec = nsec % 1000000000,
    };
    return timeutil_nanosleep_fully (&req, &rem);
}

size_t
timeutil_strftime (char *s, size_t max, const char *format, int64_t utime)
{
    struct timeval tv;
    timestamp_to_timeval (utime, &tv);

    struct tm tm;
    localtime_r (&tv.tv_sec, &tm);


    const char *formatend = format + strlen (format);
    char format2[1024], tmp[1024];
    
    // handle %i arg if present
    char *istr = strstr (format, "%i");
    if (istr != NULL) {
        if (istr > format) {
            memset (tmp, '\0', sizeof (tmp));
            strncpy (tmp, format, istr - format);
            sprintf (format2, "%s%06ld%s", tmp, tv.tv_usec, istr+2 < formatend ? istr+2 : "");
        }
        else {
            sprintf (format2, "%06ld%s", tv.tv_usec, format+2);
        }
    }
    else
        strcpy (format2, format);

    return strftime (s, max, format2, &tm);
}
