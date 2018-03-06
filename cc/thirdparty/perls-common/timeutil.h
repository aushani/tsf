#ifndef __PERLS_COMMON_TIMEUTIL_H__
#define __PERLS_COMMON_TIMEUTIL_H__

#include <time.h>

typedef int (*timeutil_timer_callback) (void *);

typedef struct timeutil_timer timeutil_timer_t;
struct timeutil_timer {
    pthread_t th;
};

/* Create a timeutil_timer_t object.  
 * struct timespec is the same as that used in nanosleep()
 * timer_callback is a function handle to the user callback function: int callback (void *user)
 * void *user is a pointer to any user data to pass to the callback function, otherwise set it to NULL
 *
 * timer object will execute callback at the sample rate specified in spec.
 *
 * callback function should return non-zero on success, if it returns 0 the timer will exit
 *
 * Alternatively the user can call timeutil_timer_destroy() from the parent process to exit the timer
 */
timeutil_timer_t
timeutil_timer_create (struct timespec spec, timeutil_timer_callback callback, void *user);

void
timeutil_timer_destroy (timeutil_timer_t *timer);


/* converts Hertz to a timespec struct */
struct timespec
timeutil_hz_to_timespec (double hz);

/* signal-tolerant version of nanosleep() */
int
timeutil_nanosleep_fully (const struct timespec *req, struct timespec *rem);

/* same as above, but takes nsecs, usecs, or secs as input */
int
timeutil_nsleep (int64_t nsec);

static inline int
timeutil_usleep (int64_t usec) 
{
    return timeutil_nsleep (usec * 1000);
}

static inline int
timeutil_sleep (int64_t sec)
{
    return timeutil_nsleep (sec * 1000000000);
}


/** Same syntax as strftime, but adds %i conversion character for microseconds since the Epoch.
 **/
size_t
timeutil_strftime (char *s, size_t max, const char *format, int64_t utime);

#endif // __PERLS_COMMON_TIMEUTIL_H__
