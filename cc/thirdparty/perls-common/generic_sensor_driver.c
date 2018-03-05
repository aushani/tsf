#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>
#include <strings.h>
#include <glib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <inttypes.h> // needed for PRId32 macros

#include "daemon.h"
#include "error.h"
#include "timestamp.h"
#include "serial.h"
#include "bot_util.h"

#include "lcmtypes/senlcm_stats_t.h"
#include "lcmtypes/senlcm_raw_t.h"

#include "generic_sensor_driver.h"


#define GSD_PUBLISH_STATS     1
#define GSD_STDOUT_MAX_RATE   2                             // Hz
#define GSD_STDOUT_MIN_PERIOD (1000000/GSD_STDOUT_MAX_RATE) // us

static GStaticMutex gsd_write_mutex = G_STATIC_MUTEX_INIT;
static generic_sensor_driver_t *GSD; // global pointer for signal handler


static gint
gsd_queue_sort (gconstpointer a, gconstpointer b, gpointer user_data)
{
    const gsd_data_t *dataA = a;
    const gsd_data_t *dataB = b;
    if (dataA->timestamp > dataB->timestamp)
        return 1;
    else if (dataA->timestamp == dataB->timestamp)
        return 0;
    else
        return -1;
}

static void
gsd_queue_free (gpointer element)
{
    gsd_data_t *data = (gsd_data_t *) element;
    g_free (data);
}

static void
gsd_open (generic_sensor_driver_t *gsd)
{
    char key[256];
    const char *io;
    if (getopt_has_flag (gsd->gopt, "io"))
        io = getopt_get_string (gsd->gopt, "io");
    else {
        snprintf (key, 255, "%s.gsd.%s", gsd->rootkey, "io");
        io = botu_param_get_str_or_default (gsd->params, key, "unknown");
    }

    if (!strcasecmp (io, "none"))
        gsd->io = GSD_IO_NONE;
    else if (!strcasecmp (io, "serial"))
        gsd->io = GSD_IO_SERIAL;
    else if (!strcasecmp (io, "udp"))
        gsd->io = GSD_IO_UDP;
    else if (!strcasecmp (io, "udp2"))
        gsd->io = GSD_IO_UDP2;
    else if (!strcasecmp (io, "tcp"))
        gsd->io = GSD_IO_TCPIP;
    else if (!strcasecmp (io, "playback"))
        gsd->io = GSD_IO_PLAYBACK;
    else {
        ERROR ("unknown io");
        exit (EXIT_FAILURE);
    }

    switch (gsd->io) {
    case GSD_IO_NONE: {
        break;
    }
    case GSD_IO_SERIAL: {
        const char *device;
        if (getopt_has_flag (gsd->gopt, "device"))
            device = getopt_get_string (gsd->gopt, "device");
        else {
            snprintf (key, 255, "%s.gsd.%s", gsd->rootkey, "device");
            device = botu_param_get_str_or_default (gsd->params, key, "/dev/ttyS99");
        }

        int baud;
        if (getopt_has_flag (gsd->gopt, "baud"))
            baud = getopt_get_int (gsd->gopt, "baud");
        else {
            snprintf (key, 255, "%s.gsd.%s", gsd->rootkey, "baud");
	    baud = 9600;
	    bot_param_get_int (gsd->params, key, &baud);
        }

        const char *parity;
        if (getopt_has_flag (gsd->gopt, "parity"))
            parity = getopt_get_string (gsd->gopt, "parity");
        else {
            snprintf (key, 255, "%s.gsd.%s", gsd->rootkey, "parity");
            parity = botu_param_get_str_or_default (gsd->params, key, "8N1");
        }

        printf ("Opening %s %d %s\n", device, baud, parity);

        gsd->fd = serial_open (device, serial_translate_speed (baud),
                               serial_translate_parity (parity), 1);
        if (gsd->fd == -1)
            exit (EXIT_FAILURE);
        break;
    }
    case GSD_IO_UDP: {
        ERROR ("GSD_IO_UDP not implemented yet");
        exit (EXIT_FAILURE);
        break;
    }
    case GSD_IO_UDP2: {
        ERROR ("GSD_IO_UDP2 not implemented yet");
        exit (EXIT_FAILURE);
        break;
    }
    case GSD_IO_TCPIP: {
        ERROR ("GSD_IO_TCPIP not implemented yet");
        exit (EXIT_FAILURE);
        break;
    }
    case GSD_IO_PLAYBACK: {
        break;
    }
    default:
        ERROR ("unknown GSD_IO");
        exit (EXIT_FAILURE);
    } // switch
}

static void
gsd_write_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_raw_t *raw, void *user)
{
    generic_sensor_driver_t *gsd = user;
    gsd_write (gsd, (char *) raw->data, raw->length);
}

static void
gsd_playback_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                       const senlcm_raw_t *raw, void *user)
{
    generic_sensor_driver_t *gsd = user;

    /* send to parent process */
    gsd_data_t *data = calloc (1, sizeof (*data));
    data->timestamp = raw->utime;
    data->len = raw->length;
    memcpy (data->buf, raw->data, raw->length);
    g_async_queue_push (gsd->gq, data);
}


static gpointer
gsd_thread (gpointer context)
{
    generic_sensor_driver_t *gsd = context;

    /* identify that we are using queue */
    g_async_queue_ref (gsd->gq);

    /* create and subscribe to lcm */
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }
    int lcm_fd = lcm_get_fileno (lcm);
    senlcm_raw_t_subscribe (lcm, gsd->write_channel, &gsd_write_callback, gsd);
    if (gsd->io == GSD_IO_PLAYBACK)
        senlcm_raw_t_subscribe (lcm, gsd->read_channel, &gsd_playback_callback, gsd);

    int32_t ntimeouts_prev = 0;
    int64_t timestamp_prev = timestamp_now ();
    while (!gsd->done) {
        /* Watch fd and lcm to see who has input. */
        fd_set rfds;
        FD_ZERO (&rfds);
        FD_SET (gsd->fd, &rfds);
        FD_SET (lcm_fd, &rfds);

        /* Wait up to some period of time. */
        struct timeval tv = {
            .tv_sec = 1,
            .tv_usec = 0,
        };
        int maxfd = MAX (gsd->fd, lcm_fd);
        int ret = select (maxfd + 1, &rfds, NULL, NULL, &tv);
        int64_t timestamp = timestamp_now ();
        if (ret < 0)
            PERROR ("select()");
        else if (ret == 0) {/* Timeout */
            gsd->stats.dt = timestamp - timestamp_prev;
            gsd->stats.latency = 0;
            gsd_update_stats (gsd, 0);
        }
        else { /* We have data. */
            if (FD_ISSET (lcm_fd, &rfds))
                lcm_handle (lcm);
            else if (FD_ISSET (gsd->fd, &rfds)) {
                gsd_data_t *data = calloc (1, sizeof (*data));

                switch (gsd->io) {
                case GSD_IO_NONE:
                    ERROR ("GSD_IO_NONE");
                    break;
                case GSD_IO_SERIAL:
                    data->len = read (gsd->fd, data->buf, GSD_DATA_BUFSIZE);
                    break;
                case GSD_IO_UDP:
                    ERROR ("GSD_IO_UDP not implemented yet");
                    break;
                case GSD_IO_UDP2:
                    ERROR ("GSD_IO_UDP2 not implemented yet");
                    break;
                case GSD_IO_TCPIP:
                    ERROR ("GSD_IO_TCPIP not implemented yet");
                    break;
                case GSD_IO_PLAYBACK:
                    ERROR ("GSD_IO_PLAYBACK got data???");
                    break;
                default:
                    ERROR ("unknown GSD_IO");
                }

                /* update our statistics */
                gsd->stats.dt = timestamp - timestamp_prev;
                if (ntimeouts_prev == gsd->stats.ntimeouts) {// last time wasn't a timeout
                    gsd->stats.dtmax = MAX (gsd->stats.dt, gsd->stats.dtmax);
                    gsd->stats.dtmin = gsd->stats.dtmin ? MIN (gsd->stats.dt, gsd->stats.dtmin) : gsd->stats.dt;
                }
                timestamp_prev = timestamp;
                ntimeouts_prev = gsd->stats.ntimeouts;

                /* publish raw data */
                senlcm_raw_t raw;
                raw.utime = timestamp;
                raw.length = data->len;
                raw.data = (uint8_t *) data->buf;
                senlcm_raw_t_publish (gsd->lcm, gsd->read_channel, &raw);

                /* send to parent process */
                data->timestamp = timestamp;
                g_async_queue_push (gsd->gq, data);
            }
        }
    } // while (1)

    lcm_destroy (lcm);

    return NULL;
}

static void
gsd_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    if (GSD != NULL) {

	if (GSD->destroy_callback)
	    (*GSD->destroy_callback) (GSD, GSD->destroy_callback_user);

	gsd_destroy (GSD);
    }
    exit (EXIT_SUCCESS);
}


generic_sensor_driver_t *
gsd_create (int argc, char *argv[], const char *rootkey, gsd_getopt_callback callback)
{
    if (!g_thread_supported ()) g_thread_init (NULL);

    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    generic_sensor_driver_t *gsd = calloc (1, sizeof (generic_sensor_driver_t));
    GSD = gsd; // global pointer

    gsd->gopt = getopt_create ();
    gsd->rootdir = strdup (get_current_dir_name ());
    gsd->basename = strdup (basename (argv[0]));

    /* Add custom command-line options */
    if (callback != NULL && callback (gsd) != 0) {
        ERROR ("user callback error.");
        exit (EXIT_FAILURE);
    }

    /* Add std command-line options */
    char myrootkey[256];
    if (rootkey)
        strncpy (myrootkey, rootkey, 255);
    else {
        const char *token = "perls-sen-";
        const char *found = strstr (gsd->basename, token);
        if (found)
            snprintf (myrootkey, 255, "sensors.%s", gsd->basename + strlen (token));
        else
            snprintf (myrootkey, 255, "sensors.%s", gsd->basename);
    }

    getopt_add_help   (gsd->gopt, NULL);
    getopt_add_bool   (gsd->gopt, 'D',  "daemon",  0,         "Run as daemon");
    getopt_add_string (gsd->gopt, 'k',  "key",     myrootkey, "Config file key");
    getopt_add_string (gsd->gopt, 'c',  "channel", "",        "LCM channel, e.g., SENSOR_FOO");
    getopt_add_string (gsd->gopt, '\0', "io",      "",        "serial, udp, udp2, tcp, playback");
    getopt_add_spacer (gsd->gopt, "IO_SERIAL:");
    getopt_add_string (gsd->gopt, '\0', "device",  "",        "Serial device, e.g. /dev/ttyS0");
    getopt_add_int    (gsd->gopt, '\0', "baud",    "",        "Baud rate, e.g. 9600");
    getopt_add_string (gsd->gopt, '\0', "parity",  "",        "Parity, e.g., 8N1, 7E1, 7O1, 7S1");
    getopt_add_spacer (gsd->gopt, "");
    botu_param_add_pserver_to_getopt (gsd->gopt);

    /* parse our flags */
    if (!getopt_parse (gsd->gopt, argc, argv, 1) || gsd->gopt->extraargs->len!=0) {
        getopt_do_usage (gsd->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gsd->gopt, "help")) {
        getopt_do_usage (gsd->gopt, NULL);
        exit (EXIT_SUCCESS);
    }
    gsd->rootkey = strdup (getopt_get_string (gsd->gopt, "key"));

    /* our glib asynchronous queue for passing data back to parent process */
    gsd->gq = g_async_queue_new_full (&gsd_queue_free);

    /* create lcm parent object */
    gsd->lcm = lcm_create (NULL);
    if (!gsd->lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    /* param server or file? */
    gsd->params = botu_param_new_from_getopt_or_fail (gsd->gopt, gsd->lcm);
    if (!gsd->params) {
        ERROR ("botu_param_new_from_getopt_or_fail() failed");
        exit (EXIT_FAILURE);
    }

    /* lcm channel names */
    char channel[LCM_MAX_CHANNEL_NAME_LENGTH];
    if (getopt_has_flag (gsd->gopt, "channel")) {
        const char *opt = getopt_get_string (gsd->gopt, "channel");
        snprintf (channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s", opt);
    }
    else {
        char key[255];
        snprintf (key, 255, "%s.gsd.%s", gsd->rootkey, "channel");
        char *opt = botu_param_get_str_or_default (gsd->params, key, "SENSOR_UNSPECIFIED");
        snprintf (channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s", opt);
    }
    snprintf (gsd->channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s", channel);
    snprintf (gsd->write_channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s.WRITE", channel);
    snprintf (gsd->read_channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s.RAW", channel);
    snprintf (gsd->stats_channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s.STATS", channel);

    /* open device (must be called after setting gsd->params)*/
    gsd_open (gsd);

    /* run as daemon? */
    if (getopt_get_bool (gsd->gopt, "daemon"))
        daemon_fork ();

    /* Install custom signal handler */
    struct sigaction act = {
        .sa_sigaction = gsd_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    //sigaction (SIGABRT, &act, NULL);
    //sigaction (SIGQUIT, &act, NULL);

    // default printing stats to on
    gsd->print_stats = 1;

    return gsd;
}


void
gsd_launch (generic_sensor_driver_t *gsd)
{
    gsd->done = 0;
    gsd->th = g_thread_create (&gsd_thread, gsd, 1, NULL);
}

void
gsd_destroy (generic_sensor_driver_t *gsd)
{
    gsd->done = 1;
    if (gsd->th)
      g_thread_join (gsd->th);

    lcm_destroy (gsd->lcm);
    bot_param_destroy (gsd->params);
    getopt_destroy (gsd->gopt);
    g_async_queue_unref (gsd->gq);

    fprintf (stderr, "\ngsd: %s: Goodbye\n", gsd->rootkey);
    fflush (stderr);

    usleep (1E5);

    g_free (gsd);
}

int
gsd_canonical (generic_sensor_driver_t *gsd, char eol, char eol2)
{
    int ret = 0;

    switch (gsd->io) {
    case GSD_IO_NONE:
        break;
    case GSD_IO_SERIAL:
        ret = serial_set_canonical (gsd->fd, eol, eol2);
        break;
    case GSD_IO_UDP:
        ERROR ("GSD_IO_UDP not implemented yet");
        break;
    case GSD_IO_UDP2:
        ERROR ("GSD_IO_UDP2 not implemented yet");
        break;
    case GSD_IO_TCPIP:
        ERROR ("GSD_IO_TCPIP not implemented yet");
        break;
    case GSD_IO_PLAYBACK:
        break;
    default:
        ERROR ("unknown io type");
    }

    return ret;
}


int
gsd_noncanonical (generic_sensor_driver_t *gsd, int min, int time)
{
    int ret = 0;

    switch (gsd->io) {
    case GSD_IO_NONE:
        break;
    case GSD_IO_SERIAL:
        ret = serial_set_noncanonical (gsd->fd, min, time);
        break;
    case GSD_IO_UDP:
        ERROR ("GSD_IO_UDP not implemented yet");
        break;
    case GSD_IO_UDP2:
        ERROR ("GSD_IO_UDP2 not implemented yet");
        break;
    case GSD_IO_TCPIP:
        ERROR ("GSD_IO_TCPIP not implemented yet");
        break;
    case GSD_IO_PLAYBACK:
        break;
    default:
        ERROR ("unknown io type");
    }

    return ret;
}


int
gsd_read (generic_sensor_driver_t *gsd, char *buf, int len, int64_t *timestamp)
{
    g_async_queue_sort (gsd->gq, &gsd_queue_sort, NULL);
    gsd_data_t *data = g_async_queue_pop (gsd->gq);

    gsd->stats.latency = timestamp_now () - data->timestamp;
    gsd->stats.latencymax = MAX (gsd->stats.latency, gsd->stats.latencymax);

    if (timestamp != NULL)
        *timestamp = data->timestamp;


    memset (buf, 0, len);
    if (len < data->len) {
        memcpy (buf, data->buf, len);
        memmove (data->buf, &data->buf[len], data->len -= len);
        g_async_queue_push (gsd->gq, data);
        return len;
    }
    else {
        memcpy (buf, data->buf, data->len);
        int len = data->len;
        g_free (data);
        return len;
    }
}


int
gsd_read_timeout (generic_sensor_driver_t *gsd, char *buf, int len,
                  int64_t *timestamp, uint64_t usTimeout)
{
    g_async_queue_sort (gsd->gq, &gsd_queue_sort, NULL);
    gsd_data_t *data = g_async_queue_timeout_pop (gsd->gq, usTimeout);

    if (data == NULL) /* timeout */
	return 0;

    gsd->stats.latency = timestamp_now () - data->timestamp;
    gsd->stats.latencymax = MAX (gsd->stats.latency, gsd->stats.latencymax);

    if (timestamp != NULL)
        *timestamp = data->timestamp;


    memset (buf, 0, len);
    if (len < data->len) {
        memcpy (buf, data->buf, len);
        memmove (data->buf, &data->buf[len], data->len -= len);
        g_async_queue_push (gsd->gq, data);
        return len;
    }
    else {
        memcpy (buf, data->buf, data->len);
        int len = data->len;
        g_free (data);
        return len;
    }
}

void
gsd_flush (generic_sensor_driver_t *gsd)
{
    gsd_data_t *data = 0;

    // clear out the g_async_queue
    bool empty=0;
    while (!empty) {
        g_async_queue_sort (gsd->gq, &gsd_queue_sort, NULL);
        data = (gsd_data_t *) g_async_queue_timeout_pop (gsd->gq, 0);
        if (data == NULL)
            empty = 1;
        g_free (data);
    }
}

int
gsd_write (generic_sensor_driver_t *gsd, const char *buf, int len)
{
    int ret = 0;

    g_static_mutex_lock (&gsd_write_mutex);

    switch (gsd->io) {
    case GSD_IO_NONE:
        break;
    case GSD_IO_SERIAL:
        ret = write (gsd->fd, buf, len);
        break;
    case GSD_IO_UDP:
        ERROR ("GSD_IO_UDP not implemented yet");
        break;
    case GSD_IO_UDP2:
        ERROR ("GSD_IO_UDP not implemented yet");
        break;
    case GSD_IO_TCPIP:
        ERROR ("GSD_IO_UDP not implemented yet");
        break;
    case GSD_IO_PLAYBACK:
        ERROR ("GSD_IO_PLAYBACK doesn't write!");
        break;
    default:
        ERROR ("unknown GSD_IO");
    }

    g_static_mutex_unlock (&gsd_write_mutex);

    return ret;
}

void
gsd_update_stats (generic_sensor_driver_t *gsd, int good)
{
    int64_t utime_prev = gsd->stats.utime;
    gsd->stats.utime = timestamp_now ();

    if (good > 0)
        gsd->stats.ngood++;
    else if (good < 0)
        gsd->stats.nbad++;
    else
        gsd->stats.ntimeouts++;

#ifdef GSD_PUBLISH_STATS
    senlcm_stats_t_publish (gsd->lcm, gsd->stats_channel, &gsd->stats);
#endif

    gsd->stats_dt_stdout += gsd->stats.utime - utime_prev;
    if (gsd->print_stats && gsd->stats_dt_stdout > GSD_STDOUT_MIN_PERIOD) {
        printf ("\rgood:%"PRId32" bad:%"PRId32" timeout:%"PRId32,
                gsd->stats.ngood, gsd->stats.nbad, gsd->stats.ntimeouts);
        gsd->stats_dt_stdout = 0;
    }
}

void
gsd_reset_stats (generic_sensor_driver_t *gsd)
{
    bzero (&gsd->stats, sizeof (gsd->stats));
}

void
gsd_print_stats (generic_sensor_driver_t *gsd, int print_stats) {
    gsd->print_stats = print_stats;
}

void
gsd_attach_destroy_callback (generic_sensor_driver_t *gsd,
                             void (*destroy_callback) (generic_sensor_driver_t *gsd, void *user),
                             void *destroy_callback_user) {
    gsd->destroy_callback = destroy_callback;
    gsd->destroy_callback_user = destroy_callback_user;

}
