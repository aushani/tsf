#ifndef __PERLS_COMMON_GENERIC_SENSOR_DRIVER_H__
#define __PERLS_COMMON_GENERIC_SENSOR_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

#include <bot_param/param_client.h>
#include <glib.h>

#include <lcm/lcm.h>
#include "lcmtypes/senlcm_stats_t.h"

#include "getopt.h"

/**
 * @defgroup PerlsCommonGsd Generic Sensor Driver
 * @brief Generic sensor driver (gsd) helper library.
 * @details The GSD library forms the backbone of most sensor drivers.
 * @ingroup PerlsCommon
 * @include: perls-common/generic_sensor_driver.h
 *
 * @{
 */

#define GSD_IO_NONE       0
#define GSD_IO_SERIAL     1
#define GSD_IO_UDP        2
#define GSD_IO_UDP2       4
#define GSD_IO_TCPIP      8
#define GSD_IO_PLAYBACK  16

#ifdef __cplusplus
extern "C" {
#endif

#define GSD_DATA_BUFSIZE 1024
typedef struct _gsd_data_t gsd_data_t;
struct _gsd_data_t {
    int64_t       timestamp;
    unsigned char buf[GSD_DATA_BUFSIZE];
    unsigned int  len;
};

typedef struct _generic_sensor_driver_t generic_sensor_driver_t;
struct _generic_sensor_driver_t
{
    BotParam    *params;
    getopt_t    *gopt;
    char        *rootkey;
    char        *rootdir;
    char        *basename;

    int         io;                // SD_IO_SERIAL, UDP, UDP2, or TCPIP
    int         fd;                // device fd

    lcm_t       *lcm;              // LCM parent object
    char        channel[LCM_MAX_CHANNEL_NAME_LENGTH];          // data channel
    char        write_channel[LCM_MAX_CHANNEL_NAME_LENGTH];    // write channel
    char        read_channel[LCM_MAX_CHANNEL_NAME_LENGTH];     // read channel
    char        stats_channel[LCM_MAX_CHANNEL_NAME_LENGTH];    // statistics channel

    GAsyncQueue *gq;               // glib asynchronous queue
    GThread     *th;               // sensor_driver_thread id
    bool        done;              // main loop

    senlcm_stats_t stats;
    int64_t        stats_dt_stdout;   // for throttling how fast we print to stdout
    int            print_stats;       // bool to disable stat printing


    void    (*destroy_callback) (generic_sensor_driver_t *gsd, void *user);
    void     *destroy_callback_user;

};

/**
 * GSD getopt callback function handle.  This callback interface can be used to add
 * additional non-default getopt options to the user's program.
 *
 * @see getopt, gsd_create()
 */
typedef int (*gsd_getopt_callback) (generic_sensor_driver_t *);

/*
 * Creates a generic sensor driver object.
 *
 * Custom command line options can be specified via the callback function pointer,
 * otherwise set to NULL.  The callback function should return 0 on success.
 */
generic_sensor_driver_t *
gsd_create (int argc, char *argv[], const char *rootkey, gsd_getopt_callback callback);

/* Launch the gsd I/O thread */
void
gsd_launch (generic_sensor_driver_t *gsd);

void
gsd_destroy (generic_sensor_driver_t *gsd);

/* Returns 0 on success, -1 on error */
int
gsd_canonical (generic_sensor_driver_t *gsd, char eol, char eol2);

/* Returns 0 on success, -1 on error */
int
gsd_noncanonical (generic_sensor_driver_t *gsd, int min, int time);

/* Returns number of chars read.
   timestamp is the the time associated with start of sensor transmission.  Set it to
    NULL if you do not want the timestamp.
 */
int
gsd_read (generic_sensor_driver_t *gsd, char *buf, int len, int64_t *timestamp);

/* Returns number of chars read, 0 if timeout */
int
gsd_read_timeout (generic_sensor_driver_t *gsd, char *buf, int len,
                  int64_t *timestamp, uint64_t usTimeout);

/* Returns number of chars written */
int
gsd_write (generic_sensor_driver_t *gsd, const char *buf, int len);

/* Flushes the read buffer */
void
gsd_flush (generic_sensor_driver_t *gsd);

/* Updates driver stats: call with good>0 if success, good<0 if error, good=0 if timeout */
void
gsd_update_stats (generic_sensor_driver_t *gsd, int good);

/* Resets driver stats */
void
gsd_reset_stats (generic_sensor_driver_t *gsd);

/* enable or disable stat printing */
void
gsd_print_stats (generic_sensor_driver_t *gsd, int print_stats);

/* attach destroy callback */
void
gsd_attach_destroy_callback (generic_sensor_driver_t *gsd,
                             void (*destroy_callback) (generic_sensor_driver_t *gsd, void *user),
                             void *destroy_callback_user);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif // __PERLS_COMMON_GENERIC_SENSOR_DRIVER_H__
