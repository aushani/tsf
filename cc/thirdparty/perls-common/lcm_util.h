#ifndef __PERLS_COMMON_LCM_UTIL_H__
#define __PERLS_COMMON_LCM_UTIL_H__

#include <sys/time.h>
#include <lcm/lcm.h>
#include <bot_param/param_client.h>

/**
 * @defgroup PerlsCommonLcmUtil LCM Utility
 * @brief LCM helper library.
 * @ingroup PerlsCommon
 * @include: perls-common/lcm_util.h
 *
 * @{
 */


/**
 * @param lcm     LCM object.
 * @param timeout Timeout structure, same as you would use with the select() system call.
 * @return        -1 error, 0 timeout, >0 LCM event.
 *
 * Performs the same function as lcm_handle(), but with a timeout.
 */
int
lcmu_handle_timeout (lcm_t *lcm, struct timeval *timeout);

/**
 * @param prefix       string to prefix, can be NULL.
 * @param base_channel base channel string name string.
 * @param postfix      string to append, can be NULL.
 * @return modified channel name, user must free().
 *
 * Convenience function that modifies the base channel name using a prefix and postfix.
 */
char *
lcmu_channel_get_prepost (const char *prefix, const char *base_channel, const char *postfix);

/**
 * Values to determine which OS_CONDUIT channel to return.
 */
typedef enum {
    /** Acknowledgement of mission control commands. **/
    LCMU_CHANNEL_OS_CONDUIT_ACK,
    /** Jump to waypoint. **/
    LCMU_CHANNEL_OS_CONDUIT_OJW,
    /** Primitive mode commands. **/
    LCMU_CHANNEL_OS_CONDUIT_OMP,
    /** Vehicle power system status. **/
    LCMU_CHANNEL_OS_CONDUIT_OPI,
    /** Send Position **/
    LCMU_CHANNEL_OS_CONDUIT_OPOS,
    /** Vehicle sensor data. **/
    LCMU_CHANNEL_OS_CONDUIT_OSD,
    /** Vehicle state data. **/
    LCMU_CHANNEL_OS_CONDUIT_OSI,
    /**Vehicle state data. **/
    LCMU_CHANNEL_OS_CONDUIT_CPRTD,

    /** Start mission. **/
    LCMU_CHANNEL_OS_CONDUIT_OMSTART,
    /** Stop current mission. **/
    LCMU_CHANNEL_OS_CONDUIT_OMSTOP,

    /** NMEA from UVC. **/
    LCMU_CHANNEL_OS_CONDUIT_RAW,
    /** NMEA to UVC. **/
    LCMU_CHANNEL_OS_CONDUIT_OUT,
} lcmu_channel_os_conduit_t;


/**
 * @param param BotParam pointer to the config file.
 * @param type  Which os-conduit channel to return.
 * @return requested os-conduit channel name, user must free().
 *
 * Convenience function that returns the requested os-conduit channel name.
 */
char *
lcmu_channel_get_os_conduit (BotParam *param, lcmu_channel_os_conduit_t type);


/**
 * @param prefix string to prefix to HEARTBEAT channel name, can be NULL.
 * @param freq frequency in Hertz of requested HEARTBEAT channel.
 * @return HEARTBEAT channel name, user must free().
 *
 * Convenience function that returns the requested heartbeat channel name.
 */
char *
lcmu_channel_get_heartbeat (const char *prefix, double freq);



typedef int (*lcmu_encode) (void *buf, int offset, int maxlen, const void *p);
typedef int (*lcmu_decode) (const void *buf, int offset, int maxlen, void *p);
typedef int (*lcmu_encoded_size) (const void *p);

/**
 * @param filename filename to read
 * @param fencode lcm encode function pointer for this type
 * @param fencoded_size lcm encoded_size function pointer for this type
 * @param in pointer to input lcm struct
 * @param fingerprint unique 64-bit number to be used as header information for this lcm type
 *
 * Writes a binary lcm file to disk.
 */
int32_t
lcmu_fwrite (const char *filename, const void *in, lcmu_encode f_encode,
             lcmu_encoded_size f_encoded_size, int64_t fingerprint);

/**
 * @param fname filename to read
 * @param in pointer to lcm struct
 * @param lcm_type type of input lcm struct
 * @param magic unique 64-bit number to be used as header information for this lcm type
 *
 * Convenience macro that writes a binary lcm file to disk.
 */
#define LCMU_FWRITE(filename, in, lcm_type)                             \
    lcmu_fwrite (filename, (void *)in,                                  \
                 (lcmu_encode) &(lcm_type ## _encode),                  \
                 (lcmu_encoded_size) &(lcm_type ## _encoded_size),      \
                 __ ## lcm_type ## _get_hash ())

/**
 * @param filename filename to read
 * @param size size of lcm struct
 * @param out address of output pointer
 * @param f_decode pointer to lcm_decode function
 * @param magic unique 64-bit number to be used as header information for this lcm type
 *
 * Reads a binary lcm file from disk.
 */
int32_t
lcmu_fread (const char *filename, void **out, int size, lcmu_decode f_decode, int64_t fingerprint);

/**
 * @param fname filename to read
 * @param lcm_type name of lcm type to read
 * @param out address of output pointer
 * @param magic unique 64-bit number to be used as header information for this lcm type
 *
 * Convenience macro that reads a binary lcm file from disk.
 */
#define LCMU_FREAD(filename, out, lcm_type)                             \
    lcmu_fread (filename, (void **)out,                                 \
                sizeof (lcm_type),                                      \
                (lcmu_decode) &(lcm_type ## _decode),                   \
                __ ## lcm_type ## _get_hash ())

#define LCMU_BUFWRITE(buf, len, msg, lcm_type)                          \
    len = lcm_type ## _encoded_size (msg);                              \
    buf = (uint8_t *) malloc (len);                                     \
    if (!buf)                                                           \
        perror ("malloc()");                                            \
    lcm_type ## _encode (buf, 0, len, msg)

/**
 * @}
 */
#endif // __PERLS_COMMON_LCM_UTIL_H__
