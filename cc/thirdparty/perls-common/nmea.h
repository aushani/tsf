#ifndef __PERLS_COMMON_NMEA_H__
#define __PERLS_COMMON_NMEA_H__

#include <stdio.h>

/**
 * @defgroup PerlsCommonNmea NMEA
 * @brief NMEA helper library.
 * @ingroup PerlsCommon
 * @include: perls-common/nmea.h
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param buf    Empty buffer.
 * @param format sprintf() format.
 *
 * Same as sprintf(), but automatically appends a checksum onto a NMEA
 * formatted payload string.  Payload string should include the `$`
 * and `*`.
 */
#define nmea_sprintf(buf, format, ...)          \
    do {                                        \
        sprintf (buf, format, ## __VA_ARGS__);  \
        char *tmp = strdup (buf);               \
        nmea_append_checksum (buf, tmp);        \
        free (tmp);                             \
    } while (0)

/**
 * @param buf    Empty buffer.
 * @param size   Empty buffer size.
 * @param format snprintf() format.
 *
 * Same as snprintf(), but automatically appends a checksum onto a
 * NMEA formatted payload string.  Payload string should include the
 * `$` and `*`.
 */
#define nmea_snprintf(buf, size, format, ...)           \
    do {                                                \
        snprintf (buf, size, format, ## __VA_ARGS__);   \
        char *tmp = strdup (buf);                       \
        nmea_append_checksum (buf, tmp);                \
        free (tmp);                                     \
    } while (0)

/**
 * @param buf NMEA buffer.
 * @param str NMEA payload string, user must free().
 * @return strlen() on succes; 0 otherwise.
 *
 * Extracts the payload string between the `$` and `*`.
 */
int
nmea_payload (const char *buf, char **str);

/**
 * @return checksum on success; -1 on error.
 *
 * Computes the NMEA checksum ignoring leading `$` and trailing `*`.
 */
int
nmea_compute_checksum (const char *buf);

/**
 * @return 1 if valid, 0 if invalid.
 *
 * Validates NMEA message checksum.
 */
int
nmea_validate_checksum (const char *buf);

/**
 * @return      strlen() on success; 0 otherwise.
 *
 * Copies src to dest with appended NMEA checksum.  The strings may
 * overlap.  
 */
int
nmea_append_checksum (char *dest, const char *src);

/**
 * @param buf NMEA buffer.
 * @param n   Field number to parse (e.g., 1).
 * @param arg Argument associated with field n.
 * @return    1 success; 0 otherwise.
 *
 * Extracts the string argument associated with field n.  Correctly
 * handles empty NMEA fields.
 */
int
nmea_arg (const char *buf, int n, char *arg);

/**
 * @param buf NMEA buffer.
 * @param n   Field number to parse (e.g., 1).
 * @param v   Character valued argument associated with field n.
 * @return    1 success; 0 otherwise.
 *
 * Extracts character argument for field n.
 */
int
nmea_argc (const char *buf, int n, char *v);

/**
 * @param buf NMEA buffer.
 * @param n   Field number to parse (e.g., 1).
 * @param v   Integer valued argument associated with field n.
 * @return    1 success; 0 otherwise.
 *
 * Extracts integer argument for field n.
 */
int
nmea_argi (const char *buf, int n, int *v);

/**
 * @param buf NMEA buffer.
 * @param n   Field number to parse (e.g., 1).
 * @param v   Double valued argument associated with field n.
 * @return 1 success; 0 otherwise
 *
 * Extracts double argument for field n.
 */
int
nmea_argf (const char *buf, int n, double *v);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif //__PERLS_COMMON_NMEA_H__
