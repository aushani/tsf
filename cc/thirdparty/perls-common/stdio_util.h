#ifndef __PERLS_COMMON_STDIO_UTIL_H__
#define __PERLS_COMMON_STDIO_UTIL_H__

#include <stdio.h>
#include <sys/types.h>
#include "ascii.h"

/* function macro that prints in bold
 */
#define stdiou_printf_bold(...)  \
    printf ("%c[1m", ASCII_ESC), printf (__VA_ARGS__), printf ("%c[0m", ASCII_ESC)

/* prints buf as a hexadecimal stream in big-endian order 
 * 
 * if prefix is NULL, the default '0x' is used
*/
int
stdiou_hexdump (const void *buf, size_t len, const char *prefix);
int
stdiou_shexdump (char *str, const void *buf, size_t len, const char *prefix);
int
stdiou_fhexdump (FILE *stream, const void *buf, size_t len, const char *prefix);

/* scans string into 
 * 
 */
int
stdiou_hexscan (const char *str, size_t len, void *buf, const char *prefix);

/* prints buf as a binary stream in big-endian order
 *
 * if prefix is NULL, the default '0b' is used
*/
int
stdiou_bindump (const void *buf, size_t len, const char *prefix);
int
stdiou_sbindump (char *str, const void *buf, size_t len, const char *prefix);
int
stdiou_fbindump (FILE *stream, const void *buf, size_t len, const char *prefix);

#endif // __PERLS_COMMON_STDIO_UTIL_H__
