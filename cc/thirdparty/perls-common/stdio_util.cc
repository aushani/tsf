#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <glib.h>

#include "stdio_util.h"
#include "error.h"


/* prints buf as a hexadecimal stream in big-endian order */
int
stdiou_shexdump (char *str, const void *buf, size_t len, const char *prefix)
{
    char *sptr = str;
    if (prefix)
        sptr += sprintf (sptr, "%s", prefix);
    else
        sptr += sprintf (sptr, "0x");

    if (G_BYTE_ORDER == G_LITTLE_ENDIAN) {
        const unsigned char *bptr = buf+len-1;
        for (int i=0; i<len; i++, bptr--)
            sptr += sprintf (sptr, "%02x", *bptr);
    }
    else { /* G_BIG_ENDIAN */
        const unsigned char *bptr = buf;
        for (int i=0; i<len; i++, bptr++)
            sptr += sprintf (sptr, "%02x", *bptr);
    }
    return sptr-str;
}


int
stdiou_hexscan (const char *str, size_t len, void *buf, const char *prefix)
{
    const char *sptr = str;
    //skip the prefix
    if (prefix) {
        sptr += strlen (prefix);
        len -= strlen (prefix);
    }
    else
        sptr += strlen ("0x");

    //len is the length of the hex string, two characters per byte of data
    if (len % 2)
        ERROR ("ERROR: Hex string length not even! ");
    int byte_len = len/2;


    unsigned int u;
    if (G_BYTE_ORDER == G_LITTLE_ENDIAN) {
        char *bptr = buf+byte_len-1;
        for (int i=0 ; i<byte_len && sscanf(sptr, "%2x", &u) == 1; i++) {
            *bptr = u; bptr--; sptr += 2;
        }
    }
    else { /* G_BIG_ENDIAN */
        char *bptr = buf;
        for (int i=0 ; i<byte_len && sscanf(sptr, "%2x", &u) == 1; i++) {
            *bptr = u; bptr++; sptr += 2;
        }
    }

    return byte_len;

}

int
stdiou_hexdump (const void *buf, size_t len, const char *prefix)
{
    size_t plen = 2; // "0x" default
    if (prefix)
        plen = strlen (prefix);
    char str[2*len+plen+1]; // '\0' terminator
    stdiou_shexdump (str, buf, len, prefix);
    return printf ("%s", str);
}

int
stdiou_fhexdump (FILE *stream, const void *buf, size_t len, const char *prefix)
{
    size_t plen = 2;
    if (prefix)
        plen = strlen (prefix);
    char str[2*len+plen+1];
    stdiou_shexdump (str, buf, len, prefix);
    return fprintf (stream, "%s", str);
}

/* prints buf as a bit stream in big-endian order */
int
stdiou_sbindump (char *str, const void *buf, size_t len, const char *prefix)
{
    char *sptr = str;
    if (prefix)
        sptr += sprintf (sptr, "%s", prefix);
    else
        sptr += sprintf (sptr, "0b");

    if (G_BYTE_ORDER == G_LITTLE_ENDIAN) {
        const unsigned char *bptr = buf+len-1;
        for (int i=0; i<len; i++, bptr--)
            for (int b=7; b>=0; b--)
                sptr += sprintf (sptr, "%d", (*bptr & (1<<b)) ? 1 : 0 );
    }
    else { /* G_BIG_ENDIAN */
        const unsigned char *bptr = buf;
        for (size_t i=0; i<len; i++, bptr++)
            sptr += sprintf (sptr, "%d", *bptr);
    }
    return sptr-str;
}

int
stdiou_bindump (const void *buf, size_t len, const char *prefix)
{
    size_t plen = 2; // "0b" default
    if (prefix)
        plen = strlen (prefix);
    char str[8*len+plen+1]; // '\0' terminator
    stdiou_sbindump (str, buf, len, prefix);
    return printf ("%s", str);
}

int
stdiou_fbindump (FILE *stream, const void *buf, size_t len, const char *prefix)
{
    size_t plen = 2;
    if (prefix)
        plen = strlen (prefix);
    char str[8*len+plen+1];
    stdiou_sbindump (str, buf, len, prefix);
    return fprintf (stream, "%s", str);
}
