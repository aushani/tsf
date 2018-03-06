#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nmea.h"

int
nmea_payload (const char *buf, char **str)
{
    char tmp[1024];
    if (1 == sscanf (buf, "$%[^*]s", tmp)) {
        *str = strdup (tmp);
        return strlen (*str);
    }
    else
        return 0;
}

int
nmea_compute_checksum (const char *buf)
{
    char *msg = NULL;
    int msglen = nmea_payload (buf, &msg);
    if (msglen == 0) {
        if (msg) free (msg);
        return -1;
    }

    int checksum = 0;
    for (int i=0; i<msglen; i++)
        checksum ^= msg[i];

    free (msg);
    return checksum;
}


int
nmea_validate_checksum (const char *buf)
{
    int cs;
    if (1 != sscanf (buf, "$%*[^*]s*%2X", &cs)) // invalid parse
        return -1;

    int checksum = nmea_compute_checksum (buf);
    if (cs == checksum)
        return 0;
    else
        return -1;
}

int
nmea_append_checksum (char *dest, const char *src)
{
    size_t len = strlen (src);
    if (src[len-1] != '*') // invalid termination
        return -1;

    int checksum = nmea_compute_checksum (src);
    char *tmp = strdup (src);
    int ret = sprintf (dest, "%s%02X\r\n", tmp, checksum);
    free (tmp);
    if (ret < 0)
        return 0;
    else
        return ret;
}

/* correctly handles empty NMEA field */
int
nmea_arg (const char *str, int n, char *out)
{
    const char *c1, *c2, *c;

    c = str;
    // find a delim
    while ((*c != ',') && (*c != '*') && (*c != '\0'))
        c++;    // $CCCCC,
    c++;        // skip the comma

    // for preceding args
    for (int i=0; i<(n-1); i++) {
        while ((*c != ',') && (*c != '*') && (*c != '\0'))
            c++; // skip preceding args
        c++;     // skip comma
    }
    char pa = *(c-1);
    if ( pa == '*' || pa == '\0') {
        return 0;
    }

    c1 = c; // points to first char of arg we want, or next comma
    while ((*c != ',') && (*c != '*') && (*c != '\0'))
        c++;     // skip to end
    c2 = c - 1;  // points to last char before comma

    if (c1 <= c2) {
        out[0] = '\0';
        strncat (out, c1, (c2-c1+1));
        return 1;
    }
    else {
        out[0] = '\0'; // empty
        return 1;
    }
}


int
nmea_argc (const char *buf, int n, char *c)
{
    char str[256];
    if (!nmea_arg (buf, n, str))
        return 0;
    else
        *c = str[0];

    return 1;
}


int
nmea_argi (const char *buf, int n, int *i)
{
    char str[256];
    if (!nmea_arg (buf, n, str))
        return 0;    // empty or missing NMEA field
    else             // field is stuffed
        *i = atoi (str);

    return 1;
}


int
nmea_argf (const char *buf, int n, double *d)
{
    char str[256];
    if (!nmea_arg (buf, n, str))
        return 0;    // empty or missing NMEA field
    else             // field is stuffed
        *d = atof (str);

    return 1;
}
