#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <glib.h>

#include "error.h"
#include "lcm_util.h"


int
lcmu_handle_timeout (lcm_t *lcm, struct timeval *timeout)
{
    int fd = lcm_get_fileno (lcm);

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (fd, &rfds);

    int ret = select (fd + 1, &rfds, NULL, NULL, timeout);

    if (ret == -1) {
#ifdef LINUX
        if (errno == EINTR)
            return lcmu_handle_timeout (lcm, timeout);
#endif
        if (errno != EINTR)
            PERROR ("select()");
    }
    else if (ret == 0) {
        /* Timeout */
    }
    else {
        /* We have data. */
        if (FD_ISSET (fd, &rfds))
            lcm_handle (lcm);
    }

    return ret;
}


char *
lcmu_channel_get_prepost (const char *prefix, const char *base_channel, const char *postfix)
{
    return g_strconcat (prefix ? : "", base_channel, postfix ? : "", NULL);
}


char *
lcmu_channel_get_os_conduit (BotParam *param, lcmu_channel_os_conduit_t type)
{
    char *basechannel = bot_param_get_str_or_fail (param, "os-conduit.gsd.channel");
    char *channel = NULL;
    switch (type) {
    case LCMU_CHANNEL_OS_CONDUIT_ACK:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_ACK");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OJW:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OJW");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OMP:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OMP");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OPI:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OPI");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OPOS:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OPOS");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OSD:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OSD");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OSI:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OSI");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_CPRTD:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_CPRTD");
        break;

    case LCMU_CHANNEL_OS_CONDUIT_OMSTART:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OMSTART");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OMSTOP:
        channel = lcmu_channel_get_prepost (NULL, basechannel, "_OMSTOP");
        break;

    case LCMU_CHANNEL_OS_CONDUIT_RAW:
        channel = lcmu_channel_get_prepost (NULL, basechannel, ".RAW");
        break;
    case LCMU_CHANNEL_OS_CONDUIT_OUT:
        channel = lcmu_channel_get_prepost (NULL, basechannel, ".OUT");
        break;

    default:
        ERROR ("unknown type [%d]", type);
        g_free (basechannel);
        exit (EXIT_FAILURE);
    }
    g_free (basechannel);
    return channel;
}


char *
lcmu_channel_get_heartbeat (const char *prefix, double freq)
{
    char freq_str[64] = {'\0'};
    snprintf (freq_str, sizeof freq_str, "%g", freq);
    if (prefix)
        return g_strconcat (prefix, "HEARTBEAT_", freq_str, "HZ", NULL);
    else
        return g_strconcat ("HEARTBEAT_", freq_str, "HZ", NULL);
}


int32_t
lcmu_fwrite (const char *filename, const void *in, lcmu_encode f_encode,
             lcmu_encoded_size f_encoded_size, int64_t fingerprint)
{
    FILE *stream = fopen (filename, "w");
    if (!stream) {
        ERROR ("unable to create file %s!", filename);
        return -1;
    }

    int32_t max_data_size = (*f_encoded_size) (in);
    uint8_t *buf = malloc (max_data_size);
    if (!buf) {
        fclose (stream);
        return -1;
    }
    int32_t data_size = (*f_encode) (buf, 0, max_data_size, in);
    if (data_size < 0) {
        free (buf);
        ERROR ("unable to encode %s!", filename);
        fclose (stream);
        return data_size;
    }

    size_t numel=0;
    numel += fwrite (&fingerprint, sizeof fingerprint, 1, stream);
    numel += fwrite (&data_size, sizeof data_size, 1, stream);
    numel += fwrite (buf, data_size, 1, stream);

    fclose (stream);
    if (numel != 3) {
        free (buf);
        ERROR ("unable to write data to %s!", filename);
        return -2;
    }

    free (buf);
    return (sizeof fingerprint + sizeof data_size + data_size);
}

int32_t
lcmu_fread (const char *filename, void **out, int size, lcmu_decode f_decode, int64_t fingerprint)
{
    FILE *stream = fopen (filename, "r");
    if (!stream) {
        ERROR ("unable to read file %s!", filename);
        return -1;
    }

    size_t numel=0;
    int64_t magic = 0;
    numel += fread (&magic, sizeof magic, 1, stream);
    if (magic != fingerprint) {
        printf ("WARNING: invalid fingerprint for %s!\n", filename);
        //ERROR ("invalid fingerprint for %s!", filename);
        // fclose (stream); // just warn
        // return -3;
    }

    *out = malloc (size);
    int32_t data_size=0;
    numel += fread (&data_size, sizeof data_size, 1, stream);
    uint8_t *buf = malloc (data_size);
    numel += fread (buf, data_size, 1, stream);
    fclose (stream);

    int ret = (*f_decode) (buf, 0, data_size, *out);
    if (numel != 3 || ret !=data_size) {
        free (buf);
        free (*out);
        *out = NULL;
        ERROR ("unable to decode %s!\n", filename);
        return -2;
    }

    free (buf);
    return data_size;
}
