#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "error.h"
#include "units.h"

/* use the installed version, so we can use the CMAKE_INSTALL_PREFIX */
#include "perls-common/bot_util.h"

int
botu_param_get_boolean_to_bool (BotParam *param, const char *key, bool *val)
{
    int tmp, bot_ret;
    bot_ret = bot_param_get_boolean (param, key, &tmp);
    if (bot_ret == 0)
        *val = tmp ? true : false;
    return bot_ret;
}

char *
botu_param_get_str_or_default (BotParam *param, const char *key, const char *def)
{
    char *str = NULL;
    if (0==bot_param_get_str (param, key, &str))
        return str;
    else
        return strdup (def);
}

int
botu_param_get_boolean_or_default (BotParam *param, const char *key, const int def)
{
    int val;
    if (0==bot_param_get_boolean (param, key, &val))
        return val;
    else
        return def;
}

double
botu_param_get_double_or_default (BotParam *param, const char *key, const double def)
{
    double val;
    if (0==bot_param_get_double (param, key, &val))
        return val;
    else
        return def;
}

int
botu_param_get_int_or_default (BotParam *param, const char *key, const int def)
{
    int val;
    if (0==bot_param_get_int (param, key, &val))
        return val;
    else
        return def;
}

void
botu_param_read_covariance (gsl_matrix *cov_out, BotParam *param, char *base_key)
{    
    int len = cov_out->size1;
    char key[256];
    
    //try to read full version
    sprintf (key, "%s.full", base_key);
    double *cov = calloc (len*len, sizeof (*cov));
    int ret = bot_param_get_double_array (param, key, cov, len*len);
    if (len*len == ret) {
        memcpy (cov_out->data, cov, len*len*sizeof (*cov_out->data));
        free (cov);
        return;
    } else
        free (cov);
    
    // try to read diagonal human readable std devation version
    sprintf (key, "%s.diag_std", base_key);
    double *diag_std = calloc (len, sizeof (*diag_std));
    ret = bot_param_get_double_array (param, key, diag_std, len);
    if (len==ret) {
        //check for a dtor vector if we need to convert some elements to radians
        sprintf (key, "%s.dtor", base_key);
        int *dtor = calloc (len, sizeof (*dtor));
        ret = bot_param_get_int_array (param, key, dtor, len);
        if (len==ret) {
            for (int i=0; i<len; i++) {
                if (dtor[i])
                    diag_std[i] = diag_std[i]*UNITS_DEGREE_TO_RADIAN;

                diag_std[i] = diag_std[i]*diag_std[i];
            }
        }
        
        gsl_vector_view diag_v = gsl_matrix_diagonal (cov_out);
        gsl_vector_view diag_std_v = gsl_vector_view_array (diag_std, len);
        gsl_vector_memcpy (&diag_v.vector, &diag_std_v.vector);

        free (diag_std);
        free (dtor);
        return;
    }
    else
        free (diag_std);
    
    ERROR ("Failed to read noise covariance %s", base_key);
}

void
botu_param_add_pserver_to_getopt (getopt_t *gopt)
{
    char strf[128];
    getopt_add_bool   (gopt, '\0', BOTU_PARAM_LONG_OPT_USE_SERVER, 1, "Use bot param server");
    getopt_add_string (gopt, '\0', BOTU_PARAM_LONG_OPT_SERVER_NAME, "", "Name of bot param server");
    snprintf (strf, 128, "Name of config file to use (ignored when %s is enabled)", BOTU_PARAM_LONG_OPT_USE_SERVER);
    getopt_add_string (gopt, '\0', BOTU_PARAM_LONG_OPT_MASTER_CFG, BOTU_PARAM_DEFAULT_CFG, strf);
}

BotParam *
botu_param_new_from_getopt_or_fail (getopt_t *gopt, lcm_t *lcm)
{
    BotParam *param = NULL;
    int keep_updated = 1;

    if (getopt_get_bool (gopt, BOTU_PARAM_LONG_OPT_USE_SERVER)) {
        printf ("Creating param from server\n");
        const char *server_name = getopt_get_string (gopt, BOTU_PARAM_LONG_OPT_SERVER_NAME);
        if (strcmp (server_name, ""))
            param = bot_param_new_from_named_server (lcm, server_name, keep_updated);
        else
            param = bot_param_new_from_server (lcm, keep_updated);
    }
    else {
        const char *cfg_file = getopt_get_string (gopt, BOTU_PARAM_LONG_OPT_MASTER_CFG);
        printf ("Creating param from file %s\n", cfg_file);
        param = bot_param_new_from_file (cfg_file);
    }

    if (!param)
        abort ();

    return param;
}
