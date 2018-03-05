#ifndef __PERLS_COMMON_BOT_UTIL_H__
#define __PERLS_COMMON_BOT_UTIL_H__

#include <stdbool.h>

#include <gsl/gsl_matrix.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "getopt.h"

/**
 * @defgroup PerlsCommonBotUtil Libbot Utility
 * @brief libbot2 helper library.
 * @ingroup PerlsCommon
 * @include: perls-common/bot_util.h
 *
 * @{
 */

#define BOTU_PARAM_DEFAULT_CFG "@CMAKE_INSTALL_PREFIX@/share/perls/config/master.cfg"
#define BOTU_PARAM_LONG_OPT_SERVER_NAME "pserver-name"
#define BOTU_PARAM_LONG_OPT_USE_SERVER "use-pserver"
#define BOTU_PARAM_LONG_OPT_MASTER_CFG "master-cfg"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @param param   The configuration
 * @param key     The key to get the value for
 * @param val     The bool to which we assign key's value
 *
 * @brief For convenience of programs using stdbool.h's bool as true/false 
 *
 * @detailed Passing bool (from stdbool.h) to bot_param_get_boolean() is unreliable
 * because the size of a boolean is 1 byte, while bot_param_get_boolean() expects a full int.
 */
int
botu_param_get_boolean_to_bool (BotParam *param, const char *key, bool *val);

/**
 * @param param   The configuration.
 * @param key     The key to get the value for.
 * @param default A "default" value.
 * @return The value associated with key, or def.  The user must free.
 *
 * @brief Like bot_param_get_str(), except it returns the value of def if key is not found.
 * @see bot_param_get_str()
 */
char *
botu_param_get_str_or_default (BotParam *param, const char *key, const char *def);

/**
 * @param param   The configuration.
 * @param key     The key to get the value for.
 * @param def     A "default" value.
 * @return The value associated with key, or def.
 *
 * @brief Like bot_param_get_boolean(), except it returns the value of def if key is not found.
 * @see bot_param_get_boolean()
 */
int
botu_param_get_boolean_or_default (BotParam *param, const char *key, const int def);

/**
 * @param param   The configuration.
 * @param key     The key to get the value for.
 * @param def     A "default" value.
 * @return The value associated with key, or def.
 *
 * @brief Like bot_param_get_double(), except it returns the value of def if key is not found.
 * @see bot_param_get_double()
 */
double
botu_param_get_double_or_default (BotParam *param, const char *key, const double def);

/**
 * @param param   The configuration.
 * @param key     The key to get the value for.
 * @param def     A "default" value.
 * @return The value associated with key, or def.
 *
 * @brief Like bot_param_get_int(), except it returns the value of def if key is not found.
 * @see bot_param_get_int()
 */
int
botu_param_get_int_or_default (BotParam *param, const char *key, const int def);

/**
 * @param cov_out   gsl_matrix to store covariance in
 * @param param     BotParam instance
 * @param base_key  config key of covariance
 *
 * @brief Read a covariance from a config file. Lets you specify full matrix or diagonal.
 */
void
botu_param_read_covariance (gsl_matrix *cov_out, BotParam *param, char *base_key);

/**
 * @param gopt      getopt_t to add options for using server
 */
void
botu_param_add_pserver_to_getopt (getopt_t *gopt);

/**
 * @param gopt      getopt_t to add options for using server
 * @param lcm       lcm_t object
 */
BotParam *
botu_param_new_from_getopt_or_fail (getopt_t *gopt, lcm_t *lcm);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif // __PERLS_COMMON_BOT_UTIL_H__
