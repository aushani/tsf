#ifndef __PERLS_COMMON_DAEMON_H__
#define __PERLS_COMMON_DAEMON_H__

#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

pid_t
daemon_fork (void);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_DAEMON_H__
