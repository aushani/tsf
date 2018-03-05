#ifndef __PERLS_COMMON_UDP_H__
#define __PERLS_COMMON_UDP_H__

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** make and bind a udp socket to an ephemeral port. Returns the fd. **/
int
udp_socket_create (void);

/** make and bind a udp socket to a specified port. Returns the fd. **/
int
udp_socket_listen (int port);

/** return the local port number for a socket. **/
int
udp_socket_get_port (int sock);

// convenience method that sends a one-off udp message
// return 0 on success
int
udp_send (const char *ipaddr, int port, const void *data, int datalen);

static inline int
udp_send_string (const char *ipaddr, int port, const char *string) 
{
    return udp_send (ipaddr, port, string, strlen (string));
}

#ifdef __cplusplus
}
#endif

#endif //__PERLS_COMMON_UDP_H__
