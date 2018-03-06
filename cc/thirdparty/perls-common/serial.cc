/* ================================================================
** serial.[ch]
**
** serial communication utilities.
**
** 07 NOV 2008  Ryan Eustice  Created and written.
** ================================================================
*/

#include <stdio.h>      // snprintf()
#include <stdlib.h>     // perror()
#include <errno.h>      // "
#include <unistd.h>     // lockf()
#include <strings.h>    // strncasecmp()

#include <sys/types.h>  // open()
#include <sys/stat.h>   // "
#include <fcntl.h>      // "

#include "error.h"
#include "serial.h"

int 
serial_open (const char *device, speed_t speed, parity_t parity, bool blocking) 
{
    struct termios tio;

    int flags = (O_RDWR | O_NOCTTY);
    if (!blocking)
        flags |= O_NONBLOCK;

    int fd = open (device, flags);
    if (fd == -1) {
        PERROR ("open (%s, %d, 0)", device, flags);
        return -1;
    }

    // lock serial device
    if (lockf (fd, F_TLOCK, 0) < 0) {
        PERROR ("%s is locked by another program (possibly another running instance?)", device);
        exit (EXIT_FAILURE);
    }


    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr");
        return -1;
    }

    /* empirically, zeroing out the termios struct seems to be very important...  
     * I was getting inconsistent i/o performance otherwise with noncanonical
     * reads
     */
    bzero (&tio, sizeof (tio));

    // set tio.cflag - Enable the receiver and set local mode
    // CLOCAL - ignore modem control lines like carrier detect
    // CREAD  - enable uart receiver
    tio.c_cflag |= (CLOCAL | CREAD);
    
    // set tio.oflag - raw
    tio.c_oflag &= ~OPOST;

    // set tio.iflag to raw
    tio.c_iflag &= ~(ICRNL | INLCR | IGNCR | INPCK | ISTRIP);

    // ignore incoming BREAK events
    tio.c_iflag |= IGNBRK;

    // disable control characters by default
    for (int i=0; i < NCCS; i++)
        tio.c_cc[i] = _POSIX_VDISABLE;
    


    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr");
        return -1;
    }

    if ( serial_set_speed (fd, speed) ) {
        PERROR ("serial_set_speed()");
        return -1;
    }

    if ( serial_set_parity (fd, parity) ) {
        PERROR ("serial_set_parity()");
        return -1;
    }

    if ( serial_set_canonical (fd, '\r', '\n') ) {
        PERROR ("serial_set_canonical()");
        return -1;
    }


    tcflush (fd, TCIOFLUSH);

    return fd;
}


int 
serial_close (int fd) 
{
    return close (fd);
}


int 
serial_set_speed (int fd, speed_t speed) 
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }

    if ( cfsetispeed (&tio, speed) ) {
        PERROR("cfsetispeed()");
        return -1;
    }

    if ( cfsetospeed (&tio, speed) ) {
        PERROR("cfsetospeed()");
        return -1;
    }

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}


int 
serial_set_parity (int fd, parity_t parity) 
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }

    switch (parity) {
    case PARITY_8N1: // no parity
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        break;
    case PARITY_7E1: // even parity
        tio.c_cflag |= PARENB;
        tio.c_cflag &= ~PARODD;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS7;
        break;
    case PARITY_7O1: // odd parity
        tio.c_cflag |= PARENB;
        tio.c_cflag |= PARODD;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS7;
        break;
    case PARITY_7S1: // space parity (setup the same as no parity)
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        break;
    default:
        ERROR ("invalid parity selection");
        return -1;
    }

    if ( tio.c_cflag & PARENB )  // enable parity checking
        tio.c_iflag |= (INPCK | ISTRIP);
    else                        // disable
        tio.c_iflag &= ~(INPCK | ISTRIP);

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}
 
speed_t
serial_translate_speed (int rate) 
{
    switch (rate) {
    case 0:      return B0;
    case 300:    return B300;
    case 1200:   return B1200;
    case 2400:   return B2400;
    case 4800:   return B4800;
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    //case 460800: return B460800; /* not support on Mac OSX */
    default:     return -1;
    }
}

parity_t 
serial_translate_parity (const char *parity) 
{
    if ( strncasecmp (parity, "8N1", 3) == 0 )
        return PARITY_8N1;
    else if ( strncasecmp (parity, "7E1", 3) == 0)
        return PARITY_7E1;
    else if ( strncasecmp (parity, "7O1", 3) == 0)
        return PARITY_7O1;
    else if ( strncasecmp (parity, "7S1", 3) == 0)
        return PARITY_7S1;
    else 
        return -1;
}

int 
serial_set_canonical (int fd, char eol, char eol2)
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }
    
    tio.c_lflag |= ICANON;
    
    const char CR = 0x0a;
    const char CL = 0x0d;

    if (eol == '\r')
        tio.c_cc[VEOL] = CR;
    else if (eol == '\n')
        tio.c_cc[VEOL] = CL;
    else
        tio.c_cc[VEOL] = eol;

    if (eol2 == '\r')
        tio.c_cc[VEOL2] = CR;
    else if (eol2 == '\n')
        tio.c_cc[VEOL2] = CL;
    else
        tio.c_cc[VEOL2] = eol2;

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}

int 
serial_set_noncanonical (int fd, int min, int time) 
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }
    
    tio.c_lflag &= ~ICANON;
    tio.c_cc[VMIN] = min;
    tio.c_cc[VTIME] = time; // deciseconds


    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }
    return 0;
}

int 
serial_set_xonxoff (int fd, bool on) 
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }

    if ( on ) // enable software flow control
        tio.c_iflag |= (IXON | IXOFF | IXANY);
    else    // disable
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}

int 
serial_set_ctsrts (int fd, bool on) 
{
   struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }

    if ( on ) // enable hardware flow control 
        tio.c_cflag |= CRTSCTS;
    else      // disable
        tio.c_cflag &= ~CRTSCTS;

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}

int 
serial_set_echo (int fd, bool on) 
{
   struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        PERROR ("tcgetattr()");
        return -1;
    }

    if ( on ) // enable echo
        tio.c_cflag |= (ECHO | ECHOE);
    else      // disable
        tio.c_cflag &= ~(ECHO | ECHOE);

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        PERROR ("tcsetattr()");
        return -1;
    }

    return 0;
}
