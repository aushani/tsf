#ifndef __PERLS_COMMON_SERIAL_H__
#define __PERLS_COMMON_SERIAL_H__

#include <stdbool.h>
#include <termios.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {PARITY_8N1=0, PARITY_7E1, PARITY_7O1, PARITY_7S1} parity_t;

/* Creates a basic fd, defaulting to 9600 N81, raw data i/o, blocking, 
   canonical reads.

   Returns the fd on success, -1 on error
*/
int 
serial_open (const char *device, speed_t speed, parity_t parity, bool blocking);

/* Returns 0 on success, -1 on error */
int 
serial_close (int fd);

/* Sets the baud rate to baud, where baud is a speed_t data type as defined
   in termios.h  (e.g., B9600)
    
   Returns 0 success, -1 on error
*/
int 
serial_set_speed (int fd, speed_t speed);

/* Sets the port parity to parity, where parity is a parity_t data type as
   defined in serial.h  (e.g., PARITY_N81)
   
   Returns 0 on succes, -1 on error
*/
int 
serial_set_parity (int fd, parity_t parity);

 
/* Provides a mapping between integer rate and speed, where speed is of type
   speed_t as defined in termios.h (e.g., 9600 & B9600)

    Returns speed on success, -1 on error
*/
speed_t 
serial_translate_speed (int rate);

/* Provides a mapping between ascii parity and integer parity, where parity
   is an integer macro as defined in serial.h (e.g., "8N1" & PARITY_8N1)

    Returns parity on success, -1 on error
*/
parity_t 
serial_translate_parity (const char *parity);

/* Enables canonical (blocking) line-based character reading.
   
   Set eol and eol2 to '\r' and '\n' (or vice versa) to use default end of line
   character processing; otherwise set eol and eol2 to the desired custom
   termination character(s).

   Returns 0 on success, -1 on error
*/
int 
serial_set_canonical (int fd, char eol, char eol2);

/* Enables noncanonical (blocking) character reading.
   
   If MIN > 0 and TIME = 0, MIN sets the number of characters to
   receive before the read is satisfied. As TIME is zero, the timer
   is not used.

   If MIN = 0 and TIME > 0, TIME serves as a timeout value. The read
   will be satisfied if a single character is read, or TIME is
   exceeded (t = TIME *0.1 s). If TIME is exceeded, no character will
   be returned.
    
   If MIN > 0 and TIME > 0, TIME serves as an inter-character
   timer. The read will be satisfied if MIN characters are received,
   or the time between two characters exceeds TIME. The timer is
   restarted every time a character is received and only becomes
   active after the first character has been received.

   If MIN = 0 and TIME = 0, read will be satisfied immediately. The
   number of characters currently available, or the number of
   characters requested will be returned.

   Returns 0 on success, -1 on error
*/
int
serial_set_noncanonical (int fd, int min, int time);

/* Enables XON/XOFF hardware flowcontrol.

   Returns 0 on success, -1 on error
*/
int 
serial_set_xonxoff (int fd, bool on);

/* Enables CTS/RTS software flowcontrol.

   Returns 0 on success, -1 on error
*/
int 
serial_set_ctsrts (int fd, bool on);

/* Enables Echo

   Returns 0 on success, -1 on error
*/
int 
serial_set_echo (int fd, bool on);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_COMMON_SERIAL_H__
