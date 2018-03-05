#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "daemon.h"

pid_t
daemon_fork (void)
{
    // Fork off the parent process
    pid_t pid = fork ();
    if (pid < 0) {
        perror ("fork()");
        exit (EXIT_FAILURE);
    }

    // If we got a good PID, then we can exit the parent process.
    if (pid > 0) {
        exit (EXIT_SUCCESS);
    }

    // Change the file mode mask
    umask (027);
                
    // Open any logs here    
                
    // Create a new SID for the child process
    pid_t sid = setsid ();
    if (sid < 0) {
        // Log the failure
        exit (EXIT_FAILURE);
    }

    /* RME: commented out chdir so that processes like prosilca can
     * write files to where they were launched
     */
    #if 0
    // Change the current working directory
    if ((chdir ("/")) < 0) {
        // Log the failure
        exit (EXIT_FAILURE);
    }
    #endif
        
    // Close out the standard file descriptors
    close (STDIN_FILENO);
    close (STDOUT_FILENO);
    //close (STDERR_FILENO);

    return sid;
}

