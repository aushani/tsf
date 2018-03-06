#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "util.h"

//int64_t utime_now() {
//    struct timeval tv;
//    gettimeofday (&tv, NULL);
//    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
//}

velodyne_returns_t*
load_velodyne_file(const char *fn, float *x_vs) {
    FILE *fp = fopen(fn, "r");

    if (!fp) {
        printf("No file %s\n", fn);
        return NULL;
    }

    char line[1024];

    // Read x_vs's
    for (int sensor=0; sensor<4; sensor++) {
        if (fgets(line, 1024, fp)) {
            int i = 0;
            for (char *tok = strtok(line, ","); tok; tok = strtok(NULL, ",")) {
                x_vs[3*sensor + i++] = atof(tok);
            }
        }
    }

    int n_points = 0;
    if (fgets(line, 1024, fp)) {
        n_points = atoi(line);
    }

    velodyne_returns_t *vr = velodyne_returns_create(n_points);
    vr->num_returns = n_points;

    for (int i=0; i<n_points; i++) {
        if (fgets(line, 1024, fp)) {
            int j = 0;
            for (char *tok = strtok(line, ","); tok; tok = strtok(NULL, ",")) {
                if (j<3) {
                    vr->xyz[3*i + j++] = atof(tok);
                } else {
                   // vr->sensor_num[i] = 0;
                    break;
                }
            }
        }
    }

    fclose(fp);

    return vr;
}
