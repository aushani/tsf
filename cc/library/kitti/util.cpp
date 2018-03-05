#include <math.h>
#include "util.hpp"

velodyne_returns_t*
read_kitti_file(FILE *fp, int64_t utime)
{

    fseek(fp, 0L, SEEK_END);
    size_t sz = ftell(fp);

    rewind(fp);

    int n_hits = sz / (sizeof(float)*4);

    velodyne_returns_t *vr = velodyne_returns_create(n_hits);

    for (int i=0; i<n_hits; i++) {

        float x, y, z, intens;
        if (fread(&x, sizeof(float), 1, fp) == 0) break;
        if (fread(&y, sizeof(float), 1, fp) == 0) break;
        if (fread(&z, sizeof(float), 1, fp) == 0) break;
        if (fread(&intens, sizeof(float), 1, fp) == 0) break;

        vr->xyz[3*i + 0] = x;
        vr->xyz[3*i + 1] = y;
        vr->xyz[3*i + 2] = z;
        vr->intensity[i] = intens*255;
    }

    vr->num_returns = n_hits;

    vr->utime = utime;

    return vr;
}
