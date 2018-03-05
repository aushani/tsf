#include <math.h>

#include "pose.hpp"

Pose::Pose()
{
}

Pose::~Pose()
{
}

Pose::oxt_t
Pose::read_oxt_file(FILE *fp)
{
    oxt_t oxt;

    int res =  fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %d %d",
                        &oxt.lat,
                        &oxt.lon,
                        &oxt.alt,
                        &oxt.roll,
                        &oxt.pitch,
                        &oxt.yaw,
                        &oxt.vn,
                        &oxt.ve,
                        &oxt.vf,
                        &oxt.vl,
                        &oxt.vu,
                        &oxt.ax,
                        &oxt.ay,
                        &oxt.az,
                        &oxt.af,
                        &oxt.al,
                        &oxt.au,
                        &oxt.wx,
                        &oxt.wy,
                        &oxt.wz,
                        &oxt.wf,
                        &oxt.wl,
                        &oxt.wu,
                        &oxt.posacc,
                        &oxt.velacc,
                        &oxt.navstat,
                        &oxt.numsats,
                        &oxt.posmode,
                        &oxt.velmode,
                        &oxt.orimode);

    if (res!=30) {
        printf("Could not parse oxts file (only got %d)\n", res);
    }

    return oxt;
}

void
Pose::lat_lon_to_mercator(double lat, double lon, double scale, double *x, double *y)
{
    // Adapted from KITTI dev kit
    // converts lat/lon coordinates to mercator coordinates using mercator scale

    double er = 6378137;
    *x = scale * lon * M_PI * er / 180;
    *y = scale * er * log( tan((90+lat) * M_PI / 360) );
}

std::vector<Pose>
Pose::load_all_poses(std::string kitti_log_dir)
{
    std::vector<Pose> poses;

    double scale = -1;

    char filename[200];

    int i = 0;
    while  (1) {
        // Load oxts
        sprintf(filename, "%s/oxts/data/%010d.txt", kitti_log_dir.c_str(), i);
        FILE *fp = fopen(filename, "r");

        if (fp == NULL) {
            // We're done
            break;
        }

        oxt_t oxt = read_oxt_file(fp);

        if (i==0) {
            scale = cos(oxt.lat * M_PI / 180.0);
        }

        double pose[6];
        lat_lon_to_mercator(oxt.lat, oxt.lon, scale, &pose[0], &pose[1]);
        pose[2] = oxt.alt;
        pose[3] = oxt.roll;
        pose[4] = oxt.pitch;
        pose[5] = oxt.yaw;

        Pose p;
        p.x = pose[0];
        p.y = pose[1];
        p.z = pose[2];
        p.r = pose[3];
        p.p = pose[4];
        p.h = pose[5];

        p.v_f = oxt.vf;
        p.v_h = oxt.wu;

        poses.push_back(p);

        fclose(fp);

        i++;
    }

    return poses;
}

std::vector<Pose>
Pose::load_all_poses(std::string kitti_log_dir, std::string kitti_log_date, int log_num)
{
    char filename[200];
    snprintf(filename, sizeof(filename), "%s/%s/%s_drive_%04d_sync/",
            kitti_log_dir.c_str(), kitti_log_date.c_str(), kitti_log_date.c_str(), log_num);

    printf("filename: %s\n", filename);

    return load_all_poses(std::string(filename));
}
